#include <include/StereoPair.h>

// Construtor default
StereoPair::StereoPair() {
	this->m_leftCam = nullptr;
	this->m_rightCam = nullptr;
	this->m_pairMerged = false;
}

// Construtor
StereoPair::StereoPair(Camera* left, Camera* right) {
	this->m_leftCam = left;
	this->m_rightCam = right;
	this->m_pairMerged = false;
}

// Destrutor
StereoPair::~StereoPair() {}

// Faz o matching entre as imagens das duas c�meras
size_t StereoPair::match() {
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING); // brute force matcher
	//cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED); // FLANN-based matcher
	std::vector<std::vector<cv::DMatch>> matches;

	// matching
	// query = descritores da imagem da esquerda
	// train = descritores da imagem da direita
	matcher->knnMatch(this->m_leftCam->m_descriptors, this->m_rightCam->m_descriptors, matches, 2);

	// filtrando os matches ruins
	std::vector<cv::DMatch> goodMatches; // lista de matches filtrados
	for (size_t i = 0; i < matches.size(); i++)
		if (matches[i][0].distance < 0.7f * matches[i][1].distance)
			goodMatches.push_back(matches[i][0]);
	matches.clear(); // limpando a lista incial de matches (liberar mem�ria)

	// recuperando os keypoints dos matches bons e suas coordenadas
	for (cv::DMatch match : goodMatches) {
		this->m_goodKeypointsL.emplace_back(this->m_leftCam->m_keypoints[match.queryIdx]);		// keypoint c�mera esq.
		this->m_goodKeypointsR.emplace_back(this->m_rightCam->m_keypoints[match.trainIdx]);		// keypoint c�mera dir.
		this->m_pointsL.emplace_back(this->m_leftCam->m_keypoints[match.queryIdx].pt);			// coord. keypoint esq.
		this->m_pointsR.emplace_back(this->m_rightCam->m_keypoints[match.trainIdx].pt);			// coord. keypoint dir.
	}
	goodMatches.clear(); // limpando a lista inicial de matches bons (liberar mem�ria)

	// obtendo a matriz essencial
	cv::Mat mask; // vetor de �ndicas resultante. 1 = inlier, 0 = outlier
	this->m_E = cv::findEssentialMat(this->m_pointsL, this->m_pointsR, this->m_leftCam->m_K, cv::RANSAC, 0.999, 1.0, mask);

	// removendo outliers
	std::vector<cv::KeyPoint> goodKeypointsL;	// temp inliers
	std::vector<cv::KeyPoint> goodKeypointsR;	// temp inliers
	std::vector<cv::Point2f> pointsL;			// temp inliers
	std::vector<cv::Point2f> pointsR;			// temp inliers

	for (int i = 0; i < mask.rows; i++) {
		if (mask.at<uchar>(i, 0) == 1) {
			// 1 == inlier, adicionar nas listas tempor�rias
			goodKeypointsL.emplace_back(this->m_goodKeypointsL[i]);
			goodKeypointsR.emplace_back(this->m_goodKeypointsR[i]);
			pointsL.emplace_back(this->m_pointsL[i]);
			pointsR.emplace_back(this->m_pointsR[i]);
		}
	}
	this->m_goodKeypointsL = goodKeypointsL;
	this->m_goodKeypointsR = goodKeypointsR;
	this->m_pointsL = pointsL;
	this->m_pointsR = pointsR;

	return this->m_pointsL.size(); // num. de matches bons filtrados sem outliers
}

// Cria as tracks para cada match
void StereoPair::createTracks() {
	for (size_t i = 0; i < this->m_goodKeypointsL.size(); i++) {
		this->m_tracks.emplace_back(new Track());
		this->m_tracks.back()->addKeypoint(new Keypoint(this->m_goodKeypointsL[i], this->m_leftCam));
		this->m_tracks.back()->addKeypoint(new Keypoint(this->m_goodKeypointsR[i], this->m_rightCam));
	}
}

// Calcula as pose entre um par de c�meras e armazena na matriz extr�nsseca P da c�mera da direita
void StereoPair::recoverPose() {
	cv::Mat R, t; // matriz de rota��o e vetor de transla��o
	cv::recoverPose(this->m_E, this->m_pointsL, this->m_pointsR, this->m_leftCam->m_K, R, t);
	this->m_rightCam->m_P = cv::Matx34f(R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0),
		R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1),
		R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2));
}

// Faz a retifica��o est�reo das imagens do par
void StereoPair::rectify() {
	cv::Mat H1, H2, Pn1, Pn2;
	MathUtils::rectifyPoses(
		this->m_leftCam->m_P, this->m_rightCam->m_P, 
		this->m_leftCam->m_K, 
		H1, H2, 
		Pn1, Pn2
	);

	MathUtils::rectifyImages(
		this->m_leftCam->m_image, this->m_rightCam->m_image, 
		H1, H2, 
		this->m_leftWarp, this->m_rightWarp, 
		this->m_leftMask, this->m_rightMask, 
		this->m_leftH, this->m_rightH
	);
}

// Detecta e computa os descritores das imagens do par
void StereoPair::detectAndComputeDense() {
	cv::Ptr<cv::AKAZE> akaze_f2d = cv::AKAZE::create();
	akaze_f2d->detectAndCompute(this->m_leftCam->m_image, cv::noArray(), this->m_leftKeyPoints, this->m_leftDescriptors);
	akaze_f2d->detectAndCompute(this->m_rightCam->m_image, cv::noArray(), this->m_rightKeyPoints, this->m_rightDescriptors);
}

// Faz o matching entre as imagens das duas c�meras (dense)
void StereoPair::matchDense() {
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
	std::vector<std::vector<cv::DMatch>> matches;

	// matching
	// query = descritores da imagem da esquerda
	// train = descritores da imagem da direita
	matcher->knnMatch(this->m_leftDescriptors, this->m_rightDescriptors, matches, 2);

	// filtrando os matches ruins
	std::vector<cv::DMatch> goodMatches; // lista de matches filtrados
	for (size_t i = 0; i < matches.size(); i++)
		if (matches[i][0].distance < 0.7f * matches[i][1].distance)
			goodMatches.push_back(matches[i][0]);
	matches.clear(); // limpando a lista incial de matches (liberar mem�ria)

	// recuperando os keypoints dos matches bons e suas coordenadas
	this->m_goodKeypointsL.clear();
	this->m_goodKeypointsR.clear();
	this->m_pointsL.clear();
	this->m_pointsR.clear();
	for (cv::DMatch match : goodMatches) {
		if (abs(this->m_leftKeyPoints[match.queryIdx].pt.y - this->m_rightKeyPoints[match.trainIdx].pt.y) < 10) { // Epipolar constraint
			this->m_goodKeypointsL.emplace_back(this->m_leftKeyPoints[match.queryIdx]);		// keypoint c�mera esq.
			this->m_goodKeypointsR.emplace_back(this->m_rightKeyPoints[match.trainIdx]);	// keypoint c�mera dir.
			this->m_pointsL.emplace_back(this->m_leftKeyPoints[match.queryIdx].pt);			// coord. keypoint esq.
			this->m_pointsR.emplace_back(this->m_rightKeyPoints[match.trainIdx].pt);		// coord. keypoint dir.
			this->m_leftMask.at<uchar>(this->m_pointsL.back()) = 0;
			this->m_rightMask.at<uchar>(this->m_pointsR.back()) = 0;
		}
	}
}

// Cria as seeds iniciais
void StereoPair::createInitialSeeds() {
	for (size_t i = 0; i < this->m_pointsL.size(); i++) {
		this->m_seeds.push_back(
			new Seed(
				this->m_pointsL[i], this->m_pointsR[i], 
				this->m_leftCam, this->m_rightCam, 
				1.0
			)
		);
	}
}

bool compareFunction(Seed* i, Seed* j) {
	return i->m_score < j->m_score;
}

// Computa a seeds adicionais
void StereoPair::computeNewSeeds() {
	int windowSize = 6, windowSize2 = windowSize + 1;
	int searchSeed = 5;
	int maxDist = 5;
	double score = 0;
	size_t seedCount;

	std::mutex mutex;
	std::vector<Seed*> newSeeds;
	std::vector<Seed*> usedSeeds;
	std::vector<Seed*> tempSeeds;

	const size_t max_itr = 20;
	for (size_t i = 0; i < max_itr; i++) {
		seedCount = this->m_seeds.size();

		cv::parallel_for_(cv::Range(0, (int)seedCount), [&](const cv::Range& range) {

			for (int r = range.start; r < range.end; r++) {
				int p1_x = this->m_seeds[r]->m_p1.x;
				int p1_y = this->m_seeds[r]->m_p1.y;
				int min_x = p1_x - searchSeed, max_x = p1_x + searchSeed;
				int min_y = p1_y - searchSeed, max_y = p1_y + searchSeed;
				int min_x2 = this->m_seeds[r]->m_p2.x - searchSeed;
				int p2_x = 0;

				for (int x = min_x; x <= max_x; x++) {
					for (int y = min_y; y <= max_y; y++) {
						if (y >= 0 && x >= 0 && y < this->m_leftMask.rows && x < this->m_leftMask.cols) {
							if ((x - windowSize) >= 0 &&
								(x + windowSize2) < this->m_leftWarp.cols &&
								(y - windowSize) >= 0 &&
								(y + windowSize2) < this->m_leftWarp.rows) {
								cv::Point2f found = MathUtils::ZNCC(
									this->m_rightWarp,
									this->m_leftWarp(
										cv::Range(y - windowSize, y + windowSize2),
										cv::Range(x - windowSize, x + windowSize2)
									),
									min_x2 + p2_x,
									windowSize,
									maxDist,
									y,
									this->m_rightWarp,
									score
								);

								if (found.x != -1) {
									mutex.lock();
									newSeeds.push_back(new Seed(
										cv::Point2f(x, y),
										found,
										this->m_leftCam,
										this->m_rightCam,
										score
									));
									mutex.unlock();
								}
							}
						}
					}
				}
			}
		});

		usedSeeds.insert(usedSeeds.end(), this->m_seeds.begin(), this->m_seeds.end());
		this->m_seeds.clear();
		std::sort(newSeeds.begin(), newSeeds.end(), compareFunction);

		for (size_t i = 0; i < newSeeds.size(); i++) {
			if (this->m_rightMask.at<uchar>(newSeeds[i]->m_p2) != 0) {
				this->m_leftMask.at<uchar>(newSeeds[i]->m_p1) = 0;
				this->m_rightMask.at<uchar>(newSeeds[i]->m_p2) = 0;
				tempSeeds.emplace_back(newSeeds[i]);
			}
			else {
				delete newSeeds[i];
				newSeeds[i] = nullptr;
			}
		}
		newSeeds.clear();
		this->m_seeds = tempSeeds;
		tempSeeds.clear();
	}

	this->m_seeds.clear();
	this->m_seeds = usedSeeds;
	usedSeeds.clear();
	std::vector<Seed*>(usedSeeds).swap(usedSeeds);

	newSeeds.clear();
	std::vector<Seed*>(newSeeds).swap(newSeeds);
	std::vector<Seed*>(tempSeeds).swap(tempSeeds);
}

void StereoPair::triangulate3DPoints() {
	cv::parallel_for_(cv::Range(0, this->m_seeds.size()), [&](const cv::Range& range) {
		for (int r = range.start; r < range.end; r++) {
			this->m_seeds[r]->triangulate3DPoints(this->m_leftH, this->m_rightH);
		}
	});
}

// void StereoPair::PCL_createPointCloud() {

// 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

// 	for (auto seed : this->m_seeds) {
// 		if (seed->m_reprojectionError < 1.0) {
// 			cloud->push_back(pcl::PointXYZ(
// 				seed->m_point3D.x, 
// 				seed->m_point3D.y, 
// 				seed->m_point3D.z
// 			));
// 		}
// 	}
// 	this->m_pclCloud = cloud;
// }

// void StereoPair::PCL_filterCloud() {
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
// 	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;

// 	// set parameters
// 	outrem.setInputCloud(this->m_pclCloud);
// 	outrem.setRadiusSearch(0.01);
// 	outrem.setMinNeighborsInRadius(50);

// 	// filter cloud
// 	outrem.filter(*cloud_filtered);
// 	this->m_pclCloud = cloud_filtered;
// }

// void StereoPair::PCL_computeNormals() {
// 	// Create the normal estimation class, and pass the input dataset to it
// 	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
// 	ne.setInputCloud(this->m_pclCloud);

// 	// Create an empty kdtree representation, and pass it to the normal estimation object.
// 	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
// 	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
// 	ne.setSearchMethod(tree);

// 	// Output dataset
// 	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

// 	// Use all neighbors in a sphere of radius 0.05cm
// 	ne.setRadiusSearch(0.05);

// 	// Determining the view point
// 	double* viewPoint = nullptr;
// 	while (!viewPoint) {
// 		// tente alocar mem�ria at� conseguir
// 		viewPoint = (double*)std::calloc(3, sizeof(double)); // calloc inicializa todo o bloco com zeros
// 	}
// 	MathUtils::transformPoint(viewPoint, this->m_rightCam->m_P);
// 	ne.setViewPoint(*viewPoint, *(viewPoint + 1), *(viewPoint + 2));
// 	std::free(viewPoint);

// 	// Compute the features
// 	ne.compute(*cloud_normals);
// 	this->m_pclCloudNormals = cloud_normals;
// }

// void StereoPair::PCL_exportCloudPLY(const fs::path& outputDir, const std::string& filename, pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud) {

// 	// cria o diret�rio se ele n�o existe
// 	if (!fs::exists(outputDir)) {
// 		fs::create_directories(outputDir);
// 	}

// 	// caminho do arquivo
// 	const fs::path outputPath = outputDir/filename;

// 	if (cloud) {
// 		// salva nuvem passada por par�metro e retorna
// 		pcl::io::savePLYFileBinary(outputPath.string(), *cloud);
// 		return;
// 	}

// 	// salva ou somente a nuvem de pontos ou nuvem de pontos + normais
// 	if (this->m_pclCloudNormals) {

// 		// concatenando nuvem de pontos com suas normais
// 		this->m_pclCloudWithNormals = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
// 		for (size_t i = 0; i < this->m_pclCloud->size(); i++)
// 		{
// 			pcl::PointNormal p;

// 			p.x = (*this->m_pclCloud)[i].x;
// 			p.y = (*this->m_pclCloud)[i].y;
// 			p.z = (*this->m_pclCloud)[i].z;

// 			p.normal_x = (*this->m_pclCloudNormals)[i].normal_x;
// 			p.normal_y = (*this->m_pclCloudNormals)[i].normal_y;
// 			p.normal_z = (*this->m_pclCloudNormals)[i].normal_z;

// 			this->m_pclCloudWithNormals->push_back(p);
// 		}

// 		// salvando nuvem com normais
// 		pcl::io::savePLYFileBinary(outputPath.string(), *this->m_pclCloudWithNormals);
// 	} else {
// 		// salvando nuvem
// 		pcl::io::savePLYFileBinary(outputPath.string(), *this->m_pclCloud);
// 	}
// 	std::cout << "  Exportado: \"" << outputPath.string() << "\"\n";
// }
