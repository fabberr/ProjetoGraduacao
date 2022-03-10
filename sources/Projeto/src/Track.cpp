#include <Track.h>

// Construtor default
Track::Track() {
	this->m_point = cv::Point3d(-1, -1, -1);
	this->m_lastAvgReprojError = VTK_DOUBLE_MAX;
}

// Destrutor
Track::~Track() {}

// Adiciona um keypoint e sua c�mera na lista de keypoints
void Track::addKeypoint(Keypoint* kpt) {
	this->m_keypoints.emplace_back(kpt);
}

// Triangula um ponto 3D
void Track::triangulatedPoints3D() {
	std::vector<cv::Mat> triangulatedPoints3D;
	cv::Mat KLeft, KRight;
	Camera* camLeft;
	Camera* camRight;

	for (size_t i = 0; i < this->m_keypoints.size(); i++) {
		std::vector<cv::Point2f> imgLeftPts;
		imgLeftPts.emplace_back(this->m_keypoints[i]->m_keypoint.pt);
		KLeft = this->m_keypoints[i]->m_camera->m_K;

		cv::Mat normalizedLeftPts;
		cv::undistortPoints(imgLeftPts, normalizedLeftPts, KLeft, cv::Mat());

		camLeft = this->m_keypoints[i]->m_camera;
		for (size_t j = i + 1; j < this->m_keypoints.size(); j++) {
			if (camLeft->angleBetween(this->m_keypoints[j]->m_camera) > 10) {
				std::vector<cv::Point2f> imgRightPts;
				imgRightPts.emplace_back(this->m_keypoints[j]->m_keypoint.pt);
				KRight = this->m_keypoints[j]->m_camera->m_K;

				cv::Mat normalizedRightPts;
				cv::undistortPoints(imgRightPts, normalizedRightPts, KRight, cv::Mat());

				camRight = this->m_keypoints[j]->m_camera;

				cv::Mat points3DHomogeneous;
				cv::triangulatePoints(camLeft->m_P, camRight->m_P, normalizedLeftPts, normalizedRightPts, points3DHomogeneous);

				cv::Mat points3D;
				cv::convertPointsFromHomogeneous(points3DHomogeneous.t(), points3D);
				triangulatedPoints3D.emplace_back(points3D);
			}
		}
	}

	double sumReproj;
	double minReproj = VTK_DOUBLE_MAX;
	size_t minReprojIdx = -1;
	for (size_t i = 0; i < triangulatedPoints3D.size(); i++) {
		sumReproj = 0;
		for (size_t j = 0; j < this->m_keypoints.size(); j++)
			sumReproj += this->m_keypoints[j]->computeReprojectionError(triangulatedPoints3D[i]);
		sumReproj /= this->m_keypoints.size();
		if (sumReproj < minReproj) {
			minReproj = sumReproj;
			minReprojIdx = i;
		}
	}

	if (minReproj < this->m_lastAvgReprojError && minReproj < 100) {
		float x = abs(triangulatedPoints3D[minReprojIdx].at<float>(0, 0));
		float y = abs(triangulatedPoints3D[minReprojIdx].at<float>(0, 1));
		float z = abs(triangulatedPoints3D[minReprojIdx].at<float>(0, 2));
		if (x < 10 && y < 10 && z < 10) {
			this->m_point = cv::Point3f(
				triangulatedPoints3D[minReprojIdx].at<float>(0, 0), 
				triangulatedPoints3D[minReprojIdx].at<float>(0, 1), 
				triangulatedPoints3D[minReprojIdx].at<float>(0, 2)
			);
			m_lastAvgReprojError = minReproj;
		}
	}
}

// Faz o merge dessa track com outra
bool Track::mergeWith(Track* other) {
	std::vector<Keypoint*> newKeypoints;
	bool change = false;

	for (Keypoint* otherKeypoint : other->m_keypoints) {
		// busca sequ�ncial em this->m_keypoints
		// chave = otherKeypoint
		std::vector<Keypoint*>::iterator thisKeypointsItr = this->m_keypoints.begin();
		while (thisKeypointsItr != this->m_keypoints.end() && **thisKeypointsItr != *otherKeypoint)
			thisKeypointsItr++;

		// se n�o encontrou a chave, adicione-a em newKeypoints
		if (thisKeypointsItr == this->m_keypoints.end())
			newKeypoints.emplace_back(otherKeypoint);
		else
			change = true;
	}

	// adicione todos os novos keypoints em this->m_keypoints
	if (change)
		for (Keypoint* k : newKeypoints)
			this->m_keypoints.emplace_back(k);
	return change;
}

// Remove pontos ruins ap�s o BA
void Track::filterPoint() {
	if (std::abs(this->m_point.x) > 10 || std::abs(this->m_point.y) > 10 || std::abs(this->m_point.z) > 10) {
		// descartar o ponto
		this->m_point = cv::Point3d(-1, -1, -1);
		return;
	}

	cv::Mat ptMat(1, 3, CV_32F, { this->m_point.x, this->m_point.y, this->m_point.z });
	double avgMinReproj = 0;
	for (Keypoint* kpt : this->m_keypoints)
		avgMinReproj += kpt->computeReprojectionError(ptMat);
	avgMinReproj /= this->m_keypoints.size();

	if (avgMinReproj > 100) // erro de reproje��o > 100
		this->m_point = cv::Point3d(-1, -1, -1); // dascartar o ponto
}
