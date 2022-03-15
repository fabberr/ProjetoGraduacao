/********** Headers **********/

// libc
#include <cstddef> // std::size_t

// headers internos
#include <PointCloud.h>

/********** Definição de Funções **********/

// Construtor
PointCloud::PointCloud(const fs::path& datasetPath, const fs::path& outputPath, size_t imgCount) : 
	m_inputPath{datasetPath}, 
	m_outputPath{outputPath}, 
	m_imgCount{imgCount}
{}

// Destrutor
PointCloud::~PointCloud() {}

// Computa a nuvem de pontos esparsa usando como entrada as imagens do dataset, retorna o caminho do diretório de saída
void PointCloud::computeSparse() {
	/*----- I. CARREGAR CÂMERAS -----*/

	// a. carregar todos os caminhos até as imagens do dataset em imagePaths
	std::vector<fs::path> imagePaths; // lista de caminhos absolutos até as imagens do dataset
	for (const auto& entry : fs::directory_iterator(this->m_inputPath)) {
		imagePaths.emplace_back(entry.path());
	}

	// num. máximo de imagens a serem carregadas
	if (this->m_imgCount == 0) {
		this->m_imgCount = imagePaths.size();
	}

	std::cout << "Dataset: \"" << this->m_inputPath.string() << "\"\n";
	std::cout << " Nmr. de imagens: " << this->m_imgCount << "\n\n";

	// b. instanciar as cameras
	std::cout << "Carregando as cameras...";

	std::vector<Camera*> cameras; // lista de câmeras, uma por imagem
	cameras.reserve(this->m_imgCount);
	for (const fs::path& path : imagePaths) {
		cameras.emplace_back(new Camera(path.string()));
	}
	imagePaths.clear();
	std::cout << "OK" << std::endl;

	// c. detectar features, computar descritores
	std::cout << "Detectando keypoints e computando seus descritores...\n\n";
	for (size_t i = 0; i < this->m_imgCount; i++) {
		Camera* cam = cameras[i];
		std::printf(">Imagem %llu de %llu: \"%s\"\n", i + 1, this->m_imgCount, cam->m_pathToImage.c_str());
		cam->detectAndCompute();
		std::printf(" Nmr. de keypoints: %llu\n\n", cam->m_keypoints.size());
	}
	std::cout.flush();

	/*----- II. MATCHING & CÁLCULO DAS POSES -----*/

	std::vector<StereoPair*> sequentialPairs;	// lista de pares subsequentes {(0, 1), (1, 2), (2, 3), ...}
	std::vector<StereoPair*> unorderedPairs;	// lista de pares aleatórios

	// reservando espaço p/ os pares
	sequentialPairs.reserve(this->m_imgCount);
	unorderedPairs.reserve(this->m_imgCount);

	std::cout << "\nMatching: comparando imagens e calculando as poses...\n";
	std::cout << "Nmr. minimo de matches: 100\n";
	for (size_t i = 0; i < this->m_imgCount - 1; i++) { // i = 0, ..., penúltimo elem.
		// a. comparar imagem com a subsequente até que número de matches seja < 100
		size_t lastNumOfMatches = 100;
		StereoPair* pair;
		for (size_t j = i + 1; j < this->m_imgCount && lastNumOfMatches >= 100; j++) { // j = i + 1, ..., �ltimo elem.
			std::cout << "\n>Par (" << i << ", " << j << ")\n";
			std::cout << " Matching...";
			pair = new StereoPair(cameras[i], cameras[j]);
			lastNumOfMatches = pair->match();

			std::cout << "OK" << std::endl;
			std::cout << " Nmr. de matches: " << lastNumOfMatches << std::endl;

			if (lastNumOfMatches >= 100) {
				// par teve pelo menos 100 matches
				// criar as tracks, calcular as poses se for par sequêncial e adicionar na lista correta de pares
				pair->createTracks();
				if (j == i + 1) {
					// sequentialPairs
					std::cout << " Calculando pose...";
					pair->recoverPose();
					std::cout << "OK\n";
					sequentialPairs.emplace_back(pair);
				} else {
					// unorderedPairs
					unorderedPairs.emplace_back(pair);
				}
			} else {
				// par teve menos que 100 matches
				// descartar par
				delete pair;
				std::cout << " Par descartado\n";
			}
		}

		lastNumOfMatches = 100;
		// b. comparar imagem com as últimas até que o número de matches seja < 100
		for (size_t j = this->m_imgCount - 1; j > i + 1 && lastNumOfMatches >= 100; j--) { // j = último elem., ..., i + 2
			std::cout << "\n>Par (" << i << ", " << j << ")\n";
			std::cout << " Matching...";
			pair = new StereoPair(cameras[i], cameras[j]);
			lastNumOfMatches = pair->match();

			std::cout << "OK\n";
			std::cout << " Nmr. de matches: " << lastNumOfMatches << std::endl;

			if (lastNumOfMatches >= 100) {
				// par teve pelo menos 100 matches
				// criar tracks e adicionar em unorderedPairs (pois (j == i + 1) nunca será verdade)
				pair->createTracks();
				unorderedPairs.emplace_back(pair);
			} else {
				// par teve menos que 100 matches
				// descartar par
				delete pair;
				std::cout << " Par descartado\n";
			}
		}
	}

	/*----- GERAÇÃO	 DA NUVEM DE PONTOS -----*/
	std::cout << "\nGeracao da nuvem de pontos (esparsa)\n";

	// a. criar as cenas
	std::vector<Graph*> graphs; // lista de cenas, uma por par inicialmente, ao final apenas uma cena será gerada a partir da fusão das demais
	graphs.reserve(sequentialPairs.size());

	// b. adicionando os pares sequenciais as cenas
	std::cout << "\nAdicionando pares sequenciais nas cenas\n";
	for (StereoPair* pair : sequentialPairs) {
		graphs.emplace_back(new Graph(pair));
	}
	std::cout << graphs.size() << " cenas geradas\n";

	// c. triangular os pontos 3D para as tracks da cena 0
	std::cout << "Triangulando pontos iniciais da cena 0...";
	graphs[0]->triangulate3DPoints();
	std::cout << "OK\n";

	// d. fazer fusão das cenas e um bundle adjustment a cada iteração até que todas as cenas sejem fundidas
	std::cout << "\nFazendo fusao das cenas...\n";
	size_t modelSize = 0;
	for (size_t i = 1; i < graphs.size(); i++) {
		// e. fazendo a fusão da cena (adiciona novas tracks na cena 0)
		std::cout << "\n>Fusao: cena " << i << " com a cena 0\n";
		graphs[0]->mergeWith(graphs[i]);

		// f. adicionando os pares sem ordem que ainda não foram fundidos (adicionar novas tracks na cena 0)
		for (StereoPair* p : unorderedPairs) {
			if (!p->m_pairMerged) {
				graphs[0]->addPair(p);
			}
		}

		// g. recalcular os pontos 3D agora que há novas tracks na cena
		std::cout << " Recalculando pontos 3D da cena 0...";
		graphs[0]->triangulate3DPoints();
		std::cout << "OK\n";

		// h. bundle adjustment parcial
		ParallelBundleAdjustment* pba = new ParallelBundleAdjustment(graphs[0]);
		if (i == 1 || (graphs[0]->getModelSize() > modelSize * 1.05 && graphs[0]->m_cameras.size() > 5)) {
			// aumento de mais de 5% na quantidade de pontos => BA completo
			std::cout << " Fazendo bundle adjustment completo parcial...\n";
			pba->runBundle(true);
			modelSize = graphs[0]->getModelSize();
		} else {
			// aumento de menos de 5% na quantidade de pontos -> BA sobre a última câmera
			std::cout << " Fazendo bundle adjustment parcial...\n";
			pba->runBundle(false);
		}
		graphs[0] = pba->getResult();
		std::cout << "OK\n";
		delete pba;

		//graphs[0]->filterPoints();
	}

	// i. bundle adjustment completo final
	std::cout << "\nTodas as cenas foram fundidas, fazendo bundle adjustment completo final...\n";
	ParallelBundleAdjustment pba(graphs[0]);
	pba.runBundle(true);
	graphs[0] = pba.getResult();
	std::cout << "OK\n";

	/*----- EXPORTAR ARQUIVOS -----*/
	std::cout << "\nExportando resultados\n";

	// a. exportar resultado da calibração das câmeras
	std::cout << "\n Exportando resultados da calibracao das cameras (.sfm)\n";
	graphs[0]->exportSFM(this->m_outputPath); // .sfm

	// b. exportar nuvem esparsa
	std::cout << "\n Exportando nuvem de pontos (.obj)\n";
	pba.exportOBJ(this->m_outputPath); // .obj (somente lista de vértices)

	// c. exportar arquivo N-View Match para visualização da nuvem gerado no VisualSFM
	std::cout << "\n Exportando N-View Match (.nvm)\n";
	pba.exportNVM(this->m_outputPath); // .nvm

	// FIM, liberar memória
	// ...
}

// // Computa a nuvem de pontos densa usando como entrada o resultado da gera��o da nuvem esparsa, retorna o caminho do diret�rio de sa�da
// const fs::path PointCloud::computeDense() {

// 	// carregando matrizes das c�meras partir do arquivo .sfm
// 	std::cout << "Carregando as poses a partir do arquivo\n";
// 	std::vector<Camera*> cameras = loadCameras();
// 	if (cameras.size() == 0) {
// 		// nenhuma c�mera carregada, encerrar
// 		std::cout << "Nao foi possivel ler nenhuma camera do arquivo\n";
// 		std::cout << "Abortando\n";
// 		std::exit(-1);
// 	}

// 	std::vector<StereoPair*> pairs;						// vetor de pares
// 	const fs::path outputDir = this->m_inputPath/"dense";	// diret�rio de sa�da

// 	// comparar pares de c�mera
// 	std::cout << "Comparando angulo entre as cameras:\n";
// 	for (size_t i = 0; i < cameras.size() - 1; i++) {

// 		double angle = 0;
// 		for (size_t j = i + 1; j < cameras.size(); j++) {

// 			std::cout << "\nCameras #" << i << " e #" << j << ":\n";
// 			angle = cameras[i]->angleBetween(cameras[j]);
// 			std::cout << "  Angulo entre as cameras: " << angle << std::endl;

// 			if (angle >= 5 && angle <= 30) {
// 				pairs.emplace_back(new StereoPair(cameras[i], cameras[j]));

// 				std::cout << "  Fazendo retificacao estereo das imagens...";
// 				pairs.back()->rectify();
// 				std::cout << "OK" << std::endl;

// 				std::cout << "  Detectando keypoints nas imagens e computando seus descritores...";
// 				pairs.back()->detectAndComputeDense();
// 				std::cout << "OK\n";

// 				std::cout << "  Fazendo matching dos keypoints...";
// 				pairs.back()->matchDense();
// 				std::cout << "OK\n";

// 				std::cout << "  Criando seeds iniciais para o par...";
// 				pairs.back()->createInitialSeeds();
// 				std::cout << "OK\n";

// 				std::cout << "  Computando seeds adicionais para o par...";
// 				pairs.back()->computeNewSeeds();
// 				std::cout << "OK\n";

// 				std::cout << "  Triangulando pontos 3D...";
// 				pairs.back()->triangulate3DPoints();
// 				std::cout << "OK\n";

// 				std::cout << "  Adicionando pontos a nuvem intermediaria...";
// 				pairs.back()->PCL_createPointCloud();
// 				std::cout << "OK\n";

// 				std::cout << "  Filtrando nuvem intermediaria\n";
// 				std::cout << "    Nmr. de pontos pre filtragem: " << pairs.back()->m_pclCloud->size() << std::endl;
// 				pairs.back()->PCL_filterCloud();
// 				std::cout << "    Nmr. de pontos pos filtragem: " << pairs.back()->m_pclCloud->size() << std::endl;

// 				std::cout << "  Calculando normais...";
// 				pairs.back()->PCL_computeNormals();
// 				std::cout << "OK\n";

// 				std::cout << "  Exportando nuvem intermediaria (.ply)\n";
// 				pairs.back()->PCL_exportCloudPLY(
// 					outputDir, 
// 					(std::stringstream() << "pair_" << i << "_" << j << "_cloud.ply").str()
// 				);
// 			} else {
// 				std::cout << "  Par descartado\n";
// 			}
// 		}
// 	}

// 	// concatenar todas as nuvens
// 	std::cout << "\nConcatenando todas as nuvens intermediarias...";
// 	pcl::PointCloud<pcl::PointNormal>::Ptr result_cloud(new pcl::PointCloud<pcl::PointNormal>);
// 	for (const auto& pair : pairs) {
// 		for (const auto& pt : *pair->m_pclCloud) {
// 			pcl::PointNormal p;
// 			p.x = pt.x; p.y = pt.y; p.z = pt.z;
// 			result_cloud->push_back(p);
// 		}

// 		for (const auto& pt : *pair->m_pclCloudWithNormals) {
// 			result_cloud->push_back(pt);
// 		}
// 	}
// 	std::cout << "OK\n";

// 	// salvar resultados
// 	std::cout << "\nExportando nuvem densa (.ply)\n";
// 	std::string result_filename("cloud_dense.ply");
// 	pairs[0]->PCL_exportCloudPLY(outputDir, result_filename, result_cloud);
// 	std::cout << "  Exportado: " << outputDir/result_filename << std::endl;

// 	// FIM, liberar recursos
// 	// retornar caminho at� o diret�rio de sa�da
// 	return outputDir;
// }

// // L� o arquivo .sfm e instancia um vetor de c�meras com as matrizes intrinseca e extrinseca carregadas do arquivo
// std::vector<Camera*> PointCloud::loadCameras() {

// 	std::string poseFilename = (this->m_inputPath/"pose.sfm").string();	// caminho do arquivo
// 	std::ifstream ifs(poseFilename);								// construtor input file stream, tenta abrir o arquivo
// 	std::vector<Camera*> cameras;									// vetor de c�meras (sa�da do m�todo)

// 	std::cout << "  Caminho: " << poseFilename << std::endl;
// 	if (ifs.is_open()) {
// 		// ler n�mero de c�meras
// 		ifs >> this->m_imgCount;

// 		std::cout << "  Nmr. de imagens: " << this->m_imgCount << std::endl;

// 		// reservando mem�ria p/ vetor de c�meras
// 		cameras.reserve(this->m_imgCount);

// 		// ler e descartar linha vazia
// 		char empty_buffer[1];			// buffer tempor�rio
// 		ifs.getline(empty_buffer, 1);	// l� 1 byte do arquivo para descartar a linha vazia

// 		// ler matrizes das c�meras e dist�ncias focais, uma c�mera/linha
// 		std::string pathToImage;
// 		for (size_t i = 0; i < this->m_imgCount && !ifs.eof(); i++) {
// 			// caminho da imagem
// 			ifs >> pathToImage;
// 			Camera* cam = new Camera(pathToImage);

// 			// matriz de rota��o R
// 			for (int j = 0; j < 3; j++)
// 				for (int k = 0; k < 3; k++)
// 					ifs >> cam->m_P(j, k);

// 			// vetor de transla��o t
// 			for (int j = 0; j < 3; j++)
// 				ifs >> cam->m_P(j, 3);

// 			// dist�ncia focal
// 			ifs >> cam->m_K.at<double>(0, 0);
// 			ifs >> cam->m_K.at<double>(1, 1);

// 			// adicionando no vetor de c�meras
// 			cameras.emplace_back(cam);
// 		}

// 		// fechar o arquivo
// 		ifs.close();
// 	} else {
// 		std::cout << "  Erro ao abrir arquivo: " << poseFilename << std::endl;
// 	}

// 	return cameras;
// }

// // Salva a nuvem no formato NVM: http://ccwu.me/vsfm/doc.html#nvm
// void PointCloud::exportNVM(const std::vector<Camera*>& cameras, pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud, const fs::path& outputDir, const std::string& filename) {

// 	// cria o diret�rio se ele n�o existe
// 	if (!fs::exists(outputDir)) {
// 		fs::create_directories(outputDir);
// 	}

// 	const fs::path outputPath = outputDir/filename; // caminho do arquivo
// 	std::ofstream ofs(outputPath.string(), ios::trunc | ios::out); // output file stream, cria ou abre o arquivo no modo truncate
// 	if (ofs.is_open()) {
// 		//==========Header section
// 		ofs << "NVM_V3_R9T\n"; // file version header

// 		//==========Camera section: <number of cameras> <list of cameras>

// 		ofs << this->m_imgCount << '\n'; // number of cameras

// 		// list of cameras: <filename> <focal length> <quaternion WXYZ> <camera center> <radial distortion> 0
// 		for (const auto& cam : cameras) {
// 			ofs << cam->m_pathToImage << ' '; // <filename>
// 			ofs << cam->m_K.at<double>(0, 0) << ' '; // <focal length>

// 			// <quaternion WXYZ> (rotation matrix)
// 			for (int i = 0; i < 3; i++) {
// 				for (int j = 0; j < 3; j++) {
// 					ofs << cam->m_P.row(i).col(j) << ' ';
// 				}
// 			}

// 			// <camera center> (translation vector)
// 			for (int i = 0; i < 3; i++) {
// 				ofs << cam->m_P.row(i).col(3) << ' ';
// 			}

// 			ofs << "0 "; // <radial distortion>
// 			ofs << "0\n"; // end
// 		}

// 		//==========Point data section: <number of points> <list of points>

// 		ofs << cloud->size() << '\n'; // <number of points>

// 		// <list of points>: <XYZ coords> <RGB value> <number of measurements> <list of measurements>
// 		for (const auto& pt : *cloud) {
// 			ofs << pt.x << ' ' << pt.y << ' ' << pt.z << ' '; // <XYZ coords>
// 			ofs << "255 255 255 "; // <RBG value>

// 			ofs << "0\n"; // measurements
// 		}

// 		ofs.close();
// 		std::cout << "  Exportado: \"" << outputPath.string() << "\"\n";
// 	} else {
// 		std::cout << "  Erro ao abrir arquivo: " << filename << std::endl;
// 	}
// }

/** @todo liberar memória alocada no heap ao destruir objeto */
