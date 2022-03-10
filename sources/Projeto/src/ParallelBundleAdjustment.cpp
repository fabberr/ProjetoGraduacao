#include <ParallelBundleAdjustment.h>

// Construtor default
ParallelBundleAdjustment::ParallelBundleAdjustment() {
	this->m_graph = nullptr;
}

// Construtor
ParallelBundleAdjustment::ParallelBundleAdjustment(Graph* graph) {
	// inicializando vari�veis membros
	this->m_graph = graph;

	// alocando mem�ria para o vetor de c�meras
	this->m_cameraData.resize(graph->m_cameras.size());
	this->m_photoNames.resize(graph->m_cameras.size());

	// carregando informa��es no vetor de c�meras (this->m_cameraData)
	for (size_t i = 0; i < graph->m_cameras.size(); i++) {
		cv::Matx34f P = graph->m_cameras[i]->m_P; // matriz extr�nsseca

		// carregando matriz de rota��o
		for (int j = 0; j < 3; j++)
			for (int k = 0; k < 3; k++)
				this->m_cameraData[i].m[j][k] = P(j, k);

		// carregando o vetor de transla��o
		for (int j = 0; j < 3; j++)
			this->m_cameraData[i].t[j] = P(j, 3);

		// carregando dist�ncia focal
		this->m_cameraData[i].SetFocalLength(graph->m_cameras[i]->m_K.at<double>(0, 0));
		this->m_photoNames[i] = graph->m_cameras[i]->m_pathToImage;
	}

	// alocando mem�ria para o vetor de pontos (this->m_pointData)
	this->m_pointData.resize(graph->m_tracks.size());

	// carregando informa��es no vetor de pontos
	for (int i = 0; i < (int)graph->m_tracks.size(); i++) {
		Keypoint* kpt;
		Track* t = graph->m_tracks[i];
		cv::Point3f pt = t->m_point;
		int cc[3]{ 255, 255, 255 };

		if (pt.x != -1 && pt.y != -1 && pt.z != -1) {
			// carregando ponto 3D
			this->m_pointData[i].SetPoint(pt.x, pt.y, pt.z);
			this->m_pointColor.insert(this->m_pointColor.end(), cc, cc + 3);

			for (size_t j = 0; j < t->m_keypoints.size(); j++) {
				// carregando os �ndices das c�meras
				// busca o �ndice da c�mera em graph->m_cameras
				kpt = t->m_keypoints[j];
				int k = 0;
				while (k < (int)graph->m_cameras.size() && graph->m_cameras[k] != kpt->m_camera)
					k++;
				if (k < (int)graph->m_cameras.size() && graph->m_cameras[k] == kpt->m_camera)
					this->m_camIdx.emplace_back(k);

				// carregando o �ndice do ponto
				this->m_ptIdx.emplace_back(i);

				// adicionando uma medida ao vetor
				Point2D pt2D(
					kpt->m_keypoint.pt.x - kpt->m_camera->m_K.at<double>(0, 2),		/* x - x0 ou x - K02 */
					kpt->m_keypoint.pt.y - kpt->m_camera->m_K.at<double>(1, 2)		/* y - y0 ou y - K12 */
				);
				this->m_measurements.emplace_back(pt2D); // ponto central da imagem
			}
		}
	}
}

// Destrutor
ParallelBundleAdjustment::~ParallelBundleAdjustment() {}

// Executa o bundle adjustment
void ParallelBundleAdjustment::runBundle(bool fullBA) {
	ParallelBA pba(ParallelBA::PBA_CPU_DOUBLE);

	if (!fullBA)
		for (CameraT cam : this->m_cameraData)
			cam.SetConstantCamera();

	pba.SetCameraData(this->m_cameraData.size(), &this->m_cameraData[0]);
	pba.SetPointData(this->m_pointData.size(), &this->m_pointData[0]);
	pba.SetProjection(this->m_measurements.size(), &this->m_measurements[0], &this->m_ptIdx[0], &this->m_camIdx[0]);

	pba.GetInternalConfig()->__lm_max_iteration = 100;

	pba.RunBundleAdjustment();
}

// Retorna a cena resultante ap�s a execu��o do BA
Graph* ParallelBundleAdjustment::getResult() {
	// atualiza as matrizes K e P das c�meras da cena
	for (int i : this->m_camIdx) {
		Camera* cam = this->m_graph->m_cameras[i];
		cv::Matx34f P;

		for (int j = 0; j < 3; j++)
			for (int k = 0; k < 3; k++)
				P(j, k) = this->m_cameraData[i].m[j][k];

		for (int j = 0; j < 3; j++)
			P(j, 3) = this->m_cameraData[i].t[j];

		cam->m_P = P;
		cam->m_K.at<double>(0, 0) = this->m_cameraData[i].f;
		cam->m_K.at<double>(1, 1) = this->m_cameraData[i].f;
	}

	// atualiza as tracks da cena
	for (int i : this->m_ptIdx) {
		this->m_graph->m_tracks[i]->m_point.x = this->m_pointData[i].xyz[0];
		this->m_graph->m_tracks[i]->m_point.y = this->m_pointData[i].xyz[1];
		this->m_graph->m_tracks[i]->m_point.z = this->m_pointData[i].xyz[2];
	}
	return this->m_graph;
}

// Exporta arquivo NView Match p/ visualiza��o no VisualSFM
void ParallelBundleAdjustment::exportNVM(const fs::path& outputDir, const std::string& filename) {
	// cria o diret�rio se ele n�o existe
	if (!fs::exists(outputDir))
		fs::create_directories(outputDir);

	fs::path outputPath = outputDir/filename; // caminho do arquivo
	std::ofstream ofs(outputPath.string(), ios::trunc | ios::out); // output file stream, cria ou abre o arquivo no modo truncate
	if (ofs.is_open()) {
		ofs << "NVM_V3_R9T\n";
		ofs << this->m_cameraData.size() << '\n' << std::setprecision(12);

		if (this->m_photoNames.size() < this->m_cameraData.size())
			this->m_photoNames.resize(this->m_cameraData.size(), std::string("unknown"));
		if (this->m_pointColor.size() < 3 * this->m_pointData.size())
			this->m_pointColor.resize(this->m_pointData.size() * 3, 0);

		for (size_t i = 0; i < this->m_cameraData.size(); ++i) {
			CameraT& cam = this->m_cameraData[i];
			ofs << this->m_photoNames[i] << ' ' << cam.GetFocalLength() << ' ';
			for (int j = 0; j < 9; j++)
				ofs << cam.m[0][j] << ' ';
			ofs << cam.t[0] << ' ' << cam.t[1] << ' ' << cam.t[2] << ' '
				<< cam.GetNormalizedMeasurementDistortion() << " 0\n";
		}

		ofs << this->m_pointData.size() << '\n';

		for (size_t i = 0, j = 0; i < this->m_pointData.size(); ++i) {
			Point3D& pt = this->m_pointData[i];
			int* pc = &this->m_pointColor[i * 3];
			ofs << pt.xyz[0] << ' ' << pt.xyz[1] << ' ' << pt.xyz[2] << ' '
				<< pc[0] << ' ' << pc[1] << ' ' << pc[2] << ' ';

			size_t je = j;
			while (je < this->m_ptIdx.size() && this->m_ptIdx[je] == (int)i)
				je++;
			ofs << (je - j) << ' ';

			for (; j < je; ++j)
				ofs << this->m_camIdx[j] << ' ' << " 0 " << this->m_measurements[j].x << ' ' << this->m_measurements[j].y << ' ';

			ofs << '\n';
		}

		ofs.close();
		std::cout << "  Exportado: \"" << outputPath.string() << "\"\n";
	} else {
		std::cout << "   Erro ao abrir arquivo: " << filename << std::endl;
	}
}

// Exporta a nuvem esparsa gerada como um arquivo .obj (somente lista de v�rtices)
void ParallelBundleAdjustment::exportOBJ(const fs::path& outputDir, const std::string& filename) {

	// cria o diret�rio se ele n�o existe
	if (!fs::exists(outputDir)) {
		fs::create_directories(outputDir);
	}

	fs::path outputPath = outputDir/filename; // caminho do arquivo
	std::ofstream ofs(outputPath.string(), ios::trunc | ios::out); // output file stream, cria ou abre o arquivo no modo truncate
	if (ofs.is_open()) {
		cv::Point3f pt;
		for (Track* t : this->m_graph->m_tracks) {
			pt = t->m_point;
			if (pt.x != -1 || pt.y != -1 || pt.z != -1)
				ofs << "v " << pt.x << " " << pt.y << " " << pt.z << '\n';
		}
		ofs.close();
		std::cout << "  Exportado: \"" << outputPath.string() << "\"\n";
	}
	else {
		std::cout << "  Erro ao abrir arquivo: " << filename << std::endl;
	}
}
