#include <include/Graph.h>

// Construtor default
Graph::Graph() {}

// Construtor
Graph::Graph(StereoPair* pair) {
	this->m_tracks = pair->m_tracks;
	this->m_cameras.emplace_back(pair->m_leftCam);
	this->m_cameras.emplace_back(pair->m_rightCam);
}

// Destrutor
Graph::~Graph() {}

// Triangula os pontos 3D para todas as tracks do graph
void Graph::triangulate3DPoints() {
	for (Track* track : this->m_tracks)
		track->triangulatedPoints3D();
}

// Faz fus�o das tracks deste graph com a tracks de outro
void Graph::mergeTracks(std::vector<Track*> newTracks) {
	size_t tracksSize = this->m_tracks.size();
	size_t newTracksSize = newTracks.size();
	for (size_t i = 0; i < tracksSize; i++)
		for (size_t j = 0; j < newTracksSize; j++)
			if (newTracks[j] != nullptr && this->m_tracks[i]->mergeWith(newTracks[j]))
				newTracks[j] = nullptr;

	for (size_t i = 0; i < newTracksSize; i++)
		if (newTracks[i] != nullptr)
			this->m_tracks.emplace_back(newTracks[i]);
}

// Faz a fus�o desse graph com outro
void Graph::mergeWith(Graph* other) {
	// atualizando a origem da c�mera
	other->m_cameras.back()->updateOrigin(this->m_cameras.back()->m_P);

	std::cout << " Adicionando camera na cena: \"" << this->m_cameras.back()->m_pathToImage << "\"\n";
	this->m_cameras.emplace_back(other->m_cameras.back());
	this->mergeTracks(other->m_tracks);
}

// Adiciona as tracks de um par nesse graph
void Graph::addTracks(const std::vector<Track*>& newTracks) {
	for (Track* t : newTracks)
		this->m_tracks.emplace_back(t);
}

// Adiciona um par no graph
void Graph::addPair(StereoPair* p) {
	bool hasLeftCam = false, hasRightCam = false;
	Camera* leftCam = nullptr;
	Camera* rightCam = nullptr;

	for (Camera* cam : this->m_cameras) {
		if (cam == p->m_leftCam) {
			hasLeftCam = true;
			leftCam = p->m_leftCam;
		}
		if (cam == p->m_rightCam) {
			hasRightCam = true;
			rightCam = p->m_rightCam;
		}
	}

	if (!hasLeftCam || !hasRightCam || p->m_pairMerged) return;

	this->addTracks(p->m_tracks);
	p->m_pairMerged = true;
}

// Retorna o tamamho do modelo (quantidade de tracks)
size_t Graph::getModelSize() {
	size_t size = 0;
	for (Track* t : this->m_tracks)
		if (t->m_point.x != -1 && t->m_point.y != -1 && t->m_point.z != -1)
			size++;
	return size;
}

// Filtra os pontos ruins
void Graph::filterPoints() {
	for (Track* t : this->m_tracks)
		t->filterPoint();
}

/*
* Exporta os resultados da calibra��o das c�meras como um arquivo .sfm
*
* Formato:
*
*	<*.sfm>
*	nCameras
*
*	pathToImage_1 P_1 f_1
*	pathToImage_2 P_2 f_2
*	...
*	pathToImage_N P_N f_N
*	<eof>
*
* Onde:
* nCameras = N
* pathToImage = path/to/image/filename.<png | jpg>
* P = matriz instr�nsseca
* f = dist�ncia focal
*/
void Graph::exportSFM(const fs::path& outputDir, const std::string& filename) {
	
	// cria o diret�rio se ele n�o existe
	if (!fs::exists(outputDir)) {
		fs::create_directories(outputDir);
	}

	fs::path outputPath = outputDir/filename; // caminho do arquivo
	std::ofstream ofs(outputPath.string(), ios::trunc | ios::out); // output file stream, cria ou abre o arquivo no modo truncate
	if (ofs.is_open()) {
		ofs << this->m_cameras.size() << "\n\n";

		for (Camera* cam : this->m_cameras)
			ofs << cam->sfmString();
		ofs.close();
		std::cout << "  Exportado: \"" << outputPath.string() << "\"\n";
	} else {
		std::cout << "  Erro ao abrir arquivo: " << filename << std::endl;
	}
}

// Exporta a nuvem esparsa gerada como um arquivo .obj (somente lista de v�rtices)
void Graph::exportOBJ(const fs::path& outputDir, const std::string& filename) {

	// cria o diret�rio se ele n�o existe
	if (!fs::exists(outputDir)) {
		fs::create_directories(outputDir);
	}

	fs::path outputPath = outputDir/filename; // caminho do arquivo
	std::ofstream ofs(outputPath.string(), ios::trunc | ios::out); // output file stream, cria ou abre o arquivo no modo truncate
	if (ofs.is_open()) {
		cv::Point3f pt;
		for (Track* t : this->m_tracks) {
			pt = t->m_point;
			if (pt.x != -1 || pt.y != -1 || pt.z != -1)
				ofs << "v " << pt.x << " " << pt.y << " " << pt.z << '\n';
		}
		ofs.close();
		std::cout << "  Exportado: \"" << outputPath.string() << "\"\n";
	} else {
		std::cout << "  Erro ao abrir arquivo: " << filename << std::endl;
	}
}
