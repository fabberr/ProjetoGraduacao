#ifndef GRAPH_H
#define GRAPH_H

#include <opencv2/opencv.hpp>
#include <filesystem>
#include <vector>

#include <StereoPair.h>

namespace fs = std::filesystem;

/*
* Graph: representa a cena recriada
* Contem uma lista de tracks e uma lista de c�meras
*/
class Graph {
public:
	//----- Membros
	std::vector<Track*>		m_tracks;	// lista de tracks
	std::vector<Camera*>	m_cameras;	// lista de c�meras

	//----- Construtores e destrutor
	Graph();
	Graph(StereoPair* pair);
	~Graph();

	//----- M�todos
	void triangulate3DPoints();
	void mergeTracks(std::vector<Track*> newTracks);
	void mergeWith(Graph* other);
	void addTracks(const std::vector<Track*>& newTracks);
	void addPair(StereoPair* p);
	size_t getModelSize();
	void filterPoints();
	void exportSFM(const fs::path& outputDir = "./output/", const std::string& filename = "pose.sfm");
	void exportOBJ(const fs::path& outputDir = "./output/", const std::string& filename = "point_cloud.obj");
};

#endif // GRAPH_H
