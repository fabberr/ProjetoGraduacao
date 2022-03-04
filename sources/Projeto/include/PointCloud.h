#ifndef POINTCLOUD_H
#define POINTCLOUD_H

// STL
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

// headers de terceiros
#include <opencv2/opencv.hpp>

// #include <pcl/point_types.h>
// #include <pcl/features/normal_3d.h>

// headers internos
#include <include/Camera.h>
#include <include/ParallelBundleAdjustment.h>

namespace fs = std::filesystem; // std::filesystem namespace alias

/*
* Implementa os algoritmos usados na gera��o da nuvem de pontos (esparsa ou densa)
*/
class PointCloud {
private:
	//----- Membros
	fs::path	m_path;			// caminho at� o diret�rio do dataset (sparse) ou diret�rio do arquivo .sfm (dense)
	size_t		m_imgCount;		// n�mero de imagens a serem usadas

public:
	//----- Construtores e destrutor
	PointCloud(const fs::path& path, int imgCount);
	PointCloud(const fs::path& path);
	PointCloud();
	~PointCloud();

	//----- M�todos
	const fs::path computeSparse();
	// const fs::path computeDense();
	// std::vector<Camera*> loadCameras();

	// void exportNVM(
	// 	const std::vector<Camera*>& cameras, 
	// 	pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud, 
	// 	const fs::path& outputDir, 
	// 	const std::string& filename = "cloud_dense.nvm"
	// );
};

#endif // POINTCLOUD_H
