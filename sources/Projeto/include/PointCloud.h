#ifndef POINTCLOUD_H
#define POINTCLOUD_H

/********** Headers **********/

// libc++
#include <iostream>
#include <string>
#include <vector>
#include <filesystem>

namespace fs = std::filesystem; // std::filesystem namespace alias

// libc
#include <cstddef> // std::size_t

// OpenCV
#include <opencv2/opencv.hpp>

// PCL
// #include <pcl/point_types.h>
// #include <pcl/features/normal_3d.h>

// headers internos
#include <Camera.h>
#include <ParallelBundleAdjustment.h>

/********** PointCloud.h **********/

/*
* Implementa os algoritmos usados na geração da nuvem de pontos (esparsa ou densa)
*/
class PointCloud {
private:
	/********** Membros Privados **********/
	
	fs::path 	m_inputPath; 	// caminho até o diretório do dataset (sparse) ou diretório do arquivo .sfm (dense)
	fs::path 	m_outputPath; 	// caminho até po diretório de saída
	size_t		m_imgCount;		// número de imagens a serem usadas

public:
	/********** Construtores e Destrutor **********/
	
	PointCloud(const fs::path& datasetPath, const fs::path& outputPath= "./output/", size_t imgCount = 0);

	PointCloud() = delete; // construtor padrão desabilitado
	~PointCloud();

public:
	/********** Métodos Públicos **********/
	
	void computeSparse();
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
