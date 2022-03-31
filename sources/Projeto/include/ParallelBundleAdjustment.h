#ifndef PARALLELBUNDLEADJUSTMENT_H
#define PARALLELBUNDLEADJUSTMENT_H

// STL
#include <vector>
#include <string>

// 3rd-party
#include <opencv4/opencv2/opencv.hpp>

#include <pba/pba.h>
#include <pba/DataInterface.h>

// #include <pcl/point_types.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/features/normal_3d_omp.h>
// #include <pcl/io/ply_io.h>
// #include <pcl/surface/mls.h>
// #include <pcl/filters/radius_outlier_removal.h>

// internal
#include <Graph.h>
#include <Track.h>
#include <Camera.h>
#include <Seed.h>

/*
* Wrapper para m�todos da biblioteca Multi-Core Bundle Adjustment (ou ParallelBA)
*/
class ParallelBundleAdjustment {
public:
	//----- Membros
	Graph*						m_graph;			// cena
	std::vector<CameraT>		m_cameraData;		// c�mera
	std::vector<Point3D>		m_pointData;		// ponto 3D
	std::vector<Point2D>		m_measurements;		// medidas/vetor de proje��o
	std::vector<int>			m_camIdx;			// �ndice das c�meras
	std::vector<int>			m_ptIdx;			// �ndice dos pontos
	std::vector<std::string>	m_photoNames;		// usado p/ gerar o arquivo .nvm
	std::vector<int>			m_pointColor;		// usado p/ gerar o arquivo .nvm

	//----- Construtores e destrutor
	ParallelBundleAdjustment();
	ParallelBundleAdjustment(Graph* graph);
	~ParallelBundleAdjustment();

	//----- M�todos
	void runBundle(bool fullBA);
	Graph* getResult();
	void exportNVM(const fs::path& outputDir, const std::string& filename = "nview_match.nvm");
	void exportOBJ(const fs::path& outputDir, const std::string& filename = "cloud.obj");
};

#endif // PARALLELBUNDLEADJUSTMENT_H
