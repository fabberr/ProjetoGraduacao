#ifndef STEREOPAIR_H
#define STEREOPAIR_H

// STL
#include <vector>
#include <mutex>
#include <string>

// 3rd party
#include <opencv2/opencv.hpp>

// #include <pcl/point_types.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/features/normal_3d_omp.h>
// #include <pcl/io/ply_io.h>
// #include <pcl/surface/mls.h>
// #include <pcl/filters/radius_outlier_removal.h>

// internal
#include <include/Camera.h>
#include <include/Track.h>
#include <include/Seed.h>

/*
* StereoPair: representa um par de c�meras
* Cont�m duas c�meras e uma lista de tracks formada pelos matches entres os keypoints das duas c�meras
*/
class StereoPair {
public:
	//----- Membros
	Camera*					m_leftCam;		// c�mera da esquerda
	Camera*					m_rightCam;		// c�mera da direita
	std::vector<Track*>		m_tracks;		// lista de tracks
	bool					m_pairMerged;	// flag - indica se j� foi feita fus�o deste par ou n�o

	// membros usados para encontrar a matriz essencial usada no c�lculo das poses das c�meras
	std::vector<cv::KeyPoint>	m_goodKeypointsL;	// lista de keypoints bons da c�mera da esquerda
	std::vector<cv::KeyPoint>	m_goodKeypointsR;	// lista de keypoints bons da c�mera da direita
	std::vector<cv::Point2f>	m_pointsL;			// lista das coordenadas 2D dos keypoints bons da c�mera da esquerda
	std::vector<cv::Point2f>	m_pointsR;			// lista das coordenadas 2D dos keypoints bons da c�mera da direita
	cv::Mat						m_E;				// matriz essencial, relaciona pontos em imagens est�reo https://en.wikipedia.org/wiki/Essential_matrix

	// membros utilizados na computa��o da nuvem densa de pontos
	cv::Mat						m_leftWarp;
	cv::Mat						m_leftMask;
	std::vector<cv::KeyPoint>	m_leftKeyPoints;
	cv::Mat						m_leftDescriptors;
	cv::Mat						m_leftH;

	cv::Mat						m_rightWarp;
	cv::Mat						m_rightMask;
	std::vector<cv::KeyPoint>	m_rightKeyPoints;
	cv::Mat						m_rightDescriptors;
	cv::Mat						m_rightH;

	std::vector<Seed*> m_seeds;

	// pcl::PointCloud<pcl::PointXYZ>::Ptr		m_pclCloud = nullptr;				// nuvem de pontos densa
	// pcl::PointCloud<pcl::Normal>::Ptr		m_pclCloudNormals = nullptr;		// vetores normais da nuvem de pontos densa
	// pcl::PointCloud<pcl::PointNormal>::Ptr	m_pclCloudWithNormals = nullptr;	// nuvem de pontos densa com suas normais

	//----- Construtores e destrutor
	StereoPair();
	StereoPair(Camera* left, Camera* right);
	~StereoPair();

	//----- M�todos
	size_t match();
	void createTracks();
	void recoverPose();

	void rectify();
	void detectAndComputeDense();
	void matchDense();
	void createInitialSeeds();
	void computeNewSeeds();
	void triangulate3DPoints();

	// void PCL_createPointCloud();
	// void PCL_filterCloud();
	// void PCL_computeNormals();

	// /**
	// * Exporta a nuvem no formato .ply no diret�rio e com nome de arquivo especificados.
	// * 
	// * @param outputDir -- Caminho at� o diret�rio no qual oa rquivo ser� salvo.
	// * @param filename -- Nome do arquivo com extens�o (.ply).
	// * @param cloud -- Opcional - Se na�o for nulo, especifica a nuvem a ser salva no lugar da nuvem membro da classe. Por padr�o, `nullptr`.
	// */
	// void PCL_exportCloudPLY(
	// 	const fs::path& outputDir, 
	// 	const std::string& filename, 
	// 	pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud = nullptr
	// );
};

#endif // STEREOPAIR_H
