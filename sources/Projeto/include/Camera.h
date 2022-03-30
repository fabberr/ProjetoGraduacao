#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <filesystem>

#include <MathUtils.h>

namespace fs = std::filesystem;

/*
* Camera - representa uma c�mera
* Cont�m uma imagem, matriz intr�nseca, lista de keypoints detectados e seus descritores
*/
class Camera {

public:
	//----- Membros
	std::string					m_pathToImage;	// caminho absoluto at� a imagem
	cv::Mat						m_image;		// imagem
	std::vector<cv::KeyPoint>	m_keypoints;	// features detectadas
	cv::Mat						m_descriptors;	// descritores computados
	cv::Mat						m_K;			// matriz intr�nseca http://ksimek.github.io/2013/08/13/intrinsic/
	cv::Matx34f					m_P;			// matriz extr�nsseca (pose) https://ksimek.github.io/2012/08/22/extrinsic/

public:
	//----- Construtores e destrutor
	Camera();
	Camera(const std::string& pathToImage);
	~Camera();

public:
	//----- M�todos
	cv::Mat_<double> defaultIntrinsic();
	void detectAndCompute(int nfeatures = 8000);
	void getViewVector(double* outViewVector);
	double angleBetween(Camera* other);
	void updateOrigin(cv::Matx34f newOrigin);
	std::string sfmString();
};

#endif // CAMERA_H
