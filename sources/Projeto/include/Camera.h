#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <filesystem>

#include <include/MathUtils.h>

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
	const static int N_FEATURES = 9000;			// n�mero de features

	//----- Construtores e destrutor
	Camera();
	Camera(const std::string& pathToImage);
	~Camera();

	//----- M�todos
	cv::Mat_<double> defaultIntrinsic();
	void detectAndCompute(bool useNfeatures = false);
	void getViewVector(double* outViewVector);
	double angleBetween(Camera* other);
	void updateOrigin(cv::Matx34f newOrigin);
	std::string sfmString();
};

#endif // CAMERA_H
