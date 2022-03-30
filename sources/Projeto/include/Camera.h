#ifndef CAMERA_H
#define CAMERA_H

/********** Headers **********/

// libc++
#include <string>
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>

/********** Camera.h **********/

/*
* Camera - representa uma câmera
* Contém uma imagem, matriz intrínseca, lista de keypoints detectados e seus descritores
*/
class Camera {

public:
	/********** Membros Públicos **********/
	
	std::string					m_pathToImage;	// caminho absoluto até a imagem
	cv::Mat						m_image;		// imagem
	std::vector<cv::KeyPoint>	m_keypoints;	// features detectadas
	cv::Mat						m_descriptors;	// descritores computados
	cv::Mat						m_K;			// matriz intrínseca http://ksimek.github.io/2013/08/13/intrinsic/
	cv::Matx34f					m_P;			// matriz extrínsseca (pose) https://ksimek.github.io/2012/08/22/extrinsic/

public:
	/********** Construtores e Destrutor **********/
	
	Camera();
	Camera(const std::string& pathToImage);
	~Camera();

public:
	/********** Funções Membro Públicas **********/
	
	cv::Mat_<double> defaultIntrinsic();
	void detectAndCompute(int nfeatures = 8000);
	void getViewVector(double* outViewVector);
	double angleBetween(Camera* other);
	void updateOrigin(cv::Matx34f newOrigin);
	std::string sfmString();
};

#endif // CAMERA_H
