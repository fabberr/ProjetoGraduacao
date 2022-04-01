#ifndef KEYPOINT_H
#define KEYPOINT_H

#include <opencv2/opencv.hpp>

#include <cmath>

#include <Camera.h>

class Keypoint {
public:
	//----- Membros
	cv::KeyPoint	m_keypoint;		// keypoint 
	Camera*			m_camera;		// c�mera da esquerda que tem vis�o desse keypoint

	//----- Construtores e destrutor
	Keypoint();
	Keypoint(cv::KeyPoint keypoint, Camera* camera);
	~Keypoint();

	//----- M�todos
	double computeReprojectionError(cv::Mat triangulatedPoint);
	bool equals(Keypoint other);
	bool operator==(const Keypoint& other);
	bool operator!=(const Keypoint& other);
};

#endif // KEYPOINT_H
