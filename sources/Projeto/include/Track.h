#ifndef TRACK_H
#define TRACK_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>

#include <Keypoint.h>

/*
* Track - representa um ponto no espa�o 3D
* Cont�m um ponto e uma lista de keypoints que correspondem a este ponto
*/
class Track {
public:
	//----- Membros
	cv::Point3f				m_point;				// ponto 3D
	std::vector<Keypoint*>	m_keypoints;			// lista de keypoints que correspondem ao ponto
	double					m_lastAvgReprojError;	// �ltima m�dia dos erros de reproje��o de um ponto

	//----- Construtores e destrutor
	Track();
	~Track();

	//----- M�todos
	void addKeypoint(Keypoint* kpt);
	void triangulatedPoints3D();
	bool mergeWith(Track* other);
	void filterPoint();
};

#endif // TRACK_H
