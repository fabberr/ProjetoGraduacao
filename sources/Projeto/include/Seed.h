#ifndef SEED_H
#define SEED_H

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

#include <Camera.h>
#include <Keypoint.h>

class Seed {
private:
	Camera* m_leftCam;
	Camera* m_rightCam;

public:
	cv::Point3f m_point3D;

	cv::Point2f m_p1;
	cv::Point2f m_p2;
	cv::Point2f m_p1Rect;
	cv::Point2f m_p2Rect;
	cv::Vec3f m_pointNormal;
	double m_score;
	double m_reprojectionError;

	Seed();
	Seed(cv::Point2f p1, cv::Point2f p2, Camera* leftCam, Camera* rightCam, double score);
	~Seed();

	double getReprojectionError(cv::Mat point3D, Camera* cam, cv::Point2f p);
	void triangulate3DPoints(cv::Mat H1, cv::Mat H2);
};

#endif // SEED_H
