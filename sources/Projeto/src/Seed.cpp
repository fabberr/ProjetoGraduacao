#include <Seed.h>

Seed::Seed() {
	this->m_p1 = cv::Point2f(-1, -1);
	this->m_p2 = cv::Point2f(-1, -1);
	this->m_p1Rect = cv::Point2f(-1, -1);
	this->m_p2Rect = cv::Point2f(-1, -1);
	this->m_pointNormal = cv::Vec3f(0, 0, 0);
	this->m_score = 0;
	this->m_reprojectionError = VTK_DOUBLE_MAX;

	this->m_leftCam = nullptr;
	this->m_rightCam = nullptr;
	this->m_point3D = cv::Point3f(-1, -1, -1);
}

Seed::Seed(cv::Point2f p1, cv::Point2f p2, Camera* leftCam, Camera* rightCam, double score) {
	this->m_p1 = p1;
	this->m_p2 = p2;
	this->m_p1Rect = cv::Point2f(-1, -1);
	this->m_p2Rect = cv::Point2f(-1, -1);
	this->m_pointNormal = cv::Vec3f(0, 0, 0);
	this->m_score = score;
	this->m_reprojectionError = VTK_DOUBLE_MAX;

	this->m_leftCam = leftCam;
	this->m_rightCam = rightCam;
	this->m_point3D = cv::Point3f(-1, -1, -1);
}

Seed::~Seed() {}

double Seed::getReprojectionError(cv::Mat point3D, Camera* cam, cv::Point2f p) {
	cv::Mat rvecLeft;
	cv::Rodrigues(cam->m_P.get_minor<3, 3>(0, 0), rvecLeft);
	cv::Mat tvecLeft(cam->m_P.get_minor<3, 1>(0, 3).t());

	std::vector<cv::Point2f> projectedOnLeft(1);
	projectPoints(point3D, rvecLeft, tvecLeft, cam->m_K, cv::Mat(), projectedOnLeft);

	return cv::norm(projectedOnLeft[0] - p);
}

void Seed::triangulate3DPoints(cv::Mat H1, cv::Mat H2) {
	std::vector<cv::Point2f> leftPoints, rightPoints;

	leftPoints.emplace_back(this->m_p1);
	rightPoints.emplace_back(this->m_p2);

	std::vector<cv::Point2f> leftPointsTransform, rightPointsTransform;
	cv::perspectiveTransform(leftPoints, leftPointsTransform, H1);
	cv::perspectiveTransform(rightPoints, rightPointsTransform, H2);

	this->m_p1Rect = leftPointsTransform[0];
	if (this->m_p1Rect.x < 0) this->m_p1Rect.x = 0;
	if (this->m_p1Rect.y < 0) this->m_p1Rect.y = 0;
	if (this->m_p1Rect.x >= this->m_leftCam->m_image.cols) this->m_p1Rect.x = this->m_leftCam->m_image.cols - 1;
	if (this->m_p1Rect.y >= this->m_leftCam->m_image.rows) this->m_p1Rect.y = this->m_leftCam->m_image.rows - 1;
	
	this->m_p2Rect = rightPointsTransform[0];
	if (this->m_p2Rect.x < 0) this->m_p2Rect.x = 0;
	if (this->m_p2Rect.y < 0) this->m_p2Rect.y = 0;
	if (this->m_p2Rect.x >= this->m_rightCam->m_image.cols) this->m_p2Rect.x = this->m_rightCam->m_image.cols - 1;
	if (this->m_p2Rect.y >= this->m_rightCam->m_image.rows) this->m_p2Rect.y = this->m_rightCam->m_image.rows - 1;

	cv::Mat leftPointsNorm, rightPointsNorm;
	cv::undistortPoints(leftPointsTransform, leftPointsNorm, this->m_leftCam->m_K, cv::Mat());
	cv::undistortPoints(rightPointsTransform, rightPointsNorm, this->m_rightCam->m_K, cv::Mat());

	cv::Mat points3DHomogeneous;
	cv::triangulatePoints(this->m_leftCam->m_P, this->m_rightCam->m_P, leftPointsNorm, rightPointsNorm, points3DHomogeneous);

	cv::Mat points3D;
	cv::convertPointsFromHomogeneous(points3DHomogeneous.t(), points3D);

	this->m_reprojectionError = (
		getReprojectionError(points3D, this->m_leftCam, leftPointsTransform[0]) + 
		getReprojectionError(points3D, this->m_rightCam, rightPointsTransform[0])
	) / 2.0;

	this->m_point3D = cv::Point3f(
		points3D.at<float>(0, 0), 
		points3D.at<float>(0, 1), 
		points3D.at<float>(0, 2)
	);
}
