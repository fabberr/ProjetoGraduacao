#include <include/Keypoint.h>

// Construtor padr�o
Keypoint::Keypoint() {
	this->m_camera = nullptr;
}

// Construtor
Keypoint::Keypoint(cv::KeyPoint keypoint, Camera* camera) {
	this->m_keypoint = keypoint;
	this->m_camera = camera;
}

// Destrutor
Keypoint::~Keypoint() {}

// Calcula o erro de reproje��o de um ponto triangulado
double Keypoint::computeReprojectionError(cv::Mat triangulatedPoint) {
	cv::Mat r; // vetor de rota��o
	cv::Mat t; // vetor de transla��o

	// obt�m a matriz de rota��o a partir da matriz extr�nsseca da c�mera da esquerda e a transforma em um vetor de rota��p
	cv::Matx33d R = this->m_camera->m_P.get_minor<3, 3>(0, 0);
	cv::Rodrigues(R, r);

	// obt�m o vetor de rota��o a partir da matriz extr�nsseca da c�mera da esquerda
	t = cv::Mat(this->m_camera->m_P.get_minor<3, 1>(0, 3).t());

	// projeta o ponto 3D triangulado num plano
	std::vector<cv::Point2f> projectedPoints(1);
	cv::Mat K = this->m_camera->m_K;
	cv::projectPoints(triangulatedPoint, r, t, K, cv::Mat(), projectedPoints);

	// calcula e retorna o erro de reproje��o
	return cv::norm(projectedPoints[0] - this->m_keypoint.pt);
}

// Verifica se este keypoint � igual a outro
bool Keypoint::equals(Keypoint other) {
	if (this->m_camera != other.m_camera) return false;

	float dist = 1e-10;
	cv::Vec2f dx = this->m_keypoint.pt - other.m_keypoint.pt;
	return std::sqrt(dx.dot(dx)) < dist;
}

// operator== overload
bool Keypoint::operator==(const Keypoint& other) {
	return this->equals(other);
}

// operator!= overload
bool Keypoint::operator!=(const Keypoint& other) {
	return !this->equals(other);
}
