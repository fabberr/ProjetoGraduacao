#include <Camera.h>

// Construtor default
Camera::Camera() {
	m_pathToImage = "";
	m_image = cv::Mat();
	m_K = cv::Mat();
}

// Construtor
Camera::Camera(const std::string& pathToImage) {
	this->m_pathToImage = pathToImage;
	this->m_image = cv::imread(pathToImage, cv::ImreadModes::IMREAD_GRAYSCALE);
	this->m_K = defaultIntrinsic();
}

// Destrutor
Camera::~Camera() {}

// Cria e retorna uma matriz intr�nseca K com valores padr�o
cv::Mat_<double> Camera::defaultIntrinsic() {
	double f = 1.2 * std::max(m_image.rows, m_image.cols); // dist�ncia focal
	return (cv::Mat_<double>(3, 3) << 
		f, 0, m_image.cols / 2,
		0, f, m_image.rows / 2,
		0, 0, 1
	);
}

// Detecta os keypoints e computa seus descritores
void Camera::detectAndCompute(bool useNfeatures) {
	cv::Ptr<cv::AKAZE> akaze_f2d = cv::AKAZE::create();
	akaze_f2d->detectAndCompute(m_image, cv::noArray(), m_keypoints, m_descriptors);
}

// Obtem vetor de visao (vetor double R3) usado no c�culo do angulo entre as c�meras
void Camera::getViewVector(double* outViewVector) {
	double* p1 = new double[3]{ .0, .0, .0 };
	double* p2 = new double[3]{ .0, .0, .5 };

	MathUtils::transformPoint(p1, this->m_P);
	MathUtils::transformPoint(p2, this->m_P);

	vtkMath::Subtract(p1, p2, outViewVector);
	delete[] p1;
	delete[] p2;
}

// Calcula o �ngulo (em graus) entre essa c�mera e outra
double Camera::angleBetween(Camera* other) {
	double* thisViewVector = new double[3];
	double* otherViewVector = new double[3];
	this->getViewVector(thisViewVector);
	other->getViewVector(otherViewVector);

	double angle = vtkMath::DegreesFromRadians(vtkMath::AngleBetweenVectors(thisViewVector, otherViewVector));

	delete[] thisViewVector;
	delete[] otherViewVector;
	return angle;
}

// Atualiza a origem da c�mera a partir de uma matriz extr�nsseca
void Camera::updateOrigin(cv::Matx34f newOrigin) {
	cv::Mat R01 = cv::Mat_<double>(3, 3);
	cv::Mat t01 = cv::Mat_<double>(3, 1);
	MathUtils::getRotationMatrix(R01, newOrigin);
	MathUtils::getTranslationVector(t01, newOrigin);

	cv::Mat R12 = cv::Mat_<double>(3, 3);
	cv::Mat t12 = cv::Mat_<double>(3, 1);
	MathUtils::getRotationMatrix(R12, this->m_P);
	MathUtils::getTranslationVector(t12, this->m_P);

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			this->m_P(i, j) = 0;
			for (int k = 0; k < 3; k++)
				this->m_P(i, j) += R01.at<double>(i, k) * R12.at<double>(k, j);
		}
	}

	for (int i = 0; i < 3; i++) {
		this->m_P(i, 3) = 0;
		for (int j = 0; j < 3; j++)
			this->m_P(i, 3) += R01.at<double>(i, j) * t12.at<double>(j, 0);
	}

	for (int i = 0; i < 3; i++)
		this->m_P(i, 3) = t01.at<double>(i, 0) + this->m_P(i, 3);
}

// Gera uma string que representa essa c�mera no formato .sfm
std::string Camera::sfmString() {
	std::string sfm("");

	sfm += this->m_pathToImage + " ";
	for (int i = 0; i < this->m_P.rows; i++)
		for (int j = 0; j < this->m_P.cols - 1; j++)
			sfm += std::to_string(this->m_P(i, j)) + " ";
	for (int i = 0; i < this->m_P.rows; i++)
		sfm += std::to_string(this->m_P(i, 3)) + " ";
	sfm += std::to_string(this->m_K.at<double>(0, 0)) + " " + std::to_string(this->m_K.at<double>(1, 1)) + '\n';
	return sfm;
}
