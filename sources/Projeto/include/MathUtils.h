#ifndef MATHUTILS_H
#define MATHUTILS_H

#include <opencv2/opencv.hpp>
#include <vtk-9.0/vtkMath.h>
#include <vtk-9.0/vtkSmartPointer.h>
#include <vtk-9.0/vtkMatrix4x4.h>

class MathUtils
{
public:
	MathUtils() {}
	~MathUtils() {}

	// Multiplica um ponto no espa�o R3 pela inversa de uma matriz extr�nsseca
	static void transformPoint(double* p, cv::Matx34f matrixRT) {
		vtkSmartPointer<vtkMatrix4x4> matrixRT4x4 = vtkSmartPointer<vtkMatrix4x4>::New();

		// transforma a matrix extr�nsseca 3x4 passada por par�metro em uma matriz 4x4
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 4; j++)
				matrixRT4x4->SetElement(i, j, matrixRT(i, j));
		matrixRT4x4->SetElement(3, 0, 0);
		matrixRT4x4->SetElement(3, 1, 0);
		matrixRT4x4->SetElement(3, 2, 0);
		matrixRT4x4->SetElement(3, 3, 1);

		// Resultado:
		//		   [ 1    0    0    t1 ]
		// [R|t] = [ 0    1    0    t2 ]
		//		   [ 0    0    1    t3 ]
		//		   [ 0    0    0    1  ]

		matrixRT4x4->Invert(); // calcula a matriz invertida ([R|t] = [R|t]^I)

		// multiplica o vetor pela matriz inversa
		double x = (matrixRT4x4->Element[0][0] * p[0] + matrixRT4x4->Element[0][1] * p[1] + matrixRT4x4->Element[0][2] * p[2] + matrixRT4x4->Element[0][3]);
		double y = (matrixRT4x4->Element[1][0] * p[0] + matrixRT4x4->Element[1][1] * p[1] + matrixRT4x4->Element[1][2] * p[2] + matrixRT4x4->Element[1][3]);
		double z = (matrixRT4x4->Element[2][0] * p[0] + matrixRT4x4->Element[2][1] * p[1] + matrixRT4x4->Element[2][2] * p[2] + matrixRT4x4->Element[2][3]);
		p[0] = x;
		p[1] = y;
		p[2] = z;
	}

	// extrai a matriz de rota��o da matrix extr�nsseca P
	static void getRotationMatrix(cv::Mat& R, cv::Matx34d P) {
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				R.at<double>(i, j) = P(i, j);
	}

	static void getRotationMatrix(cv::Mat& R, cv::Mat P)
	{
		for (unsigned int i = 0; i < 3; i++)
			for (unsigned int j = 0; j < 3; j++)
				R.at<double>(i, j) = P.at<double>(i, j);
	}

	// extrai o vetor de transla��o da matrix extr�nsseca P
	static void getTranslationVector(cv::Mat& t, cv::Matx34d P) {
		for (int i = 0; i < 3; i++)
			t.at<double>(i, 0) = P(i, 3);
	}

	// compoe a matriz da c�mera usando os componentes Re t
	static cv::Mat composeCameraMatrix(cv::Mat R, cv::Mat t) {
		cv::Mat RT = cv::Mat_<double>(3, 4);
		for (int i = 0; i < R.rows; i++)
			for (int j = 0; j < R.cols; j++)
				RT.at<double>(i, j) = R.at<double>(i, j);

		for (int i = 0; i < 3; i++)
			RT.at<double> (i, 3) = t.at<double>(i, 0);

		return RT;
	}

	// Multiplica uma matriz intrinsseca mat pela matriz extrinsseca P
	static cv::Mat multiplyPose(cv::Mat mat, cv::Matx34f P) {
		if (mat.cols != P.rows)
			return cv::Mat(); // imposs�vel multiplicar, retorne matriz vazia

		cv::Mat res = cv::Mat_<double>(mat.rows, mat.cols);
		double sum = 0;
		for (int i = 0; i < mat.rows; i++) {
			for (int j = 0; j < mat.cols; j++) {
				for (int k = 0; k < P.rows; k++)
					sum += mat.at<double>(i, k) * P(k, j);
				res.at<double>(i, j) = sum;
				sum = 0;
			}
		}

		return res;
	}

	static cv::Mat createRT(cv::Mat R, cv::Mat t)
	{
		cv::Mat RT = cv::Mat_<double>(3, 4);
		for (int i = 0; i < R.rows; i++)
		{
			for (int j = 0; j < R.cols; j++)
			{
				RT.at<double>(i, j) = R.at<double>(i, j);
			}
		}
		for (int i = 0; i < 3; i++)
		{
			RT.at<double>(i, 3) = t.at<double>(i, 0);
		}
		return RT;
	}

	static cv::Mat htx(cv::Mat H, cv::Mat X) {
		cv::Mat newX;

		X.copyTo(newX);
		newX.push_back(cv::Mat_<double>::ones(1, X.cols));

		cv::Mat Y = H * newX;

		cv::Mat newY;
		cv::divide(Y, cv::repeat(Y.rowRange(Y.rows - 1, Y.rows), Y.rows, 1), newY);

		return newY.rowRange(0, newY.rows - 1);
	}

	static cv::Mat findBB(cv::Mat H, cv::Size imageSize)
	{
		cv::Mat corners = (cv::Mat_<double>(2, 4) << 0, 0, imageSize.width, imageSize.width, 0, imageSize.height, 0, imageSize.height);
		corners = htx(H, corners);

		double minX, maxX, minY, maxY;

		cv::minMaxLoc(corners.rowRange(0, 1), &minX, &maxX);
		cv::minMaxLoc(corners.rowRange(1, 2), &minY, &maxY);

		cv::Mat bb = (cv::Mat_<double>(4, 1) << floor(minX), floor(minY), ceil(maxX), ceil(maxY));

		return bb;
	}

	static void rectifyPoses(cv::Matx34f P1, cv::Matx34f P2, cv::Mat K, cv::Mat& H1, cv::Mat& H2, cv::Mat& Pn1, cv::Mat& Pn2) {
		cv::Mat R1 = cv::Mat_<double>(3, 3);
		cv::Mat R2 = cv::Mat_<double>(3, 3);
		cv::Mat t1 = cv::Mat_<double>(3, 1);
		cv::Mat t2 = cv::Mat_<double>(3, 1);

		MathUtils::getRotationMatrix(R1, P1);
		MathUtils::getRotationMatrix(R2, P2);
		MathUtils::getTranslationVector(t1, P1);
		MathUtils::getTranslationVector(t2, P2);

		cv::Mat c1 = -1 * R1.t() * t1;
		cv::Mat c2 = -1 * R2.t() * t2;

		cv::Mat v1 = (c2 - c1);
		cv::Mat aux3 = R1.rowRange(1, 2).cross(R1.rowRange(2, 3)) * c2;
		double res = aux3.at<double>(0, 0);
		if (res > 0) {
			res = 1;
		} else if (res < 0) {
			res = -1;
		}
		v1 *= res;

		cv::Mat v2 = R1.rowRange(2, 3).t().cross(v1);

		cv::Mat v3 = v1.cross(v2);

		double normV1 = cv::norm(v1, cv::NORM_L2);
		double normV2 = cv::norm(v2, cv::NORM_L2);
		double normV3 = cv::norm(v3, cv::NORM_L2);
		cv::Mat R = cv::Mat_<double>(3, 3);
		for (int i = 0; i < R.rows; i++) {
			for (int j = 0; j < R.cols; j++) {
				if (i == 0) {
					R.at<double>(i, j) = v1.at<double>(j, 0) / normV1;
				} else if (i == 1) {
					R.at<double>(i, j) = v2.at<double>(j, 0) / normV2;
				} else if (i == 2) {
					R.at<double>(i, j) = v3.at<double>(j, 0) / normV3;
				}
			}
		}

		Pn1 = K * MathUtils::createRT(R, -1 * R * c1);
		Pn2 = K * MathUtils::createRT(R, -1 * R * c2);

		cv::Mat P1org = multiplyPose(K, (cv::Mat)P1);
		cv::Mat P2org = multiplyPose(K, (cv::Mat)P2);

		cv::Mat R_P1org = cv::Mat_<double>(3, 3);
		cv::Mat R_P2org = cv::Mat_<double>(3, 3);
		cv::Mat R_Pn1 = cv::Mat_<double>(3, 3);
		cv::Mat R_Pn2 = cv::Mat_<double>(3, 3);

		MathUtils::getRotationMatrix(R_P1org, P1org);
		MathUtils::getRotationMatrix(R_P2org, P2org);
		MathUtils::getRotationMatrix(R_Pn1, Pn1);
		MathUtils::getRotationMatrix(R_Pn2, Pn2);

		H1 = R_Pn1 * R_P1org.inv();
		H2 = R_Pn2 * R_P2org.inv();
	}

	static void rectifyImages(cv::Mat img1, cv::Mat img2, cv::Mat H1, cv::Mat H2, 
		cv::Mat& img1Warp, cv::Mat& img2Warp, cv::Mat& img1Mask, cv::Mat& img2Mask, 
		cv::Mat& newH1, cv::Mat& newH2) {

		cv::Mat bb1 = MathUtils::findBB(H1, img1.size());
		cv::Mat bb2 = MathUtils::findBB(H2, img2.size());

		bb1.at<double>(1, 0) = (bb1.at<double>(1, 0) < bb2.at<double>(1, 0)) 
			? bb1.at<double>(1, 0) 
			: bb2.at<double>(1, 0)
		;
		bb1.at<double>(3, 0) = (bb1.at<double>(3, 0) > bb2.at<double>(3, 0)) 
			? bb1.at<double>(3, 0) 
			: bb2.at<double>(3, 0)
		;
		bb2.at<double>(1, 0) = bb1.at<double>(1, 0);
		bb2.at<double>(3, 0) = bb1.at<double>(3, 0);

		double w1 = bb1.at<double>(2, 0) - bb1.at<double>(0, 0);
		double w2 = bb2.at<double>(2, 0) - bb2.at<double>(0, 0);

		double w = w1 > w2 ? w1 : w2;

		double c1 = floor((bb1.at<double>(2, 0) + bb1.at<double>(0, 0)) / 2.0);
		double c2 = floor((bb2.at<double>(2, 0) + bb2.at<double>(0, 0)) / 2.0);

		bb1.at<double>(2, 0) = c1 + floor(w / 2.0);
		bb2.at<double>(0, 0) = c2 - floor(w / 2.0);
		bb2.at<double>(2, 0) = c2 + floor(w / 2.0);

		cv::Mat A = cv::Mat::eye(3, 3, CV_64F); A.at<double>(0, 2) = -bb1.at<double>(0, 0); A.at<double>(1, 2) = -bb1.at<double>(1, 0);
		newH1 = A * H1;
		cv::Mat img1MaskInit = cv::Mat(img1.size(), CV_8UC1, cv::Scalar(255, 255, 255));
		cv::warpPerspective(img1, img1Warp, newH1, cv::Size(bb1.at<double>(2, 0) - bb1.at<double>(0, 0), bb1.at<double>(3, 0) - bb1.at<double>(1, 0)));
		cv::warpPerspective(img1MaskInit, img1Mask, newH1, img1Warp.size());

		A.at<double>(0, 2) = -bb2.at<double>(0, 0); A.at<double>(1, 2) = -bb2.at<double>(1, 0);
		newH2 = A * H2;
		cv::Mat img2MaskInit = cv::Mat(img2.size(), CV_8UC1, cv::Scalar(255, 255, 255));
		cv::warpPerspective(img2, img2Warp, newH2, cv::Size(bb2.at<double>(2, 0) - bb2.at<double>(0, 0), bb2.at<double>(3, 0) - bb2.at<double>(1, 0)));
		cv::warpPerspective(img2MaskInit, img2Mask, newH2, img2Warp.size());

		newH1 = newH1.inv();
		newH2 = newH2.inv();
	}

	static cv::Point2f ZNCC(cv::Mat image, cv::Mat temp, int xFeature, int windowSize, int maxDist, int searchY, cv::Mat imgMask, double& score) {
		int u, v;
		cv::Point2f best;
		double sum = 0;
		double sumSq1 = 0;
		double sumSq2 = 0;
		double med1 = 0;
		double med2 = 0;
		double ZNCC = 0;
		double bestMatch = -1;
		int y = searchY;

		int yStart = searchY;
		int yEnd = searchY;
		if ((y - windowSize) < 0 && (y + windowSize) >= image.rows) {
			return cv::Point2f(-1, -1);
		}

		int xStart = xFeature - maxDist;
		int xEnd = xFeature + maxDist;
		if ((xStart - windowSize) < 0) {
			xStart = windowSize;
		}
		if ((xEnd + windowSize) >= image.cols) {
			xEnd = image.cols - 1 - windowSize;
		}

		unsigned int windowSize2 = windowSize * 2 + 1;
		double res1;
		double res2;
		for (unsigned int k = 0; k < windowSize2; k++) {
			for (unsigned int j = 0; j < windowSize2; j++) {
				med2 += temp.at<uchar>(k, j);
			}
		}
		med2 = med2 / (windowSize2 * windowSize2);

		for (y = yStart; y <= yEnd; y++) {
			for (int x = xStart; x <= xEnd; x++) {
				if (imgMask.at<uchar>(y, x) != 0) {
					v = y - windowSize;
					u = x - windowSize;
					for (unsigned int k = 0; k < windowSize2; k++) {
						for (unsigned int j = 0; j < windowSize2; j++) {
							med1 += image.at<uchar>(v + k, u + j);
						}
					}
					med1 = med1 / (windowSize2 * windowSize2);

					for (unsigned int k = 0; k < windowSize2; k++) {
						for (unsigned int j = 0; j < windowSize2; j++) {
							res1 = image.at<uchar>(v + k, u + j) - med1;
							res2 = temp.at<uchar>(k, j) - med2;
							sum += res1 * res2;
							sumSq1 += res1 * res1;
							sumSq2 += res2 * res2;
						}
					}

					ZNCC = sum / sqrt(sumSq1 * sumSq2);
					if (ZNCC > bestMatch) {
						bestMatch = ZNCC;
						best = cv::Point2f(x, y);
					}
					sum = 0;
					sumSq1 = 0;
					sumSq2 = 0;
					med1 = 0;
				}
			}
		}

		if (bestMatch >= 0.5) {
			score = bestMatch;
			return best;
		}

		return cv::Point2f(-1, -1);
	}
};

#endif // MATHUTILS_H
