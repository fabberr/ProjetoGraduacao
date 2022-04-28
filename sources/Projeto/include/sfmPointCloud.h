#ifndef SFM_POINT_CLOUD_H
#define SFM_POINT_CLOUD_H

/********** Headers **********/

// libc++
#include <vector>
#include <string>
#include <filesystem>

namespace fs = std::filesystem;

// OpenCV
#include <opencv2/core.hpp> 	// core functionality and types
#include <opencv2/calib3d.hpp> 	// cv::Affine

// internos
#include <cmdline_parser.h>

/********** sfmPointCloud.h **********/

/**
 * Wrapper class para os algoritmos de reconstrução do módulo cv::sfm.
*/
class sfmPointCloud {
private:
	/********** Membros Privados **********/
	const ctrl::args& 			_args; 			/** Argumentos validados extraídos da linha de comando. */
	std::vector<cv::String> 	_images; 		/** Caminhos absolutos até as imagens do dataset. */
	cv::Mat 					_K; 			/** Matriz intrínseca K estimada. Inicialmente derivada dos argumentos <f>, <cx> e <cy>. */
	std::vector<cv::Affine3d> 	_Rts; 			/** Matrizes extrínsecas Rt estimadas para cada câmera. */
	std::vector<cv::Vec3f> 		_point_cloud; 	/** Nuvem de pontos esparsa. */

public:
	/********** Construtores & Destrutor **********/

	sfmPointCloud() = delete;
	sfmPointCloud(const sfmPointCloud&) = delete;
	sfmPointCloud(sfmPointCloud&&) = delete;
	sfmPointCloud(const ctrl::args&);

	virtual ~sfmPointCloud();

public:
	/********** Funções Membro Públicas **********/
	
	void compute_sparse();

private:
	/********** Funções Membro Privadas **********/

	void export_results() const;
};

#endif // SFM_POINT_CLOUD_H
