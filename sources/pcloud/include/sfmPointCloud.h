#ifndef SFM_POINT_CLOUD_H
#define SFM_POINT_CLOUD_H

/********** Headers **********/

// libc++
#include <fstream>
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
	fs::path 					_output_dir; 	/** Caminho até o diretório de saída final (com nome do dataset). */

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

	bool read_cloud(std::ifstream& file);
	bool write_cloud(std::ofstream& file);
	bool write_pose(std::ofstream& file);
	void export_results(bool cloud_only = false, const char* obj_filename = "point_cloud.obj", const char* sfm_filename = "pose.sfm");
};

#endif // SFM_POINT_CLOUD_H
