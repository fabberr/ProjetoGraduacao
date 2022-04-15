/********** Headers **********/

// libc++
#include <iostream> 		// i/o streams
#include <fstream> 			// std::ofstream
#include <algorithm> 		// std::count_if, std::transform
#include <functional> 		// std::hash
#include <string_view> 		// std::string_view
#include <unordered_set> 	// std::unordered_set
#include <system_error> 	// std::error_code
#include <filesystem> 		// filesystem utilities

namespace fs = std::filesystem; // filesystem namespace alias

// libc
#include <cstddef> 	// std::size_t
#include <cstring> 	// strncpm

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/sfm.hpp>

// internos
#include <sfmPointCloud.h>
#include <logger.h>

/********** Implementação Construtores & Destrutor **********/

/**
 * Construtor.
 * Instancia um objeto sfmPointCloud a partir dos argumentos da linha de comando.
*/
sfmPointCloud::sfmPointCloud(const ctrl::args& args) : 
	// inicializando variáveis
	_args(args), 
	_images(),
	_K(), 
	_Rts(),
	_points3d(),
	_point_cloud()
{
	// executa uma função anônima que popula o vetor `_images` e retorna true se falhou
	bool load_failed = [this]{
		// reservando espaço para o vetor `_images`, contando apenas arquivos regulares
		typedef bool (*is_reg_file_fptr)(const fs::path&); // function pointer type alias
		is_reg_file_fptr pred = fs::is_regular_file;
		const size_t count = std::count_if(fs::directory_iterator(_args.input_path), fs::directory_iterator{}, pred); // begin, end, predicate
		_images.reserve(count);

		// functor de hashing p/ usar containers associativos (set, map, ...) não ordenados com strings estilo C
		struct cstr_hash {
			inline std::size_t operator()(const char* str) const noexcept { return std::hash<std::string_view>{}(str); }
		};

		// functor de comparação p/ usar containers associativos (set, map, ...) não ordenados com strings estilo C
		struct cstr_equal_to {
			inline bool operator()(const char* a, const char* b) const noexcept { return std::strcmp(a, b) == 0; }
		};

		// inserindo arquivos de imagem no vetor
		typedef std::unordered_set<const char*, cstr_hash, cstr_equal_to> whitelist_t;
		const whitelist_t ext_whitelist{ ".png", ".jpg", ".tiff", ".webp" };
		for (const auto& entry : fs::directory_iterator(_args.input_path)) {
			auto ext = entry.path().extension().native();
			std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
			auto ext_lower = ext.c_str();
			if (ext_whitelist.find(ext_lower) != ext_whitelist.end()) {
				_images.emplace_back(entry.path().string());
			}
		}

		return _images.size(); // 0 == false
	}();
	if (load_failed) {
		log_error_and_exit("Nao foi possivel carregar imagens");
	}

	// construir K
	_K = cv::Mat_<double>(3, 3) << (
		(_args.f,    0   , _args.cx), 
		(   0   , _args.f, _args.cy), 
		(   0   ,    0   ,    1    )
	);
}

/** Destrutor (noop) */
sfmPointCloud::~sfmPointCloud() { }

/********** Implementação Funções Membro Públicas **********/

/** Computa a nuvem de pontos esparsa e esporta os resultados. */
void sfmPointCloud::compute_sparse() {

	// reconstruir a cena
	std::vector<cv::Mat> Rs_est, ts_est; 	// matrizes de rotação e vetores de translação estimados p/ cada câmera
	std::vector<cv::Mat> points3d_est; 		// pontos 3d estimados
	cv::sfm::reconstruct(_images, Rs_est, ts_est, _K, points3d_est, true);

	std::cout << 
		"Pontos 3D estimados: "  << points3d_est.size() << "\n" 
		"Cameras estimadas: " << Rs_est.size() << "\n" 
		"Valores intrinsecos refinados:\n" << _K << "\n" 
	<< std::endl;

	// construir Rt p/ cada câmera
	std::cout << "Recuperando valores extrinsecos [R|t] (pose)...";
	for (size_t i = 0; i < Rs_est.size(); ++i) {
		_Rts.emplace_back(Rs_est[i], ts_est[i]);
	}
	Rs_est.clear();
	ts_est.clear();
	std::cout << "ok\n";

	// extrair nuvem de pontos
	std::cout << "Recuperando nuvem de pontos (esparsa)...";
	for (const auto& pt : points3d_est) {
		_point_cloud.emplace_back(pt);
	}
	points3d_est.clear();
	std::cout << "ok\n";

	// determinando nome do dataset p/ caminho final de saída
	fs::path dataset_name = (_args.input_path.has_filename() ? // verifica se `input_path` termina em `/` (empty filename)
		_args.input_path.filename() : 								// use nome do diretório .
		_args.input_path.parent_path().filename() 					// use nome do diretório ..
	);
	fs::path out_dir = _args.output_path/dataset_name;

	// exportar resultados
	export_cloud_OBJ("point_cloud.obj", out_dir);
	export_pose_SFM("pose.sfm", out_dir);
}

/********** Implementação Funções Membro Privadas **********/

/** Exporta a nuvem de pontos como um arquivo .obj (apens lista de vértices). */
void sfmPointCloud::export_cloud_OBJ(const std::string& filename, const fs::path& output_dir) const {

	// verifica se o diretório de saída existe e tenta criá-lo caso contrário
	if (!fs::exists(output_dir)) {
		std::error_code ec;
		if (!fs::create_directories(output_dir, ec)) {
			log_error_and_exit(
				"sfmPointCloud::export_cloud_OBJ - Nao foi possivel criar diretorio de saida `%s`:\n"
				"%s\n", 
				output_dir.c_str(), 
				ec.message().c_str()
			);
		}
	}

	fs::path file_path = output_dir/filename;
	if (std::ofstream file{file_path}; file.is_open()) { // cria/recria arquivo ("w" mode)
		for (const auto& pt : _point_cloud) {
			file << "v " << pt[0] << ' ' << pt[1] << ' ' << pt[2] << '\n'; // formato: "v <x> <y> <z>\n"
		}
		file.close();
	} else {
		log_error("Nao foi possivel abrir arquivo de saida `%s`\n", file_path.c_str());
	}
}

/** Exporta as matrizes extrínsecas [R|t] estimados de cada câmera como um arquivo .sfm. */
void sfmPointCloud::export_pose_SFM(const std::string& filename, const fs::path& output_dir) const {
	// ...
}
