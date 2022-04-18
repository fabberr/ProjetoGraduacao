/********** Headers **********/

// libc++
#include <iostream> 		// i/o streams
#include <fstream> 			// std::ofstream
#include <algorithm> 		// std::count_if, std::transform
#include <functional> 		// std::hash
#include <string_view> 		// std::string_view
#include <unordered_set> 	// std::unordered_set
#include <system_error> 	// std::error_code
#include <utility> 			// std::make_pair
#include <functional> 		// function objects support
#include <filesystem> 		// filesystem utilities

namespace fs = std::filesystem; // filesystem namespace alias

// libc
#include <cstddef> 	// std::size_t
#include <cstring> 	// strncpm
#include <cstdint> 	// uint_t

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
	_point_cloud()
{
	// executa uma função anônima que popula o vetor `_images` e retorna true se falhou
	bool load_failed = [this]{
		// reservando espaço para o vetor `_images`, contando apenas arquivos regulares
		bool (*pred)(const fs::path&) = fs::is_regular_file;
		const size_t count = std::count_if(fs::directory_iterator(_args.input_path), fs::directory_iterator{}, pred); // begin, end, predicate
		_images.reserve(count);
		_Rts.reserve(count);

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
		log_error_and_exit("sfmPointCloud::sfmPointCloud: Nao foi possivel carregar imagens\n");
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

	// extrair nuvem de pontos
	std::cout << "Recuperando nuvem de pontos (esparsa)...";
	_point_cloud.reserve(points3d_est.size());
	for (const auto& pt : points3d_est) {
		_point_cloud.emplace_back(pt);
	}
	points3d_est.clear();
	std::cout << "ok\n";

	// construir Rt p/ cada câmera
	std::cout << "Recuperando valores extrinsecos [R|t] (pose)...";
	for (size_t i = 0; i < Rs_est.size(); ++i) {
		_Rts.emplace_back(Rs_est[i], ts_est[i]);
	}
	Rs_est.clear();
	ts_est.clear();
	std::cout << "ok\n";

	// exporta nuvem de pontos e pose das câmeras
	export_results();
}

/********** Implementação Funções Membro Privadas **********/

/** Exporta a nuvem de pontos como um arquivo .obj (apens lista de vértices) e as poses estimadas de cada câmera como um arquivo .sfm. */
void sfmPointCloud::export_results() const {

	/**
	 * Helper function: determina o caminho final de saída e verifica se ele existe,
	 * tenta criá-lo caso contrário.
	 * 
	 * @returns Um std::pair contendo `true` e o caminho até o diretório de saída 
	 * validado ou `false` e um caminho vazio caso falhe em criar o diretório.
	*/
	const auto validate_output_dir = [this]{
		// determinando nome do dataset
		fs::path dataset_name = (_args.input_path.has_filename() ? // verifica se `input_path` termina em `/` (empty filename)
			_args.input_path.filename() : 								// use nome do diretório .
			_args.input_path.parent_path().filename() 					// use nome do diretório ..
		);
		fs::path out_dir = _args.output_path/dataset_name; // caminho final
		
		// verifica se o diretório de saída existe, tenta criá-lo caso contrário
		if (!fs::exists(out_dir)) {
			std::error_code ec;
			if (!fs::create_directories(out_dir, ec)) {
				// failure state
				log_error(
					"sfmPointCloud::export_results: Nao foi possivel criar diretorio de saida `%s`\n"
					"\t%s\n", 
					out_dir.c_str(), 
					ec.message().c_str()
				);
				return std::make_pair(false, fs::path{});
			}
		}
		return std::make_pair(true, out_dir);
	};

	/** Helper function: exporta a nuvem de pontos para uma stream de arquivo aberta. */
	const auto write_cloud = [&cloud = _point_cloud](std::ofstream& file){
		// lista de vértices, formato: "v <x> <y> <z>\n"
		for (const auto& pt : cloud) {
			file << "v " << pt[0] << ' ' << pt[1] << ' ' << pt[2] << '\n';
		}
	};

	/** Helper function: exporta a pose das câmeras para uma stream de arquivo aberta. */
	const auto write_pose = [&paths = _images, &Rts = _Rts](std::ofstream& file){
		// número de câmeras
		file << paths.size() << "\n\n";

		// sfm string p/ cada câmera, formato: "<path> <rotation> <translation>\n"
		for (std::size_t i = 0; i < paths.size(); ++i) {
			// caminho absoluto até a imagem
			file << paths[i] << ' ';

			// componente R (matriz de rotação)
			const auto& R = Rts[i].rotation();
			for (std::uint8_t j = 0; j < 3; ++j) {
				for (std::uint8_t k = 0; k < 3; ++k) {
					// [0][0] [0][1] [0][2] [1][0] [1][1] [1][2] [2][0] [2][1] [2][2] (primerias 3 linhas e colunas)
					file << R(j, k) << ' ';
				}
			}

			// componente t (vetor de translação)
			const auto& t = Rts[i].translation();
			for (std::uint8_t j = 0; j < 3; ++j) {
				// [0][3] [1][3] [2][3] (última coluna)
				file << t[j] << ' ';
			}

			file << '\n';
		}
	};

	/** Helper function: abre o arquivo de saída `filename` e escreve seus conteúdos de acordo com a função `write_fn` */
	const auto export_to = [](const fs::path& filename, const std::function<void(std::ofstream&)>& write_fn){
		if (std::ofstream file{filename}; file.is_open()) { // cria/recria arquivo ("w" mode)
			write_fn(file);
			file.close();
		} else {
			log_error("Nao foi possivel abrir arquivo de saida `%s`\n", filename.c_str());
		}
	};

	if (const auto& [path_exists, out_dir] = validate_output_dir(); path_exists) {
		export_to(out_dir/"point_cloud.obj", write_cloud);
		export_to(out_dir/"pose.sfm", write_pose);
	} else {
		log_info("Impossivel exportar resultados\n");
	}
}
