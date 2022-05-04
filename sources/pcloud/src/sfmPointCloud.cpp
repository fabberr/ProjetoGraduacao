/********** Headers **********/

// libc++
#include <iostream> 		// i/o streams
#include <string> 			// std::string
#include <fstream> 			// std::ofstream, std::ifstream
#include <algorithm> 		// std::count_if, std::transform, std::sort
#include <functional> 		// std::hash
#include <string_view> 		// std::string_view
#include <unordered_set> 	// std::unordered_set
#include <system_error> 	// std::error_code
#include <utility> 			// std::make_pair
#include <filesystem> 		// filesystem utilities

namespace fs = std::filesystem; // filesystem namespace alias

// libc
#include <cstddef> 	// std::size_t
#include <cstring> 	// strncpm
#include <cstdint> 	// fixed width integer types and C numeric limits

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/sfm.hpp>

// internos
#include <sfmPointCloud.h>
#include <logger.h>
#include <io.h>

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
	_point_cloud(),
	_output_dir()
{
	// executa uma função anônima que popula o vetor `_images` e retorna false se falhou
	bool load_success = [this] {
		// reservando espaço para o vetor `_images`, contando apenas arquivos regulares
		bool (*pred)(const fs::path&) = fs::is_regular_file;
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

		// extensões de arquivo permitidas
		typedef std::unordered_set<const char*, cstr_hash, cstr_equal_to> whitelist_t;
		const whitelist_t ext_whitelist{ ".png", ".jpg", ".tiff", ".webp" };

		// inserindo caminhos até os arquivos de imagem no vetor
		for (const auto& entry : fs::directory_iterator(_args.input_path)) {
			// filtrandoa penas extensões permitidas
			auto ext = entry.path().extension().native();
			std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
			auto ext_lower = ext.c_str();
			
			if (ext_whitelist.find(ext_lower) != ext_whitelist.end()) {
				// garante que o caminho seja absoluto
				const auto path = (entry.path().is_absolute()) ? entry.path() : fs::absolute(entry.path());
				_images.emplace_back(path.c_str());
			}
		}
		_images.shrink_to_fit();
		_Rts.reserve(_images.size());

		// ordenar por ordem alfabética
		std::sort(_images.begin(), _images.end());

		return _images.size(); // 0 == false
	}();
	if (!load_success) {
		log_error_and_exit("sfmPointCloud::sfmPointCloud: Não foi possível carregar imagens\n");
	}

	// construir K
	_K = (cv::Mat_<double>(3, 3) << 
		_args.f,    0   , _args.cx, 
		   0   , _args.f, _args.cy, 
		   0   ,    0   ,    1
	);

#if defined(_DEBUG) || defined(DEBUG)
	// debug build
	std::cout << 
		"Objeto sfmPointCloud instanciado:\n" 
		"    Imagens carregadas: " << _images.size()
	<< std::endl;
	for (const auto& path : _images) {
		std::cout << "        >`" << path << "`\n";
	}
	std::cout << 
		"    _K (inicial):\n" 
		<< _K << '\n' 
	<< std::endl;
#endif
}

/** Destrutor (noop) */
sfmPointCloud::~sfmPointCloud() { return; }

/********** Implementação Funções Membro Públicas **********/

/** Computa a nuvem de pontos esparsa e exporta os resultados. */
void sfmPointCloud::compute_sparse() {

	/**
	 * Helper function: computa uma nuvem de pontos parcial, usando apenas uma parte
	 * do dataset (recomendado: janela de 3-4 imagens).
	 *
	 * @param lower Limite inferior da janela.
	 * @param upper Limite superior da janela.
	 * @param size Tamanho da janela.
	*/
	const auto compute_partial = [this](std::size_t lower, std::size_t upper, std::uint8_t size) {
		
		// criando dataset parcial
		std::vector<cv::String> partial{};
		partial.reserve(size);
		if (upper > lower) {
			partial.assign(&_images[lower], &_images[upper]); // range: [lower, upper)
		} else {
			partial.assign(&_images[lower], &_images[_images.size()]); 		// lower half range: [lower, END)
			partial.insert(partial.end(), &_images[0], &_images[upper]); 	// upper half range: [BEGIN, upper)
		}

		std::cout << "Reconstruindo a cena para o dataset parcial:\n";
		for (const auto& path : partial) {
			std::cout << "    >`" << path << "`\n";
		}
		std::cout << std::endl;

		// reconstruir a cena
		std::vector<cv::Mat> Rs_est, ts_est; 	// matrizes de rotação e vetores de translação estimados p/ cada câmera
		std::vector<cv::Mat> points3d_est; 		// pontos 3d estimados
		cv::Mat K_refined = _K.clone(); 		// matriz intrínseca refinada
		cv::sfm::reconstruct(partial, Rs_est, ts_est, K_refined, points3d_est, true);

		std::cout << "\n" 
			"Cena reconstruída:\n" 
			"    Pontos estimados: "  << points3d_est.size() << "\n" 
			"    Poses estimadas: " << Rs_est.size() << "\n" 
			"    Valores intrínsecos refinados:\n" << K_refined << "\n" 
		<< std::endl;

		// extrair nuvem de pontos
		std::cout << "Recuperando nuvem de pontos...";
		_point_cloud.clear();
		_point_cloud.reserve(points3d_est.size());
		for (const auto& pt : points3d_est) {
			_point_cloud.emplace_back(pt);
		}
		points3d_est.clear();
		std::cout << "OK\n";

		// construir Rt p/ cada câmera
		std::cout << "Recuperando valores extrínsecos [R|t] (pose)...";
		for (size_t i = 0; i < Rs_est.size(); ++i) {
			_Rts.emplace_back(Rs_est[i], ts_est[i]);
		}
		Rs_est.clear();
		ts_est.clear();
		std::cout << "OK\n";

		// exporta nuvem de pontos parcial
		std::cout << "Exportando nuvem de pontos parcial\n";
		export_results(true, std::string{"partial_cloud_0" + std::to_string(lower + 1) + ".obj"}.c_str());
	};

	// iterando sobre o vetor de caminhos `_images` usando um algoritmo janela deslizante circular com interseção
	const std::uint8_t SLIDING_WINDOW_SIZE 			= 3; 	// tamanho do segmento
	const std::uint8_t SLIDING_WINDOW_INTERSECTION 	= 2; 	// quantidade de elementos na interseção entre o segmento atual e o anterior
	const std::uint8_t SLIDING_WINDOW_STEP 			= 
		SLIDING_WINDOW_SIZE - SLIDING_WINDOW_INTERSECTION; 	// quantidade de índices a incrementar a cada iteração
	std::size_t lower = 0, upper = SLIDING_WINDOW_SIZE; 	// limites inferior e superior da janela
	while (lower < _images.size()) {

		compute_partial(lower, upper, SLIDING_WINDOW_SIZE);

		lower += SLIDING_WINDOW_STEP;
		upper = (upper + SLIDING_WINDOW_STEP) % _images.size();
	}

	// concatenar as nuvens de pontos
	if (fs::exists(_output_dir)) {
		_point_cloud.clear();
		
		// lista de vértices, formato: "v <x> <y> <z>\n"
		for (const auto& entry : fs::directory_iterator(_output_dir)) {
			if (std::strncmp(entry.path().extension().c_str(), ".obj", 4) == 0) {
				using namespace std::placeholders;
				io::import_from_file(entry.path(), std::bind(&sfmPointCloud::read_cloud, this, _1));
			}
		}
	} else {
		log_error_and_exit("Diretório de saída `%s` não existe\n", _output_dir.c_str());
	}
}

/********** Implementação Funções Membro Privadas **********/

/** Importa a nuvem de pontos parcial para uma stream de arquivo aberta. */
bool sfmPointCloud::read_cloud(std::ifstream& file) {
	// lista de vértices, formato: "v <x> <y> <z>\n"
	while (file) {
		char v[1];
		cv::Point3f pt{};
		file >> v >> pt.x >> pt.y >> pt.z;

		_point_cloud.emplace_back(pt.x, pt.y, pt.z);
	}
	return true;
}

/** Exporta a nuvem de pontos para uma stream de arquivo aberta. */
bool sfmPointCloud::write_cloud(std::ofstream& file) {
	// lista de vértices, formato: "v <x> <y> <z>\n"
	for (const auto& pt : _point_cloud) {
		file << "v " << pt[0] << ' ' << pt[1] << ' ' << pt[2] << '\n';
	}
	return true;
}

/** Helper function: exporta a pose das câmeras para uma stream de arquivo aberta. */
bool sfmPointCloud::write_pose(std::ofstream& file) {
	// número de câmeras
	file << _images.size() << "\n\n";

	// sfm string p/ cada câmera, formato: "<path> <cam_rotation> <cam_translation>\n"
	for (std::size_t i = 0; i < _images.size(); ++i) {
		// caminho absoluto até a imagem
		file << _images[i] << ' ';

		// componente R (matriz de rotação)
		const auto& R = _Rts[i].rotation();
		for (std::uint8_t j = 0; j < 3; ++j) {
			for (std::uint8_t k = 0; k < 3; ++k) {
				// [0][0] [0][1] [0][2] [1][0] [1][1] [1][2] [2][0] [2][1] [2][2] (primerias 3 linhas e colunas)
				file << R(j, k) << ' ';
			}
		}

		// componente t (vetor de translação)
		const auto& t = _Rts[i].translation();
		for (std::uint8_t j = 0; j < 3; ++j) {
			// [0][3] [1][3] [2][3] (última coluna)
			file << t[j] << ' ';
		}

		file << '\n';
	}
	return true;
}


/**
 * Exporta a nuvem de pontos como um arquivo .obj (apens lista de vértices) e as
 * poses estimadas de cada câmera como um arquivo .sfm.
 *
 * @param cloud_only Determina se apenas a nuvem de pontos deve ser exportada. 
 *        Opcional, `false` por padrão.
 * @param obj_filename Nome do arquivo .obj. Opcional, `point_cloud.obj` por 
 *        padrão.
 * @param sfm_filename Nome do arquivo .sfm. Opcional, `pose.sfm` por padrão.
*/
void sfmPointCloud::export_results(bool cloud_only, const char* obj_filename, const char* sfm_filename) {

	/**
	 * Helper function: determina o caminho de saída final e tenta criá-lo caso ele 
	 * não exista.
	 * 
	 * @returns Um std::pair contendo `true` e o caminho até o diretório de saída 
	 * validado ou `false` e um caminho vazio caso falhe em criar o diretório.
	*/
	const auto validate_output_dir = [this] {
		// determinando nome do dataset
		fs::path dataset_name = (_args.input_path.has_filename() ? // verifica se `input_path` tem componente filename (não termina em `/` ou `\`)
			_args.input_path.filename() : 								// se sim, use nome do diretório atual
			_args.input_path.parent_path().filename() 					// senão, use nome do diretório pai
		);
		fs::path out_dir = _args.output_path/dataset_name; // caminho final
		
		// tenta criar o diretório de saída caso ele não exista
		if (!fs::exists(out_dir)) {
			std::error_code ec;
			if (!fs::create_directories(out_dir, ec)) {
				// failure state
				log_error(
					"sfmPointCloud::export_results: Não foi possível criar diretório de saída `%s`\n"
					"\t%s\n", 
					out_dir.c_str(), 
					ec.message().c_str()
				);
				return std::make_pair(false, fs::path{});
			}
		}
		return std::make_pair(true, out_dir);
	};

	// tenta exportar os resultados
	if (const auto& [path_exists, out_dir] = validate_output_dir(); path_exists) {
		_output_dir = out_dir;
		
		using namespace std::placeholders;
		io::export_to_file(out_dir/obj_filename, std::bind(&sfmPointCloud::write_cloud, this, _1));
		if (!cloud_only) {
			io::export_to_file(out_dir/sfm_filename, std::bind(&sfmPointCloud::write_pose, this, _1));
		}
		std::cout << std::endl;
	} else {
		log_error("Impossível exportar resultados\n");
	}
}

/** @todo permitir que os parâmetros do algoritmo de janela deslizante sejam alterados através da linha de comando */
