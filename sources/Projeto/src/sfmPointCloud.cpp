/********** Headers **********/

// libc++
#include <algorithm> 		// std::count_if, std::transform
#include <functional> 		// std::hash
#include <string_view> 		// std::string_view
#include <unordered_set> 	// std::unordered_set

// libc
#include <cstddef> 	// std::size_t
#include <cstring> 	// strncpm

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
	bool load_failed = [this](){
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
				_images.emplace_back(entry);
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

/********** Implementação Funções Membro Públicas **********/

/** Computa a nuvem de pontos esparsa e esporta os resultados. */
void sfmPointCloud::compute_sparse() {

	// chamar cv::sfm::reconstruct

	// construir Rt p/ cada câmera

	// extrair nuvem de pontos

	// chamar sfmPointCloud::export_cloud_OBJ
	// chamar sfm::PointCloud::export_pose_SFM
}

/********** Implementação Funções Membro Privadas **********/

/** Exporta a nuvem de pontos como um arquivo .obj (apens lista de vérticas). */
void sfmPointCloud::export_cloud_OBJ(const fs::path& output_dir) const {
}

/** Exporta os dados extrínsecos (matriz [R|t]) estimados de cada câmera como um arquivo .sfm. */
void sfmPointCloud::export_pose_SFM(const fs::path& output_dir) const {
}
