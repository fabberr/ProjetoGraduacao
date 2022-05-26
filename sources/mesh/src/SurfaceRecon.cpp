/********** Headers **********/

// libc++
#include <iostream> 	// i/o streams
#include <vector> 		// std::vector
#include <system_error> // std::error_code
#include <map> 			// std::map
#include <functional> 	// std::function
#include <filesystem> 	// filesystem utilities

namespace fs = std::filesystem; // filesystem namespace alias

// libc
#include <cmath> // M_PI

// libpcl
#include <pcl/point_cloud.h> 			// pcl::PointCloud<>
#include <pcl/point_types.h> 			// pcl::PointXYZ, pcl::Normal, pcl::PointNormal
#include <pcl/io/obj_io.h> 				// pcl::io::loadOBJFile, pcl::io::saveOBJFile
#include <pcl/io/ply_io.h> 				// pcl::io::loadPLYFile, pcl::io::savePLYFile
#include <pcl/features/normal_3d_omp.h> // pcl::NormalEstimationOMP
#include <pcl/common/io.h> 				// pcl::concatenateFields
#include <pcl/search/kdtree.h> 			// pcl::KdTree
#include <pcl/surface/gp3.h> 			// pcl::GreedyProjection

// internos
#include <logger.h>
#include <SurfaceRecon.h>

/********** Construtores & Destrutor **********/

/**
 * Construtor.
 * Instancia um objeto SurfaceRecon a partir dos parâmetros.
*/
SurfaceRecon::SurfaceRecon(const fs::path& cloud_path, const fs::path& output_dir, const std::string& method) :
	_cloud_path(cloud_path),
	_output_dir(output_dir),
	_method(evaluate_method(method))
{
	if (!load_cloud()) {
		log_error_and_exit("Não foi possível carregar a nuvem de pontos, terminando\n");
	}
}

/**
 * Construtor.
 * Instancia um objeto SurfaceRecon a partir dos argumentos da linha de comando.
*/
SurfaceRecon::SurfaceRecon(const ctrl::args& args) : 
	_cloud_path(args.cloud_path),
	_output_dir(args.output_dir),
	_method(evaluate_method(args.method))
{
	if (!load_cloud()) {
		log_error_and_exit("Não foi possível carregar a nuvem de pontos, terminando\n");
	}
}

/**
 * Destrutor.
 * Libera os recursos instanciados pelo objeto.
*/
SurfaceRecon::~SurfaceRecon() {
	_cloud = nullptr;

	_cloud_normals->clear();
	_cloud_normals = nullptr;

	_mesh->cloud.data.clear();
	_mesh->polygons.clear();
	_mesh = nullptr;
}

/********** Funções Membro Privadas **********/

/** Avalia e retorna o método de reconstrução a ser usado. */
SurfaceRecon::recon_t SurfaceRecon::evaluate_method(const std::string& method) {
	
	static std::map<std::string, recon_t> method_map{
		{ "GREEDY_PROJECTION", recon_t::GREEDY_PROJECTION },
		{ "POISSON"          , recon_t::POISSON           }
	};
	return method_map[method];
}

/**
 * Lê o arquivo especificado em `_cloud_path` e armazena a nuvem de pontos em 
 * `_cloud`.
 * 
 * @returns `true` se o arquivo foi lido com sucesso, `false` caso contrário.
*/
bool SurfaceRecon::load_cloud() {

	/**
	 * Helper function: define qual método de leitura será usado (OBJ ou PLY).
	 * 
	 * @returns `true` se o arquivo foi lido com sucesso, `false` caso contrário.
	*/
	const auto read = (_cloud_path.extension() == ".obj") 
		? std::function<bool()>{[this] {
			return (pcl::io::loadOBJFile(_cloud_path, *_cloud) == 0); // 0 on success.
		}} 
		: std::function<bool()>{[this] {
			return (pcl::io::loadPLYFile(_cloud_path, *_cloud) == 0); // 0 on success.
		}}
	;

	if (!fs::exists(_cloud_path)) {
		log_error("Arquivo especificado `%s` não existe\n", _cloud_path.c_str());
		return false;
	}
	_cloud = cloud_t::Ptr(new cloud_t);
	return read();
}

/**
 * Exporta a malha reconstruída como um arquivo .obj ou .ply.
 * 
 * @param filename Nome do arquivo. Opcional, `mesh.ply` por padrão.
 * 
 * @returns `true` se o arquivo foi escrito com sucesso, `false` caso contrário.
*/
bool SurfaceRecon::export_mesh(const char* filename) {

	/**
	 * Helper function: define qual método de escrita será usado (OBJ ou PLY).
	 *
	 * @param filename Nome do arquivo.
	 * 
	 * @returns `true` se o arquivo foi escrito com sucesso, `false` caso contrário.
	*/
	const auto write = (_cloud_path.extension() == ".obj") 
		? std::function<bool(const fs::path&)>{[this](const fs::path& filename) {
			return (pcl::io::saveOBJFile(filename, *_mesh) == 0); // 0 on success.
		}} 
		: std::function<bool(const fs::path&)>{[this](const fs::path& filename) {
			return (pcl::io::savePLYFile(filename, *_mesh) == 0); // 0 on success.
		}}
	;

	/** 
	 * Helper function: Tenta criar o diretório de saída caso ele não exista.
	 * 
	 * @returns `true` se o diretório foi criado com sucesso, `false` caso contrário.
	*/
	const auto try_mkdir = [](const fs::path& output_dir) {
		std::error_code ec;
		if (!fs::create_directories(output_dir, ec)) {
			// failure state
			log_error(
				"Não foi possível criar diretório de saída `%s`\n"
				"\t%s\n",
				output_dir.c_str(),
				ec.message().c_str()
			);
			return false;
		}
		return true;
	};

	if (not (fs::exists(_output_dir) || try_mkdir(_output_dir))) {
		// failure state
		log_error("Impossível exportar malha\n");
		return false;
	}
	const auto p = _output_dir/filename;
	log_info("Exportando malha para `%s`\n", p.c_str());
	return write(p);
}

/**
 * Estima as normais da superfície para a nuvem de pontos lida do arquivo.
 * As normais estimadas serão concatenadas com o campo XYZ da nuvem numa segunda
 * nuvem `_cloud_normals`. Após a concatenação, `_cloud` ficará em um estado 
 * vazio.
*/
void SurfaceRecon::estimate_normals() {

	tree_t::Ptr kdtree(new tree_t);
	kdtree->setInputCloud(_cloud);
	
	pcl::NormalEstimationOMP<point_t, pcl::Normal> norm_estimator{};
	norm_estimator.setInputCloud(_cloud);
	norm_estimator.setSearchMethod(kdtree);
	norm_estimator.setKSearch(20);

	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	norm_estimator.compute(*normals);

	_cloud_normals = cloud_norm_t::Ptr(new cloud_norm_t);
	pcl::concatenateFields(*_cloud, *normals, *_cloud_normals);
	_cloud->clear();
}

/**
 * Utiliza um algoritmo de projeção gulosa para reconstruir a malha triangular 
 * para a nuvem de pontos.
*/
void SurfaceRecon::reconstruct_greedy_projection() {
	log_info("called `SurfaceRecon::reconstruct_greedy_projection`\n");
}

/**
 * Utiliza o algoritmo de poisson para reconstruir a malha triangular para a 
 * nuvem de pontos.
*/
void SurfaceRecon::reconstruct_poisson() {
	log_info("called `SurfaceRecon::reconstruct_poisson`\n");
}

void SurfaceRecon::reconstruct_fail() {
	log_error_and_exit("Método especificado não existe\n");
}

/********** Funções Membro Públicas **********/

/** Reconstrói a superfície do ojeto e exporta a malha para o diretório de saída. */
void SurfaceRecon::reconstruct() {

	/** Helper function: chama o método de reconstrução especificado. */
	const auto evaluate_and_call = [this](recon_t method) {
		// map methods
		typedef void (SurfaceRecon::* mem_fptr)();
		static std::map<recon_t, mem_fptr> recon_map {
			{ NONE             , &SurfaceRecon::reconstruct_fail              }, 
			{ GREEDY_PROJECTION, &SurfaceRecon::reconstruct_greedy_projection }, 
			{ POISSON          , &SurfaceRecon::reconstruct_poisson           }
		};
		return (this->*recon_map[method])();
	};

	estimate_normals();
	evaluate_and_call(_method);
	export_mesh();
}
