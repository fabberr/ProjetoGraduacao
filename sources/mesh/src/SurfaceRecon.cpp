/********** Headers **********/

// libc++
#include <iostream> 	// i/o streams
#include <vector> 		// std::vector
#include <filesystem> 	// filesystem utilities

namespace fs = std::filesystem; // filesystem namespace alias

// libc
#include <cmath> // M_PI

// libpcl
#include <pcl/point_cloud.h> 		// pcl::PointCloud<>
#include <pcl/point_types.h> 		// pcl::PointXYZ, pcl::Normal, pcl::PointNormal
#include <pcl/io/obj_io.h> 			// pcl::io::loadOBJFile, pcl::io::saveOBJFile
#include <pcl/features/normal_3d.h> // pcl::NormalEstimation
#include <pcl/common/io.h> 			// pcl::concatenateFields
#include <pcl/search/kdtree.h> 		// pcl::KdTree
#include <pcl/surface/gp3.h> 		// pcl::GreedyProjection

// internos
#include <logger.h>
#include <SurfaceRecon.h>

/********** Construtores & Destrutor **********/

/** Construtor. */
SurfaceRecon::SurfaceRecon(const fs::path& cloud_path) 
	: _cloud_path(cloud_path)
{
	if (!load_cloud_obj()) {
		log_error_and_exit("Não foi possível carregar a nuvem de pontos.\n");
	}
}

/** Destrutor. */
SurfaceRecon::~SurfaceRecon() {
	_cloud = nullptr;
	_cloud_normals = nullptr;
	_mesh = nullptr;
}

/********** Funções Membro Privadas **********/

/** Lê o arquivo .obj contendo a nuvem de pontos. */
bool SurfaceRecon::load_cloud_obj() {

	if (!fs::exists(_cloud_path) || _cloud_path.extension() != ".obj") {
		log_error("Arquivo especificado `%s` não existe ou é inválido.\n", _cloud_path.c_str());
		return false;
	}

	_cloud = cloud_t::Ptr(new cloud_t);
	return (pcl::io::loadOBJFile(_cloud_path, *_cloud) == 0); // 0 on success.
}

/** Salva a malha reconstruida em um arquivo .obj. */
void SurfaceRecon::export_mesh_obj(const fs::path& filename) {
	pcl::io::saveOBJFile(filename, *_mesh);
}

/** Estima as normais da */
void SurfaceRecon::estimate_normals() {
	// type aliases
	using normals_t = pcl::PointCloud<pcl::Normal>;
	using kdtree_t = pcl::search::KdTree<pcl::PointXYZ>;

	// normal stimation
	normals_t::Ptr normals(new normals_t);
	pcl::NormalEstimation<point_t, pcl::Normal> norm_estimator{};
	kdtree_t::Ptr tree(new kdtree_t);

	std::cerr << "Estimando normais dos vértices..." << std::endl;
	tree->setInputCloud(_cloud);
	norm_estimator.setInputCloud(_cloud);
	norm_estimator.setSearchMethod(tree);
	norm_estimator.setKSearch(20);
	norm_estimator.compute(*normals);
	std::cerr << "    OK\n" << std::endl;

	// concatenate XYZ and normal fields
	std::cerr << "Concatenando campos XYZ com as normais estimadas..." << std::endl;
	_cloud_normals = cloudNormals_t::Ptr(new cloudNormals_t);
	pcl::concatenateFields(*_cloud, *normals, *_cloud_normals);
	std::cerr << "    OK" << std::endl;
	
	// deallocate memory
	_cloud->clear();
	normals->clear();
	_cloud = nullptr;
	normals = nullptr;
}

/********** Funções Membro Públicas **********/

/** Utiliza um algoritmo de Projeção Gulosa para obter a malha da superfície. */
void SurfaceRecon::computeMeshGreedyProjection() {
	// type aliases
	using gp3_t = pcl::GreedyProjectionTriangulation<pcl::PointNormal>;

	estimate_normals();

	// initialize objects
	kdtree_t::Ptr tree(new kdtree_t);
	tree->setInputCloud(_cloud_normals);
	gp3_t gp3;

	gp3.setSearchRadius(0.025);
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45°
	gp3.setMinimumAngle(M_PI/18); // 10°
	gp3.setMinimumAngle(2*M_PI/3); // 120°
	gp3.setNormalConsistency(false);

	// run
	std::cerr << "Pontos: " << _cloud_normals->points.size() << std::endl;
	std::cerr << "Reconstruindo superfície..." << std::endl;
	gp3.setInputCloud(_cloud_normals);
	gp3.setSearchMethod(tree);
	gp3.reconstruct(*_mesh);
	std::cerr << "    OK\n" << std::endl;

	// aditional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	std::cerr << "Exportando resultados\n" << std::endl;
	export_mesh_obj();
}

/** Utiliza o algoritmo de Poisson para obter a malha da superfície. */
void SurfaceRecon::computeMeshPoisson() { }
