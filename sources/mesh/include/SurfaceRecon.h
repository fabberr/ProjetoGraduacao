#ifndef SURFACERECON_H
#define SURFACERECON_H

/********** Headers **********/

// libc++
#include <filesystem> // filesystem utilities

namespace fs = std::filesystem; // filesystem namespace alias

// libpcl
#include <pcl/point_cloud.h> 	// pcl::PointCloud<>
#include <pcl/point_types.h> 	// pcl::PointXYZ, pcl::Normal, pcl::PointNormal
#include <pcl/PolygonMesh.h> 	// pcl::PolygonMesh
#include <pcl/search/kdtree.h> 	// pcl::KdTree

/********** SurfaceRecon.h **********/

/** Wrapper para algoritmos de reconstrução de superfície do PCL. */
class SurfaceRecon {
public:
	/********** Enums Públicos **********/
	typedef enum { NONE, GREEDY_PROJECTION, POISSON } recon_t;

private:
	/********** Tipos Membro Privados **********/

	using point_t 	= pcl::PointXYZ;
	using mesh_t 	= pcl::PolygonMesh;

	typedef pcl::PointCloud<point_t> 				cloud_t;
	typedef pcl::PointCloud<pcl::PointNormal> 		cloud_norm_t;
	typedef pcl::search::KdTree<point_t> 			tree_t;
	typedef pcl::search::KdTree<pcl::PointNormal> 	tree_norm_t;

private:
	/********** Membros Privados **********/
	
	fs::path _cloud_path; /** Caminho até o arquivo .obj contendo a nuvem de pontos. */

	cloud_t::Ptr 		_cloud 			= nullptr; /** Nuvem de pontos carregada do arquivo. */
	cloud_norm_t::Ptr 	_cloud_normals 	= nullptr; /** Nuvem de pontos com as normais da superfície estimadas. */
	mesh_t::Ptr 		_mesh 			= nullptr; /** Malha poligonal reconstruída. */

public:
	/********** Construtores & Destrutor **********/
	
	SurfaceRecon() 						= delete; /** Default Constructor (deleted). */
	SurfaceRecon(const SurfaceRecon&) 	= delete; /** Copy Constructor (deleted). */
	SurfaceRecon(SurfaceRecon&&) 		= delete; /** Move Constructor (deleted). */

	SurfaceRecon(const fs::path& cloud_path);
	virtual ~SurfaceRecon();

private:
	/********** Funções Membro Privadas **********/
	
	bool load_cloud_OBJ();
	bool export_mesh_OBJ(const fs::path& output_dir, const char* filename = "mesh.obj");
	
	void estimate_normals();
	void reconstruct_greedy_projection();
	void reconstruct_poisson();
	void reconstruct_fail();

public:
	/********** Funções Membro Públicas **********/
	
	void reconstruct(recon_t method = recon_t::GREEDY_PROJECTION);
}; // class SurfaceRecon

#endif // SURFACERECON_H
