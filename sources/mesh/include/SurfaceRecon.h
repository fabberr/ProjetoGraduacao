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
	/********** Tipos Membro Públicos **********/

	using point_t = pcl::PointXYZ;
	using mesh_t = pcl::PolygonMesh;

	typedef pcl::PointCloud<point_t> 				cloud_t;
	typedef pcl::PointCloud<pcl::PointNormal> 		cloudNormals_t;
	typedef pcl::search::KdTree<pcl::PointNormal> 	kdtree_t;
	
private:
	/********** Membros Privados **********/

	fs::path _cloud_path; /** Caminho ate o arquivo .obj contendo a nuvem de pontos. */

	cloud_t::Ptr 		_cloud			= nullptr; /** Nuvem de pontos carregada do arquivo. */
	cloudNormals_t::Ptr _cloud_normals 	= nullptr; /** Nuvem de pontos com as normais dos vértices. */
	mesh_t::Ptr 		_mesh 			= nullptr; /** Superfície gerada. */

public:
	/********** Construtores & Destrutor (declaração) **********/

	SurfaceRecon() 					= delete; /** Default constructor(deleted) */
	SurfaceRecon(SurfaceRecon&) 	= delete; /** Copy constructor (deleted) */
	SurfaceRecon(SurfaceRecon&&) 	= delete; /** Move constructor (deleted) */

	SurfaceRecon(const fs::path& path);
	virtual ~SurfaceRecon();

private:
	/********** Funções Membro Privadas **********/
	bool load_cloud_obj();
	void export_mesh_obj(const fs::path& filename = "output/packt/gargoyle/SIFT_mesh.obj");
	void estimate_normals();

public:
	/********** Funções Membro Públicas **********/

	void computeMeshGreedyProjection();
	void computeMeshPoisson();
}; // class SurfaceRecon

#endif // SURFACERECON_H
