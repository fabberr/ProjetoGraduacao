// #include <SurfaceRecon.h>
#include <SurfaceReconstruction.h>

int main() {
	// SurfaceRecon{"output/packt/gargoyle/SIFT_point_cloud_filtered.obj"}.computeMeshGreedyProjection();
	
	SurfaceReconstruction{fs::path{"output/packt/gargoyle"}}.computeMesh();

	return 0;
}
