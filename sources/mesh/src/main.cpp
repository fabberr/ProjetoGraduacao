/********** Headers **********/

// internos
#include <parser.h>
#include <SurfaceRecon.h>

/********** main.cpp **********/

/** ./mesh output/packt/crazyhorse/AKAZE_dense_cloud.ply */
int main(int argc, char** argv) {
	SurfaceRecon{ctrl::parse({argv, argv + argc})}.reconstruct();
	return 0;
}
