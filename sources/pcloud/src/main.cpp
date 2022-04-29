/********** Headers **********/

// internos
#include <cmdline_parser.h>
#include <sfmPointCloud.h>

// main entry point
// ./pcloud datasets/gargoyle 800 576.0 1024.0 -o ~/dev/cpp/ProjetoGraduacao/output
int main(int argc, char** argv) {

	sfmPointCloud{ctrl::parse(argc, argv)}.compute_sparse();

	return 0;
}
