/********** Headers **********/

// internos
#include <cmdline_parser.h>
#include <sfmPointCloud.h>

// main entry point
// ./Projeto datasets/gargoyle <f> <cx> <cy> -o ~/dev/cpp/ProjetoGraduacao/output
int main(int argc, char** argv) {

	sfmPointCloud{ctrl::parse(argc, argv)}.compute_sparse();

	return 0;
}
