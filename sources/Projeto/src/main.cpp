/********** Headers **********/

// internos
#include <cmdline_parser.h>

// main entry point
// ./Projeto datasets/gargoyle/00_absolute_paths.txt <f> <cx> <cy> -o ~/dev/cpp/ProjetoGraduacao/output
int main(int argc, char** argv) {

	ctrl::parse(argc, argv);

	return 0;
}
