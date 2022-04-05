/********** Headers **********/

// libc++
#include <iostream>		// input/output streams
#include <filesystem>	// filesystem library

namespace fs = std::filesystem; // filesystem namespace alias

// libc
#include <cstdio> 	// fprintf
#include <cstddef> 	// size_t
#include <cstring> 	// strncmp
#include <cstdlib>	// strtof, exit

// internos
#include <PointCloud.h>

/********** Lógica de Controle **********/

namespace ctrl {

// índices dos argumentos da linha de comando na lista de argumentos
const int _INPUT_PATH_IDX 	= 1; 	/** Índice do argumento orbigatório <input_path> na lista de argumentos. */
const int _FOCAL_LENGTH_IDX = 2; 	/** Índice do argumento orbigatório <f> na lista de argumentos. */
const int _PRINCIPAL_X_IDX 	= 3; 	/** Índice do argumento orbigatório <cx> na lista de argumentos. */
const int _PRINCIPAL_Y_IDX 	= 4; 	/** Índice do argumento orbigatório <cy> na lista de argumentos. */
const int _MIN_ARGS 		= 5; 	/** Número mínimo de argumentos, também representa o índice do primeiro argumento subsequente aos obrigatórios. */

// macros
#define ARG_INPUT_PATH 		argv[ctrl::_INPUT_PATH_IDX]
#define ARG_FOCAL_LENGTH 	argv[ctrl::_FOCAL_LENGTH_IDX]
#define ARG_CX 				argv[ctrl::_PRINCIPAL_X_IDX]
#define ARG_CY 				argv[ctrl::_PRINCIPAL_Y_IDX]

// function-like macros
#define MSG_AND_EXIT_FAILURE(msg, ...) std::fprintf(stderr, msg __VA_OPT__(,) __VA_ARGS__); std::exit(-1);
#define MSG_AND_EXIT_SUCCESS(msg, ...) std::fprintf(stdout, msg __VA_OPT__(,) __VA_ARGS__); std::exit(0);

// variáveis de conrtrole: caminhos de entrada e saída
fs::path input_path{}; 				/* Caminho até o arquivo que lista os caminhos das imagens do dataset. */
fs::path output_path{"./output/"}; 	/* Caminho até o diretório de saída. */

// variáveis de controle: parâmetros intrínsecos da câmera
float f{}; 			/** Distância focal da camera. */
float cx{}, cy{}; 	/** Offset do ponto principal (x, v) da câmera. */

// funções

void help(std::ostream& os = std::cout);
void parse(const int argc, const char* const* argv);

} // namespace ctrl

/********** Funções **********/

/* Insere a mensagem de ajuda do programa num objeto std::ostream. */
void ctrl::help(std::ostream& os) {
	os << 
		"Uso: ./Projeto <input_path> <f> <cx> <cy> [OPTIONS]\n\n" 
		"<input_path>:\n" 
		"    Arquivo que contem os caminhos ate as imagens a serem usadas para computar a\n" 
		"    nuvem de pontos esparsa, um por linha. Ao final da execucao, sera gerado um\n" 
		"    arquivo *.obj contendo somente a lista de vertices representando a nuvem de\n" 
		"    pontos esparsa." 
		"OPTIONS:\n" 
		"    --help, -h:\n" 
		"        Exibe esta mensagem de ajuda e termina a execucao sem computar nada.\n" 
		"    -o <output_path>:\n" 
		"        Caminho ate o diretorio onde serao armazenados os resultados.\n" 
		"        `$PWD/output/`, por padrao.\n" 
	<< std::endl;
}

/* Analisa os argumentos da linha de comando e modifica as variáveis de controle. */
void ctrl::parse(const int argc, const char* const* argv) {

	// verificando número mínimo de argumentos
	if (argc < ctrl::_MIN_ARGS) {
		ctrl::help();
		std::exit(-1);
	}

	// extraindo e validando argumento obrigatório <input_path>
	if (ctrl::input_path = ARG_INPUT_PATH; !fs::exists(ctrl::input_path) || !fs::is_regular_file(ctrl::input_path)) {
		MSG_AND_EXIT_FAILURE("[ERRO] Arquivo de entrada `%s` nao existe ou nao e valido\n", ARG_INPUT_PATH)
	}

	// extraindo e validando argumento obrigatório <f>
	if (ctrl::f = std::strtof(ARG_FOCAL_LENGTH, nullptr); ctrl::f == 0) {
		MSG_AND_EXIT_FAILURE("[ERRO] Distancia focal `%s` e invalida\n", ARG_FOCAL_LENGTH)
	}

	// extraindo e validando argumentos obrigatórios <cx> e <cy>
	if (ctrl::cx = std::strtof(ARG_CX, nullptr), ctrl::cy = std::strtof(ARG_CY, nullptr); ctrl::cx == 0 || ctrl::cy == 0) {
		MSG_AND_EXIT_FAILURE("[ERRO] Ponto principal (%s, %s) e invalido\n", ARG_CX, ARG_CY)
	}

	// verificando e extraindo argumentos opcionais, se existirem
	for (int arg_idx = ctrl::_MIN_ARGS; arg_idx < argc; ++arg_idx) { // intervalo da busca: [_MIN_ARGS, argc)
		const char* current_arg = argv[arg_idx];
		if (std::strncmp(current_arg + 1, "-h", 2) == 0) { 		// strings que correspondem ao padrão `?-h*`
			ctrl::help();
			std::exit(0);
		} else if (std::strncmp(current_arg, "-o", 2) == 0) { 	// -o <output_path>
			// incremente arg_idx e verifica se o próximo argumento existe
			if (current_arg = argv[++arg_idx]; arg_idx < argc) {
				// validação do diretório é feita nas funções que exportam os resultados
				/**
				 * FIXME:
				 * É possível que `current_arg` não seja um caminho, mas sim outro argumento 
				 * opcional passado pelo usuário (no caso `... -o -h`, por exemplo). Neste caso o
				 * argumento opcional que segue `-o` será ignorado e o programa vai tentar usar 
				 * este argumento como diretório de saída e pode causar um erro durante a execução.
				*/
				ctrl::output_path = current_arg;
			} else {
				std::cout << "[INFO]: Nenhum valor foi passado para a opcao `-o`, usando `./output/` como caminho de saida\n";
				ctrl::output_path = "./output/";
			}
		}
	}
}

// main entry point
// build/Projeto/Projeto ~/dev/cpp/ProjetoGraduacao/datasets/gargoyle/00_index.txt <f> <cx> <cy> -o ~/dev/cpp/ProjetoGraduacao/output
int main(int argc, char** argv) {

	ctrl::parse(argc, argv);

	return 0;
}
