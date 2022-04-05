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

/********** Lógica de Controle **********/

namespace ctrl {

// índices dos argumentos da linha de comando na lista de argumentos
const int ARG_INPUT_FILE_IDX 	= 1; /** Índice do argumento orbigatório <input_file> na lista de argumentos. */
const int ARG_FOCAL_LENGTH_IDX 	= 2; /** Índice do argumento orbigatório <f> na lista de argumentos. */
const int ARG_PRINCIPAL_X_IDX 	= 3; /** Índice do argumento orbigatório <cx> na lista de argumentos. */
const int ARG_PRINCIPAL_Y_IDX 	= 4; /** Índice do argumento orbigatório <cy> na lista de argumentos. */
const int ARG_MINIMUM 			= 5; /** Número mínimo de argumentos, também representa o índice do primeiro argumento subsequente aos obrigatórios. */

// macros
#define ARG_INPUT_FILE 		argv[ctrl::ARG_INPUT_FILE_IDX]
#define ARG_FOCAL_LENGTH 	argv[ctrl::ARG_FOCAL_LENGTH_IDX]
#define ARG_CX 				argv[ctrl::ARG_PRINCIPAL_X_IDX]
#define ARG_CY 				argv[ctrl::ARG_PRINCIPAL_Y_IDX]

// function-like macros
#define log_error_and_exit(msg, ...) 	std::fprintf(stderr, "[ERROR]: " msg __VA_OPT__(,) __VA_ARGS__); std::exit(-1)
#define log_info(msg, ...) 				std::fprintf(stdout, "[INFO]: " msg __VA_OPT__(,) __VA_ARGS__)
#define display_help_and_exit() 		ctrl::help(); std::exit(0)

// variáveis de conrtrole: caminhos de entrada e saída
fs::path input_file{}; 				/* Caminho até o arquivo que lista os caminhos das imagens do dataset. */
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
		"Uso: ./Projeto <input_file> <f> <cx> <cy> [OPTIONS]\n" 
		"    Computa uma nuvem de pontos esparsa para um conjunto de imagens.\n" 
		"    Ao final da execucao, um arquivo .obj contendo apenas uma lista de vertices\n" 
		"    representando a nuvem de pontos sera gerado.\n" 
		"\n" 
		"<input_file>:\n" 
		"    Arquivo contendo os caminhos absolutos ate as imagens do dataset a serem\n" 
		"    usadas, um por linha.\n" 
		"<f>:\n" 
		"    Distancia Focal da camera.\n" 
		"<cx> e <cy>:\n" 
		"    Coordenadas X e Y do Ponto Central da camera, respectivamente.\n" 
		"\n" 
		"OPTIONS:\n" 
		"    --help, -h:\n" 
		"        Exibe esta mensagem de ajuda e termina a execucao sem computar nada.\n" 
		"    -o <output_path>:\n" 
		"        Caminho ate o diretorio onde serao armazenados os resultados. Se nenhum\n" 
		"        caminho for fornecido, por padrao sera definido como `$PWD/output/`.\n" 
	<< std::endl;
}

/* Analisa os argumentos da linha de comando e modifica as variáveis de controle. */
void ctrl::parse(const int argc, const char* const* argv) {

	// verificando número mínimo de argumentos
	if (argc < ctrl::ARG_MINIMUM) {
		display_help_and_exit();
	}

	// extraindo e validando argumento obrigatório <input_file>
	if (ctrl::input_file = ARG_INPUT_FILE; !fs::exists(ctrl::input_file) || !fs::is_regular_file(ctrl::input_file)) {
		log_error_and_exit("Arquivo de entrada `%s` nao existe ou nao e valido\n", ARG_INPUT_FILE);
	}

	// extraindo e validando argumento obrigatório <f>
	if (ctrl::f = std::strtof(ARG_FOCAL_LENGTH, nullptr); ctrl::f == 0) {
		log_error_and_exit("Distancia focal `%s` e invalida\n", ARG_FOCAL_LENGTH);
	}

	// extraindo e validando argumentos obrigatórios <cx> e <cy>
	if (ctrl::cx = std::strtof(ARG_CX, nullptr), ctrl::cy = std::strtof(ARG_CY, nullptr); ctrl::cx == 0 || ctrl::cy == 0) {
		log_error_and_exit("Ponto principal (%s, %s) e invalido\n", ARG_CX, ARG_CY);
	}

	// verificando e extraindo argumentos opcionais, se existirem
	for (int arg_idx = ctrl::ARG_MINIMUM; arg_idx < argc; ++arg_idx) { // intervalo da busca: [ARG_MINIMUM, argc)
		const char* current_arg = argv[arg_idx];
		if (std::strncmp(current_arg + 1, "-h", 2) == 0) { 		// strings que correspondem ao padrão `?-h*`
			display_help_and_exit();
		} else if (std::strncmp(current_arg, "-o", 2) == 0) { 	// -o <output_path>
			// incremente arg_idx e verifica se o próximo argumento existe
			if (current_arg = argv[++arg_idx]; arg_idx < argc) {
				/** FIXME:
				 * É possível que `current_arg` não seja um caminho, mas sim outro argumento 
				 * opcional passado pelo usuário (no caso `... -o -h`, por exemplo). Neste caso o
				 * argumento opcional que segue `-o` será ignorado e o programa vai tentar usar 
				 * este argumento como diretório de saída e pode causar um erro durante a execução.
				*/
				ctrl::output_path = current_arg;
				log_info("Usando `%s` como caminho de saida\n", ctrl::output_path.c_str());
			} else {
				ctrl::output_path = "./output/";
				log_info("Nenhum valor passado para a opcao `-o`, usando `%s` como caminho de saida\n", ctrl::output_path.c_str());
			}
		}
	}
}

// main entry point
// ./Projeto datasets/gargoyle/00_absolute_paths.txt <f> <cx> <cy> -o ~/dev/cpp/ProjetoGraduacao/output
int main(int argc, char** argv) {

	ctrl::parse(argc, argv);

	return 0;
}
