/********** Headers **********/

// libc++
#include <iostream>		// input/output streams
#include <filesystem>	// filesystem utilities

namespace fs = std::filesystem; // filesystem namespace alias

// libc
#include <cstring> 	// strncmp
#include <cstdlib>	// strtof

// internos
#include <cmdline_parser.h>

/********** Funções (definição) **********/

/** Insere a mensagem de ajuda do programa em um objeto std::ostream (stc::cout por padrão). */
void ctrl::help(std::ostream& os) {
	os << 
		"Uso: ./Projeto <input_file> <f> <cx> <cy> [OPTIONS]\n" 
		"    Computa uma nuvem de pontos esparsa para um conjunto de imagens.\n" 
		"    Ao final da execucao, dois arquivos serao gerados: um arquivo .obj contendo\n" 
		"    apenas a lista de vertices, representando a nuvem de pontos, e um arquivo .sfm\n" 
		"    detalhando a pose (matrix de transformacao Rt) estimada para cada camera.\n" 
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

/** 
 * Analisa a linha de comando e modifica as variáveis de controle de acordo com 
 * os argumentos presentes.
 * 
 * Caso um erro de validação seja encontrado, o programa termina a execução com 
 * código de status `EXIT_FAILURE`.
 * 
 * Caso o argumento ocional `--help` esteja presente ou argumentos obrigatórios 
 * insuficientes foram passados, o programa simplesmente exibe a mensagem de 
 * ajuda e termina a execução com código de status `EXIT_SUCCESS`.
*/
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
				 * É possível que neste estado, `current_arg` não seja um caminho, mas sim outro 
				 * argumento opcional passado pelo usuário (no caso `... -o -h`, por exemplo). 
				 * Neste caso o argumento opcional que segue `-o` (se existir) será ignorado na 
				 * próxima iteração pois `arg_idx` é incrementado no if statement. Além disso, o
				 * programa ira tentar usar este argumento como o diretório de saída, o que pode
				 * ocasionar um erro de execução.
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
