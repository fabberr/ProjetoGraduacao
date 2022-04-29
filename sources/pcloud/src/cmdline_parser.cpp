/********** Headers **********/

// libc++
#include <iostream>		// input/output streams
#include <filesystem>	// filesystem utilities

namespace fs = std::filesystem; // filesystem namespace alias

// libc
#include <cstdlib> 	// strtof, exit, EXIT_FAILURE, EXIT_SUCCESS
#include <cstring> 	// strncmp

// internos
#include <cmdline_parser.h>
#include <logger.h>

/********** Macros **********/

// macros para acesso à índices específicos da lista de argumentos
#define ARG_INPUT_PATH 		argv[ctrl::args::INPUT_PATH_IDX]
#define ARG_FOCAL_LENGTH 	argv[ctrl::args::FOCAL_LENGTH_IDX]
#define ARG_CX 				argv[ctrl::args::PRINCIPAL_X_IDX]
#define ARG_CY 				argv[ctrl::args::PRINCIPAL_Y_IDX]

// function-like macros
#define display_help_and_exit() ctrl::help(); std::exit(EXIT_SUCCESS)

/********** Funções (definição) **********/

/** Insere a mensagem de ajuda do programa em um objeto std::ostream (std::cout por padrão). */
void ctrl::help(std::ostream& os) {
	os << 
		"Uso: ./pcloud <input_path> <f> <cx> <cy> [OPTIONS]\n" 
		"    Computa uma nuvem de pontos esparsa para um conjunto de imagens.\n" 
		"    Ao final da execucao, dois arquivos serao gerados: um arquivo .obj contendo\n" 
		"    apenas a lista de vertices, representando a nuvem de pontos, e um arquivo\n" 
		"    .sfm detalhando a pose (matriz de transformacao Rt) estimada para cada\n" 
		"    camera.\n" 
		"\n" 
		"<input_path>:\n" 
		"    Caminho ate o diretorio contendo as imagens do dataset.\n"
		"<f>:\n" 
		"    Distancia Focal da camera, em pixels.\n" 
		"<cx> e <cy>:\n" 
		"    Offsets X e Y do Ponto Principal da camera, em pixels.\n" 
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
 * Analisa a linha de comando e retorna um objeto contendo os argumentos 
 * extraídos.
 * 
 * Caso um erro de validação seja encontrado, o programa termina a execução com 
 * código de status `EXIT_FAILURE`.
 * 
 * Caso o argumento opcional `--help` esteja presente ou argumentos obrigatórios 
 * insuficientes foram passados, o programa simplesmente exibe a mensagem de 
 * ajuda e termina a execução com código de status `EXIT_SUCCESS`.
*/
ctrl::args ctrl::parse(const int argc, const char* const* argv) {

	// verificando número mínimo de argumentos
	if (argc < ctrl::args::MIN_ARGS) {
		display_help_and_exit();
	}

	// instanciando variavés de controle com valores padrão
	ctrl::args extracted{};

	// extraindo e validando argumento obrigatório <input_path>
	if (extracted.input_path = ARG_INPUT_PATH; !fs::exists(extracted.input_path) || !fs::is_directory(extracted.input_path)) {
		log_error_and_exit("Diretorio de entrada `%s` nao existe ou nao e valido\n", ARG_INPUT_PATH);
	}

	// extraindo e validando argumento obrigatório <f>
	if (extracted.f = std::strtof(ARG_FOCAL_LENGTH, nullptr); extracted.f == 0) {
		log_error_and_exit("Distancia focal `%s` e invalida\n", ARG_FOCAL_LENGTH);
	}

	// extraindo e validando argumentos obrigatórios <cx> e <cy>
	if (extracted.cx = std::strtof(ARG_CX, nullptr), extracted.cy = std::strtof(ARG_CY, nullptr); extracted.cx == 0 || extracted.cy == 0) {
		log_error_and_exit("Ponto principal (%s, %s) e invalido\n", ARG_CX, ARG_CY);
	}

	// verificando e extraindo argumentos opcionais, se existirem
	for (int arg_idx = ctrl::args::MIN_ARGS; arg_idx < argc; ++arg_idx) { // intervalo da busca: [MIN_ARGS, argc)
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
				extracted.output_path = current_arg;
				log_info("Usando `%s` como caminho de saida\n", extracted.output_path.c_str());
			} else {
				extracted.output_path = "./output/";
				log_info("Nenhum valor passado para a opcao `-o`, usando `%s` como caminho de saida\n", extracted.output_path.c_str());
			}
		}
	}

	return extracted;
}

/** @todo Adicionar acentuação às mensagens. */
