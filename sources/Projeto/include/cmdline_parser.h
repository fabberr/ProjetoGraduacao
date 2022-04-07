#ifndef CMDLINE_PARSER_H
#define CMDLINE_PARSER_H

/********** Headers **********/

// libc++
#include <iostream>		// input/output streams
#include <filesystem> 	// filesystem utilities

namespace fs = std::filesystem; // filesystem namespace alias

// libc
#include <cstdio> 	// fprintf, stdout, stderr, EXIT_FAILURE, EXIT_SUCCESS
#include <cstdlib> 	// exit

/********** cmdline_parser.h **********/

/** Lógica de Controle. */
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
#define log_error_and_exit(msg, ...) 	std::fprintf(stderr, "[ERROR]: " msg __VA_OPT__(,) __VA_ARGS__); std::exit(EXIT_FAILURE)
#define log_info(msg, ...) 				std::fprintf(stdout, "[INFO]: " msg __VA_OPT__(,) __VA_ARGS__)
#define display_help_and_exit() 		ctrl::help(); std::exit(EXIT_SUCCESS)

// variáveis de controle: caminhos de entrada e saída
fs::path input_file{}; 				/* Caminho até o arquivo que lista os caminhos das imagens do dataset. */
fs::path output_path{"./output/"}; 	/* Caminho até o diretório de saída. */

// variáveis de controle: parâmetros intrínsecos da câmera
float f{}; 			/** Distância focal da câmera. */
float cx{}, cy{}; 	/** Offset do ponto principal (x, v) da câmera. */

// funções (declaração)

void help(std::ostream& os = std::cout);
void parse(const int argc, const char* const* argv);

} // namespace ctrl

#endif // CMDLINE_PARSER_H
