#ifndef CMDLINE_PARSER_H
#define CMDLINE_PARSER_H

/********** Headers **********/

// libc++
#include <iostream>		// input/output streams
#include <filesystem> 	// filesystem utilities

namespace fs = std::filesystem; // filesystem namespace alias

/********** cmdline_parser.h **********/

/** Lógica de Controle. */
namespace ctrl {

/** Armazena os argumentos validados extraídos da linha de comando em variáveis de controle. */
struct args {
	// índices dos argumentos da linha de comando na lista de argumentos
	static const int INPUT_PATH_IDX 	= 1; 	/** Índice do argumento orbigatório <input_path> na lista de argumentos. */
	static const int FOCAL_LENGTH_IDX 	= 2; 	/** Índice do argumento orbigatório <f> na lista de argumentos. */
	static const int PRINCIPAL_X_IDX 	= 3; 	/** Índice do argumento orbigatório <cx> na lista de argumentos. */
	static const int PRINCIPAL_Y_IDX 	= 4; 	/** Índice do argumento orbigatório <cy> na lista de argumentos. */
	static const int MIN_ARGS 			= 5; 	/** Número mínimo de argumentos. Também representa o índice do primeiro argumento subsequente aos obrigatórios. */

	// caminhos de entrada e saída
	fs::path input_path{}; 				/* Caminho até o diretório de entrada que contém as imagens do dataset. */
	fs::path output_path{"./output"}; 	/* Caminho até o diretório de saída onde os resultados serão exportados para. */

	// Parâmetros intrínsecos da câmera
	float f{}; 			/** Distância focal da câmera. */
	float cx{}, cy{};	/** Offset do ponto principal (x, y) da câmera. */
};

/********** Funções (declaração) **********/

void help(std::ostream& os = std::cout);
args parse(const int argc, const char* const* argv);

} // namespace ctrl

#endif // CMDLINE_PARSER_H
