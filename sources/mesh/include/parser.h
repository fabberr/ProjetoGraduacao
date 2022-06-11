#ifndef PARSER_H
#define PARSER_H

/********** Headers **********/

// libc++
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>

namespace fs = std::filesystem;

// libc
#include <cstdlib>

/********** parser.h **********/

/** Lógica de controle. */
namespace ctrl {

	/** Argumentos extraídos e validados. */
	struct args {
		fs::path cloud_path; /** Caminho até o arquivo contendo a nuvem de pontos. */
		fs::path output_dir; /** Caminho até o diretório de saída. */

		std::string method = "GREEDY_PROJECTION"; /** Método usado para reconstruir a superfície. */

		static const int cloud_path_idx = 1; /** Índice esperado do argumento <cloud_path> na lista de argumentos. */
		static const int min_args 		= 2; /** Minimo de argumentos. Também representa o índice do primeiro argumento subsequente ao obrigatório. */
	}; // struct args

	void help(std::ostream& out = std::cout);
	args parse(const std::vector<std::string>& args);
	
} // namespace ctrl

#endif // PARSER_H
