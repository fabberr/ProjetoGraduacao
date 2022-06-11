/********** Headers **********/

// libc++
#include <vector>
#include <array>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <filesystem>

namespace fs = std::filesystem;

// internos
#include <parser.h>
#include <logger.h>

/********** parser.cpp **********/

/**
 * Insere a mensagem de ajuda na stream especificada.
 *
 * @param out Objeto `std::ostream` onde a mensagem de ajuda será inserida.
 *            Opcional, `std::cout` por padrão.
*/
void ctrl::help(std::ostream& out) {
	out <<
		"Uso: ./mesh <cloud_path> [-h] [--out=<output_dir>] [--method=<recon_method>]\n"
		"    Reconstroi a superfície para a nuvem de pontos especificada.\n"
		"\n"
		"    cloud_path:\n"
		"        Caminho até um arquivo do tipo OBJ ou PLY contendo a nuvem de pontos.\n"
		"\n"
		"    -h, --help:\n"
		"        Exibe essa mensagem de ajuda e termina o programa.\n"
		"    output_dir:\n"
		"        Caminho até o diretório onde a cena com a superfície reconstruida será\n"
		"        salva. Opcional, caso nenhum valor seja passado, usa o mesmo diretório\n"
		"        onde <cloud_path> está localizado ou `./` caso isso não seja possível.\n"
		"    recon_method:\n"
		"        Metodologia usada para reconstruir a superfície, valores aceitos são\n"
		"        GREEDY_PROJECTION e POISSON. Opcional, GREEDY_PROJECTION por padrão.\n"
	<< std::endl;
}

/**
 * Analisa e extrai os argumentos da linha de comando.
 *
 * @param args Lista de argumentos.
 *
 * @returns Uma struct contendo os argumentos extraídos.
*/
ctrl::args ctrl::parse(const std::vector<std::string>& args) {

	// mínimo de argumentos
	if (args.size() < ctrl::args::min_args) {
		ctrl::help(std::cerr);
		std::exit(EXIT_FAILURE);
	}

	// verificando [-h, --help]
	if (std::find_if(args.cbegin(), args.cend(),
		[](const std::string& v) { return (v == "-h" || v == "--help"); }
	) != args.cend()) {
		ctrl::help();
		std::exit(EXIT_SUCCESS);
	}

	ctrl::args extracted{};

	// extraindo e validando argumento obrigatório <cloud_path>
	struct valid_file {
		const fs::path& p;
		explicit operator bool() const {
			return (fs::is_regular_file(p) && (p.extension() == ".obj" || p.extension() == ".ply"));
		}
	};
	if (extracted.cloud_path = args[args::cloud_path_idx]; not valid_file{extracted.cloud_path}) {
		log_error_and_exit("Arquivo especificado `%s` não existe ou é inválido\n", args[args::cloud_path_idx].c_str());
	}

	// iterando sobre o resto da lista buscando argumentos opcionais
	for (auto it = args.begin() + args::min_args; it != args.end(); ++it) {

		/**
		 * Helper function: Separa o argumento atual (apontado por `it`) em 2 tokens de
		 * acordo com o delimitador especificado.
		*/
		const auto tokenize = [&it](char delim = '=') {
			std::array<std::string, 2U> tokens{};
			tokens.fill("");

			auto i = 0U;
			std::stringstream ss{*it};
			for (std::string token{}; i < 2 && std::getline(ss, token, delim); ) {
				tokens[i++] = token;
			}
			return tokens;
		};
		const auto& [left, right] = tokenize();
		if (left == "" && right == "") { continue; }

		if (left == "--out") {
			if (fs::is_directory(right)) {
				extracted.output_dir = right;
			}
		} else if (left == "--method") {
			if (right == "GREEDY_PROJECTION" || right == "POISSON") {
				extracted.method = right;
			}
		}
	}

	// certificando-se que o argumento <output_dir> tem um valor válido
	if (extracted.output_dir == "") {
		extracted.output_dir = (extracted.cloud_path.has_parent_path()) ? extracted.cloud_path.parent_path() : "./";
		log_info("Usando `%s` como diretório de saída\n", extracted.output_dir.c_str());
	}

#if defined (DEBUG) || defined (_DEBUG)
	std::cerr << "{ cloud_path=" << extracted.cloud_path << ", output_dir=" << extracted.output_dir << ", method=" << extracted.method << " }\n";
#endif
	return extracted;
}
