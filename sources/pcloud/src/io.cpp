/********** Headers **********/

// libc++
#include <iostream> 	// i/o streams
#include <fstream> 		// i/o file streams
#include <functional> 	// function objects support
#include <filesystem> 	// filesystem utilities

namespace fs = std::filesystem; // filesystem namespace alias

// internos
#include <io.h>
#include <logger.h>

/********** Funções (definição) **********/

/**
 * Abre o arquivo indicado por `filename` para leitura e lê seus conteúdos de 
 * acordo com `read_fn`.
 * Note que esta função é responsável pelas operações de abertura/fechamento de 
 * arquivo e tratamento de erros, e que portanto, `read_fn` deve apenas realizar 
 * operações de leitura sobre o arquivo.
 *
 * @param filename Nome do arquivo para leitura.
 * @param read_fn Função que define as operações de leitura. Esta função deve 
 *        receber como seu único arugumento uma referência à um objeto 
 *        std::ifstream e retornar `true` apenas caso seja bem sucedida em 
 *        realizar todas as suas operações de leitura.
 * @param mode Indica o modo de abertura do arquivo. Opcional, 
 *        `std::ios_base::in` por padrão
 *
 * @returns `true` se a operação de leitura foi bem sucedida, `false` caso
 *          contrário.
*/
bool io::import_from_file(const fs::path& filename, const std::function<bool(std::ifstream&)>& read_fn, std::ios_base::openmode mode) {

	mode |= std::ios_base::in;

	bool success{};
	if (std::ifstream file{filename, mode}; file.is_open()) {
		success = read_fn(file);
		file.close();
	} else {
		log_error("Não foi possível abrir arquivo `%s`\n", filename.c_str());
		return false;
	}
	return success;
}

/**
 * Abre o arquivo indicado por `filename` para escrita e lê seus conteúdos de 
 * acordo com `write_fn`.
 * Note que esta função é responsável pelas operações de abertura/fechamento de 
 * arquivo e tratamento de erros, e que portanto, `write_fn` deve apenas 
 * realizar operações de escrita sobre o arquivo.
 *
 * @param filename Nome do arquivo para escrita.
 * @param write_fn Função que define as operações de escrita. Esta função deve 
 *        receber como seu único arugumento uma referência à um objeto 
 *        std::ofstream e retornar `true` apenas caso seja bem sucedida em 
 *        realizar todas as suas operações de escrita.
 * @param mode Indica o modo de abertura do arquivo. Opcional, 
 *        `std::ios_base::out` por padrão
 *
 * @returns `true` se a operação de escrita foi bem sucedida, `false` caso
 *          contrário.
*/
bool io::export_to_file(const fs::path& filename, const std::function<bool(std::ofstream&)>& write_fn, std::ios_base::openmode mode) {

	mode |= std::ios_base::out;

	bool success{};
	if (std::ofstream file{filename, mode}; file.is_open()) {
		success = write_fn(file);
		file.close();
		std::cout << "    Exportado: `" << filename.c_str() << "`\n";
	} else {
		log_error("Não foi possível abrir arquivo `%s`\n", filename.c_str());
		return false;
	}
	return success;
}
