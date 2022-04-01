/********** Headers **********/

// libc++
#include <iostream>		// input/output streams
#include <filesystem>	// filesystem library

namespace fs = std::filesystem; // filesystem namespace alias

// libc
#include <cstdio> 	// fprintf
#include <cstddef> 	// size_t
#include <cstring> 	// strncmp
#include <cstdlib>	// strtol, exit

// internos
#include <PointCloud.h>
// #include <SurfaceReconstruction.h>

/********** Variáveis de controle **********/

namespace ctrl {

// input/output paths
fs::path input_path{}; 				/* Caminho até diretório do dataset (sparse) ou do arquivo .sfm (dense). */
fs::path output_path{"./output/"}; 	/* Caminho até o diretório de saída. */

// [--count]
long int count{0}; /* Número máximo de imagens que devem ser usadas (sparse). */

// [--dense]
bool compute_dense{false}; /* Flag -- indica se deve computar a nuvem densa ao invés da esparsa. */

} // namespace ctrl

/********** Funções **********/

/* Exibe a mensagem de ajuda do programa. */
void printHelp() {
	std::cout << 
		"Uso: ./Projeto <input_path> [output_path] [OPTIONS]\n\n" 
		"<input_path>:\n" 
		"    Caminho do diretorio contendo o dataset usado para computar a nuvem de\n" 
		"    pontos esparsa. Ao final da execucao, serao gerados tres arquivos:\n" 
		"        -`point_cloud.obj`: Arquivo no formato *.obj contendo apenas a lista de\n" 
		"         vertices do objeto.\n" 
		"        -`nview_match.nvm`: Arquivo no formato *.nvm contendo as cameras e suas\n" 
		"         respectivas matrizes intrinsecas e extrinsecas e listas de vertices.\n" 
		"        -`pose.sfm`: Arquivo no formato *.sfm contendo informacoes sobre a pose\n" 
		"         estimada de cada camera (matrizes intrinseca e extrinseca e ponto\n" 
		"         focal).\n" 
		"[output_path]:\n" 
		"    Opcional. Caminho ate o diretorio onde serao armazenados os resultados.\n" 
		"    `./output/`, por padrao\n"
		"OPTIONS:\n" 
		"    --help, -h:\n" 
		"        Exibe esta mensagem de ajuda.\n" 
		"    --count:\n" 
		"        Especifica o total de imagens do dataset que serao usadas.\n" 
		"    --dense:\n" 
		"        Computa a nuvem de pontos densa, usando o diretorio especificado em `input_path`\n" 
		"        para localizar o arquivo de calibracao das cameras (*.sfm).\n"
	<< std::endl;
}

/* Analisa os argumentos da linha de comando e modifica as variáveis de controle. */
void parse(int argc, const char* const* argv) {

	// processando flags e argumentos de controle, se existirem
	for (int arg_idx = 2; arg_idx < argc; ++arg_idx) {
		const char* current_arg = argv[arg_idx];

		if (std::strncmp(current_arg + 1, "-h", 2) == 0) { 			// --help, -h
			printHelp();
			std::exit(0);
		} else if (std::strncmp(current_arg, "--count", 7) == 0) { 	// --count
			// incremente arg_idx e verifica se o próximo argumento existe
			if (current_arg = argv[++arg_idx]; arg_idx < argc) {
				// tenta converter em um valor inteiro na base10, se falhar, strol retorna 0
				ctrl::count = std::strtol(current_arg, nullptr, 10);
			}
		} else if (std::strncmp(current_arg, "--dense", 7) == 0) { 	// --dense
			// set flag compute_dense
			ctrl::compute_dense = true;
		}
	}

	// verificando número mínimo de argumentos
	if (argc < 2) {
		printHelp();
		std::exit(-1);
	}

	// extraíndo e validando argumento obrigatório input_path
	ctrl::input_path = argv[1];
	if (!fs::exists(ctrl::input_path) || !fs::is_directory(ctrl::input_path)) {
		std::cout << "[ERRO] Caminho de entrada especificado nao existe ou nao e valido\n";
		std::exit(-1);
	}

	// extraíndo argumento opcional output_path, se existir
	if (argc > 2) {
		// validação nas funções que exportam os resultados
		ctrl::output_path = argv[2];
	}
}

// main entry point
// build/Projeto/Projeto ~/dev/cpp/ProjetoGraduacao/datasets/gargoyle ~/dev/cpp/ProjetoGraduacao/output
int main(int argc, char** argv) {
	
	// analisa os argumentos da linha de comando
	parse(argc, argv);

	// instanciando objeto PointCloud sem nome e computando nuvem de pontos
	PointCloud{ 
		ctrl::input_path.make_preferred(), 		// std::filesystem::path
		ctrl::output_path.make_preferred(), 	// std::filesystem::path
		static_cast<std::size_t>(ctrl::count) 	// std::size_t
	}.computeSparse();

	return 0;
}
