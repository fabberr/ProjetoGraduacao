/********** Headers **********/

// STL
#include <filesystem>	// filesystem library
#include <iostream>		// cout
#include <cstring>		// strncmp
#include <cstdlib>		// strtol

// internal
#include <include/PointCloud.h>
// #include <include/SurfaceReconstruction.h>

namespace fs = std::filesystem; // filesystem namespace alias

/********** Vari�veis globais de controle **********/

fs::path path("./");		// obrigat�rio	<path>:				indica o caminho at� diret�rio das imagens (sparse) ou do arquivo .sfm (dense)

long int n = 0;				// opcional		[--count <N>]:		indica o n�mero m�ximo de imagens que devem ser usadas (sparse)

bool computeDense = false;	// opcional		[--dense]:			flag -- indica se deve computar a nuvem densa ao inv�s da esparsa

fs::path cloudPath("./");	// opcional		[--cloud <path>]:	indica o caminho at� o arquivo .obj (sparse) e pula a etapa de gera��o da nuvem de pontos
bool skipCloud = false;		//									flag -- indica se deve pular a etapa de gera��o da nuvem de pontos

/********** Fun��es **********/

/* Exibe a mensagem de ajuda do programa e termina a execu��o. */
void printHelp() {
	std::cout << 
		"Uso: ./ProjetoGrad <PATH> [OPTIONS]\n\n" 
		"PATH:\n" 
		"    Caminho do diretorio contendo o dataset usado para computar a nuvem de\n" 
		"    pontos esparsa. Ao final da execucao, serao gerados tres arquivos:\n" 
		"        -\"point_cloud.obj\": Arquivo no formato *.obj contendo apenas a lista de\n" 
		"         vertices do objeto.\n" 
		"        -\"nview_match.nvm\": Arquivo no formato *.nvm contendo as cameras e suas\n" 
		"         respectivas matrizes intrinsecas e extrinsecas e listas de vertices.\n" 
		"         Usado para visualizacao da nuvem de pontos esparsa.\n" 
		"        -\"pose.sfm\": Arquivo no formato *.sfm contendo informacoes sobre a pose\n" 
		"         estimada de cada camera (matrizes intrinseca e extrinseca e ponto\n" 
		"         focal). Usado para computar a nuvem de pontos densa.\n" 
		"OPTIONS:\n" 
		"--help, -h:\n" 
		"    Exibe esta mensagem de ajuda.\n" 
		"--count <N>:\n" 
		"    Especifica o total de imagens do dataset que serao usadas.\n" 
		"--dense:\n" 
		"    Computa a nuvem de pontos densa, usando o diretorio especificado em PATH\n" 
		"    para localizar o arquivo necessario (pose.sfm).\n"
	<< std::endl;

	std::exit(0);
}

/*
* Analisa os argumentos da linha de comando e modifica as vari�veis de controle.
* 
* @param argc -- N�mero de argumentos.
* @param argv -- Vetor de C-strings contendo argc argumentos.
*/
void parse(int argc, char* const* argv) {

	// verificando m�nimo de argumentos ou chamado de ajuda
	if (argc < 2 ||
		std::strncmp(argv[1], "--help", 6) == 0 ||
		std::strncmp(argv[1], "-h", 2) == 0) {

		printHelp();
	}

	// verificando e extra�ndo argumento obrigat�rio
	path = fs::path(argv[1]);
	if (!fs::is_directory(path)) {
		std::cout << "ERRO: caminho especificado nao existe.\n";
		std::exit(-1);
	}

	// extrair argumentos opcionais, se existirem
	for (int i = 2; i < argc; i++) {
		if (argv[i] && !std::strncmp(argv[i], "--count", 7)) {
			// --count <N>

			// incrementa i e extrai pr�ximo argumento, se existir
			if (++i < argc) {
				// tenta converter em um valor inteiro, sem retornar ponteiro para o �ltimo char ap�s o n�mero, na base10
				// se falhar, retorna 0
				n = std::strtol(argv[i], nullptr, 10);
			}
		} else if (argv[i] && !std::strncmp(argv[i], "--dense", 7)) {
			// --dense
			
			// set computeDense flag
			computeDense = true;
		} else if (argv[i] && !std::strncmp(argv[i], "--cloud", 7)) {
			// --cloud <path>

			// incrementa i e extrai o pr�ximo argumento, se existir e set skipCloud flag
			if (++i < argc) {
				cloudPath = fs::path(argv[i]);
				skipCloud = true;
			}
		}
	}
}

// main entry point
int main(int argc, char** argv) {

	// analisa os argumentos da linha de comando
	parse(argc, argv);

	if (!skipCloud) {
		// instanciando objeto PointCloud e computando nuvem de pontos
		PointCloud cloud(path, n);
		cloudPath = fs::path(
			(computeDense) ? cloud.computeDense() : cloud.computeSparse()
		);
	}

	// // instanciando objeto SurfaceReconstruction com caminho at� arquivos exportados da �ltima etapa
	// SurfaceReconstruction recon(cloudPath, computeDense);
	// recon.computeMesh(computeDense);

	return 0;
}
