/********** Headers **********/

// libc++
#include <iostream> 	// i/o streams
#include <fstream> 		// file streams
#include <sstream>	 	// string streams
#include <string> 		// std::string, std::getline
#include <tuple> 		// std::tuple
#include <vector> 		// std::vector
#include <filesystem> 	// filesystem utilities

// libc
#include <cstdlib> 	// std::exit, std::atoi, std::size_t
#include <cstdio> 	// stderr, EXIT_FAILURE, EXIT_SUCCESS
#include <cmath> 	// std::sqrt, std::pow
#include <cstring> 	// std::strncmp
#include <cstdint> 	// width-based integer types

namespace fs = std::filesystem; // filesystem namespace alias

// internos
#include <callable_utils.h>

/********** main.cpp **********/

/** Lógica de controle. */
namespace ctrl {
	/**
	 * Analisa e extrai os argumentos da linha de comando.
	 * Se o Número de argumentos for menor que o mínimo ou se o argumento obrigatório
	 * não for válido, termina a execução do programa.
	 *
	 * @returns Um std::tuple contendo os argumentos extraídos e validados na ordem: 
	 *          `{ <path_to_obj_file>, <threshold=15> }`.
	*/
	std::tuple<fs::path, std::uint16_t> parse_cmd_line(int argc, const char* const* argv) {		
		/** 
		 * Functor:
		 * Valida o caminho até o arquivo de entrada.
		 * 
		 * @returns `true` se o caminho é válido.
		 */
		struct validate_path {
			const fs::path& p;
			explicit operator bool() const {
				return (
					fs::exists(p) && fs::is_regular_file(p) && 
					(std::strncmp(p.extension().c_str(), ".obj", 4) == 0)
				);
			}
		};

		// inicializando variáveis de controle com valores padrão
		fs::path path{};
		const std::uint16_t threshold_default = 15;
		uint16_t threshold = threshold_default;

		// verificando mínimo de argumentos
		if (argc < 2) {
			std::cout <<
				"Uso: " << argv[0] << " <path_to_obj_file> [threshold]\n"
				"    Filtra uma nuvem de pontos calculando a Distância Euclidiana entre cada um\n"
				"    dos pontos até a origem. Pontos que excedem o limiar serão descartados.\n"
				"    A nuvem filtrada sera exportada como `<path_to_obj_file>_filtered.obj`\n"
				"\n"
				"<path_to_obj_file>:\n"
				"    Caminho até um arquivo .obj seguindo o schema: {\n"
				"        # <número de vértices>\n"
				"        v <x> <y <z>\n"
				"        ...\n"
				"    }\n"
				"\n"
				"[threshold]:\n"
				"    Limiar da busca, pontos os quais excederem este limiar de distância do ponto\n"
				"    origem (0, 0, 0), serão descartados.\n"
				"    Deve ser um valor inteiro não sinalinzado no intervalo (0, UINT16_MAX].\n"
				"    Opcional, 15 por padrão.\n"
			<< std::endl;
			std::exit(EXIT_SUCCESS);
		}

		// extraindo e validando argumento obrigatório <path_to_obj_file>
		if (path = argv[1]; !validate_path{path}) {
			std::fprintf(stderr, "[ERRO] Caminho até o arquivo .obj `%s` não é válido\n", argv[1]);
			std::exit(EXIT_FAILURE);
		}

		// verificando argumento opcional [threshold]
		if (argc > 2) {
			threshold = std::atoi(argv[2]);
			threshold = threshold == 0 ? threshold_default : threshold;
		}

		return { path, threshold };
	}
} // namespace ctrl

/** Filtra a nuvem de pontos. */
void filter_cloud(const fs::path& input_path, const int& threshold) {

	/** Armazena coordenadas em um espaço 3D. */
	typedef struct point3f { float x, y, z; } point3f_t;

	/**
	 * Helper function:
	 * Retorna a Distância Euclidiana em valor integral entre um ponto no espaço 3D
	 * e a origem (0, 0, 0).
	 * Valores reais serão truncados ao maior valor inteiro menor que o resultado.
	*/
	const auto dist_from_origin = [](const point3f_t& pt) {
		return static_cast<std::uint32_t>(std::floor(std::sqrt(std::pow(0.f - pt.x, 2.f) + std::pow(0.f - pt.y, 2.f) + std::pow(0.f - pt.z, 2.f))));
	};

	/**
	 * Helper function:
	 * Tenta abrir o arquivo `filename` no modo `mode` e retorna a fstream aberta.
	 * Se falhar, termina a execução do programa com código EXIT_FAILURE.
	*/
	const auto try_open = [](const fs::path& filename, std::ios_base::openmode mode) {
		if (std::fstream file{filename, mode}; file) {
			return file;
		} else {
			std::cerr << "[ERRO] Não foi possível abrir arquivo `" << filename.c_str() << "`\n";
			std::exit(EXIT_FAILURE);
		}
	};
	
	/**
	 * Helper function:
	 * Processa a nuvem de pontos, eliminando os pontos mais distantes da origem.
	 * Os arquivos passados por referência serão fechados.
	*/
	const auto filter_cloud = [&input_path, &threshold, &dist_from_origin](std::fstream& input_cloud, std::fstream& output_cloud) {

		std::vector<point3f_t> cloud_filtered{};
		std::size_t size, size_cpy;
		input_cloud.get(); 		// ignora `#`
		input_cloud >> size; 	// número de vértices
		size_cpy = size;

		std::cout << 
			"Processando nuvem `" << input_path.c_str() << "`\n" 
			"    Pontos = " << size << "\n" 
			"    threshold = " << threshold << "\n" 
		<< std::endl;
		cloud_filtered.reserve(size);

		// lista de vértices, formato: `v <x> <y> <z>\n`
		for (std::string line{}; std::getline(input_cloud, line); ) {
			std::istringstream ss{line};
			point3f_t pt{};

			ss.get(); 					// ignora `v`
			ss >> pt.x >> pt.y >> pt.z; // coordenadoas do ponto
			
			// filtra a nuvem
			if (dist_from_origin(pt) <= threshold) {
				cloud_filtered.emplace_back(pt);
			} else {
				--size;
			}
		}
		std::cout << "Pontos descartados: " << size_cpy - size << '\n';
		input_cloud.close();
		cloud_filtered.shrink_to_fit();

		// salva a nuvem filtrada
		output_cloud << "# " << cloud_filtered.size() << '\n'; 	// número de vértices
		for (const auto& pt : cloud_filtered) { 				// lista de vértices
			output_cloud << "v " << pt.x << ' ' << pt.y << ' ' << pt.z << '\n';
		}
		output_cloud.close();
		cloud_filtered.clear();
	};

	// abrir arquivos de entrada e saída
	fs::path output_path{input_path.parent_path()/input_path.stem() += "_filtered.obj"};
	std::fstream input_cloud = try_open(input_path, std::ios_base::in);
	std::fstream output_cloud = try_open(output_path, std::ios_base::out);

	// processa a nuvem de pontos
	filter_cloud(input_cloud, output_cloud);
	std::cout << "Nuvem filtrada salva em: `" << output_path.c_str() << "`\n";
}

// ./filter-cloud output/gargoyle/point_cloud.obj
int main(int argc, char** argv) {
	utils::invoke(filter_cloud, ctrl::parse_cmd_line(argc, argv));
	return 0;
}
