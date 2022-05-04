#ifndef IO_H
#define IO_H

/********** Headers **********/

// libc++

#include <iostream> // i/o streams
#include <fstream> // i/o file streams
#include <functional> // function objects support
#include <filesystem> // filesystem utilities

namespace fs = std::filesystem; // filesystem namespace alias

/********** io.h **********/

/** Fornece funcionalidade I/O em arquivos. */
namespace io {

	/********** Funções (declaração) **********/

	bool import_from_file(const fs::path& filename, const std::function<bool(std::ifstream&)>& read_fn);
	bool export_to_file(const fs::path& filename, const std::function<bool(std::ofstream&)>& write_fn);
	
} // namespace io

#endif // IO_H
