#ifndef LOGGER_H
#define LOGGER_H

/********** Headers **********/

// libc
#include <cstdlib> 	// exit, EXIT_FAILURE, EXIT_SUCCESS
#include <cstdio> 	// fprintf, stdout, stderr

/********** logger.h **********/

#define log_error_and_exit(msg, ...) 	std::fprintf(stderr, "[ERRO]: " msg __VA_OPT__(,) __VA_ARGS__); std::exit(EXIT_FAILURE)
#define log_info(msg, ...) 				std::fprintf(stdout, "[INFO]: " msg __VA_OPT__(,) __VA_ARGS__)

#endif // LOGGER_H
