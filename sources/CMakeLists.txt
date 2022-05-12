#
# Top-level CMakeLists.txt
#

cmake_minimum_required(VERSION 3.21.4)

# Project name
project(ProjetoGraduacao)

# Adding targets
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/pcloud") 			# target: pcloud (executable)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/filter-cloud") 	# target: filter-cloud (executable)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/packt-sfm") 		# target: packt-sfm (executable)
