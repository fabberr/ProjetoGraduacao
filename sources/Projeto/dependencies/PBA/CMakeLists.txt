# No PBA CMake config file is provided by the developer, adding PBA as an Imported Library
# https://cmake.org/cmake/help/latest/command/add_library.html#imported-libraries
message(STATUS "Importing PBA library")
add_library(libpba SHARED IMPORTED GLOBAL)
set_target_properties(
	libpba PROPERTIES 
	LINKER_LANGUAGE CXX 
	IMPORTED_LOCATION "${CMAKE_CURRENT_LIST_DIR}/lib/libpba.so"
)
