message("-- Setting up Parallel Bundle Adjustment shared library")

add_library(pba SHARED IMPORTED GLOBAL)
set_target_properties(
	pba PROPERTIES 
	LINKER_LANGUAGE CXX 
	IMPORTED_IMPLIB "${CMAKE_CURRENT_LIST_DIR}/lib/pba_x64.lib" 
	IMPORTED_LOCATION "${CMAKE_CURRENT_LIST_DIR}/bin/pba_x64.dll"
)
