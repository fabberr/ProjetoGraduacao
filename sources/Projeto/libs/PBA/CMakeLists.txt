message("-- Setting up Parallel Bundle Adjustment")

add_library(
	PBA SHARED IMPORTED GLOBAL 
	"${CMAKE_CURRENT_LIST_DIR}/bin/pba_x64.dll"
)

set_target_properties(
	PBA PROPERTIES 
	LINKER_LANGUAGE CXX 
	IMPORTED_IMPLIB "${CMAKE_CURRENT_LIST_DIR}/lib/pba_x64.lib"
)
