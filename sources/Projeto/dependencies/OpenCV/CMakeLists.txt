message("-- Setting up OpenCV shared library")

add_library(opencv SHARED IMPORTED GLOBAL)
set_target_properties(
	opencv PROPERTIES 
	LINKER_LANGUAGE CXX 
	IMPORTED_IMPLIB "${CMAKE_CURRENT_LIST_DIR}/lib/opencv_world455d.lib" 
	IMPORTED_LOCATION "${CMAKE_CURRENT_LIST_DIR}/bin/opencv_world455d.dll"
)
