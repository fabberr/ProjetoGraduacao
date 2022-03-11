message("-- Setting up OpenCV shared libraries")

add_library(opencv_core SHARED IMPORTED GLOBAL)
set_target_properties(
	opencv_core PROPERTIES 
	LINKER_LANGUAGE CXX 
	IMPORTED_IMPLIB "${CMAKE_CURRENT_LIST_DIR}/lib/libopencv_core455.dll.a" 
	IMPORTED_LOCATION "${CMAKE_CURRENT_LIST_DIR}/bin/libopencv_core455.dll"
)
