message("-- Setting up OpenCV")

add_library(
	OpenCV SHARED IMPORTED GLOBAL 
	"${CMAKE_CURRENT_LIST_DIR}/bin/opencv_world450d.dll" # with debug symbols
)
# add_library(
# 	OpenCV SHARED IMPORTED
# 	"${CMAKE_CURRENT_LIST_DIR}/bin/opencv_world450.dll" # release (no debug symbols)
# )

set_target_properties(
	OpenCV PROPERTIES 
	LINKER_LANGUAGE CXX 
	IMPORTED_IMPLIB "${CMAKE_CURRENT_LIST_DIR}/lib/opencv_world450d.lib" 	# with debug symbols
	# IMPORTED_IMPLIB "${CMAKE_CURRENT_LIST_DIR}/lib/opencv_world450.lib" 	# release (no debug symbols)
)
