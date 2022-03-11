message("-- Setting up OpenCV shared libraries")

add_library(opencv_core SHARED IMPORTED GLOBAL)
set_target_properties(
	opencv_core PROPERTIES 
	LINKER_LANGUAGE CXX 
	IMPORTED_IMPLIB "${CMAKE_CURRENT_LIST_DIR}/lib/libopencv_core455.dll.a" 
	IMPORTED_LOCATION "${CMAKE_CURRENT_LIST_DIR}/bin/libopencv_core455.dll"
)

add_library(opencv_imgcodecs SHARED IMPORTED GLOBAL)
set_target_properties(
	opencv_imgcodecs PROPERTIES 
	LINKER_LANGUAGE CXX 
	IMPORTED_IMPLIB "${CMAKE_CURRENT_LIST_DIR}/lib/libopencv_imgcodecs455.dll.a" 
	IMPORTED_LOCATION "${CMAKE_CURRENT_LIST_DIR}/bin/libopencv_imgcodecs455.dll"
)

add_library(opencv_features2d SHARED IMPORTED GLOBAL)
set_target_properties(
	opencv_features2d PROPERTIES 
	LINKER_LANGUAGE CXX 
	IMPORTED_IMPLIB "${CMAKE_CURRENT_LIST_DIR}/lib/libopencv_features2d455.dll.a" 
	IMPORTED_LOCATION "${CMAKE_CURRENT_LIST_DIR}/bin/libopencv_features2d455.dll"
)

add_library(opencv_calib3d SHARED IMPORTED GLOBAL)
set_target_properties(
	opencv_calib3d PROPERTIES 
	LINKER_LANGUAGE CXX 
	IMPORTED_IMPLIB "${CMAKE_CURRENT_LIST_DIR}/lib/libopencv_calib3d455.dll.a" 
	IMPORTED_LOCATION "${CMAKE_CURRENT_LIST_DIR}/bin/libopencv_calib3d455.dll"
)

add_library(opencv_imgproc SHARED IMPORTED GLOBAL)
set_target_properties(
	opencv_imgproc PROPERTIES 
	LINKER_LANGUAGE CXX 
	IMPORTED_IMPLIB "${CMAKE_CURRENT_LIST_DIR}/lib/libopencv_imgproc455.dll.a" 
	IMPORTED_LOCATION "${CMAKE_CURRENT_LIST_DIR}/bin/libopencv_imgproc455.dll"
)
