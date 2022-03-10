message("-- Setting up Visualization Toolkit")

add_library(vtkCommonCore SHARED IMPORTED GLOBAL "${CMAKE_CURRENT_LIST_DIR}/bin/vtkCommonCore-9.0.dll")
add_library(vtkCommonMath SHARED IMPORTED GLOBAL "${CMAKE_CURRENT_LIST_DIR}/bin/vtkCommonMath-9.0.dll")
add_library(vtkloguru     SHARED IMPORTED GLOBAL "${CMAKE_CURRENT_LIST_DIR}/bin/vtkloguru-9.0.dll")

set_target_properties(
	vtkCommonCore PROPERTIES 
	LINKER_LANGUAGE CXX 
	IMPORTED_IMPLIB "${CMAKE_CURRENT_LIST_DIR}/lib/vtkCommonCore-9.0.lib"
)

set_target_properties(
	vtkCommonMath PROPERTIES 
	LINKER_LANGUAGE CXX 
	IMPORTED_IMPLIB "${CMAKE_CURRENT_LIST_DIR}/lib/vtkCommonMath-9.0.lib"
)

set_target_properties(
	vtkloguru PROPERTIES 
	LINKER_LANGUAGE CXX 
	IMPORTED_IMPLIB "${CMAKE_CURRENT_LIST_DIR}/lib/vtkloguru-9.0.lib"
)
