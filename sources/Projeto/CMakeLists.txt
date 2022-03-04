#
# CMake target: Projeto (executable)
#

# Target name
project(Projeto)

# Creating executable target
add_executable(
	${PROJECT_NAME} 
	./src/main.cpp
)

# Setting target's properties
set_target_properties(
	${PROJECT_NAME} PROPERTIES
	LINKER_LANGUAGE CXX
	CXX_STANDARD 20
)

# Setting target's include dirs
target_include_directories(
	${PROJECT_NAME} PUBLIC
	C:/libs/opencv/build/include 	# OpenCV 4.5.0
	C:/libs/vtk/VTK-9.0.1/include 	# VTK 9.0.1
	C:/libs/pba/include 			# PBA 1.0.5
	. 								# internal header files
)

# Linking Libraries
target_link_libraries(
	${PROJECT_NAME}
	PUBLIC C:/libs/opencv/build/x64/vc15/bin/opencv_world450.dll
	PUBLIC C:/libs/vtk/VTK-9.0.1/bin/vtkCommonCore-9.0.dll
	PUBLIC C:/libs/vtk/VTK-9.0.1/bin/vtkCommonMath-9.0.dll
	PUBLIC C:/libs/vtk/VTK-9.0.1/bin/vtkloguru-9.0.dll
	PUBLIC C:/libs/pba/bin/pba.dll
)
