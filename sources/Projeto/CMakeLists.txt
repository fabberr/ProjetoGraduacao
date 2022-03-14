# Target name
project(Projeto)

# Creating executable target
add_executable(
	${PROJECT_NAME} 
	"${CMAKE_CURRENT_LIST_DIR}/src/main.cpp"
	"${CMAKE_CURRENT_LIST_DIR}/src/Camera.cpp"
	"${CMAKE_CURRENT_LIST_DIR}/src/Graph.cpp"
	"${CMAKE_CURRENT_LIST_DIR}/src/Keypoint.cpp"
	"${CMAKE_CURRENT_LIST_DIR}/src/ParallelBundleAdjustment.cpp"
	"${CMAKE_CURRENT_LIST_DIR}/src/PointCloud.cpp"
	"${CMAKE_CURRENT_LIST_DIR}/src/Seed.cpp"
	"${CMAKE_CURRENT_LIST_DIR}/src/StereoPair.cpp"
	"${CMAKE_CURRENT_LIST_DIR}/src/Track.cpp"
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
	"${CMAKE_CURRENT_LIST_DIR}/dependencies/OpenCV/include/" 	# OpenCV 4.5.5
	"${CMAKE_CURRENT_LIST_DIR}/dependencies/VTK/include/" 		# VTK 9.1.0
	"${CMAKE_CURRENT_LIST_DIR}/dependencies/PBA/include/" 		# PBA 1.0.5
	"${CMAKE_CURRENT_LIST_DIR}/include/"						# internal header files
)

# Adding dependencies, each directory adds one or more imported shared library 
# targets to be passed to `target_link_libraries()` call
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/dependencies/OpenCV/")
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/dependencies/PBA/")
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/dependencies/VTK/")

# Linking libraries with "Library Target Names" signature specified in: 
# <https://cmake.org/cmake/help/latest/command/target_link_libraries.html#overview>
target_link_libraries(${PROJECT_NAME}
	PUBLIC opencv 
	PUBLIC pba 
	PUBLIC vtkCommonCore 
	PUBLIC vtkCommonMath 
	PUBLIC vtkkissfft 
	PUBLIC vtkloguru 
	PUBLIC vtksys
)
