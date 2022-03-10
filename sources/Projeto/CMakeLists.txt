# Target name
project(Projeto)

# Creating executable target
add_executable(
	${PROJECT_NAME} 
	"${CMAKE_CURRENT_LIST_DIR}/src/main.cpp "
	"${CMAKE_CURRENT_LIST_DIR}/src/Camera.cpp "
	"${CMAKE_CURRENT_LIST_DIR}/src/Graph.cpp "
	"${CMAKE_CURRENT_LIST_DIR}/src/Keypoint.cpp "
	"${CMAKE_CURRENT_LIST_DIR}/src/ParallelBundleAdjustment.cpp "
	"${CMAKE_CURRENT_LIST_DIR}/src/PointCloud.cpp "
	"${CMAKE_CURRENT_LIST_DIR}/src/Seed.cpp "
	"${CMAKE_CURRENT_LIST_DIR}/src/StereoPair.cpp "
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
	"${CMAKE_CURRENT_LIST_DIR}/libs/OpenCV/include/" 	# OpenCV 4.5.0
	"${CMAKE_CURRENT_LIST_DIR}/libs/VTK/include/" 		# VTK 9.0.1
	"${CMAKE_CURRENT_LIST_DIR}/libs/PBA/include/" 		# PBA 1.0.5
	"${CMAKE_CURRENT_LIST_DIR}/include/"				# internal header files
)

# Adding dependency subdirectories, each of these adds one or more imported 
# shared library targets to be passed to `target_link_libraries()`
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/libs/OpenCV/")
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/libs/PBA/")
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/libs/VTK/")

# Linking libraries
target_link_libraries(${PROJECT_NAME}
	PUBLIC OpenCV 
	PUBLIC PBA 
	PUBLIC vtkCommonCore 
	PUBLIC vtkCommonMath 
	PUBLIC vtkloguru
)
