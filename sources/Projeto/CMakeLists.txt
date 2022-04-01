# Target name
project(Projeto)

# Creating executable target
add_executable(
	${PROJECT_NAME} 
	"${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/src/Camera.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/src/Graph.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/src/Keypoint.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/src/ParallelBundleAdjustment.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/src/PointCloud.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/src/Seed.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/src/StereoPair.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/src/Track.cpp"
)

# Setting target's properties
set_target_properties(
	${PROJECT_NAME} PROPERTIES
	LINKER_LANGUAGE CXX
	CXX_STANDARD 20
)

# Instruct basic signature of `find_package` function to fall back to "Config" 
# mode immediately instead of trying "Module" mode first since no libraries used
# provide a Find Module
set(CMAKE_FIND_PACKAGE_PREFER_CONFIG TRUE)

# Setting up OpenCV
message(STATUS "Searching for OpenCV package")
find_package(
	OpenCV 4.5.5 REQUIRED 
	COMPONENTS core imgcodecs imgproc features2d flann calib3d
)

# Setting up VTK
message(STATUS "Searching for VTK package")
find_package(
	VTK 9.1.0 REQUIRED 
	COMPONENTS CommonCore CommonMath kissfft loguru vtksys
)

# Setting up PBA
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/dependencies/PBA")

# Setting include directories and linking libraries
target_include_directories(
	${PROJECT_NAME} 
	PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include" 					# internal header files
	PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/PBA/include" 	# pba/
	PRIVATE "/usr/local/include" 									# vtk-9.1/
	PRIVATE "/usr/local/include/opencv4" 							# opencv2/
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${VTK_LIBRARIES} libpba)
