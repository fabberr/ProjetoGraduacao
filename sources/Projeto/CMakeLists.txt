# Target name
project(Projeto)

# Setting variables
set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")

# Creating executable target
add_executable(
	${PROJECT_NAME} 
	"${SRC_DIR}/main.cpp"
	"${SRC_DIR}/cmdline_parser.cpp"
	"${SRC_DIR}/Camera.cpp"
	"${SRC_DIR}/Graph.cpp"
	"${SRC_DIR}/Keypoint.cpp"
	"${SRC_DIR}/ParallelBundleAdjustment.cpp"
	"${SRC_DIR}/PointCloud.cpp"
	"${SRC_DIR}/Seed.cpp"
	"${SRC_DIR}/StereoPair.cpp"
	"${SRC_DIR}/Track.cpp"
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
	COMPONENTS core imgcodecs imgproc features2d flann calib3d xfeatures2d sfm
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
	PRIVATE ${INCLUDE_DIR} 											# internal header files
	PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/PBA/include" 	# pba/
	PRIVATE "/usr/local/include" 									# vtk-9.1/
	PRIVATE "/usr/local/include/opencv4" 							# opencv2/
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${VTK_LIBRARIES} libpba)
