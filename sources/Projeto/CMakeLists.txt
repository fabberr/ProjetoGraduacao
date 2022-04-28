# Target name
project(Projeto)

# Setting variables
set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")

# Creating executable target
add_executable(
	${PROJECT_NAME} 
	"${INCLUDE_DIR}/logger.h" 
	"${SRC_DIR}/main.cpp" 
	"${SRC_DIR}/cmdline_parser.cpp" 
	"${SRC_DIR}/sfmPointCloud.cpp"
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
	OpenCV 3.4 REQUIRED 
	COMPONENTS core imgcodecs imgproc features2d flann calib3d xfeatures2d sfm
)

# Setting include directories and linking libraries
target_include_directories(
	${PROJECT_NAME} 
	PRIVATE ${INCLUDE_DIR} # internal header files
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
