# Target name
project(packt-sfm)

# Setting variables
set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")

# Creating executable target
add_executable(
	${PROJECT_NAME} 
	"${SRC_DIR}/main.cpp"
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
	COMPONENTS core imgproc imgcodecs calib3d viz features2d xfeatures2d sfm
)

# Setting up Eigen
find_package(Eigen3 REQUIRED)

# Setting up OpenMVS
find_package(OpenMVS REQUIRED)

# Setting up Boost
find_package(Boost REQUIRED filesystem graph)

# Setting include directories and linking libraries
target_include_directories(
	${PROJECT_NAME} 
	PRIVATE ${INCLUDE_DIR} 			# internal header files
	PRIVATE ${EIGEN3_INCLUDE_DIR} 	# libeigen3
	PRIVATE ${OpenMVS_INCLUDE_DIRS} # libopenmvs
	PRIVATE ${Boost_INCLUDE_DIR} 	# libboost
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${OpenMVS_LIBRARIES})
