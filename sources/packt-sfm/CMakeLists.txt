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
find_package(
	OpenCV 3.4 REQUIRED 
	COMPONENTS core imgproc imgcodecs calib3d viz features2d xfeatures2d sfm
)

# Setting up Boost
find_package(Boost REQUIRED filesystem graph iostreams program_options system serialization)

# Setting up OpenMVS
find_package(OpenMVS REQUIRED)

# Setting up include directories, definitions and linking libraries
target_include_directories(
	${PROJECT_NAME} 
	PRIVATE ${INCLUDE_DIR} 			# internal header files
	PRIVATE ${Boost_INCLUDE_DIR} 	# libboost
	PRIVATE ${OpenMVS_INCLUDE_DIRS} # libopenmvs
)
target_compile_definitions(${PROJECT_NAME} 
	PRIVATE -DCERES_FOUND
	PRIVATE ${OpenMVS_DEFINITIONS}
)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${Boost_LIBRARIES} OpenMVS::MVS)
