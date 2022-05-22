# Target name
project(mesh)

# Setting variables
set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")

# Creating executable target
add_executable(
	${PROJECT_NAME} 
	"${INCLUDE_DIR}/logger.h"
	"${SRC_DIR}/main.cpp"
	"${SRC_DIR}/SurfaceRecon.cpp"
)

# Setting target's properties
set_target_properties(
	${PROJECT_NAME} PROPERTIES
	LINKER_LANGUAGE CXX
	CXX_STANDARD 20
)

# Setting up PCL
find_package(
	PCL 1.12 REQUIRED
	COMPONENTS common io surface features
)

# Setting up include directories, definitions and linking libraries
target_include_directories(
	${PROJECT_NAME} 
	PRIVATE ${INCLUDE_DIR} 		# internal header files
	PRIVATE ${PCL_INCLUDE_DIRS} # libpcl
)
target_link_directories(
	${PROJECT_NAME}
	PRIVATE ${PCL_LIBRARY_DIRS}
)
target_compile_definitions(${PROJECT_NAME} 
	PRIVATE ${PCL_DEFINITIONS}
)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
