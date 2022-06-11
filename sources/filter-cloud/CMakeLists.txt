# Target name
project(filter-cloud)

# Setting variables
set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")

# Creating executable target
add_executable(
	${PROJECT_NAME} 
	"${SRC_DIR}/main.cpp"
)

# Seting target properties
set_target_properties(
	${PROJECT_NAME} PROPERTIES 
	LINKER_LANGUAGE CXX 
	CXX_STANDARD 20
)

# Setting include directories
target_include_directories(
	${PROJECT_NAME} 
	PRIVATE "${INCLUDE_DIR}" # internal header files
)
