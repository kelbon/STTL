cmake_minimum_required(VERSION 3.21)
project(STTL) # standard template types library

include(CTest)

if(BUILD_TESTING)
	set(${PROJECT_NAME}_TEST ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_traits.cpp)
endif()
file(GLOB ${PROJECT_NAME}_SRCES ${CMAKE_CURRENT_SOURCE_DIR}/include/* )
add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SRCES} ${${PROJECT_NAME}_TEST})
add_library(${PROJECT_NAME}lib INTERFACE ${${PROJECT_NAME}_SRCES})
target_include_directories(${PROJECT_NAME}lib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

source_group(Include REGULAR_EXPRESSION ${CMAKE_CURRENT_SOURCE_DIR}/include/*)
if(BUILD_TESTING)
	source_group(Tests FILES ${${PROJECT_NAME}_TEST})
	add_subdirectory(tests)
endif()
set_target_properties(${PROJECT_NAME} ${PROJECT_NAME}lib PROPERTIES
						CMAKE_CXX_STANDARD_REQUIRED ON
						CXX_STANDARD 23
						CMAKE_CXX_EXTENSIONS OFF
						LINKER_LANGUAGE CXX)