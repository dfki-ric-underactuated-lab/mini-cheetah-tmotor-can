cmake_minimum_required(VERSION 3.0)

project(motor_driver)

set (CMAKE_CXX_STANDARD 11)

set(SOURCE_DIR ${PROJECT_SOURCE_DIR}/src)
set(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include/motor_driver)
set(lib_INCLUDE_DIR ${INCLUDE_DIR} CACHE INTERNAL "")

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${INCLUDE_DIR})

include_directories(
	            ${PROJECT_SOURCE_DIR}/include            
	            ${PROJECT_SOURCE_DIR}/src            
	            )

set(LIBRARY_NAME ${PROJECT_NAME} CACHE INTERNAL "")

set(LIBRARY_SOURCE ${SOURCE_DIR}/MotorDriver.cpp
	           ${SOURCE_DIR}/CANInterface.cpp)

set(LIBRARY_HEADER ${INCLUDE_DIR}/MotorDriver.hpp
	           ${INCLUDE_DIR}/CANInterface.hpp
		   ${INCLUDE_DIR}/CANCommand.hpp
	           ${INCLUDE_DIR}/Parameters.hpp)

add_library(${PROJECT_NAME} ${LIBRARY_SOURCE} ${LIBRARY_HEADER})

INSTALL(TARGETS ${PROJECT_NAME}
	LIBRARY DESTINATION lib
	PUBLIC_HEADER DESTINATION include)

