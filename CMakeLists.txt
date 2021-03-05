cmake_minimum_required(VERSION 3.0)

project(mini_cheetah_motor_driver)

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

# Create an Object to Make both Static and Shared Libraries in the same CMakeLists File.
add_library(${PROJECT_NAME}-object OBJECT ${LIBRARY_SOURCE} ${LIBRARY_HEADER})

# this is only needed for older compilers but doesn't hurt either to have it
set_target_properties(${PROJECT_NAME}-object PROPERTIES POSITION_INDEPENDENT_CODE 1)

add_library(${PROJECT_NAME} SHARED $<TARGET_OBJECTS:${PROJECT_NAME}-object>)

add_library(${PROJECT_NAME}-static STATIC $<TARGET_OBJECTS:${PROJECT_NAME}-object>)

add_executable(MotorDriverTest ${SOURCE_DIR}/MotorDriverTest.cpp)

target_link_libraries(MotorDriverTest ${PROJECT_NAME}-static)

INSTALL(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
       )

INSTALL(FILES ${LIBRARY_HEADER}
        DESTINATION include/motor_driver
        )

set(PKGCONFIG_REQUIRES_PRIVATE "")
set(PKGCONFIG_EXTRA_LINK_FLAGS "")
set(PKGCONFIG_LINK_FLAGS_PRIVATE "")

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc @ONLY)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION lib/pkgconfig)
