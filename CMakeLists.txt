cmake_minimum_required(VERSION 3.0)

project(mini_cheetah_motor_driver)

if(WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
else(WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif(WIN32)


set (CMAKE_CXX_STANDARD 11)

include(GNUInstallDirs)

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

# find threads - needed for header only csv parser used in this project
find_package (Threads)

# Create an Object to Make both Static and Shared Libraries in the same CMakeLists File.
add_library(${PROJECT_NAME}-object OBJECT ${LIBRARY_SOURCE})

# this is only needed for older compilers but doesn't hurt either to have it
set_target_properties(${PROJECT_NAME}-object PROPERTIES POSITION_INDEPENDENT_CODE 1)

add_library(${PROJECT_NAME} SHARED $<TARGET_OBJECTS:${PROJECT_NAME}-object>)

add_library(${PROJECT_NAME}-static STATIC $<TARGET_OBJECTS:${PROJECT_NAME}-object>)

# Add execuatble for a simple on/off/zero position driver test
add_executable(MotorDriverTest ${SOURCE_DIR}/MotorDriverTest.cpp)
target_link_libraries(MotorDriverTest ${PROJECT_NAME}-static ${CMAKE_THREAD_LIBS_INIT})

# Add execuatble for a trajectory following test
# add_executable(MotorTrajTest ${SOURCE_DIR}/MotorDriverTrajTest.cpp)
# target_link_libraries(MotorTrajTest ${PROJECT_NAME}-static ${CMAKE_THREAD_LIBS_INIT})

# Add PUBLIC_HEADERS so that other applications on the computer can use this library.
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "include/motor_driver/MotorDriver.hpp;include/motor_driver/CANInterface.hpp")

INSTALL(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME} 
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} 
        ARCHIVE DESTINATION lib
       )