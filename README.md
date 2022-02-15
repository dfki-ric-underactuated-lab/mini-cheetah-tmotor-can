# C++ Motor Driver

This repository contains the C++ driver for the T-Motor mini-cheetah type actuators. Currently, the following motors are supported: AK80-6 Version 1.0, AK80-6 Version 1.1, AK80-6 Version 2.0, AK80-9 Version 1.1, and AK80-9 Version 2.0

## Dependencies
It has no external library dependencies as it only depends on the *socket* library provided by Linux and communicates via **SocketCAN** with the motor. Currently, a CAN-USB adapter is used (from either PEAK or ESD GmbH) which shows up as a SocketCAN interface on a Linux computer. 

## Build Instructions
```
mkdir build
cd build
cmake ..
make
sudo make install
```

## Testing with a real motor
See `MotorDriverTest.cpp` which is the provided test script. 

**TODO**: Adapt `MotorDriverTrajTest.cpp` to the new driver.


## Using in other projects (ROS2 drivers, etc..)
In the project's `CMakeLists.txt` add the following lines after compiling and installing the library on the computer:
```
link_directories(/usr/local/lib)
find_library(mini_cheetah_motor_driver REQUIRED)
target_link_libraries(${YOUR_TARGET} mini_cheetah_motor_driver ${OTHER_LIBS})
```
And add the following header to your project code:
```
#include "mini_cheetah_motor_driver/MotorDriver.hpp"
```