# C++ Motor Driver for Mini Cheetah-type Actuators from T-Motor/CubeMars

This driver was developed at the Underactuated Lab in Robotics Innovation Center at DFKI GmbH, Bremen.

It assumes the use of a CAN to USB adapter (such as [PEAK System's PCAN-USB](https://www.peak-system.com/PCAN-USB.199.0.html?&L=1) or [ESD's CAN-USB/2](https://esd.eu/produkte/can-usb-2)) connected to a linux (tested on Ubuntu) computer. The SocketCAN interface is used which is supported natively in the Linux Kernel. For more information on SocketCAN in Linux, see [here](https://www.kernel.org/doc/html/latest/networking/can.html).

The communication send-reply frequencies are set up using the Serial Port of the motor and until now, this C++ driver was tested to achieve over 2KHz with single motor connected.

# Dependencies
It has no external library dependencies as it only depends on the *socket* library provided by Linux and communicates via **SocketCAN** with the motor. Currently, a CAN-USB adapter is used (from either PEAK or ESD GmbH) which shows up as a SocketCAN interface on a Linux computer. `CMake` build system is used for compilation/installation.

# Documentation

- Useful videos: 
    - [From T-Motor](https://www.youtube.com/watch?v=hbqQCgebaF8)
    - [From Skyentific](https://www.youtube.com/watch?v=HzY9vzgPZkA)
- [Motor Datasheets](https://store.cubemars.com/images/file/20220307/1646619452473352.pdf)
- [Ben Katz Documentation](https://docs.google.com/document/d/1dzNVzblz6mqB3eZVEMyi2MtSngALHdgpTaDJIW_BpS4/edit)

# Pre-requisites:

* Setting up the CAN interface:

  * Run this command and make sure that `can0` (or any other can interface depending on the system)shows up as an interface after connecting the USB cable to your laptop: `ip link show`

  * Configure the `can0` interface to have a 1 Mbaud communication frequency: `sudo ip link set can0 type can bitrate 1000000`

  * To bring up the `can0` interface, run: `sudo ip link set up can0`

* To change motor parameters such as CAN ID or to calibrate the encoder, a serial connection is used. The serial terminal used on linux for this purpose is `screen`. Example usage:
```
sudo apt-get install screen
screen /dev/ttyUSB0 921600
```

# Build Instructions
```
mkdir build
cd build
cmake ..
make
sudo make install
```

# Testing with a real motor and Usgae
See `MotorDriverTest.cpp` which is the provided test script. 

Add the following header to your project code:
```
#include "mini_cheetah_motor_driver/MotorDriver.hpp"
```

For the available functions, see `include/motor_driver/MotorDriver.hpp`.

**TODO**: Adapt `MotorDriverTrajTest.cpp` to the new driver interface.

# Using in other projects (ROS2 drivers, etc..)
This driver was tested and used in ROS2 for certain projects. 

**TODO**: Create a generic ROS2 node for the actuators.

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

# Supported Motor Configurations:

- AK80-6 (From Cubemars, Firmware versions V1, V1.1, and V2): `motor_type='AK80_6_V1'`, `motor_type='AK80_6_V1p1'` and `motor_type='AK80_6_V2'`
- AK80-9 (From Cubemars, Firmware version V1.1 and V2): `motor_type='AK80_9_V1p1'` and `motor_type='AK80_9_V2'`
- AK10-9 (From Cubemars, Firmware version V1.1): `motor_type='AK10_9_V1p1'`. **TODO:** The temperature and error codes are received but not yet decoded for the new firmware for this motor.

```
# Working parameters for AK80-6 V1.0 firmware
AK80_6_V1_PARAMS = {
                "P_MIN" : -95.5,
                "P_MAX" : 95.5,
                "V_MIN" : -45.0,
                "V_MAX" : 45.0,
                "KP_MIN" : 0.0,
                "KP_MAX" : 500,
                "KD_MIN" : 0.0,
                "KD_MAX" : 5.0,
                "T_MIN" : -18.0,
                "T_MAX" : 18.0,
                "AXIS_DIRECTION" : -1
                }

# Working parameters for AK80-6 V1.1 firmware
AK80_6_V1p1_PARAMS = {
                "P_MIN" : -12.5,
                "P_MAX" : 12.5,
                "V_MIN" : -22.5,
                "V_MAX" : 22.5,
                "KP_MIN" : 0.0,
                "KP_MAX" : 500,
                "KD_MIN" : 0.0,
                "KD_MAX" : 5.0,
                "T_MIN" : -12.0,
                "T_MAX" : 12.0,
                "AXIS_DIRECTION" : -1
                }

# Working parameters for AK80-6 V2.0 firmware
AK80_6_V2_PARAMS = {
                "P_MIN" : -12.5,
                "P_MAX" : 12.5,
                "V_MIN" : -38.2,
                "V_MAX" : 38.2,
                "KP_MIN" : 0.0,
                "KP_MAX" : 500.0,
                "KD_MIN" : 0.0,
                "KD_MAX" : 5.0,
                "T_MIN" : -12.0,
                "T_MAX" : 12.0,
                "AXIS_DIRECTION" : 1
                }

# Working parameters for AK80-9 V1.1 firmware
AK80_9_V1p1_PARAMS = {
                "P_MIN" : -12.5,
                "P_MAX" : 12.5,
                "V_MIN" : -22.5,
                "V_MAX" : 22.5,
                "KP_MIN" : 0.0,
                "KP_MAX" : 500,
                "KD_MIN" : 0.0,
                "KD_MAX" : 5.0,
                "T_MIN" : -18.0,
                "T_MAX" : 18.0,
                "AXIS_DIRECTION" : 1
                }

# Working parameters for AK80-9 V2.0 firmware
AK80_9_V2_PARAMS = {
                    "P_MIN" : -12.5,
                    "P_MAX" : 12.5,
                    "V_MIN" : -25.64,
                    "V_MAX" : 25.64,
                    "KP_MIN" : 0.0,
                    "KP_MAX" : 500.0,
                    "KD_MIN" : 0.0,
                    "KD_MAX" : 5.0,
                    "T_MIN" : -18.0,
                    "T_MAX" : 18.0,
                    "AXIS_DIRECTION" : 1
                    }

# Working parameters for AK10-9 V1.1 firmware
AK10_9_V1p1_PARAMS = {
                "P_MIN" : -12.5,
                "P_MAX" : 12.5,
                "V_MIN" : -50.0,
                "V_MAX" : 50.0,
                "KP_MIN" : 0.0,
                "KP_MAX" : 500,
                "KD_MIN" : 0.0,
                "KD_MAX" : 5.0,
                "T_MIN" : -65.0,
                "T_MAX" : 65.0,
                "AXIS_DIRECTION" : -1
                }

```

To add a new constants configuration use the `change_motor_constants` function or create an issue with the constants and motor information on the GitHub page to be added to the driver.

# Known Issues

When having 2 motors on the CAN bus with either PCAN CAN-USB or ESD CAN-USB/2, sometimes the motors experience an initial short *kick/impulse* at when they are enabled again after being disabled. One workaround is power cycling them. This is probably due to separate grounds for the power and CAN communication on the motors. As the ground for the CAN can be via the control computer ground and the power ground is via the power supply ground, these can have small voltage differences which can cause the initial kick. This is the current best guess for when experiencing the issue. 

As this is experimental software, there might be other unknown issues.