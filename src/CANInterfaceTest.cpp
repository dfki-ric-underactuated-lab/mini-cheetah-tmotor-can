#include "CANInterface.hpp"
#include "csv.h"

#include "motor_driver/MotorDriver.hpp"

#include <string.h>
#include <fstream>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <chrono>


int main(int argc, char **argv)
{
    CAN_interface::CANInterface TestCANInterface1 ("vcan0");
    CAN_interface::CANInterface TestCANInterface2 ("vcan0");

    unsigned char CANMsg [8];

    return 0;
}