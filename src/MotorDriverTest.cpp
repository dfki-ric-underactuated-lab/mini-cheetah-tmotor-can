#include "CANInterface.hpp"

#include "motor_driver/MotorDriver.hpp"

#include <string.h>
#include <iostream>

int main(int argc, char **argv)
{
    // std::cout << "Test Executable" << std::endl;
    // CAN_interface::CANInterface TestCANInterface ("vcan0");
    // motor_driver::MotorDriver TestMotor;

    // std::cout << "Ready to Test CAN Message on vcan0. Press Enter..." << std::endl; 
    // getchar();

    // uint32_t can_id = 0x01;
    // unsigned char CANMsgSend [] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    // if (TestCANInterface.sendCANFrame(can_id, CANMsgSend)) {
    //     std::cout << "Sent Message" << std::endl;
    // }
    // else {
    //     std::cout << "Error Sending Message" << std::endl;
    // }

    // std::cout << "Waiting to recevie a CAN Message on vcan0..." << std::endl; 

    // unsigned char CANMsg [8];
    // if (TestCANInterface.receiveCANFrame(can_id, CANMsg)) {
    //     std::cout << "Received Message" << std::endl;
    //     for (int i = 0; i < 8; i++)
    //     {
    //         printf("%02X ",CANMsg[i]);
    //     }
    // }
    // else {
    //     std::cout << "Error Receiving Message" << std::endl;
    // }

    return 0;
}