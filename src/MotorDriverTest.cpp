#include "CANInterface.hpp"

#include "motor_driver/MotorDriver.hpp"

#include <string.h>
#include <iostream>

using namespace std;

int main(int argc, char **argv)
{
	motor_driver::MotorDriver motor_controller(0x01, "can0");
	
	cout<<"Enabling Motor"<<endl;
	motor_driver::motorState state1 = motor_controller.enableMotor();
	cout<<"Position: "<<state1.position<<"Velocity: "<<state1.velocity<<"Torque: "<<state1.torque<<endl;
	
	cout<<"Disabling Motor"<<endl;	
	motor_driver::motorState state2 = motor_controller.disableMotor();
	cout<<"Position: "<<state2.position<<"Velocity: "<<state2.velocity<<"Torque: "<<state2.torque<<endl;
	
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
