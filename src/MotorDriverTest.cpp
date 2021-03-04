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
	cout<<"Position: "<<state1.position<<" Velocity: "<<state1.velocity<<" Torque: "<<state1.torque<<endl;
	
    cout<<"Setting Zero Position..."<<endl;
    motor_driver::motorState stateZero = motor_controller.setZeroPosition();

    cout<<"Position: "<<stateZero.position<<" Velocity: "<<stateZero.velocity<<" Torque: "<<stateZero.torque<<endl; 

	cout<<"Disabling Motor"<<endl;	
	motor_driver::motorState state2 = motor_controller.disableMotor();
	cout<<"Position: "<<state2.position<<" Velocity: "<<state2.velocity<<" Torque: "<<state2.torque<<endl;

    return 0;
}
