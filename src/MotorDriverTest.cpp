#include "CANInterface.hpp"
#include "csv.h"

#include "motor_driver/MotorDriver.hpp"

#include <string.h>
#include <fstream>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <chrono>

using namespace std;

using namespace std::chrono;

// Get time stamp in microseconds.
uint64_t get_time_in_microseconds()
{
    uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::
                  now().time_since_epoch()).count();
    return us; 
}

int main(int argc, char **argv)
{
    vector<int> motor_ids = {0x01};
	motor_driver::MotorDriver motor_controller(motor_ids, "can0");
	
	// cout<<"Enabling Motor..."<<endl;
	// motor_driver::motorState start_state = motor_controller.enableMotor();
	// cout<<"Position: "<<start_state.position<<" Velocity: "<<start_state.velocity<<" Torque: "<<start_state.torque<<endl;
	
 //    cout<<"Setting Zero Position..."<<endl;
 //    motor_driver::motorState stateZero = motor_controller.setZeroPosition();
 //    cout<<"Position: "<<stateZero.position<<" Velocity: "<<stateZero.velocity<<" Torque: "<<stateZero.torque<<endl; 
	
	// cout<<"Disabling Motor..."<<endl;	
	// motor_driver::motorState end_state = motor_controller.disableMotor();
	// cout<<"Position: "<<end_state.position<<" Velocity: "<<end_state.velocity<<" Torque: "<<end_state.torque<<endl;
	
    return 0;
}
