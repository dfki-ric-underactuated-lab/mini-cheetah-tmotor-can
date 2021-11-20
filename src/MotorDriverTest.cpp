#include "CANInterface.hpp"
#include "csv.h"

#include "motor_driver/MotorDriver.hpp"

#include <string.h>
#include <fstream>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>

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
    vector<int> motor_ids = {0x06, 0x07};
	motor_driver::MotorDriver motor_controller(motor_ids, "can0", motor_driver::MotorType::AK80_6_V2);
	
	cout<<"Enabling Motor..."<<endl;
	auto start_state = motor_controller.enableMotor(motor_ids);
	cout<<"Position: "<<start_state[motor_ids[0]].position<<" Velocity: "<<start_state[motor_ids[0]].velocity<<" Torque: "<<start_state[motor_ids[0]].torque<<endl;
	cout<<"Position: "<<start_state[motor_ids[1]].position<<" Velocity: "<<start_state[motor_ids[1]].velocity<<" Torque: "<<start_state[motor_ids[1]].torque<<endl;
	
    cout<<"Setting Zero Position..."<<endl;
    auto stateZero = motor_controller.setZeroPosition(motor_ids);
    cout<<"Position: "<<stateZero[motor_ids[0]].position<<" Velocity: "<<stateZero[motor_ids[0]].velocity<<" Torque: "<<stateZero[motor_ids[0]].torque<<endl; 
	cout<<"Position: "<<stateZero[motor_ids[1]].position<<" Velocity: "<<stateZero[motor_ids[1]].velocity<<" Torque: "<<stateZero[motor_ids[1]].torque<<endl; 
	
	motor_driver::motorCommand commandStruct = {1.57, 0, 50, 2, 0}; 

	std::map<int, motor_driver::motorCommand> commandMap = {{6, commandStruct}, {7, commandStruct}};
	auto startT = get_time_in_microseconds();
	auto commandState = motor_controller.sendRadCommand(commandMap);
	auto endT = get_time_in_microseconds();

	auto dt = (endT - startT);

	std::cout << "Time Taken for Command: " << dt << std::endl; 

	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	cout<<"Disabling Motor..."<<endl;	
	auto end_state = motor_controller.disableMotor(motor_ids);
	cout<<"Position: "<<end_state[motor_ids[0]].position<<" Velocity: "<<end_state[motor_ids[0]].velocity<<" Torque: "<<end_state[motor_ids[0]].torque<<endl;
	cout<<"Position: "<<end_state[motor_ids[1]].position<<" Velocity: "<<end_state[motor_ids[1]].velocity<<" Torque: "<<end_state[motor_ids[1]].torque<<endl;
	
    return 0;
}
