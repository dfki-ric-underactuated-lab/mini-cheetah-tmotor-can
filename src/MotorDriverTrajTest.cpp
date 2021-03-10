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

void extract_joint_traj(string filename, std::vector<float>& time_vec, std::vector<float>& pos_vec, std::vector<float>& vel_vec, std::vector<float>& effort_vec)
{
	io::CSVReader<4> in(filename);
	in.read_header(io::ignore_extra_column, 
	"time", "pos", "vel", "torque");

	float time,pos,vel,effort;	
			  
	while(in.read_row(time, pos, vel, effort)){
		time_vec.push_back(time);	
		pos_vec.push_back(pos);
		vel_vec.push_back(vel);
		effort_vec.push_back(effort);	
	}  
}

// Get time stamp in microseconds.
uint64_t get_time_in_microseconds()
{
    uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::
                  now().time_since_epoch()).count();
    return us; 
}

int main(int argc, char **argv)
{
	
	string traj_filepath = "/home/dfki.uni-bremen.de/skumar/repos/githb/underactuated-robotics/simple_pendulum/trajectory_optimisation/traj_opt_traj.csv";
	std::vector<float> desired_time_vec, desired_pos_vec, desired_vel_vec,  desired_effort_vec;
	extract_joint_traj(traj_filepath, desired_time_vec, desired_pos_vec, desired_vel_vec,  desired_effort_vec);
	for(uint i = 0; i < desired_time_vec.size(); i++){
	cout << desired_time_vec[i] << "," << desired_pos_vec[i] << "," << desired_vel_vec[i] << "," << desired_effort_vec[i] << endl;		
	}
	
	motor_driver::MotorDriver motor_controller(0x01, "can0");
	
	cout<<"Enabling Motor..."<<endl;
	motor_driver::motorState start_state = motor_controller.enableMotor();
	cout<<"Position: "<<start_state.position<<" Velocity: "<<start_state.velocity<<" Torque: "<<start_state.torque<<endl;
	
    cout<<"Setting Zero Position..."<<endl;
    motor_driver::motorState stateZero = motor_controller.setZeroPosition();
    cout<<"Position: "<<stateZero.position<<" Velocity: "<<stateZero.velocity<<" Torque: "<<stateZero.torque<<endl; 
	
	
    /*
    cout<<"Sending rad command..."<<endl;
	motor_driver::motorState state = motor_controller.sendRadCommand(0.1, 0.0, 10.0, 0.1, 0.0);
    cout<<"Position: "<<state.position<<" Velocity: "<<state.velocity<<" Torque: "<<state.torque<<endl; 
    */

	std::vector<float> des_pos, des_vel, des_effort;	// desired data
	std::vector<float> pos, vel, effort;	// measured data
		
	uint64_t current_time_us, last_time_us, dt_us; 
	
	uint64_t elapsed_time = 0;
	
	//dt_us = 1000;	
	dt_us = (desired_time_vec[1] - desired_time_vec[0])*1e06;
	
	uint64_t total_time_s;
	
	//total_time_s = 20;
	total_time_s = desired_time_vec.back()*1e06;
	
	float Kp, Kd;
	Kp = 50;
	Kd = 2.0;
	
	cout<<"Recording Data..."<<endl;	
	
	uint i = 0;
	
	while (elapsed_time < total_time_s) {
		current_time_us = get_time_in_microseconds();
		if(current_time_us - last_time_us > dt_us){
			// Do Stuff here at 1KHz
			//cout<<"I say Hello at 1 KHz"<<endl;			
			motor_driver::motorState state = motor_controller.sendRadCommand(desired_pos_vec[i], desired_vel_vec[i], Kp, Kd, desired_effort_vec[i]);
			pos.push_back(state.position);
			vel.push_back(state.velocity);
			effort.push_back(state.torque);
			
			des_pos.push_back(desired_pos_vec[i]);
			des_vel.push_back(desired_vel_vec[i]);
			des_effort.push_back(desired_effort_vec[i]);
					
			last_time_us = current_time_us;
			elapsed_time += dt_us;
			i += 1;
		}
	}

	cout<<"Disabling Motor..."<<endl;	
	motor_driver::motorState end_state = motor_controller.disableMotor();
	cout<<"Position: "<<end_state.position<<" Velocity: "<<end_state.velocity<<" Torque: "<<end_state.torque<<endl;
	
	cout<<"Exporting the csv files..."<<endl;	
	ofstream measured_data_file("measured_data.csv");
	ofstream desired_data_file("desired_data.csv");

	measured_data_file << "pos" << "," << "vel" << "," << "effort" << endl;		
	desired_data_file << "pos" << "," << "vel" << "," << "effort" << endl;		

	for(uint i = 0; i < pos.size(); i++){
	measured_data_file << pos[i] << "," << vel[i] << "," << effort[i] << endl;		
	desired_data_file << des_pos[i] << "," << des_vel[i] << "," << des_effort[i] << endl;	
	}
	
	measured_data_file.close();
	desired_data_file.close();
	
	cout<<"Finshed csv exports..."<<endl;	
	
	system("python3 analyze_data.py");
	
    return 0;
}
