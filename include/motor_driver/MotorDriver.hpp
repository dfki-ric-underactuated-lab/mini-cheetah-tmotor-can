#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <vector>
#include <map>
#include "Parameters.hpp"
#include "CANInterface.hpp"

namespace motor_driver
{
    struct motorState{
        int motor_id;
        float position;
        float velocity;
        float torque;
    };

    struct motorCommand{
        float p_des;
        float v_des;
        float kp;
        float kd;
        float tau_ff;       
    };

    class MotorDriver{

    public:
        MotorDriver(std::vector<int> motor_ids, const char* motor_can_socket);

    	~MotorDriver();

    	std::map<int, motorState> disableMotor(std::vector<int> disable_motor_ids);
    	std::map<int, motorState> enableMotor(std::vector<int> enable_motor_ids);
    	std::map<int, motorState> setZeroPosition(std::vector<int> zero_motor_ids);
        std::map<int, motorState> sendRadCommand(std::map<int, motorCommand>);
    	std::map<int, motorState> sendDegreeCommand(std::map<int, motorCommand>);
    	
		// Fixed Messages for Enabling, Disabling, and setting Zero Position on the Motor

		unsigned char motorEnableMsg[8];

		unsigned char motorDisableMsg[8];

		unsigned char motorSetZeroPositionMsg[8];

		// The usleep() is not very accurate on non-realtime systems. So the actual sleep time is 
        // higher than asked for. The Google Docs by Ben Katz shows round trip time to be ~230us.
        // Looking at the oscilliscope image, the time taken to reply is ~120us after the message
        // is sent. Hence here we set it to 100us given that the Ubuntu system always takes longer
        // than what is asked for.
        // Adjust this parameter if running on real-time system.
		// https://docs.google.com/document/d/1dzNVzblz6mqB3eZVEMyi2MtSngALHdgpTaDJIW_BpS4/edit
		unsigned int motorReplyWaitTime = 100;

    private:
        double pi = 3.14159265359;
        std::map<int, bool> isMotorEnabled;
        const std::vector<int> motor_ids_;
        unsigned char CANReplyMsg_ [8];
        CAN_interface::CANInterface MotorCANInterface_;
        motorState decodeCANFrame(unsigned char* CANReplyMsg_);
        unsigned char* encodeCANFrame(motorState state);
        // Taken from Ben Katz mbed repo https://os.mbed.com/users/benkatz/code/MotorModuleExample/
        int float_to_uint(float x, float x_min, float x_max, int bits);
        float uint_to_float(int x_int, float x_min, float x_max, int bits);
    };

}
#endif // MOTOR_DRIVER_HPP
