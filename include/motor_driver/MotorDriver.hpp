#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include "Parameters.hpp"
#include "CANInterface.hpp"

namespace motor_driver
{
    struct motorState{
        float position;
        float velocity;
        float torque;
    };

    class MotorDriver{

    public:
        MotorDriver(const uint32_t motor_id, const char* motor_can_socket);

    	~MotorDriver();

    	motorState disableMotor();
    	motorState enableMotor();
    	motorState setZeroPosition();
        motorState sendRadCommand(float p_des, float v_des, float kp, float kd, float i_ff);
    	motorState sendDegreeCommand();
    	motorState sendTorqueCommand();
    	
		// Fixed Messages for Enabling, Disabling, and setting Zero Position on the Motor

		unsigned char motorEnableMsg[8];

		unsigned char motorDisableMsg[8];

		unsigned char motorSetZeroPositionMsg[8];

		// Motor takes about 220 micro-seconds to respond. This delay ensures that the motor gets enough
		// time to respond. From Ben Katz Google Docs Documentation: 
		// https://docs.google.com/document/d/1dzNVzblz6mqB3eZVEMyi2MtSngALHdgpTaDJIW_BpS4/edit
		unsigned int motorReplyWaitTime;

    private:
        bool isEnabled;
        uint32_t motor_id_;
        unsigned char CANReplyMsg_ [8];
        CAN_interface::CANInterface MotorCANInterface_;
        motorState decodeCANFrame(unsigned char* CANReplyMsg_);
        unsigned char* encodeCANFrame(motorState state);
        // Taken from Ben Katz mbed repo https://os.mbed.com/users/benkatz/code/MotorModuleExample/
        int float_to_uint(float x, float x_min, float x_max, int bits);
        float uint_to_float(int x_int, float x_min, float x_max, int bits);
        // Already in math.h
        // float fmaxf(float x, float y);
        // float fminf(float x, float y);
        float fmaxf3(float x, float y, float z);
        float fminf3(float x, float y, float z);
        void limit_norm(float *x, float *y, float limit);
    };

}
#endif // MOTOR_DRIVER_HPP
