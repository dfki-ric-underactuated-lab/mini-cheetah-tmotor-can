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
        float current;
    };

    class MotorDriver{

    public:
        MotorDriver(const uint32_t motor_id, const char* motor_can_socket);

    	~MotorDriver();

    	motorState disableMotor();
    	motorState enableMotor();
    	motorState setZeroPosition();
    	motorState sendDegreeCommand();
        motorState sendRadCommand();
    	motorState sendTorqueCommand();

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
