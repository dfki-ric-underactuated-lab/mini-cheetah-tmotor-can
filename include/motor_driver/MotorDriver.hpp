#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <vector>
#include <map>

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

    struct motorParams{
        float P_MIN;
        float P_MAX;
        float V_MIN;
        float V_MAX;
        float T_MIN;
        float T_MAX;
        float KP_MIN;
        float KP_MAX;
        float KD_MIN;
        float KD_MAX;
        int AXIS_DIRECTION;
    };

    enum MotorType {AK80_6_V1,
                    AK80_6_V1p1,
                    AK80_6_V2,
                    AK80_9_V1p1,
                    AK80_9_V2,
                    AK10_9_V1p1};


    class MotorDriver{

    public:
        MotorDriver(std::vector<int> motor_ids, const char* motor_can_socket, MotorType motor_type);

    	~MotorDriver();

    	std::map<int, motorState> disableMotor(std::vector<int> disable_motor_ids);
    	std::map<int, motorState> enableMotor(std::vector<int> enable_motor_ids);
    	std::map<int, motorState> setZeroPosition(std::vector<int> zero_motor_ids);
        std::map<int, motorState> sendRadCommand(std::map<int, motorCommand>);
    	std::map<int, motorState> sendDegreeCommand(std::map<int, motorCommand>);

        motorParams getMotorParams();

        void setMotorParams(motorParams newParams);

		// The usleep() is not very accurate on non-realtime systems. So the actual sleep time is 
        // higher than asked for. The Google Docs by Ben Katz shows round trip time to be ~230us.
        // Looking at the oscilloscope image, the time taken to reply is ~120us after the message
        // is sent. Hence here we set it to 100us given that the Ubuntu system always takes longer
        // than what is asked for.
        // Adjust this parameter if running on real-time system.
		unsigned int motorReplyWaitTime = 10;

    private:
        // unsigned char motorEnableMsg[8];
		// unsigned char motorDisableMsg[8];
		// unsigned char motorSetZeroPositionMsg[8];
        motorParams currentParams;
        MotorType motor_type_;
        double pi = 3.14159265359;
        std::map<int, bool> isMotorEnabled;
        const std::vector<int> motor_ids_;
        // Pre-allocate memory for CAN messages which are overwritten by functions.
        unsigned char CANReplyMsg_ [8];
        unsigned char CANMsg_[8];
        CAN_interface::CANInterface MotorCANInterface_;
        motorState decodeCANFrame(unsigned char* CANReplyMsg_);
        bool encodeCANFrame(motorCommand cmdToSend, unsigned char* CANMsg_);
        // Taken from Ben Katz mbed repo https://os.mbed.com/users/benkatz/code/MotorModuleExample/
        int float_to_uint(float x, float x_min, float x_max, int bits);
        float uint_to_float(int x_int, float x_min, float x_max, int bits);

        // Constants for conversions.
        // Working Parameters for AK80-6 V1.0 Firmware
        motorParams AK80_6_V1_params = {
            -95.5,        // P_MIN
            95.5,         // P_MAX
            -45.0,        // V_MIN
            45.0,         // V_MAX
            -18.0,        // T_MIN
            18.0,         // T_MAX
            0.0,          // KP_MIN
            500.0,        // KP_MAX
            0,            // KD_MIN
            5,            // KD_MAX
            -1            // AXIS_DIRECTION
        };

        // Working Parameters for AK80-6 V1.1 Firmware
        motorParams AK80_6_V1p1_params = {
            -12.5,        // P_MIN
            12.5,         // P_MAX
            -22.5,        // V_MIN
            22.5,         // V_MAX
            -12.0,        // T_MIN
            12.0,         // T_MAX
            0.0,          // KP_MIN
            500.0,        // KP_MAX
            0,            // KD_MIN
            5,            // KD_MAX
            -1            // AXIS_DIRECTION
        };

        // Working parameters for AK80-6 V2.0 firmware
        motorParams AK80_6_V2_params = {
            -12.5,        // P_MIN
            12.5,         // P_MAX
            -38.2,        // V_MIN
            38.2,         // V_MAX
            -12.0,        // T_MIN
            12.0,         // T_MAX
            0.0,          // KP_MIN
            500.0,        // KP_MAX
            0,            // KD_MIN
            5,            // KD_MAX
            1             // AXIS_DIRECTION
        };

        // Working parameters for AK80-9 V1.1 firmware
        motorParams AK80_9_V1p1_params = {
            -12.5,        // P_MIN
            12.5,         // P_MAX
            -22.5,       // V_MIN
            22.5,        // V_MAX
            -18.0,        // T_MIN
            18.0,         // T_MAX
            0.0,          // KP_MIN
            500.0,        // KP_MAX
            0,            // KD_MIN
            5,            // KD_MAX
            1             // AXIS_DIRECTION
        };
        
        // Working parameters for AK80-9 V2.0 firmware
        motorParams AK80_9_V2_params = {
            -12.5,        // P_MIN
            12.5,         // P_MAX
            -25.64,       // V_MIN
            25.64,        // V_MAX
            -18.0,        // T_MIN
            18.0,         // T_MAX
            0.0,          // KP_MIN
            500.0,        // KP_MAX
            0,            // KD_MIN
            5,            // KD_MAX
            1             // AXIS_DIRECTION
        };

        // Working parameters for AK10-9 V1.1 firmware
        motorParams AK10_9_V1p1_params = {
            -12.5,        // P_MIN
            12.5,         // P_MAX
            -50,       // V_MIN
            50,        // V_MAX
            -65.0,        // T_MIN
            65.0,         // T_MAX
            0.0,          // KP_MIN
            500.0,        // KP_MAX
            0,            // KD_MIN
            5,            // KD_MAX
            1             // AXIS_DIRECTION
        };

        // Default Motor Messages
        unsigned char motorEnableMsg[8] = {0xFF,
                                           0xFF,
                                           0xFF,
                                           0xFF,
                                           0xFF,
                                           0xFF,
                                           0xFF,
                                           0xFC};

        unsigned char motorDisableMsg[8] = {0xFF,
                                            0xFF,
                                            0xFF,
                                            0xFF,
                                            0xFF,
                                            0xFF,
                                            0xFF,
                                            0xFD};
        
        unsigned char motorSetZeroPositionMsg[8] = {0xFF,
                                                    0xFF,
                                                    0xFF,
                                                    0xFF,
                                                    0xFF,
                                                    0xFF,
                                                    0xFF,
                                                    0xFE};
    };

}
#endif // MOTOR_DRIVER_HPP
