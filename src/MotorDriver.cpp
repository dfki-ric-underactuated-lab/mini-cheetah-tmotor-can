#include "motor_driver/MotorDriver.hpp"

namespace motor_driver
{

    MotorDriver::MotorDriver(const uint32_t motor_id, const char* motor_can_socket) : MotorCANInterface_(motor_can_socket, motor_id)
    {
        motor_id_ = motor_id;
    }

    MotorDriver::~MotorDriver()
    {
    }

    motorState MotorDriver::enableMotor()
    {
        motorState state;
        MotorCANInterface_.sendCANFrame(motorEnableMsg);
        usleep(motorReplyWaitTime);
        if (MotorCANInterface_.receiveCANFrame(CANReplyMsg_))
        {
            state = decodeCANFrame(CANReplyMsg_);
            isEnabled = true;
        }
        else
        {
            perror("MotorDriver: Unable to Receive CAN Reply.");
        }
        return state;
    }

    motorState MotorDriver::disableMotor()
    {
        motorState state;
        MotorCANInterface_.sendCANFrame(motorDisableMsg);
        usleep(motorReplyWaitTime);
        if (MotorCANInterface_.receiveCANFrame(CANReplyMsg_))
        {
            state = decodeCANFrame(CANReplyMsg_);
            isEnabled = false;
        }
        else
        {
            perror("MotorDriver: Unable to Receive CAN Reply.");
        }
        return state;
    }

    motorState MotorDriver::setZeroPosition()
    {
        motorState state;
        if (isEnabled)
        {
            MotorCANInterface_.sendCANFrame(motorSetZeroPositionMsg);
            usleep(motorReplyWaitTime);
            if (MotorCANInterface_.receiveCANFrame(CANReplyMsg_))
            {
                state = decodeCANFrame(CANReplyMsg_);    
            }
            else
            {
                perror("MotorDriver: Unable to Receive CAN Reply.");
            }
        }
        return state;
    }

    motorState MotorDriver::sendDegreeCommand()
    {
        motorState state;
        return state;
    }

    motorState MotorDriver::sendRadCommand()
    {
        motorState state;
        return state;
    }

    motorState MotorDriver::sendTorqueCommand()
    {
        motorState state;
        return state;
    }

    motorState MotorDriver::decodeCANFrame(unsigned char* CANReplyMsg_)
    {
        motorState state;
        // unpack ints from can buffer
        // int id = motor->rxMsg.data[0];
        int p_int = (CANReplyMsg_[1]<<8)|CANReplyMsg_[2];
        int v_int = (CANReplyMsg_[3]<<4)|(CANReplyMsg_[4]>>4);
        int i_int = ((CANReplyMsg_[4]&0xF)<<8)|CANReplyMsg_[5];
        // convert unsigned ints to floats
        float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
        float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
        float i = uint_to_float(i_int, -I_MAX, I_MAX, 12);

        state.position = p;
        state.velocity = v;
        state.current = i;

        return state;
    }

    int MotorDriver::float_to_uint(float x, float x_min, float x_max, int bits)
    {
        /// Converts a float to an unsigned int, given range and number of bits ///
        float span = x_max - x_min;
        float offset = x_min;
        return (int) ((x-offset)*((float)((1<<bits)-1))/span);
    }

    float MotorDriver::uint_to_float(int x_int, float x_min, float x_max, int bits)
    {
        /// converts unsigned int to float, given range and number of bits ///
        float span = x_max - x_min;
        float offset = x_min;
        return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }

    float MotorDriver::fmaxf3(float x, float y, float z)
    {
        /// Returns maximum of x, y, z ///
        return (x > y ? (x > z ? x : z) : (y > z ? y : z));
    }

    float MotorDriver::fminf3(float x, float y, float z)
    {
        /// Returns minimum of x, y, z ///
        return (x < y ? (x < z ? x : z) : (y < z ? y : z));
    }

    void MotorDriver::limit_norm(float *x, float *y, float limit)
    {
        /// Scales the lenght of vector (x, y) to be <= limit ///
        float norm = sqrt(*x * *x + *y * *y);
        if (norm > limit)
        {
            *x = *x * limit/norm;
            *y = *y * limit/norm;
        }
    }
} // motor driver namespace
