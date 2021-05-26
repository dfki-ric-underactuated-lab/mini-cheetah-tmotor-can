#include "motor_driver/MotorDriver.hpp"

namespace motor_driver
{

    MotorDriver::MotorDriver(const std::vector<int> motor_ids, const char* motor_can_socket) : 
                            MotorCANInterface_(motor_can_socket), motor_ids_{motor_ids}
    {
		
		motorEnableMsg[0] = 0xFF;
		motorEnableMsg[1] = 0xFF;
		motorEnableMsg[2] = 0xFF;
		motorEnableMsg[3] = 0xFF;
		motorEnableMsg[4] = 0xFF;
		motorEnableMsg[5] = 0xFF;
		motorEnableMsg[6] = 0xFF;
		motorEnableMsg[7] = 0xFC;

		motorDisableMsg[0] = 0xFF;
		motorDisableMsg[1] = 0xFF;
		motorDisableMsg[2] = 0xFF;
		motorDisableMsg[3] = 0xFF;
		motorDisableMsg[4] = 0xFF;
		motorDisableMsg[5] = 0xFF;
		motorDisableMsg[6] = 0xFF;
		motorDisableMsg[7] = 0xFD;

		motorSetZeroPositionMsg[0] = 0xFF;
		motorSetZeroPositionMsg[1] = 0xFF;
		motorSetZeroPositionMsg[2] = 0xFF;
		motorSetZeroPositionMsg[3] = 0xFF;
		motorSetZeroPositionMsg[4] = 0xFF;
		motorSetZeroPositionMsg[5] = 0xFF;
		motorSetZeroPositionMsg[6] = 0xFF;
		motorSetZeroPositionMsg[7] = 0xFE;

        // Initialize all Motors to not enabled.
        for (int idIdx = 0; idIdx < motor_ids.size(); idIdx++)
        {
            isMotorEnabled[motor_ids[idIdx]] = false;
        }
    }


    MotorDriver::~MotorDriver()
    {
    }


    std::map<int, motorState> MotorDriver::enableMotor(std::vector<int> enable_motor_ids)
    {
        std::map<int, motorState> motor_status_map;
        motorState state;
        for (int iterId = 0; iterId < enable_motor_ids.size(); iterId++)
        {
            // TODO: Add check that enable motor id is in initialized motor_ids_ vector
            MotorCANInterface_.sendCANFrame(enable_motor_ids[iterId], motorEnableMsg);
            usleep(motorReplyWaitTime);
            if (MotorCANInterface_.receiveCANFrame(CANReplyMsg_))
            {
                state = decodeCANFrame(CANReplyMsg_);
                isMotorEnabled[enable_motor_ids[iterId]] = true;
            }
            else
            {
                perror("MotorDriver::enableMotor() Unable to Receive CAN Reply.");
            }
            if (enable_motor_ids[iterId] != state.motor_id)
            {
                perror("MotorDriver::enableMotor() Received message does not have the same motor id!!");
            }
            motor_status_map[enable_motor_ids[iterId]] = state;
        }
        return motor_status_map;
    }


    std::map<int, motorState> MotorDriver::disableMotor(std::vector<int> disable_motor_ids)
    {
        std::map<int, motorState> motor_status_map;
        motorState state;
        for (int iterId = 0; iterId < disable_motor_ids.size(); iterId++)
        {
            // TODO: Add check that enable motor id is in initialized motor_ids_ vector
            if (isMotorEnabled[disable_motor_ids[iterId]])
            {
                MotorCANInterface_.sendCANFrame(disable_motor_ids[iterId], motorDisableMsg);
                usleep(motorReplyWaitTime);
                if (MotorCANInterface_.receiveCANFrame(CANReplyMsg_))
                {
                    state = decodeCANFrame(CANReplyMsg_);
                    isMotorEnabled[disable_motor_ids[iterId]] = false;
                }
                else
                {
                    perror("MotorDriver::disableMotor() Unable to Receive CAN Reply.");
                }
                if (disable_motor_ids[iterId] != state.motor_id)
                {
                    perror("MotorDriver::disableMotor() Received message does not have the same motor id!!");
                }
                motor_status_map[disable_motor_ids[iterId]] = state;
            }
            else
            {
                perror("MotorDriver::disableMotor() Motor was already in disabled state.");
            }
        }
        return motor_status_map;
    }


    std::map<int, motorState> MotorDriver::setZeroPosition(std::vector<int> zero_motor_ids)
    {
        std::map<int, motorState> motor_status_map;
        motorState state;
        for (int iterId = 0; iterId < zero_motor_ids.size(); iterId++)
        {
            // TODO: Add check that enable motor id is in initialized motor_ids_ vector
            if (isMotorEnabled[zero_motor_ids[iterId]])
            {
                MotorCANInterface_.sendCANFrame(zero_motor_ids[iterId], motorSetZeroPositionMsg);
                usleep(motorReplyWaitTime);
                if (MotorCANInterface_.receiveCANFrame(CANReplyMsg_))
                {
                    state = decodeCANFrame(CANReplyMsg_);
                    motor_status_map[zero_motor_ids[iterId]] = state;
                }
                else
                {
                    perror("MotorDriver::setZeroPosition() Unable to Receive CAN Reply.");
                }

                while (state.position > (5 * (pi / 180)))
                {
                    MotorCANInterface_.sendCANFrame(zero_motor_ids[iterId], motorSetZeroPositionMsg);
                    usleep(motorReplyWaitTime);
                    if (MotorCANInterface_.receiveCANFrame(CANReplyMsg_))
                    {
                        state = decodeCANFrame(CANReplyMsg_);
                        motor_status_map[zero_motor_ids[iterId]] = state;
                    }
                    else
                    {
                        perror("MotorDriver::setZeroPosition() Unable to Receive CAN Reply.");
                    }
                }
            }
            else
            {
                perror("MotorDriver::setZeroPosition() Motor in disabled state.");
            }
        }
       return motor_status_map;
    }


    std::map<int, motorState> MotorDriver::sendRadCommand(std::map<int, motorCommand> motorRadCommands)
    {
        std::map<int, motorState> motor_status_map;
        motorState state;
        int cmdMotorID;
        motorCommand cmdToSend;
        for (std::pair<int, motorCommand> commandIter : motorRadCommands)
        {
            cmdMotorID = commandIter.first;
            cmdToSend = commandIter.second;
            // Apply Saturation based on the limits
            cmdToSend.p_des = fminf(fmaxf(P_MIN, cmdToSend.p_des), P_MAX);
            cmdToSend.v_des = fminf(fmaxf(V_MIN, cmdToSend.v_des), V_MAX);
            cmdToSend.kp = fminf(fmaxf(KP_MIN, cmdToSend.kp), KP_MAX);
            cmdToSend.kd = fminf(fmaxf(KD_MIN, cmdToSend.kd), KD_MAX);
            cmdToSend.tau_ff = fminf(fmaxf(I_MIN, cmdToSend.tau_ff), I_MAX);
            // convert floats to unsigned ints
            int p_int = float_to_uint(cmdToSend.p_des, P_MIN, P_MAX, 16);            
            int v_int = float_to_uint(cmdToSend.v_des, V_MIN, V_MAX, 12);
            int kp_int = float_to_uint(cmdToSend.kp, KP_MIN, KP_MAX, 12);
            int kd_int = float_to_uint(cmdToSend.kd, KD_MIN, KD_MAX, 12);
            int t_int = float_to_uint(cmdToSend.tau_ff, I_MIN, I_MAX, 12);

            // pack ints into the can message
            unsigned char CANMsg_ [8];
            CANMsg_[0] = p_int>>8;                                       
            CANMsg_[1] = p_int&0xFF;
            CANMsg_[2] = v_int>>4;
            CANMsg_[3] = ((v_int&0xF)<<4)|(kp_int>>8);
            CANMsg_[4] = kp_int&0xFF;
            CANMsg_[5] = kd_int>>4;
            CANMsg_[6] = ((kd_int&0xF)<<4)|(t_int>>8);
            CANMsg_[7] = t_int&0xff;

            if (isMotorEnabled[cmdMotorID])
            {
                MotorCANInterface_.sendCANFrame(cmdMotorID, CANMsg_);
                usleep(motorReplyWaitTime);
                if (MotorCANInterface_.receiveCANFrame(CANReplyMsg_))
                {
                    state = decodeCANFrame(CANReplyMsg_);
                    motor_status_map[cmdMotorID] = state;   
                }
                else
                {
                    perror("MotorDriver::sendRadCommand() Unable to Receive CAN Reply.");
                }
            }
            else
            {
                perror("MotorDriver::sendRadCommand() Motor in disabled state.");
            }
        }

        return motor_status_map;
    }

    std::map<int, motorState> MotorDriver::sendDegreeCommand(std::map<int, motorCommand> motorDegCommands)
    {
        std::map<int, motorState> motor_status_map;
        std::map<int, motorCommand> motorRadCommands;
        int cmdMotorID;
        motorCommand cmdToSend;
        for (std::pair<int, motorCommand> commandIter : motorDegCommands)
        {
            cmdMotorID = commandIter.first;
            cmdToSend = commandIter.second;
            cmdToSend.p_des = cmdToSend.p_des * (pi / 180);
            cmdToSend.v_des = cmdToSend.v_des * (pi / 180);
            motorRadCommands[cmdMotorID] = cmdToSend;
        }

        motor_status_map = sendRadCommand(motorRadCommands);

        return motor_status_map;
    }


    motorState MotorDriver::decodeCANFrame(unsigned char* CANReplyMsg_)
    {
        motorState state;
        // unpack ints from can buffer
        int id = CANReplyMsg_[0];
        int p_int = (CANReplyMsg_[1]<<8)|CANReplyMsg_[2];
        int v_int = (CANReplyMsg_[3]<<4)|(CANReplyMsg_[4]>>4);
        int i_int = ((CANReplyMsg_[4]&0xF)<<8)|CANReplyMsg_[5];
        // convert unsigned ints to floats
        float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
        float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
        float i = uint_to_float(i_int, -I_MAX, I_MAX, 12);

        state.motor_id = id;
        state.position = p;
        state.velocity = v;
        state.torque = i;

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

} // motor driver namespace
