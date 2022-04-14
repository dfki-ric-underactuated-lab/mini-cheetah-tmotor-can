#include "motor_driver/MotorDriver.hpp"

namespace motor_driver
{

    MotorDriver::MotorDriver(const std::vector<int> motor_ids, const char* motor_can_socket, MotorType motor_type=MotorType::AK80_6_V1p1) : 
                            MotorCANInterface_(motor_can_socket), motor_ids_{motor_ids}, motor_type_{motor_type}
    {
        // Set Motor Parameters According to Motor Type

        if (motor_type_ == MotorType::AK80_6_V1)
        {
            std::cout << "Using Motor Type AK80-6 V1" << std::endl;
            currentParams = AK80_6_V1_params;
        }
        else if (motor_type_ == MotorType::AK80_6_V1p1)
        {
            std::cout << "Using Motor Type AK80-6 V1.1" << std::endl;
            currentParams = AK80_6_V1p1_params;
        }
        else if (motor_type_ == MotorType::AK80_6_V2)
        {
            std::cout << "Using Motor Type AK80-6 V2" << std::endl;
            currentParams = AK80_6_V2_params;
        }
        else if (motor_type_ == MotorType::AK80_9_V1p1)
        {
            std::cout << "Using Motor Type AK80-9 V1.1" << std::endl;
            currentParams = AK80_9_V1p1_params;
        }
        else if (motor_type_ == MotorType::AK80_9_V2)
        {
            std::cout << "Using Motor Type AK80-9 V2" << std::endl;
            currentParams = AK80_9_V2_params;
        }
        else if (motor_type_ == MotorType::AK10_9_V1p1)
        {
            std::cout << "Using Motor Type AK10-9 V1.1" << std::endl;
            currentParams = AK10_9_V1p1_params;
        }
        else 
        {
            perror("Specified Motor Type Not Found!!");
        }

        // Initialize all Motors to not enabled.
        // TODO: Enable enabled check better across multiple objects of this class.
        for (int idIdx = 0; idIdx < motor_ids_.size(); idIdx++)
        {
            isMotorEnabled[motor_ids_[idIdx]] = false;
        }
    }


    MotorDriver::~MotorDriver()
    {
    }


    std::map<int, motorState> MotorDriver::enableMotor(std::vector<int> enable_motor_ids)
    {
        std::map<int, motorState> motor_state_map;
        motorState state;
        for (int iterId = 0; iterId < enable_motor_ids.size(); iterId++)
        {
            // TODO: Add check that enable motor id is in initialized motor_ids_ vector
            // using std::find
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
            motor_state_map[enable_motor_ids[iterId]] = state;
        }
        return motor_state_map;
    }


    std::map<int, motorState> MotorDriver::disableMotor(std::vector<int> disable_motor_ids)
    {
        std::map<int, motorState> motor_state_map;
        motorState state;
        for (int iterId = 0; iterId < disable_motor_ids.size(); iterId++)
        {
            // TODO: Add check that enable motor id is in initialized motor_ids_ vector
            // using std::find.
            // TODO: Enable enabled check better across multiple objects of this class.
            // if (isMotorEnabled[disable_motor_ids[iterId]])
            // {
            //     std::cout << "MotorDriver::disableMotor() Motor seems to already be in disabled state. \
            //                   Did you want to really do this?" << std::endl;
            // }
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
            motor_state_map[disable_motor_ids[iterId]] = state;
        }
        return motor_state_map;
    }


    std::map<int, motorState> MotorDriver::setZeroPosition(std::vector<int> zero_motor_ids)
    {
        std::map<int, motorState> motor_state_map;
        motorState state;
        for (int iterId = 0; iterId < zero_motor_ids.size(); iterId++)
        {
            // TODO: Add check that enable motor id is in initialized motor_ids_ vector
            // using std::find
            // TODO: Enable enabled check better across multiple objects of this class.
            // if (isMotorEnabled[zero_motor_ids[iterId]])
            // {
            //     std::cout << "MotorDriver::setZeroPosition() Motor in disabled state.\
            //                   Did you want to really do this?" << std::endl;
            // }
            MotorCANInterface_.sendCANFrame(zero_motor_ids[iterId], motorSetZeroPositionMsg);
            usleep(motorReplyWaitTime);
            if (MotorCANInterface_.receiveCANFrame(CANReplyMsg_))
            {
                state = decodeCANFrame(CANReplyMsg_);
                motor_state_map[zero_motor_ids[iterId]] = state;
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
                    motor_state_map[zero_motor_ids[iterId]] = state;
                }
                else
                {
                    perror("MotorDriver::setZeroPosition() Unable to Receive CAN Reply.");
                }
            }
        }
       return motor_state_map;
    }


    std::map<int, motorState> MotorDriver::sendRadCommand(std::map<int, motorCommand> motorRadCommands)
    {
        std::map<int, motorState> motor_state_map;
        motorState state;
        int cmdMotorID;
        motorCommand cmdToSend;
        for (std::pair<int, motorCommand> commandIter : motorRadCommands)
        {
            cmdMotorID = commandIter.first;
            cmdToSend = commandIter.second;
            bool return_val = encodeCANFrame(cmdToSend, CANMsg_);
            // TODO: Enable enabled check better across multiple objects of this class.
            // if (isMotorEnabled[cmdMotorID])
            // {
            //     std::cout << "MotorDriver::sendRadCommand() Motor in disabled state.\
            //                   Did you want to really do this?" << std::endl;
            // }
            MotorCANInterface_.sendCANFrame(cmdMotorID, CANMsg_);
            usleep(motorReplyWaitTime);
            if (MotorCANInterface_.receiveCANFrame(CANReplyMsg_))
            {
                state = decodeCANFrame(CANReplyMsg_);
                motor_state_map[cmdMotorID] = state;
            }
            else
            {
                perror("MotorDriver::sendRadCommand() Unable to Receive CAN Reply.");
            }
        }

        return motor_state_map;
    }

    std::map<int, motorState> MotorDriver::sendDegreeCommand(std::map<int, motorCommand> motorDegCommands)
    {
        std::map<int, motorState> motor_state_map;
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

        motor_state_map = sendRadCommand(motorRadCommands);

        return motor_state_map;
    }

    motorParams MotorDriver::getMotorParams()
    {
        return currentParams;
    }

    void MotorDriver::setMotorParams(motorParams newParams)
    {   
        currentParams = newParams;
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
        float p = uint_to_float(p_int, currentParams.P_MIN, currentParams.P_MAX, 16);
        float v = uint_to_float(v_int, currentParams.V_MIN, currentParams.V_MAX, 12);
        float i = uint_to_float(i_int, -currentParams.T_MAX, currentParams.T_MAX, 12);

        state.motor_id = id;
        state.position = p * currentParams.AXIS_DIRECTION;;
        state.velocity = v * currentParams.AXIS_DIRECTION;;
        state.torque = i * currentParams.AXIS_DIRECTION;;

        return state;
    }

    bool MotorDriver::encodeCANFrame(motorCommand cmdToSend, unsigned char* CANMsg_)
    {
        cmdToSend.p_des = cmdToSend.p_des * currentParams.AXIS_DIRECTION;
        cmdToSend.v_des = cmdToSend.v_des * currentParams.AXIS_DIRECTION;
        cmdToSend.tau_ff = cmdToSend.tau_ff * currentParams.AXIS_DIRECTION;
        // Apply Saturation based on the limits      
        cmdToSend.p_des = fminf(fmaxf(currentParams.P_MIN, cmdToSend.p_des), currentParams.P_MAX);
        cmdToSend.v_des = fminf(fmaxf(currentParams.V_MIN, cmdToSend.v_des), currentParams.V_MAX);
        cmdToSend.kp = fminf(fmaxf(currentParams.KP_MIN, cmdToSend.kp), currentParams.KP_MAX);
        cmdToSend.kd = fminf(fmaxf(currentParams.KD_MIN, cmdToSend.kd), currentParams.KD_MAX);
        cmdToSend.tau_ff = fminf(fmaxf(currentParams.T_MIN, cmdToSend.tau_ff), currentParams.T_MAX);
        // convert floats to unsigned ints
        int p_int = float_to_uint(cmdToSend.p_des, currentParams.P_MIN, currentParams.P_MAX, 16);            
        int v_int = float_to_uint(cmdToSend.v_des, currentParams.V_MIN, currentParams.V_MAX, 12);
        int kp_int = float_to_uint(cmdToSend.kp, currentParams.KP_MIN, currentParams.KP_MAX, 12);
        int kd_int = float_to_uint(cmdToSend.kd, currentParams.KD_MIN, currentParams.KD_MAX, 12);
        int t_int = float_to_uint(cmdToSend.tau_ff, currentParams.T_MIN, currentParams.T_MAX, 12);

        // pack ints into the can message
        CANMsg_[0] = p_int>>8;                                       
        CANMsg_[1] = p_int&0xFF;
        CANMsg_[2] = v_int>>4;
        CANMsg_[3] = ((v_int&0xF)<<4)|(kp_int>>8);
        CANMsg_[4] = kp_int&0xFF;
        CANMsg_[5] = kd_int>>4;
        CANMsg_[6] = ((kd_int&0xF)<<4)|(t_int>>8);
        CANMsg_[7] = t_int&0xff;

        return true;
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
