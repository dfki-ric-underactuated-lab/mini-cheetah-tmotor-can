#include "motor_driver/MotorDriver.hpp"

namespace motor_driver
{

    MotorDriver::MotorDriver(const std::vector<int>& motor_ids, const char* motor_can_socket, MotorType motor_type=MotorType::AK80_6_V1p1) 
        : MotorCANInterface_(motor_can_socket), motor_ids_{motor_ids}, motor_type_{motor_type}
    {
        // Set Motor Parameters According to Motor Type

        switch (motor_type_)
        {
            case MotorType::AK80_6_V1:
                std::cout << "Using Motor Type AK80-6 V1" << std::endl;
                currentParams = AK80_6_V1_params;
                break;
            case MotorType::AK80_6_V1p1:
                std::cout << "Using Motor Type AK80-6 V1.1" << std::endl;
                currentParams = AK80_6_V1p1_params;
                break;
            case MotorType::AK80_6_V2:
                std::cout << "Using Motor Type AK80-6 V2" << std::endl;
                currentParams = AK80_6_V2_params;
                break;
            case MotorType::AK80_9_V1p1:
                std::cout << "Using Motor Type AK80-9 V1.1" << std::endl;
                currentParams = AK80_9_V1p1_params;
                break;
            case MotorType::AK80_9_V2:
                std::cout << "Using Motor Type AK80-9 V2" << std::endl;
                currentParams = AK80_9_V2_params;
                break;
            case MotorType::AK70_10V1p1:
                std::cout << "Using Motor Type AK70-10 V1.1" << std::endl;
                currentParams = AK70_10_V1p1_params;
                break;
            case MotorType::AK10_9_V1p1:
                std::cout << "Using Motor Type AK10-9 V1.1" << std::endl;
                currentParams = AK10_9_V1p1_params;
                break;
            default:
                perror("Specified Motor Type Not Found!!");
        }

        // Initialize all Motors to not enabled.
        // TODO: Enable enabled check better across multiple objects of this class.
        for (int motor_id : motor_ids_)
            isMotorEnabled[motor_id] = false;
    }


    std::map<int, motorState> MotorDriver::enableMotor(const std::vector<int>& enable_motor_ids)
    {
        std::map<int, motorState> motor_state_map;
        
        motorState state;
        for (int motor_id : enable_motor_ids)
        {
            // TODO: Add check that enable motor id is in initialized motor_ids_ vector
            // using std::find
            MotorCANInterface_.sendCANFrame(motor_id, motorEnableMsg);
            usleep(motorReplyWaitTime);
            if (MotorCANInterface_.receiveCANFrame(CANReplyMsg_))
            {
                state = decodeCANFrame(CANReplyMsg_);
                isMotorEnabled[motor_id] = true;
            }
            else
            {
                perror("MotorDriver::enableMotor() Unable to Receive CAN Reply.");
            }

            if (motor_id != state.motor_id)
                perror("MotorDriver::enableMotor() Received message does not have the same motor id!!");

            motor_state_map[motor_id] = state;
        }
        return motor_state_map;
    }


    std::map<int, motorState> MotorDriver::disableMotor(const std::vector<int>& disable_motor_ids)
    {
        std::map<int, motorState> motor_state_map;
        
        motorState state;
        for (int motor_id : disable_motor_ids)
        {
            // TODO: Add check that enable motor id is in initialized motor_ids_ vector
            // using std::find.
            // TODO: Enable enabled check better across multiple objects of this class.
            // if (isMotorEnabled[disable_motor_ids[iterId]])
            // {
            //     std::cout << "MotorDriver::disableMotor() Motor seems to already be in disabled state. \
            //                   Did you want to really do this?" << std::endl;
            // }

            // Bugfix: To remove the initial kick at motor start.
            // The current working theory is that the motor "remembers" the last command. And this
            // causes an initial kick as the motor controller starts. The fix is then to set the 
            // last command to zero so that this does not happen. For the user, the behaviour does
            // not change as zero command + disable is same as disable.
            bool return_val = encodeCANFrame(zeroCmdStruct, CANMsg_);
            MotorCANInterface_.sendCANFrame(motor_id, CANMsg_);
            usleep(motorReplyWaitTime);
            
            if (MotorCANInterface_.receiveCANFrame(CANReplyMsg_))
            {
                state = decodeCANFrame(CANReplyMsg_);
            }
            else
            {
                perror("MotorDriver::disableMotor() Unable to Receive CAN Reply.");
            }

            // Do the actual disabling after zero command.
            MotorCANInterface_.sendCANFrame(motor_id, motorDisableMsg);
            usleep(motorReplyWaitTime);
            if (MotorCANInterface_.receiveCANFrame(CANReplyMsg_))
            {
                state = decodeCANFrame(CANReplyMsg_);
                isMotorEnabled[motor_id] = false;
            }
            else
            {
                perror("MotorDriver::disableMotor() Unable to Receive CAN Reply.");
            }

            if (motor_id != state.motor_id)
                perror("MotorDriver::disableMotor() Received message does not have the same motor id!!");
                
            motor_state_map[motor_id] = state;
        }
        return motor_state_map;
    }


    std::map<int, motorState> MotorDriver::setZeroPosition(const std::vector<int>& zero_motor_ids)
    {
        std::map<int, motorState> motor_state_map;

        motorState state;
        for (int motor_id : zero_motor_ids)
        {
            // TODO: Add check that enable motor id is in initialized motor_ids_ vector
            // using std::find
            // TODO: Enable enabled check better across multiple objects of this class.
            // if (isMotorEnabled[motor_id])
            // {
            //     std::cout << "MotorDriver::setZeroPosition() Motor in disabled state.\
            //                   Did you want to really do this?" << std::endl;
            // }
            MotorCANInterface_.sendCANFrame(motor_id, motorSetZeroPositionMsg);
            usleep(motorReplyWaitTime);
            if (MotorCANInterface_.receiveCANFrame(CANReplyMsg_))
            {
                state = decodeCANFrame(CANReplyMsg_);
                motor_state_map[motor_id] = state;
            }
            else
            {
                perror("MotorDriver::setZeroPosition() Unable to Receive CAN Reply.");
            }

            while (state.position > (1 * (pi / 180)))
            {
                MotorCANInterface_.sendCANFrame(motor_id, motorSetZeroPositionMsg);
                usleep(motorReplyWaitTime);
                if (MotorCANInterface_.receiveCANFrame(CANReplyMsg_))
                {
                    state = decodeCANFrame(CANReplyMsg_);
                    motor_state_map[motor_id] = state;
                }
                else
                {
                    perror("MotorDriver::setZeroPosition() Unable to Receive CAN Reply.");
                }
            }
        }
       return motor_state_map;
    }


    std::map<int, motorState> MotorDriver::sendRadCommand(const std::map<int, motorCommand>& motorRadCommands)
    {
        motorState state;
        std::map<int, motorState> motor_state_map;

        for (const std::pair<int, motorCommand>& commandIter : motorRadCommands)
        {
            int cmdMotorID = commandIter.first;
            const motorCommand& cmdToSend = commandIter.second;

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

    std::map<int, motorState> MotorDriver::sendDegreeCommand(const std::map<int, motorCommand>& motorDegCommands)
    {
        std::map<int, motorCommand> motorRadCommands = motorDegCommands;

        for (auto& command_pair : motorRadCommands)
        {
            command_pair.second.p_des *= (pi / 180);
            command_pair.second.v_des *= (pi / 180);
        }

        return sendRadCommand(motorRadCommands);
    }

    const motorParams& MotorDriver::getMotorParams() const
    {
        return currentParams;
    }

    void MotorDriver::setMotorParams(const motorParams& newParams)
    {   
        currentParams = newParams;
    }

    motorState MotorDriver::decodeCANFrame(unsigned char* CANReplyMsg_)
    {
        // unpack ints from can buffer
        int id = CANReplyMsg_[0];
        int p_int = (CANReplyMsg_[1] << 8) | CANReplyMsg_[2];
        int v_int = (CANReplyMsg_[3] << 4) | (CANReplyMsg_[4] >> 4);
        int i_int = ((CANReplyMsg_[4] & 0xF) << 8) | CANReplyMsg_[5];
        // convert unsigned ints to floats
        float p = uint_to_float(p_int, currentParams.P_MIN, currentParams.P_MAX, 16);
        float v = uint_to_float(v_int, currentParams.V_MIN, currentParams.V_MAX, 12);
        float i = uint_to_float(i_int, -currentParams.T_MAX, currentParams.T_MAX, 12); // here -T_MAX, in encode T_MIN

        motorState state {
            .motor_id = id,
            .position = p * currentParams.AXIS_DIRECTION,
            .velocity = v * currentParams.AXIS_DIRECTION,
            .torque = i * currentParams.AXIS_DIRECTION
        };

        return state;
    }

    bool MotorDriver::encodeCANFrame(const motorCommand& cmdToSend, unsigned char* CANMsg_)
    {
        float p_des = cmdToSend.p_des * currentParams.AXIS_DIRECTION;
        float v_des = cmdToSend.v_des * currentParams.AXIS_DIRECTION;
        float tau_ff = cmdToSend.tau_ff * currentParams.AXIS_DIRECTION;

        // Apply Saturation based on the limits      
        p_des = fminf(fmaxf(currentParams.P_MIN, p_des), currentParams.P_MAX);
        v_des = fminf(fmaxf(currentParams.V_MIN, v_des), currentParams.V_MAX);
        tau_ff = fminf(fmaxf(currentParams.T_MIN, tau_ff), currentParams.T_MAX);
        float kp = fminf(fmaxf(currentParams.KP_MIN, cmdToSend.kp), currentParams.KP_MAX);
        float kd = fminf(fmaxf(currentParams.KD_MIN, cmdToSend.kd), currentParams.KD_MAX);

        // convert floats to unsigned ints
        int p_int = float_to_uint(p_des, currentParams.P_MIN, currentParams.P_MAX, 16);            
        int v_int = float_to_uint(v_des, currentParams.V_MIN, currentParams.V_MAX, 12);
        int kp_int = float_to_uint(kp, currentParams.KP_MIN, currentParams.KP_MAX, 12);
        int kd_int = float_to_uint(kd, currentParams.KD_MIN, currentParams.KD_MAX, 12);
        int t_int = float_to_uint(tau_ff, currentParams.T_MIN, currentParams.T_MAX, 12);

        // pack ints into the can message
        CANMsg_[0] = p_int >> 8;                                       
        CANMsg_[1] = p_int & 0xFF;
        CANMsg_[2] = v_int >> 4;
        CANMsg_[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
        CANMsg_[4] = kp_int & 0xFF;
        CANMsg_[5] = kd_int >> 4;
        CANMsg_[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
        CANMsg_[7] = t_int & 0xff;

        return true;
    }


    int MotorDriver::float_to_uint(float x, float x_min, float x_max, int bits)
    {
        /// Converts a float to an unsigned int, given range and number of bits ///
        float span = x_max - x_min;
        float offset = x_min;
        return (int) ((x-offset) * ((float)((1 << bits)-1)) / span);
    }


    float MotorDriver::uint_to_float(int x_int, float x_min, float x_max, int bits)
    {
        /// converts unsigned int to float, given range and number of bits ///
        float span = x_max - x_min;
        float offset = x_min;
        return ((float)x_int) * span / ((float)((1 << bits)-1)) + offset;
    }

} // motor driver namespace
