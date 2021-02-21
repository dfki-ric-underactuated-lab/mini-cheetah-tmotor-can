#include "motor_driver/MotorDriver.hpp"

namespace motor_driver
{

MotorDriver::MotorDriver()
{
}

MotorDriver::~MotorDriver()
{
}

motorState MotorDriver::enableMotor()
{
motorState state;
unsigned char MotorEnableMsg [] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
return state;
}

motorState MotorDriver::disableMotor()
{
motorState state;
unsigned char MotorDisableMsg [] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
return state;
}

motorState MotorDriver::setZeroPosition()
{
motorState state;
unsigned char MotorSetZeroPositionMsg [] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
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

} // motor driver namespace
