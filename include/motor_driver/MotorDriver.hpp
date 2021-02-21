#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include <stdint.h>

namespace motor_driver
{
    struct motorState{
        uint16_t position;
        uint16_t velocity;
        uint16_t current;
    };

    class MotorDriver{

    public:
        MotorDriver();

    	~MotorDriver();

    	motorState disableMotor();
    	motorState enableMotor();
    	motorState setZeroPosition();
    	motorState sendDegreeCommand();
        motorState sendRadCommand();
    	motorState sendTorqueCommand();

    private:

	bool isEnabled;
    };

}
#endif // MOTOR_DRIVER_HPP
