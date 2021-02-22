#ifndef MOTOR_PARAMETERS_HPP
#define MOTOR_PARAMETERS_HPP

// Fixed Messages for Enabling, Disabling, and setting Zero Position on the Motor

unsigned char motorEnableMsg [] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

unsigned char motorDisableMsg [] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

unsigned char motorSetZeroPositionMsg [] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};

// Motor takes about 220 micro-seconds to respond. This delay ensures that the motor gets enough
// time to respond. From Ben Katz Google Docs Documentation: 
// https://docs.google.com/document/d/1dzNVzblz6mqB3eZVEMyi2MtSngALHdgpTaDJIW_BpS4/edit
unsigned int motorReplyWaitTime = 300;

// Motor Value Limits
#define P_MIN -95.5f        // Radians
#define P_MAX 95.5f        
#define V_MIN -45.0f       // Rad/s
#define V_MAX 45.0f
#define KP_MIN 0.0f        // N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f        // N-m/rad/s
#define KD_MAX 5.0f
#define I_MIN -18.0f
#define I_MAX 18.0f

#endif // MOTOR_PARAMETERS_HPP