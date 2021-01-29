# Requirements For the Functionality of the C+ Driver

- The driver should be able to communicate with multiple motors (addresed using different CAN IDs) on the same socket.
- The driver should provice the following basic functions to the user for each motor:
    - Enable Motor
    - Disable Motor
    - Set Zero Position
- The driver shall allow the following command functions to the user for each motor:
    - Send full command in `degrees` i.e. send goal position, goal velocity, Kp, Kd, feedforward torque
    - Send full command in `radians` i.e. send goal position, goal velocity, Kp, Kd, feedforward torque
    - Send only feedforward torque in `Nm`
- The driver should allow communication with at least 2 motors on the same socket at 1KHz (assuming the computer is capable of this).
- The driver shall use only the standard C++/linux libraries available not depend on 3rd party libraries for operation.