#ifndef CAN_INTERFACE_HPP
#define CAN_INTERFACE_HPP

#include <stdint.h>

namespace CAN_interface
{
    class CANInterface{

    public:
        CANInterface(const char* socketName);

        ~CANInterface();
        bool sendCANFrame(uint32_t can_id, unsigned char* CANMsg);
        bool receiveCANFrame(uint32_t can_id, unsigned char* CANMsg);

    private:
        // bool sendRawCANFrame(int can_id);
        // bool receiveRawCANFrame(int can_id);
        // bool encodeMsg();
        // bool decodeMsg();
        // bool receivedData;
        int socket_descrp; // File descriptor for the socket as everything in Linux/Unix is a file.

    };

}
#endif // CAN_INTERFACE_HPP
