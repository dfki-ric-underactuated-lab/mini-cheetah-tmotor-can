#include "CANInterface.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

namespace CAN_interface
{
    CANInterface::CANInterface(const char* socketName)
    {
        // const char* socketIfName = &socketName;  
        // int s;  // File descriptor for the socket as everything in Linux/Unix is a file. 
        struct sockaddr_can addr; // structure for CAN sockets : address family number AF_CAN
        struct ifreq ifr; // from if.h Interface Request structure used for all socket ioctl's. All interface ioctl's must have parameter definitions which begin with ifr name. The remainder may be interface specific.

        // socket(int domain, int type, int protocol): returns file descriptor int or -1 if fail
        if ((socket_descrp = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
        {
            perror("CANInterface: Error While Opening CAN Socket");
        }
        else {
            // Retrieve the interface index for the interface name (can0, can1, vcan0) to be used to the ifreq struct
            strcpy(ifr.ifr_name, socketName);

            // Send an I/O control call and pass an ifreq structure containing the interface name
            // ioctl() system call manipulates the underlying device parameters of special files. 
            // SIOCGIFINDEX Retrieve the interface index of the interface into ifr_ifindex insude ifr struct.
            ioctl(socket_descrp, SIOCGIFINDEX, &ifr);

            // with the interface index, now bind the socket to the CAN Interface
            struct sockaddr_can addr;

            // set address to all zeros. Done in example/man pages. But why?
            memset(&addr, 0, sizeof(addr));

            // Setup the interface parameters in the socketcan address struct
            addr.can_family = AF_CAN;
            addr.can_ifindex = ifr.ifr_ifindex;

            if (bind(socket_descrp, (struct sockaddr *)&addr, sizeof(addr)) < 0)
            {
               perror("CANInterface: Error while binding to the CAN Socket.");
            }
        }
    }

    bool CANInterface::sendCANFrame(uint32_t can_id, unsigned char* CANMsg)
    {
        struct can_frame frame;
        frame.can_id = can_id;
        frame.can_dlc = 8;
        memcpy(frame.data, CANMsg, 8);

        if (write(socket_descrp, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
        {
            perror("CANInterface: Error writing to CAN Interface.");
            return false;
        }
        else
        {
            return true;
        }
    }

    bool CANInterface::receiveCANFrame(uint32_t can_id, unsigned char* CANMsg)
    {
        // Need to implement the filterting of the messages from the CAN ID requested.
        // Currently the CAN ID is not used at all here as it listens to all CAN messages.

        struct can_frame frame;

        if (read(socket_descrp, &frame, sizeof(struct can_frame)) < 0)
        {
            perror("CANInterface: Error Reading Data.");
            return false;
        }
        else
        {
            memcpy(CANMsg, frame.data, frame.can_dlc);
            return true;
        }
    }

    CANInterface::~CANInterface() {

        if (close(socket_descrp) < 0) {
            perror("CANInterface: Error Closing CAN Socket.");
        }

    }
}