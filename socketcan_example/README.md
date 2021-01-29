# SocketCAN Example

A Simple SocketCAN example in `C`. No extra dependencies required.

For the original code, see [this](https://github.com/craigpeacock/CAN-Examples) Github repository.

The tutorial that uses this code to explain SocketCAN usage can be found [here](https://www.beyondlogic.org/example-c-socketcan-code/).

SocketCAN Linux Documentation can be found [here](https://www.kernel.org/doc/html/latest/networking/can.html).


## Setup Virtual CAN Interface for testing

To enable virtual SocketCAN interface run:

```
$ sudo ip link add dev vcan0 type vcan

$ sudo ip link set up vcan0
```

## Build Instructions

Use GCC to build the examples:

```
$ gcc cantransmit.c -o cantransmit

$ gcc canreceive.c -o canreceive

$ gcc canfilter.c -o canfilter
```