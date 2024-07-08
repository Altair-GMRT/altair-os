import os
from dynamixel_sdk import *

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch



PROTOCOL_VERSION = 2.0
BAUDRATE         = 1000000
DEVICENAME       = '/dev/ttyUSB0'


def scan():
    for id in range(1, 253):
        model_number, comm_result, dxl_error = packetHandler.ping(portHandler, id)
        if (comm_result == COMM_SUCCESS):
            print("\n                                          ... SUCCESS \r")
            print(" [ID:%d] Model No : %d \n", id, model_number)
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print(".")
        getch()
        quit()
        

try:
    portHandler = PortHandler(DEVICENAME)
except:
    print("Device Not Found\n")
    print("Press any key to terminate...")
    getch()
    quit()

packetHandler = PacketHandler(PROTOCOL_VERSION)

print("\n***********************************************************************\n");
print("*                            DXL Monitor                              *\n");
print("***********************************************************************\n\n");
# Open port
try:
    if portHandler.openPort():
        print("Succeeded to open the port")
except:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
try:
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
except:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

scan()

portHandler.closePort()

