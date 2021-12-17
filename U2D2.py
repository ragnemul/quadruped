from dynamixel_sdk import *


class U2D2:
    # Protocol version
    __PROTOCOL_VERSION = 1.0  # See which protocol version is used in the Dynamixel

    # Default setting
    __BAUDRATE = 1000000  # Dynamixel default baudrate : 57600

    __portHandler = None
    __packetHandler = None

    def __init__(self, device_name):
        self.__device_name = device_name  # Check which port is being used on your controller

        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.__portHandler = PortHandler(self.__device_name)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.__packetHandler = PacketHandler(self.__PROTOCOL_VERSION)

        # Open port
        if self.__portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            quit()

        # Set port baudrate
        if self.__portHandler.setBaudRate(self.__BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            quit()


    def close_port(self):
        self.__portHandler.closePort()


    def get_portHandler(self):
        return self.__portHandler

    def get_packetHandler(self):
        return self.__packetHandler
