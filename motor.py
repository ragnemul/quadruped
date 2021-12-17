from dynamixel_sdk import *


class Motor:

    # Control table address
    __ADDR_AX_TORQUE_ENABLE = 24  # Control table address is different in Dynamixel model
    __ADDR_AX_GOAL_POSITION = 30
    __ADDR_AX_MOVING_SPEED = 32
    __ADDR_AX_TORQUE_LIMIT = 34
    __ADDR_AX_PRESENT_POSITION = 36

    # Default setting

    __TORQUE_ENABLE = 1  # Value for enabling the torque
    __TORQUE_DISABLE = 0  # Value for disabling the torque

    __DXL_MINIMUM_POSITION_VALUE = 450  # Dynamixel will rotate between this value
    __DXL_MAXIMUM_POSITION_VALUE = 550  # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
    __DXL_MOVING_STATUS_THRESHOLD = 5  # Dynamixel moving status threshold

    __dxl_id = None
    __packetHandler = None
    __portHandler = None
    __speed = None


    def __init__(self, packetHandler, portHandler, dxl_id):
        self.__packetHandler = packetHandler
        self.__portHandler = portHandler
        self.__dxl_id = dxl_id


    def enable_torque(self):
        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = self.__packetHandler.write1ByteTxRx(self.__portHandler,
                                                                         self.__dxl_id, self.__ADDR_AX_TORQUE_ENABLE,
                                                                         self.__TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.__packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.__packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel has been successfully connected")
        return dxl_error


    def disable_torque(self):
        # Disable Dynamixel Torque
        dxl_comm_result, dxl_error = self.__packetHandler.write1ByteTxRx(self.__portHandler,
                                                                         self.__dxl_id, self.__ADDR_AX_TORQUE_ENABLE,
                                                                         self.__TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.__packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.__packetHandler.getRxPacketError(dxl_error))
        return dxl_error


    def set_max_torque(self, torque):
        dxl_comm_result, dxl_error = self.__packetHandler.write2ByteTxRx(self.__portHandler,
                                                                         self.__dxl_id, self.__ADDR_AX_TORQUE_LIMIT,
                                                                         torque)
        return dxl_error


    def set_goal_position(self, goal_position):
        # Write goal position
        dxl_comm_result, dxl_error = self.__packetHandler.write2ByteTxRx(self.__portHandler, self.__dxl_id,
                                                                         self.__ADDR_AX_GOAL_POSITION,
                                                                         goal_position)
        """
        while 1:
            # Read present position
            dxl_present_position, dxl_error = self.get_present_position()
            #print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (self.__dxl_id, goal_position, dxl_present_position))

            if not (abs(goal_position - dxl_present_position) > self.__DXL_MOVING_STATUS_THRESHOLD):
                break
        """

        if dxl_comm_result != COMM_SUCCESS:
            print("%s " % self.__packetHandler.getTxRxResult(dxl_comm_result))
            print (self.__dxl_id)
        elif dxl_error != 0:
            print("%s " % self.__packetHandler.getRxPacketError(dxl_error))
            print(self.__dxl_id)
        return dxl_error


    def get_present_position(self):
        # Read present position
        dxl_present_position, dxl_comm_result, dxl_error = self.__packetHandler.read2ByteTxRx(self.__portHandler,
                                                                                              self.__dxl_id,
                                                                                              self.__ADDR_AX_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.__packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.__packetHandler.getRxPacketError(dxl_error))
        return dxl_present_position, dxl_error


    def set_moving_speed(self,speed):
        dxl_comm_result, dxl_error = self.__packetHandler.write2ByteTxRx(self.__portHandler,
                                                                        self.__dxl_id, self.__ADDR_AX_MOVING_SPEED,
                                                                        speed)
        return dxl_error


    def get_moving_speed(self):
        dxl_present_speed, dxl_comm_result, dxl_error = self.__packetHandler.read2ByteTxRx(self.__portHandler,
                                                                        self.__dxl_id,
                                                                        self.__ADDR_AX_MOVING_SPEED)
        if (dxl_present_speed == 0):
            dxl_present_speed = 1023

        return dxl_present_speed, dxl_comm_result, dxl_error



