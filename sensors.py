import sys
import RTIMU
import os.path
import time
import math
import threading
import numpy as np
from collections import deque
import spidev


sys.path.append('.')


SETTINGS_FILE = "RTIMULib"

class SENSORS (threading.Thread):

    __init = False
    __imu = None
    __pool_interval = None
    __fusionPose = None
    __gyro = None

    __lock = threading.Lock()
    __kill = threading.Event()

    __pose_offset = [0,0,0]
    __log = deque(maxlen=50000)

    # FS specific variables
    __spi = spidev.SpiDev()
    __spi.open(0, 0)
    __spi.max_speed_hz = 1200000
    __fsr = None


    def __init__(self, group = None, target = None, name=None, args=(), kwargs=None, verbose=None):
        super(SENSORS, self).__init__()
        self.target = target
        self.name = name


    def run(self):
        print("Using settings file " + SETTINGS_FILE + ".ini")
        if not os.path.exists(SETTINGS_FILE + ".ini"):
            print("Settings file does not exist, will be created")

        s = RTIMU.Settings(SETTINGS_FILE)
        self.__imu = RTIMU.RTIMU(s)

        print("IMU Name: " + self.__imu.IMUName())

        if (not self.__imu.IMUInit()):
            print("IMU Init Failed")
            sys.exit(1)
        else:
            print("IMU Init Succeeded")

        # this is a good time to set any fusion parameters

        self.__imu.setSlerpPower(0.02)
        self.__imu.setGyroEnable(True)
        self.__imu.setAccelEnable(True)
        self.__imu.setCompassEnable(True)

        self.__poll_interval = self.__imu.IMUGetPollInterval()
        print("Recommended Poll Interval: %dmS\n" % self.__poll_interval)

        self.__init = True

        while True:
            if self.__imu.IMURead():
                data = self.__imu.getIMUData()
                with self.__lock:
                    self.__fusionPose = data["fusionPose"]
                    self.__gyro = data["gyro"]
                    self.__fsr = [self.__read_adc(0), self.__read_adc(1), self.__read_adc(2), self.__read_adc(3)]
                    self.__log.append([time.time(), self.__fusionPose, self.__gyro, self.__fsr])

                is_killed = self.__kill.wait(self.__poll_interval * 1.0 / 1000.0)
                if is_killed:
                    print ("IMU class stopped!\n")
                    break
        return

    def kill(self):
        self.__kill.set()


    def readIMU (self):
        with self.__lock:
            value = self.__fusionPose
        #print (value)
        return value


    def reset_offset (self):
        # waits until IMU is ready to read values
        time.sleep(2)
        with self.__lock:
            self.__pose_offset = self.__fusionPose
        return


    def reset_log(self):
        self.__log.clear()
        return

    def get_log(self):
        return self.__log.copy()


    # private read_adc reads the FSR sensor specified by adc_num
    def __read_adc(self, adc_num):
        # read SPI data from the MCP3008, 8 channels in total
        if adc_num > 7 or adc_num < 0:
            return -1
        r = self.__spi.xfer2([1, 8 + adc_num << 4, 0])

        data = ((r[1] & 3) << 8) + r[2]
        return data


    # returns the FSR value
    def get_fsr(self):
        with self.__lock:
            pad_value = [self.__read_adc(0), self.__read_adc(1), self.__read_adc(2), self.__read_adc(3)]
        return pad_value







