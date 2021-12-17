import spidev
import threading


# FSR class to read values from FAR sensors
# index of the sensor goes from 0 to 3
class FS (threading.Thread):

    def __init__(self):
        super(FS, self).__init__()

        self.__spi = spidev.SpiDev()
        self.__spi.open(0, 0)
        self.__spi.max_speed_hz = 1200000

        self.__fsr = None
        self.__lock = threading.Lock()
        self.__kill = threading.Event()
        self.__poll_interval = 0


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
            pad_value = [self.__read_adc(0),self.__read_adc(1),self.__read_adc(2),self.__read_adc(3)]
        return pad_value


    def kill(self):
        self.__kill.set()


    def run(self):
        print("Initiating FSR class")
        while True:
            with self.__lock:
                self.__fsr = [self.__read_adc(0),self.__read_adc(1),self.__read_adc(2),self.__read_adc(3)]

            is_killed = self.__kill.wait(self.__poll_interval * 1.0 / 1000.0)
            if is_killed:
                print("FSR class stopped!\n")
                break





