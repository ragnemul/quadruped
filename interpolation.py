import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt

class Interpolation:
    # initial Y nudes coordinate
    __Y = None
    # initial Z nudes coordinate
    __Z = None
    # step for interpolation
    __step = None

    # interpolation function
    __f = None

    # nudes coordinates (interpolated)
    __y = None
    __z = None

    def __init__(self):
        pass


    def interpolate(self, Y, Z, step):
        self.__Y = Y
        self.__Z = Z
        self.__step = step

        self.__f = interpolate.PchipInterpolator(self.__Y, self.__Z)


    def get_interpolated_points(self):
        self.__y = []
        self.__z = []
        for i in np.arange(self.__Y[0], self.__Y[len(self.__Y)-1] + self.__step, self.__step):
            self.__y.append(i)
            self.__z.append(np.round(self.__f(i),2))

        return [self.__y, self.__z]



