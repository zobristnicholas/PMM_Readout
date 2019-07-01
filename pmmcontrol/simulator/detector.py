from pmmcontrol.simulator.hysteresis import Hysteresis
import numpy as np

class Detector():
    def __init__(self, rows=9, cols=10):
        self.rows = rows
        self.cols = cols

        self.row_currents = np.zeros(rows)
        self.col_currents = np.zeros(cols)

        #arbitrary matrix of evenly spaced frequencies
        self.fstart = 0
        self.fstop = 20
        self.frequencies = np.linspace(self.fstart, self.fstop, self.rows*self.cols).reshape(self.rows, self.cols)

        self.resArray = np.empty((self.rows, self.cols), dtype=object)
        self.resArray[:,:] = np.vectorize(Resonator)(self.frequencies)

    def setRowCurrent(self, I, row):
        self.row_currents[row] = I
        for col in range(self.cols):
            self.__updateRes(row, col)

    def setColCurrent(self, I, col):
        self.col_currents[col] = I
        for row in range(self.rows):
            self.__updateRes(row, col)

    def displayCurrents(self):
        return False

    def __updateRes(self, row, col):
        self.resArray[row, col].setCurrent(self.row_currents[row] + self.col_currents[col])

class Resonator(Hysteresis):
    def __init__(self, freq, N=10, sat_current=30):
        self.__sat_current = sat_current

        self.__freq = freq
        self.__field = 0

        self.__size = N

        Hysteresis.__init__(self, N)

    def setCurrent(self, I):
        self.freq = self.__currentToFreq(I)

        return True

    def getFrequency(self):
        return self.__freq

    def __currentToFreq(self, I):
        scale_factor = (self.__size / 2) / self.__max_current
        self.__field = self.setX(int(scale_factor * I))

        return self.__fieldToFreq(self.__field)

    def __fieldToFreq(self, B):
        self.__freq = self.__freq + B