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
    def __init__(self, freq, N=10):
        self.freq = freq

        Hysteresis.__init__(self, N)

    def setCurrent(self, I):
        self.freq = self.__currentToFreq(I)

    def __currentToFreq(self, I):
        return I

    def __hystCurve(self, x):
        return False