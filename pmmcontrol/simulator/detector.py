from pmmcontrol.simulator.hysteresis import Hysteresis
from scipy.constants import mu_0, pi
import numpy as np

class Detector():
    def __init__(self, rows=9, cols=10):
        self.rows = rows
        self.cols = cols

        self.row_currents = np.zeros(rows)
        self.col_currents = np.zeros(cols)

        #arbitrary matrix of evenly spaced frequencies
        self.fstart = 0 #MHz
        self.fstop = 20 #MHz
        self.frequencies = np.linspace(self.fstart, self.fstop, self.rows*self.cols).reshape(self.rows, self.cols)

        self.resArray = np.empty((self.rows, self.cols), dtype=object)
        self.resArray[:,:] = np.vectorize(Resonator)(self.frequencies)

    def setRowCurrent(self, I, row):
        self.row_currents[row] = I

    def setColCurrent(self, I, col):
        self.col_currents[col] = I

    def resPlot(self, row, col):
        self.resArray[row, col].plot()

    def displayCurrents(self):
        return False

    def updateRes(self):
        for row in range(self.rows):
            for col in range(self.cols):
                self.resArray[row, col].setCurrent(self.row_currents[row] + self.col_currents[col])

class Resonator(Hysteresis):
    def __init__(self, baseFreq, N=200, satField=0.0025, satMag=0.5,
                 remanence=0, coercivity=0.002):

        #Resonator properties
        self.__baseFreq = baseFreq
        self.__nLoops = 10
        self.__loopRadius = 0.0001
        self.__distLoopMag = 0

        self.__satMag = satMag
        self.__remanence = remanence
        self.__coercivity = coercivity
        self.__satField = satField

        #State variables
        self.__current = 0
        self.__B_ext = 0
        self.__mag = 0
        self.__B_res = 0
        self.__deltaFreq = 0
        self.__freq = self.__baseFreq

        self.__size = N

        Hysteresis.__init__(self, self.__satField, self.__satMag, self.__coercivity,
                            self.__remanence, self.__size, 'Applied Field (T)', 'Magnetization (T)')

    def setCurrent(self, I):

        self.__current = I

        #Compute B-field from current flow
        self.__B_ext = self.__currToField(I)

        #Compute resulting magnetization using hysteresis curve
        self.__mag = self.setX(self.__B_ext)

        self.__B_res = self.__magToField(self.__mag)

        self.__deltaFreq = self.__fieldToDeltaFreq(self.__B_res)

        self.__freq = self.__freq + self.__deltaFreq

        stateData = {'Current': self.__current,
                     'Exterior Field': self.__B_ext,
                     'Magnetization': self.__mag,
                     'Resonator Field': self.__B_res,
                     'Shift in Frequency': self.__deltaFreq,
                     'Current frequency': self.__freq}

        return stateData

    def __currToField(self, I): #Amps -> Tesla
        return (0.001) * (I / 0.0025)

    def __magToField(self, M): #Tesla -> Tesla
        return (0.00021) * (M / 0.5)

    def __fieldToDeltaFreq(self, B): #Tesla -> MHz
        slope = -(.08)/(0.0003**2)
        return slope * B