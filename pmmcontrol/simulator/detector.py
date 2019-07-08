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
        self.fstart = 0
        self.fstop = 20
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
    def __init__(self, baseFreq, N=100, satCurrent=0.020, satMag=1401.4, remanence=949.36):

        #Resonator properties
        self.__baseFreq = baseFreq
        self.__nLoops = 10
        self.__loopRadius = 0.0001
        self.__distLoopMag = 0

        self.__satCurrent = satCurrent
        self.__satMag = satMag
        self.__remanence = remanence
        self.__satField = self.__loopField(self.__satCurrent, self.__loopRadius, self.__distLoopMag,
                                                self.__nLoops)

        #State variables
        self.__freq = self.__baseFreq
        self.__mag = 0
        self.__fieldAtMagnet = 0
        self.__fieldAtRes = 0

        self.__size = N

        self.__satField = self.__loopField(self.__satCurrent, self.__loopRadius, self.__distLoopMag,
                                                self.__nLoops)

        Hysteresis.__init__(self, self.__satField, self.__satMag, self.__remanence, self.__size, 'Applied Field (T)',
                            'Magnetization (A/m)')

    def setCurrent(self, I):

        #Comute B-field from current flow
        self.__fieldAtMagnet = self.__loopField(I, self.__loopRadius,
                                                     self.__distLoopMag, self.__nLoops)

        #Compute resulting magnetization using hysteresis curve
        magnetization = self.setX(self.__fieldAtMagnet)
        self.__fieldAtRes = mu_0 * magnetization

        return self.__fieldToFreq(self.__fieldAtRes)

    def __loopField(self, I, R, Z, N):
        return N * (mu_0 / 2) * ((R**2 * I)/(Z**2 + R**2)**(3/2))

    def __fieldToFreq(self, H):
        deltaFreq = H**2
        self.__freq = self.__baseFreq + deltaFreq