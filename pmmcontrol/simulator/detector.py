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

class Resonator():
    def __init__(self, freq):
        self.freq = freq

    def setCurrent(self, I):
        self.freq = self.__currentToFreq(I)

    def __currentToFreq(self, I):
        return I

    def __hystCurve(self, x):
        return False

class HystCurve():
    def __init__(self, N, alpha=1, beta=1):
        self.x = int(0)
        self.y = int(0)
        self.fill = np.ones((N,N))
        self.weights = np.ones((N,N))

    # DONT USE -- DOESNT WORK
    def move(self, x):
        if x > self.x:
            for row in range(0, int(x)):
                row.fill(1)

        y = np.sum([np.dot(frow, wrow) for frow, wrow in zip(self.fill, self.weights)])

        return y