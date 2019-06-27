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

        vResonator = np.vectorize(Resonator)

        resArray = np.empty((self.rows, self.cols), dtype=object)
        resArray[:,:] = vResonator(self.frequencies)

    def setRowCurrent(self, I, row):

    def setColCurrent(self, I, col):

    def setFrequency(self, row, col):

    def __currentToFreq(self, I):

    def __freqToCurrent(self, f):

class Resonator():
    def __init__(self, freq):
        self.freq = freq

    def hystCurve: