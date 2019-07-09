from pmmcontrol.simulator.hysteresis import Hysteresis
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

from random import randrange

class Detector():
    def __init__(self, rows=9, cols=10):
        self.rows = rows
        self.cols = cols

        self.row_currents = np.zeros(rows)
        self.col_currents = np.zeros(cols)

        # arbitrary matrix of evenly spaced frequencies
        self.fstart = 1 #MHz
        self.fstop = 3 #MHz
        self.baseFrequencies = np.linspace(self.fstart, self.fstop, self.rows*self.cols).reshape(self.rows, self.cols)

        # scatter base frequencies
        for idx, freq in enumerate(self.baseFrequencies):
            self.baseFrequencies[idx] = freq + (randrange(-1000, 1000)/1000) * 2 * ((self.fstop - self.fstart) /
                                                    (self.rows*self.cols -1))

        self.resArray = np.empty((self.rows, self.cols), dtype=object)
        self.resArray[:,:] = np.vectorize(Resonator)(self.baseFrequencies)

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

    def getState(self, param):
        # labels = np.array([])
        data = np.array([])
        for idx, res in np.ndenumerate(self.resArray):
            # labels = np.append(labels, str(idx))
            data = np.append(data, res.state[param])

        return data

    def plotState(self, param):
        #labels = np.array([])
        data = np.array([])
        for idx, res in np.ndenumerate(self.resArray):
            #labels = np.append(labels, str(idx))
            data = np.append(data, res.state[param])

        fig = plt.figure(figsize=(8, 1))
        ax = fig.add_subplot(1, 1, 1)

        ax.spines['right'].set_color('none')
        ax.spines['left'].set_color('none')
        ax.yaxis.set_major_locator(ticker.NullLocator())
        ax.spines['top'].set_color('none')
        ax.xaxis.set_ticks_position('bottom')
        ax.tick_params(which='major', width=1.00)
        ax.tick_params(which='major', length=5)
        ax.tick_params(which='minor', width=0.75)
        ax.tick_params(which='minor', length=2.5)
        ax.set_xlim(self.fstart - 0.5, self.fstop + 0.5)
        ax.set_ylim(0, 1)
        ax.patch.set_alpha(0.0)

        ax.xaxis.set_major_locator(ticker.AutoLocator())
        ax.xaxis.set_minor_locator(ticker.AutoMinorLocator())

        ax.plot(data, np.zeros(data.size), '-bD', linestyle='')

        #for idx, label in enumerate(labels):
        #    ax.annotate(label, xy=(frequencies[idx], 0),
        #                xytext=(0,10), textcoords='offset pixels')

        ax.set_xlabel('Frequency (MHz)')

        plt.show()

        return True


class Resonator(Hysteresis):
    def __init__(self, baseFreq, N=200, satField=25, satMag=0.5,
                 remanence=0, coercivity=20):

        #Resonator properties
        self.__baseFreq = baseFreq
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
                            self.__remanence, self.__size, 'Applied Field (G)', 'Magnetization (T)')

    def setCurrent(self, I):

        self.__current = I

        #Compute B-field from current flow
        self.__B_ext = self.__currToField(I)

        #Compute resulting magnetization using hysteresis curve
        self.__mag = self.setX(self.__B_ext)

        self.__B_res = self.__magToField(self.__mag)

        self.__deltaFreq = self.__fieldToDeltaFreq(self.__B_res)

        self.__freq = self.__baseFreq + self.__deltaFreq

        stateData = {'Current': self.__current,
                     'ExtField': self.__B_ext,
                     'Magnetization': self.__mag,
                     'ResField': self.__B_res,
                     'DeltaFrequency': self.__deltaFreq,
                     'Frequency': self.__freq}

        return stateData

    def __currToField(self, I): #Amps -> Gauss
        return (10) * (I / 0.0025)

    def __magToField(self, M): #Tesla -> Tesla
        return (0.00021) * (M / 0.5)

    def __fieldToDeltaFreq(self, B): #Tesla -> MHz
        slope = -(.08)/(0.0003**2)
        return slope * (B)**2

    @property
    def state(self):
        stateData = {'Current': self.__current,
                     'ExtField': self.__B_ext,
                     'Magnetization': self.__mag,
                     'ResField': self.__B_res,
                     'DeltaFrequency': self.__deltaFreq,
                     'Frequency': self.__freq}

        return stateData

    @property
    def properties(self):
        propertyData = {'BaseFrequnecy': self.__baseFreq,
                      'SatField': self.__satField,
                      'SatMagnetization': self.__satMag,
                      'Remanence': self.__remanence,
                      'Coercivity': self.__coercivity}

        return propertyData

