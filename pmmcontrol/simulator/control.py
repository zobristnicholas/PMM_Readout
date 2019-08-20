from pmmcontrol.simulator.detector import Detector
import numpy as np
from time import time, sleep
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from scipy.signal import find_peaks

class Control(Detector):

    def __init__(self):
        # define number of rows and columns
        self.rows = 3
        self.columns = 4

        # define resistor values
        self.R_primary = 620
        self.R_auxiliary = 2400

        #The furthest we expect to be able to shift frequency
        self.__maxFreqShift = 0.2 #MHz

        self.max_voltage = 14

        Detector.__init__(self, self.rows, self.columns)

    def selectMagnet(self, row, column, isArrayMode=False, isPrimary=True, sign='positive'):

        # check inputs
        if sign != 'positive' and sign != 'negative':
            raise ValueError("parameter 'sign' must be either 'positive' or 'negative'")
        if (not isinstance(row, int)) or row < 0 or row > self.rows - 1:
            raise ValueError("parameter 'row' must be an integer and between 0 and " +
                             str(self.rows - 1))
        if (not isinstance(column, int)) or column < 0 or column > self.columns - 1:
            raise ValueError("parameter 'column' must be an integer and between 0 and " +
                             str(self.columns - 1))

        # record state
        self.curr_row = row
        self.curr_column = column
        self.curr_sign = sign
        self.curr_isPrimary = isPrimary
        self.curr_isArrayMode = isArrayMode

        # state values to be set later
        self.curr_posDAC = None
        self.curr_voltage = None

        return True

    def setCurrent(self, current):
        '''
        Set current in milli amps
        '''
        if not hasattr(self, 'curr_sign'):
            raise AttributeError("Some attributes have not been set. " +
                                 " Run 'selectMagnet()' first")
        if not self.curr_isPrimary and self.curr_isArrayMode:
            raise ValueError("Can only set currents in array mode if magnet has been selected as primary")

        #in mA
        set_current = current / 1000

        #adjust sign of current
        if self.curr_sign == 'negative':
            set_current = -set_current

        max_current = round((2*self.max_voltage) /
                            self.__chooseResistor(self.curr_isPrimary), 4)

        if set_current > max_current:
            raise ValueError('The maximum current allowed on any magnet is ' +
                             str(max_current * 1000)[:5] + ' mA')

        # set all rows and columns to zero current
        for row in range(self.rows):
            self.setRowCurrent(0, row)
        for col in range(self.cols):
            self.setColCurrent(0, col)

        # simulate turning off the DAC before adjusting pins
        self.updateRes()

        # to achieve I current on current magnet, we send I/2 down the row and the column
        self.setRowCurrent(set_current/2, self.curr_row)
        self.setColCurrent(set_current/2, self.curr_column)

        # if in array mode we need to send (I/2)/3 in the opposite direction across all other rows/cols
        if self.curr_isArrayMode:
            for row in range(self.rows):
                if row != self.curr_row:
                    self.setRowCurrent(-set_current/6, row)
            for col in range(self.cols):
                if col != self.curr_column:
                    self.setColCurrent(-set_current/6, col)

        # simulate turning on the DAC after setting all pins
        self.updateRes()

        return True

    def resetMagnet(self):
        '''
        Removes the magnetization in the currently selected magnet by oscillating an
        exponentially decaying current. DAC finishes at 0V.
        '''

        if not hasattr(self, 'curr_sign'):
            raise AttributeError("Some attributes have not been set. " +
                                 " Run 'selectMagnet()' first")
        if self.curr_isArrayMode:
            raise ValueError("Can not reset magnet that has been selected in array mode. Please reselect magnet.")

        # set oscillating and exponentially decaying current through (row, column) magnet
        tt = np.arange(0, 70)
        max_current = self.__fieldToCurrent(self.resArray[self.curr_row, self.curr_column].properties['SatField'])
        current_list = np.exp(-tt / 20.0) * np.cos(tt / 3.0) * max_current
        current_list = np.append(current_list, 0)

        # call setCurrent() first to allow updateCurrent()

        print("Resetting...")
        self.setCurrent(max_current)
        for current in 1000 * current_list:
            self.setCurrent(current)
            #sleep(.1)
        print("Reset complete.")

        return True

    def resetMagnetRect(self):
        '''
        Duplicates resetMagnet() for rectangular model
        '''

        if not hasattr(self, 'curr_sign'):
            raise AttributeError("Some attributes have not been set. " +
                                 " Run 'selectMagnet()' first")
        #if self.curr_isArrayMode:
        #    raise ValueError("Can not reset magnet that has been selected in array mode. Please reselect magnet.")

        sat_current = self.__fieldToCurrent(self.resArray[self.curr_row, self.curr_column].properties['SatField'])
        co_current = self.__fieldToCurrent(self.resArray[self.curr_row, self.curr_column].properties['Coercivity'])

        #print("Saturation Current:", sat_current)
        #print("Coercivity Current:", co_current)

        #print("Resetting...")
        self.setCurrent(sat_current * 1000)
        self.setCurrent(-co_current * 1000)
        self.setCurrent(0)
        #print("Reset complete.")

        return True

    def resetAllMagnet(self):

        for row in range(self.rows):
            for col in range(self.columns):
                self.selectMagnet = None

        raise NotImplementedError

    def findRes(self):

        f_array, s_array = self.readOut(4925, 5075)

        peaks, properties = find_peaks(-s_array, height=5)

        print("Found " + str(len(peaks)) + " out of " + str(self.rows * self.cols) + " peaks:")
        print(np.take(f_array, peaks))

        plt.figure()
        plt.plot(f_array, s_array)
        plt.plot(np.take(f_array, peaks), np.take(s_array, peaks), linestyle='', marker='x', color='green')
        plt.xlabel("Frequency [MHz]")
        plt.ylabel("|S21| [Db]")
        plt.show()


    def resId(self):
        deltaFreq_sat = .0392

        # shift from saturation field to remanence field and see which frequency matches both guesses
        #deltaFreq_rem = 4

        indexRange = 200

        freqList = np.ones((self.rows, self.columns), dtype='float') * -1

        for row in range(self.rows):
            for col in range(self.columns):

                freqInitial = self.frequencies

                indexGuess = (row * self.columns) + col

                idxLower = indexGuess - indexRange
                idxUpper = indexGuess + indexRange

                #searchFreqInitial = freqInitial[idxLower:idxUpper]
                searchFreqInitial = freqInitial

                sat_current = self.__fieldToCurrent(self.resArray[row, col].properties['SatField'])

                self.selectMagnet(row, col, True)
                self.setCurrent(sat_current * 1000)

                freqFinal = self.frequencies

                #searchFreqFinal = freqFinal[idxLower:idxUpper]
                searchFreqFinal = freqFinal

                searchFreqDiff = np.ones((len(searchFreqInitial),len(searchFreqFinal)))

                for finalIdx, finalFreq in enumerate(searchFreqFinal):
                    for initialIdx, initialFreq in enumerate(searchFreqInitial):
                        searchFreqDiff[initialIdx, finalIdx] = finalFreq-initialFreq


                searchFreqErr = np.abs(np.round(searchFreqDiff + deltaFreq_sat,5))

                searchFreqCoord = np.unravel_index(np.argmin(searchFreqErr, axis=None),
                                                   searchFreqErr.shape)

                idxInitial = searchFreqCoord[0]
                idxFinal = searchFreqCoord[1]


                #indexFound = idxInitial + indexLower
                indexFound = idxInitial
                resFreq = freqInitial[indexFound]

                freqList[row, col] = resFreq

                self.resetMagnetRect()

        self.__plotFreqs(freqList)

        return freqList



    def __plotFreqs(self, freqs):
        labels = np.array([])
        data = np.array([])
        for idx, freq in np.ndenumerate(freqs):
            labels = np.append(labels, str(idx))
            data = np.append(data, freq)

        fig = plt.figure(figsize=(8, 1.5))
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
        ax.set_xlim(int(np.amin(data)), int(np.amax(data)) + 1)
        ax.set_ylim(0, 1)
        ax.patch.set_alpha(0.0)

        ax.xaxis.set_major_locator(ticker.AutoLocator())
        ax.xaxis.set_minor_locator(ticker.AutoMinorLocator())

        ax.plot(data, np.zeros(data.size), '-bD', linestyle='')

        for idx, label in enumerate(labels):
            ax.annotate(label, xy=(data[idx], 0),
                        xytext=(0, 10), textcoords='offset pixels',
                        rotation=90, ha='left', va='bottom')

        ax.set_xlabel('Frequency (MHz)')
        ax.set_title('')

        plt.show()

        return True


    def testFreqShift(self, I):
        '''
        reset all magnets
        find first isolated frequency (none in +/- max shift)
        shift frequencies in the area and record the max and min shifts
        These are the expected primary and secondary shifts for this current
        return {"PrimaryShift": primaryShift, "SecondaryShift": secondaryShift}
        '''

        frequencies = self.frequencies
        resIdx = None

        # find suitable resonator to test
        if len(frequencies) < 1:
            raise ValueError("There are no resonators to test")
        elif len(frequencies) == 1:
            resIdx = 0
        elif len(frequencies) == 2:
            if (frequencies[1] - frequencies[0]) > self.__maxFreqShift:
                resIdx = 0
        else:
            for idx, f in enumerate(frequencies[1,-1]):
                if ((frequencies[idx+1] - frequencies[idx]) > self.__maxFreqShift) and \
                        ((frequencies[idx+1] - frequencies[idx]) > self.__maxFreqShift):
                    resIdx = idx
                    break

        if resIdx == None:
            raise ValueError("There are no suitable resonators to test")

        raise NotImplementedError

    #
    # Data retrieval
    #

    def showHistory(self):
        '''
        Shows plot of the history of the selected magnet
        '''

        if not hasattr(self, 'curr_sign'):
            raise AttributeError("Some attributes have not been set. " +
                                 " Run 'selectMagnet()' first")

        self.resPlot(self.curr_row, self.curr_column)

    def showState(self):
        return self.resArray[self.curr_row, self.curr_column].state

    def showProperties(self):
        return self.resArray[self.curr_row, self.curr_column].properties

    #
    # Utility functions
    #

    def testId(self):
        pass1, pass2 = 0, 0
        tSum1, tSum2 = 0, 0
        trials = 5

        for n in range(trials):
            print("Trial #", n+1)

            Detector.__init__(self, self.rows, self.columns)

            self.plotState('Frequency', 'Trial #' + str(n+1))

            freq = self.getState('Frequency', False)
            sortIdx = np.argsort(freq)
            freqIdx = np.argsort(sortIdx)
            realOrder = freqIdx.reshape(self.rows, self.columns)

            print("Looking for...")
            print(realOrder)
            print()

            start1 = time()
            m1 = self.resIdBasic()
            end1 = time()
            elapsed1 = end1-start1

            if np.array_equal(realOrder, m1):
                print("Good! \n")
                pass1 = pass1 + 1
            else:
                print("Wrong! \n")

            start2 = time()
            m2 = self.resId()
            end2 = time()
            elapsed2 = end2-start2

            if np.array_equal(realOrder, m2):
                print("Good! \n")
                pass2 = pass2 + 1
            else:
                print("Wrong! \n")

            tSum1 = tSum1 + elapsed1
            tSum2 = tSum2 + elapsed2

            print()

        print("Successes for #1: " + str(pass1) + " out of " + str(trials))
        print("Successes for #2: " + str(pass2) + " out of " + str(trials))
        print("Average Time #1: ", tSum1/trials)
        print("Average Time #2: ", tSum2/trials)

    def hystTest(self):
        xsat_arr = np.linspace(1, 10, 10)
        ysat = 1

        yrem_arr = []

        for s in xsat_arr:
            hyst = Hysteresis(s, ysat, s/2)
            hyst.setX(s)
            yrem = hyst.setX(0)
            hyst.plot()
            yrem_arr.append(yrem)

        yrem_arr = np.array(yrem_arr)
        rem_ratio_arr = yrem_arr / ysat

        print(rem_ratio_arr)

        plt.figure()
        plt.plot(xsat_arr, rem_ratio_arr)
        plt.title("Remanence Ratio vs. Saturation X-Value")
        plt.show()


        xcoerc_arr = np.linspace(0.5, 0.9, 10)
        ysat = 1
        xsat = 1

        yrem_arr = []

        for c in xcoerc_arr:
            hyst = Hysteresis(xsat, ysat, c)
            hyst.setX(xsat)
            yrem = hyst.setX(0)
            hyst.plot()
            yrem_arr.append(yrem)

        yrem_arr = np.array(yrem_arr)
        rem_ratio_arr = yrem_arr / ysat
        coer_ratio_arr = xcoerc_arr / xsat

        print(rem_ratio_arr)

        plt.figure()
        plt.plot(xcoerc_arr, rem_ratio_arr)
        plt.title("Remanence Ratio vs. Coercivity X-Value")
        plt.show()

        plt.figure()
        plt.plot(coer_ratio_arr, rem_ratio_arr)
        plt.title("Remanence Ratio vs. Coercivity Ratio")
        plt.plot()

    def __currentToVoltage(self, current):
        '''
        Converts the requested current into a voltage required to be output by the DAC.
        Each row has a large resistor whose value is recorded in R_row.
        '''

        voltage = self.__chooseResistor(self.curr_isPrimary) * current

        return voltage

    def __chooseResistor(self, primary):
        if primary:
            return self.R_primary
        if not primary:
            return self.R_auxiliary

    def __fieldToCurrent(self, B): #G -> Amp
        return (0.0025) * (B / 10)