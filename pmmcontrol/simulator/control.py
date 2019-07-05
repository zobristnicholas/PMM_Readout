from pmmcontrol.simulator.detector import Detector
import numpy as np
from time import sleep

class Control(Detector):

    def __init__(self):
        # define number of rows and columns
        self.rows = 9
        self.columns = 10

        # define resistor values
        self.R_primary = 620
        self.R_auxiliary = 2400

        self.max_voltage = 14

        Detector.__init__(self, self.rows, self.columns)

    def selectMagnet(self, row, column, isPrimary=True, isArrayMode=False, sign='positive'):

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

        max_current = round(self.max_voltage /
                            self.__chooseResistor(self.curr_isPrimary), 4)

        if set_current > max_current:
            raise ValueError('The maximum current allowed on this row is ' +
                             str(max_current * 1000)[:5] + ' mA')

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
        max_current = round(self.max_voltage /
                            self.__chooseResistor(self.curr_isPrimary), 4)
        current_list = np.exp(-tt / 20.0) * np.cos(tt / 3.0) * max_current
        current_list = np.append(current_list, 0)

        # call setCurrent() first to allow updateCurrent()
        self.setCurrent(max_current)
        for current in 1000 * current_list:
            print("Setting current to: ", current)
            self.setCurrent(current)
            sleep(0.1)

        return True

    def showHistory(self):
        '''
        Shows plot of the history of the selected magnet
        '''

        if not hasattr(self, 'curr_sign'):
            raise AttributeError("Some attributes have not been set. " +
                                 " Run 'selectMagnet()' first")

        self.resPlot(self.curr_row, self.curr_column)

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