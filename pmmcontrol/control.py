import numpy as np
from pmmcontrol.arduino import Arduino
from time import sleep
from scipy.interpolate import interp1d


class Control(Arduino):
    '''
    Class for controlling the PMM readout. Built on the Arduino class from arduino.py
    '''
    def __init__(self, port, baud_rate=115200, self_test=False):
        # define number of rows and columns
        self.rows = 9
        self.cols = 10

        # define resistor values in np array
        self.R_row = 356 * np.ones(10)

        # define resistor values
        self.R_primary = 620
        self.R_auxiliary = 2400

        # value of current sense resistor
        self.R_sense = 0.1

        self.max_voltage = 4

        # PIN CONFIGURATION

        # COM line for each row/column
        row_primary_COM = [1, 3, 5, 7, 9, 11, 13, 15, 17]
        col_primary_COM = [19, 21, 23, 25, 27, 29, 31, 33, 35, 37]
        row_auxiliary_COM = [2, 4, 6, 8, 10, 12, 14, 16, 18]
        col_auxiliary_COM = [20, 22, 24, 26, 28, 30, 32, 34, 36, 38]
        sign_COM = {'positive': 39, 'negative': 40}

        # COMX: JX
        headerMap = {1:1,   2:3,   3:5,   4:7,   5:9,   6:11,  7:13,  8:15,  9:17,  10:19,
                     11:21, 12:23, 13:25, 14:27, 15:29, 16:31, 17:33, 18:35, 19:37, 20:39,
                     21:2,  22:4,  23:6,  24:8,  25:10, 26:12, 27:14, 28:16, 29:18, 30:20,
                     31:22, 32:24, 33:26, 34:28, 35:30, 36:32, 37:34, 38:36, 39:38, 40:40}

        # JX: PINX
        pinMap = {1:6,   2:7,   3:8,   4:9,   5:5,   6:3,   7:2,   8:4,   9:22,  10:23,
                  11:24, 12:25, 13:26, 14:27, 15:28, 16:29, 17:30, 18:31, 19:32, 20:33,
                  21:34, 22:35, 23:36, 24:37, 25:38, 26:39, 27:40, 28:41, 29:42, 30:43,
                  31:44, 32:45, 33:46, 34:47, 35:48, 36:49, 37:50, 38:51, 39:52, 40:53}

        # pins for enabling row and column switches at large current (small resistance)
        self.row_primary_pins = [pinMap[headerMap[row]] for row in row_primary_COM]
        self.col_primary_pins = [pinMap[headerMap[col]] for col in col_primary_COM]

        # pins for enabling row and column switches at small current (large resistance)
        self.row_auxiliary_pins = [pinMap[headerMap[row]] for row in row_auxiliary_COM]
        self.col_auxiliary_pins = [pinMap[headerMap[col]] for col in col_auxiliary_COM]

        # pins for enabling positive or negative current
        self.sign_pins = {'positive': pinMap[headerMap[sign_COM['positive']]], 'negative': pinMap[headerMap[sign_COM['negative']]]}

        # all output pins enabled for use
        self.enable_pins = self.row_primary_pins + self.row_auxiliary_pins + self.col_primary_pins + self.col_auxiliary_pins + list(self.sign_pins.values())

        # analog pin for current sense
        self.sense_pin = 1

        # initialize Arduino
        Arduino.__init__(self, port, baud_rate=baud_rate)

        # define output pins
        self.output(self.enable_pins)

        # calibrate max voltage for DAC output
        sleep(0.5)  # let Vcc equilibriate

        # ENABLE IF DAC ATTACHED
        #self.__calibrateDACVoltage()

        if self_test:
            self.testConfig()

    def __str__(self):
        # add digital readout info later
        return "Arduino is on port %s at %d baudrate" % (self.serial.port,
                                                         self.serial.baudrate)

    def testConfig(self):
        '''
        Performs a test of each row/column combination.
        '''

        # measure null current
        print("Measuring null current")
        self.null_current = self.readTotalCurrent()

        # results of testing each magnet at vmax and vmin
        self.pos_test = np.zeros((self.rows, self.cols))
        self.neg_test = np.zeros((self.rows, self.cols))

        # test each magnet positive and negative
        for row in range(0, self.rows):
            for col in range(0, self.cols):
                # select magnet
                print("Selecting magnet at coordinate {}, {}".format(row, col))
                self.selectMagnet(row, col)

                # set magnet high, low, then off
                print("Setting voltage high")
                self.setVoltage(5)
                sleep(0.01)
                print("Reading current")
                self.pos_test[row][col] = self.readTotalCurrent()

                print("Setting voltage low")
                self.setVoltage(-5)
                sleep(0.01)
                print("Reading total current")
                self.neg_test[row][col] = self.readTotalCurrent()

                self.setVoltage(0)
                sleep(0.01)

        # for debugging purposes
        print("Null current: ", self.null_current)
        print("High test: ", self.pos_test)
        print("Low test: ", self.neg_test)

        return True

    def selectMagnet(self, row, column, isPrimary=True, isArrayMode=False, sign='positive'):
        '''
        Selects the magnet in (row,column) by setting the row and column switch to enable
        in either the 'positive' or 'negative' configuration, and all of the other
        switches to disable (open circuit). A magnet selected as primary will be
        enabled with maximum current (small resistor).
        '''

        # check inputs
        if sign != 'positive' and sign != 'negative':
            raise ValueError("parameter 'sign' must be either 'positive' or 'negative'")
        if (not isinstance(row, int)) or row < 0 or row > self.rows - 1:
            raise ValueError("parameter 'row' must be an integer and between 0 and " +
                             str(self.rows - 1))
        if (not isinstance(column, int)) or column < 0 or column > self.cols - 1:
            raise ValueError("parameter 'column' must be an integer and between 0 and " +
                             str(self.cols - 1))

        # record state
        self.current_row = row
        self.current_column = column
        self.current_sign = sign
        self.current_isPrimary = isPrimary
        self.current_isArrayMode = isArrayMode

        # state values to be set later
        self.current_posDAC = None
        self.current_voltage = None

        return True

    def setCurrent(self, current):
        '''
        Sets a current by selecting an appropriate binary number for the DAC. The current
        flows through whichever magnet has been enabled by running selectMagnet first.
        Calling this with array parameter False will disable all other switches. With array
        parameter True, it will set all other magnets to minimum current in the opposite
        direction.
        '''
        if not hasattr(self, 'current_sign'):
            raise AttributeError("Some attributes have not been set. " +
                                 " Run 'selectMagnet()' first")
        if not self.current_isPrimary and self.current_isArrayMode:
            raise ValueError("Can only set currents in array mode if magnet has been selected as primary")

        # get required voltage
        voltage = self.__currentToVoltage(current)

        if np.abs(voltage) > self.max_voltage[self.current_row]:
            max_current = round(self.max_voltage[self.current_row] /
                                self.R_row[self.current_row], 4)
            raise ValueError('The maximum current allowed on this row is ' +
                             str(max_current)[:5] + ' A')

        self.setVoltage(voltage)

        return True

    def setVoltage(self, voltage):
        '''
        Sets a DAC voltage by selecting an appropriate binary number. The current
        flows through whichever magnet has been enabled by running selectMagnet first.
        Depending on the sign of the voltage, this is not necessarily the voltage accross
        the device.
        '''
        if not hasattr(self, 'current_sign'):
            raise AttributeError("Some attributes have not been set. " +
                                 " Run 'selectMagnet()' first")
        if not self.current_isPrimary and self.current_isArrayMode:
            raise ValueError("Can only set voltages in array mode if magnet has been selected as primary")
        if np.abs(voltage) > self.max_voltage:
            raise ValueError('The maximum voltage output is ' +
                             str(self.max_voltage) + ' V')

        # disable all switches
        for pin in self.enable_pins:
            self.setLow(pin)

        # set DAC to zero
        self.writeDAC(0)

        # enable switches (row, column) depending on whether the magnet is primary
        if self.current_isPrimary:
            self.setHigh(self.col_primary_pins[self.current_column])
            self.setHigh(self.row_primary_pins[self.current_row])
        else:
            self.setHigh(self.col_auxiliary_pins[self.current_column])
            self.setHigh(self.row_auxiliary_pins[self.current_row])

        self.setHigh(self.sign_pins[self.current_sign])

        # need to configure all other magnets if array mode is True
        if self.current_isArrayMode:
            # set all rows except selected to auxiliary function
            for row_pin_auxiliary, row_pin_primary in zip(self.row_auxiliary_pins, self.row_primary_pins):
                if row_pin_auxiliary != self.row_auxiliary_pins[self.current_row]:
                    self.setLow(row_pin_primary)
                    self.setHigh(row_pin_auxiliary)

            # set all columns except selected to auxiliary function
            for column_pin_auxiliary, column_pin_primary in zip(self.col_auxiliary_pins, self.col_primary_pins):
                if column_pin_auxiliary != self.col_auxiliary_pins[self.current_column]:
                    self.setLow(column_pin_primary)
                    self.setHigh(column_pin_auxiliary)


        # change enable pins if sign change is necessary
        if (np.sign(voltage) == -1 and self.current_sign == 'positive') or \
                (np.sign(voltage) == 1 and self.current_sign == 'negative'):
            self.setLow(self.sign_pins['positive'])
            self.setHigh(self.sign_pins['negative'])
            posDAC = False
        else:
            self.setLow(self.sign_pins['negative'])
            self.setHigh(self.sign_pins['positive'])
            posDAC = True

        # set voltage on DAC
        binary = int(np.abs(voltage)* 2**16 / self.Vcc)
        self.writeDAC(binary)

        self.current_posDAC = posDAC # keep track of sign of DAC
        self.current_voltage = voltage # keep track of voltage

        return True

    def updateCurrent(self, current):
        '''
        Updates the current of the currently selected magnet without resetting all the enables.
        Essentially just updates the DAC but cares for the sign of the voltage.
        '''

        if not hasattr(self, 'current_sign'):
            raise AttributeError("Some attributes have not been set. " +
                                 " Run 'selectMagnet()' first")
        if self.current_voltage == None:
            raise AttributeError("Initial current has not been set. " +
                                 " Run 'setCurrent()' first")
        if not self.current_isPrimary and self.isArrayMode:
            raise ValueError("Can only set currents in array mode if magnet has been selected as primary")

        # get required voltage
        voltage = self.__currentToVoltage(current)

        if np.abs(voltage) > self.max_voltage[self.current_row]:
            max_current = round(self.max_voltage[self.current_row] /
                                self.R_row[self.current_row], 4)
            raise ValueError('The maximum current allowed on this row is ' +
                             str(max_current)[:5] + ' A')

        self.updateVoltage(voltage)

        return True

    def updateVoltage(self, voltage):
        '''
        Updates the voltage of the currently selected magnet without resetting all the enables.
        Essentially just updates the DAC but cares for the sign of the voltage.
        '''

        if not hasattr(self, 'current_sign'):
            raise AttributeError("Some attributes have not been set. " +
                                 " Run 'selectMagnet()' first")
        if self.current_voltage == None:
            raise AttributeError("Initial voltage has not been set. " +
                                 " Run 'setVoltage()' first")
        if not self.current_isPrimary and self.isArrayMode:
            raise ValueError("Can only set voltages in array mode if magnet has been selected as primary")
        if np.abs(voltage) > self.max_voltage:
            raise ValueError('The maximum voltage output is ' +
                             str(self.max_voltage) + ' V')

        # change enable pins if sign change is necessary
        if (np.sign(voltage) == -1 and self.current_sign == 'positive') or \
                (np.sign(voltage) == 1 and self.current_sign == 'negative'):
            if self.current_posDAC: # only change enables if DAC is set positive
                self.setLow(self.sign_pins['positive'])
                self.setHigh(self.sign_pins['negative'])
                self.current_posDAC = False # update state
        else:
            if not self.current_posDAC: # only change enables if DAC is set negative
                self.setLow(self.sign_pins['negative'])
                self.setHigh(self.sign_pins['positive'])
                self.current_posDAC = True # update state

        # set voltage on DAC
        binary = int(np.abs(voltage) * 2 ** 16 / self.Vcc)
        self.writeDAC(binary)

        self.current_voltage = voltage

        return True

    def readTotalCurrent(self):
        '''
        Uses current sense circuit to measure the total current flowing to ground.
        '''

        sense_voltage = self.analogRead(self.sense_pin)

        # differential voltage across current sense IC with gain of 50 and offset of 2.5V
        diff_voltage = (sense_voltage - 7.5) / 50

        return diff_voltage / self.R_sense

    def resetMagnet(self):
        '''
        Removes the magnetization in the currently selected magnet by oscillating an
        exponentially decaying current. DAC finishes at 0V.
        '''

        if not hasattr(self, 'current_sign'):
            raise AttributeError("Some attributes have not been set. " +
                                 " Run 'selectMagnet()' first")
        if self.isArrayMode:
            raise ValueError("Can not reset magnet that has been selected in array mode. Please reselect magnet.")

        # set oscillating and exponentially decaying current through (row, column) magnet
        tt = np.arange(0, 70)
        max_current = round(self.max_voltage[self.current_row] /
                            self.R_row[self.current_row], 4)
        current_list = np.exp(-tt / 20.0) * np.cos(tt / 3.0) * max_current
        current_list = np.append(current_list, 0)

        # call setCurrent() first to allow updateCurrent()
        self.setCurrent(max_current)
        for current in current_list:
            self.updateCurrent(current)
            sleep(0.1)

        return True

    def resetAllMagnet(self):
        '''
        Resets the field of each magnet by iterating through rows and columns
        '''

        raise NotImplementedError

    def findResonances(self, frequency_range):
        '''
        Returns a list of resonance frequencies and IQ sweep data for the frequency range
        in frequency_range.
        '''
        raise NotImplementedError

    def identifyResonance(self, IQ_sweep, resonances, row, column):
        '''
        Uses the IQ_sweep and resonance list from findResonances to determine the
        resonance frequency of the resonator at (row, column) and outputs its sensitivity
        to the magnetic field.
        '''
        raise NotImplementedError

    def alignResonances(self, IQ_sweep, resonances, sensitivities):
        '''
        Uses the IQ_sweep and resonance list from findResonances and the sensitivities
        list from identifyResonance to compute and set the appropriate magnetizations to
        align the resonances.
        '''
        raise NotImplementedError

    # DEPRECATED
    def __changeSign(self):
        '''
        Changes the sign of the row and column currently selected by selectMagnet.
        selectMagnet must be run first. The DAC voltage remains at 0 V until reset.
        '''
        if hasattr(self, 'current_row') and hasattr(self, 'current_column') and \
           hasattr(self, 'current_sign'):
            # enforce going through 0
            self.writeDAC(0)
            # disable switches
            self.setLow(self.column_pins[self.current_column])
            self.setLow(self.row_pins[self.current_row])
            self.setLow(self.sign_pins[self.current_sign])
            # switch current sign variable
            if self.current_sign == 'positive':
                self.current_sign = 'negative'
            else:
                self.current_sign = 'positive'
            # enable switches
            self.setHigh(self.column_pins[self.current_column])
            self.setHigh(self.row_pins[self.current_row])
            self.setHigh(self.sign_pins[self.current_sign])
        else:
            raise AttributeError("Some attributes have not been set. " +
                                 " Run 'selectMagnet()' first")

        return True

    def __currentToVoltage(self, current):
        '''
        Converts the requested current into a voltage required to be output by the DAC.
        Each row has a large resistor whose value is recorded in R_row.
        '''

        voltage = self.R_row[self.current_row] * current

        return voltage

    def __calibrateDACVoltage(self):
        '''
        Measures the DAC nonlinearity and generates a 'linearity_correction' function for
        each row. This function compensates for the two main sources of nonlinearity:
        voltage errors and current draw errors. Needs to be run in the __init__()
        statement and assumes DAC is on, so run it after 'self.output()'.
        '''
        # ensure all switches are off
        for pin in self.enable_pins:
            self.setLow(pin)

        # read the voltage powering arduino to appropriately rescale results
        self.Vcc = self.readVcc()

        # write values to the DAC and read the result with the Arduino
        binary_list = np.arange(0, 2**16 + 3854, 3855)
        voltage_list = np.zeros(np.size(binary_list))
        for index, value in enumerate(binary_list):
            self.writeDAC(value)
            sleep(0.1)
            voltage_list[index] = self.readDAC()
        # return DAC to zero Volts
        self.writeDAC(0)

        # modify the read voltages to take into account DAC current nonlinearity
        poly = [5.03514759e+19, -2.77472015e+18, 6.27230841e+16, -7.51573164e+14,
                5.14163336e+12, -2.01184144e+10, 4.26235161e+07, -4.46109983e+04,
                1.65115877e+01, 0.0]
        current_correction = lambda x: np.polyval(poly, x)
        real_voltages = [voltage_list + current_correction(voltage_list / R)
                         for R in self.R_row]

        # change the numbers sent to the DAC to target voltages
        target_voltages = binary_list * self.Vcc / 2**16

        # create a list of linearity corrections, one for each row
        self.linearity_correction = []
        for index, _ in enumerate(self.R_row):
            self.linearity_correction.append(interp1d(real_voltages[index],
                                                      target_voltages, kind='linear'))

        # record the max voltage for each row
        self.max_voltage = []
        for voltages in real_voltages:
            self.max_voltage.append(voltages[-1])

        return True
