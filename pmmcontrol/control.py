import numpy as np
from pmmcontrol.arduino import Arduino
from time import sleep, time
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from scipy import stats


class Control(Arduino):
    '''
    Class for controlling the PMM readout. Built on the Arduino class from arduino.py
    '''
    def __init__(self, port, baud_rate=115200, self_test=False):
        # define number of rows and columns
        self.rows = 9
        self.cols = 10

        # define resistor values
        self.R_primary = 620
        self.R_auxiliary = 2400

        self.row_primary_res = np.ones(self.rows) * self.R_primary
        self.row_primary_res = [635.06933744, 637.68953069, 637.68953069, 636.376737, 632.79426817, 633.76729882, 636.376737, 636.04938272, 635.39568345]
        self.row_auxiliary_res = np.ones(self.rows) * self.R_auxiliary
        self.row_auxiliary_res = [2415, 2419.7260274, 2410.29239766, 2410.29239766, 2391.64410058, 2434.01574803, 2419.7260274 , 2415, 2434.01574803]
        self.col_primary_res = np.ones(self.cols) * self.R_primary
        self.col_primary_res = [620, 620, 638.01857585, 636.04938272, 636.70442842, 635.72236504, 637.68953069, 637.0324575, 637.0324575, 632.79426817]
        self.col_auxiliary_res = np.ones(self.cols) * self.R_auxiliary
        self.col_auxiliary_res = [2400, 2400, 2405.60311284, 2438.81656805, 2415, 2424.47058824, 2434.01574803, 2463.10756972, 2448.47524752, 2458.21073559]

        # value of current sense resistor
        self.R_sense = 0.115

        self.max_voltage = 5
        self.max_voltage_linear = 3.5

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

        # set DAC to zero
        self.writeDAC(0)

        # calibrate max voltage for DAC output
        self.Vcc = self.readVcc()
        sleep(0.5)  # let Vcc equilibriate

        # ENABLE IF DAC ATTACHED
        #self.__calibrateDACVoltage()

        # calibrate current sense while DAC is at 0
        self.calibrateOffCurrent()

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
            for col in range(2, self.cols):
                # select magnet
                print("Selecting magnet at coordinate {}, {}".format(row, col))
                self.selectMagnet(row, col)

                # set magnet high, low, then off
                self.setVoltage(self.max_voltage_linear)
                sleep(0.01)
                self.pos_test[row][col] = self.readTotalCurrent()

                self.setVoltage(0)
                sleep(0.01)

                self.setVoltage(-self.max_voltage_linear)
                sleep(0.01)
                self.neg_test[row][col] = self.readTotalCurrent()

                self.setVoltage(0)
                sleep(0.01)

        # for debugging purposes
        print("Null current: ", self.null_current)
        print("High test: ", self.pos_test)
        print("Low test: ", self.neg_test)

        plt.figure(1)
        plt.plot(self.pos_test.flatten())
        plt.title('Maximum Current')

        plt.figure(2)
        plt.plot(self.neg_test.flatten())
        plt.title('Minimum Current')

        plt.show()

        return True

    def readTotalCurrent(self, seconds=2):
        '''
        Uses current sense circuit to measure the total current flowing to ground.
        '''

        sense_voltage_sum = 0
        sense_voltage_inc = 0

        self.Vcc = self.readVcc()

        t1 = time()
        while time() - t1 < seconds:
            sense_voltage_sum = sense_voltage_sum + self.analogRead(self.sense_pin) * (self.Vcc / 1023)
            sense_voltage_inc = sense_voltage_inc + 1

        sense_voltage = sense_voltage_sum / sense_voltage_inc

        # differential voltage across current sense IC with gain of 50 and offset of 2.5V
        diff_voltage = (sense_voltage - (self.Vcc/2)) / 50

        return (diff_voltage / self.R_sense) - self.sense_offset

    def readTotalCurrentError(self, seconds=2):

        sense_voltages = np.array([])
        self.Vcc = self.readVcc()

        t1= time()
        while time() - t1 < seconds:
            sense_voltages = np.append(sense_voltages, self.analogRead(self.sense_pin) * (self.Vcc / 1023))

        sense_voltage_avg = np.mean(sense_voltages)
        sense_voltage_error = np.std(sense_voltages)

        diff_voltage_avg = (sense_voltage_avg - (self.Vcc/2)) / 50
        diff_voltage_error = sense_voltage_error / 50

        return {
            "Average": (diff_voltage_avg / self.R_sense) - self.sense_offset,
            "Error": (diff_voltage_error / self.R_sense) - self.sense_offset,
        }


    def calibrateOffCurrent(self, seconds=5):
        # set DAC to zero
        self.writeDAC(0)

        self.sense_offset = 0
        self.sense_offset = self.readTotalCurrent(seconds)

        return True

    def voltageSweep(self):
        '''
        '''

        vStep = np.linspace(0, self.max_voltage_linear, 30)
        cData_pos = np.array([])
        cData_neg = np.array([])

        # enable any magnet so we can read current
        self.selectMagnet(3,3)

        for v in vStep:
            self.setVoltage(v, True)
            cData_pos = np.append(cData_pos, self.readTotalCurrent(1))

        self.setVoltage(0)

        for v in vStep:
            self.setVoltage(-v, True)
            cData_neg = np.append(cData_neg, abs(self.readTotalCurrent(1)))

        self.setVoltage(0)

        #save the data
        self.cData_pos = cData_pos
        self.cData_neg = cData_neg

        plt.plot(vStep, cData_pos, label='Positive Voltages')
        plt.plot(vStep, cData_neg, label='Negative Voltages')
        plt.ylabel('|Current| [A]')
        plt.xlabel('Voltage [V]')
        plt.title('Output Current Linearity')
        plt.grid(True)
        plt.legend()
        plt.show()

        return True

    def currentSweep(self):
        '''
        '''

        cMin = -30
        cMax = 30
        cSteps = 30

        cStep = np.linspace(cMin, cMax, cSteps)
        cData = np.array([])
        cData_percent_error = np.array([])
        cData_err = np.array([])

        # enable any magnet so we can read current
        self.selectMagnet(3,3)

        for c in cStep:
            self.setCurrent(c)
            readData = self.readTotalCurrentError(1)
            cData = np.append(cData, readData["Average"]*1000)
            cData_percent_error = np.append(cData_percent_error, ((readData["Average"]*1000-c)/c)*100)
            cData_err = np.append(cData_err, readData["Error"]*1000)


        self.setCurrent(0)

        #save the data
        self.cStep = cStep
        self.cData = cData
        self.cData_err = cData_err

        slope, intercept, r_value, p_value, std_err = stats.linregress(cStep, cData)

        plt.figure(1)
        plt.plot(cStep, cData_percent_error, linestyle='', marker='o', color='blue')
        plt.ylabel('Error in Current [%]')
        plt.xlabel('Set Current [mA]')
        plt.title('Error in Current Control')
        plt.ylim(-10, 10)
        plt.grid(True)

        plt.figure(2)
        plt.errorbar(cStep, cData, cData_err, marker='x', linestyle='', color='blue')
        plt.plot(np.linspace(cMin, cMax, cSteps*100), intercept+slope*np.linspace(cMin, cMax, cSteps*100), linestyle='--', color='orange', label='Linear Fit r={}'.format(str(round(r_value,2))))
        plt.ylabel('Measured Current [mA]')
        plt.xlabel('Set Current [mA]')
        plt.title('Current Control')
        plt.grid(True)

        plt.show()

        return True


    def measureResistors(self):

        print("MEASURING RESISTORS...")

        set_voltage = self.max_voltage_linear

        # set DAC to zero
        self.writeDAC(0)

        # disable all switches
        for pin in self.enable_pins:
            self.setLow(pin)

        self.setHigh(self.sign_pins['positive'])
        self.setLow(self.sign_pins['negative'])

        for row in range(0, self.rows):

            # configure to measure primary resistor with positive voltage
            self.setHigh(self.row_primary_pins[row])

            # set voltage on DAC
            binary = int(np.abs(set_voltage) * (2 ** 16 - 1) / self.Vcc)
            self.writeDAC(binary)
            sleep(.1)
            self.row_primary_res[row] = (set_voltage * 3) / abs(self.readTotalCurrent())
            print(self.row_primary_res[row])
            # set DAC to zero
            self.writeDAC(0)

            self.setLow(self.row_primary_pins[row])
            self.setHigh(self.row_auxiliary_pins[row])

            # set voltage on DAC

            self.writeDAC(binary)
            sleep(.1)
            self.row_auxiliary_res[row] = (set_voltage * 3) / abs(self.readTotalCurrent())
            print(self.row_auxiliary_res[row])
            # set DAC to zero
            self.writeDAC(0)

            self.setLow(self.row_auxiliary_pins[row])

        for col in range(2, self.cols):

            # configure to measure primary resistor with positive voltage
            self.setHigh(self.col_primary_pins[col])

            # set voltage on DAC
            binary = int(np.abs(set_voltage) * (2 ** 16 - 1) / self.Vcc)
            self.writeDAC(binary)
            sleep(.1)
            self.col_primary_res[col] = (set_voltage * 3) / abs(self.readTotalCurrent())
            print(self.col_primary_res[col])
            # set DAC to zero
            self.writeDAC(0)

            self.setLow(self.col_primary_pins[col])
            self.setHigh(self.col_auxiliary_pins[col])

            # set voltage on DAC
            binary = int(np.abs(set_voltage) * (2 ** 16 - 1) / self.Vcc)
            self.writeDAC(binary)
            sleep(.1)
            self.col_auxiliary_res[col] = (set_voltage * 3) / abs(self.readTotalCurrent())
            print(self.col_auxiliary_res[col])
            # set DAC to zero
            self.writeDAC(0)

            self.setLow(self.col_auxiliary_pins[col])

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
        self.state_row = row
        self.state_col = column
        self.state_sign = sign
        self.state_isPrimary = isPrimary
        self.state_isArrayMode = isArrayMode

        if self.state_isPrimary:
            r1 = self.row_primary_res[self.state_row]
            r2 = self.col_primary_res[self.state_col]
        else:
            r1 = self.row_auxiliary_res[self.state_row]
            r2 = self.col_auxiliary_res[self.state_col]

        self.state_effective_res = ((r1*r2)/(r1+r2))

        # state values to be set later
        self.state_posDAC = None
        self.state_voltage = None

        return True

    def setCurrent(self, current, allowNL=False):
        '''
        Sets a current (in mA) by selecting an appropriate binary number for the DAC. The current
        flows through whichever magnet has been enabled by running selectMagnet first.

        allowNL is set to True if voltages in the non-linear range are allowed
        '''
        if not hasattr(self, 'state_sign'):
            raise AttributeError("Some attributes have not been set. " +
                                 " Run 'selectMagnet()' first")
        if not self.state_isPrimary and self.state_isArrayMode:
            raise ValueError("Can only set currents in array mode if magnet has been selected as primary")

        set_current = (current / 1000)

        # get required voltage

        voltage = (set_current * self.state_effective_res) / 3

        self.setVoltage(voltage, allowNL)

    def setVoltage(self, voltage, allowNL=False):
        '''
        Sets a DAC voltage by selecting an appropriate binary number. The current
        flows through whichever magnet has been enabled by running selectMagnet first.
        Depending on the sign of the voltage, this is not necessarily the voltage accross
        the device.
        '''
        if not hasattr(self, 'state_sign'):
            raise AttributeError("Some attributes have not been set. " +
                                 " Run 'selectMagnet()' first")
        if not self.state_isPrimary and self.state_isArrayMode:
            raise ValueError("Can only set voltages in array mode if magnet has been selected as primary")

        if not allowNL:
            voltage_max = self.max_voltage_linear
        else:
            voltage_max = self.max_voltage

        current_max = voltage_max * self.state_effective_res

        if np.abs(voltage) > voltage_max:
            error = "The maximum output is {} V per row/col which is {} mA per magnet".format(str(round(voltage_max,3)), str(round(current_max,3)))
            raise ValueError(error)

        # disable all switches
        for pin in self.enable_pins:
            self.setLow(pin)

        # set DAC to zero
        self.writeDAC(0)

        # enable switches (row, column) depending on whether the magnet is primary
        if self.state_isPrimary:
            self.setHigh(self.col_primary_pins[self.state_col])
            self.setHigh(self.row_primary_pins[self.state_row])
        else:
            self.setHigh(self.col_auxiliary_pins[self.state_col])
            self.setHigh(self.row_auxiliary_pins[self.state_row])

        self.setHigh(self.sign_pins[self.state_sign])

        # need to configure all other magnets if array mode is True
        if self.state_isArrayMode:
            # set all rows except selected to auxiliary function
            for row_pin_auxiliary, row_pin_primary in zip(self.row_auxiliary_pins, self.row_primary_pins):
                if row_pin_auxiliary != self.row_auxiliary_pins[self.state_row]:
                    self.setLow(row_pin_primary)
                    self.setHigh(row_pin_auxiliary)

            # set all columns except selected to auxiliary function
            for column_pin_auxiliary, column_pin_primary in zip(self.col_auxiliary_pins, self.col_primary_pins):
                if column_pin_auxiliary != self.col_auxiliary_pins[self.state_col]:
                    self.setLow(column_pin_primary)
                    self.setHigh(column_pin_auxiliary)


        # change enable pins if sign change is necessary
        if (np.sign(voltage) == -1 and self.state_sign == 'positive') or \
                (np.sign(voltage) == 1 and self.state_sign == 'negative'):
            self.setLow(self.sign_pins['positive'])
            self.setHigh(self.sign_pins['negative'])
            posDAC = False
        else:
            self.setLow(self.sign_pins['negative'])
            self.setHigh(self.sign_pins['positive'])
            posDAC = True

        # set voltage on DAC
        binary = int(np.abs(voltage)* (2**16 - 1) / self.Vcc)
        self.writeDAC(binary)
        sleep(1)

        self.state_posDAC = posDAC # keep track of sign of DAC
        self.state_voltage = voltage # keep track of voltage

        print("Total current is now " + str(self.readTotalCurrent() * 1000) + ' mA')

    def updateCurrent(self, current, allowNL=False):
        '''
        Updates the current of the currently selected magnet without resetting all the enables.
        Essentially just updates the DAC but cares for the sign of the voltage.
        '''

        if not hasattr(self, 'state_sign'):
            raise AttributeError("Some attributes have not been set. " +
                                 " Run 'selectMagnet()' first")
        if self.state_voltage == None:
            raise AttributeError("Initial current has not been set. " +
                                 " Run 'setCurrent()' first")
        if not self.state_isPrimary and self.isArrayMode:
            raise ValueError("Can only set currents in array mode if magnet has been selected as primary")

        set_current = current / 1000

        # get required voltage
        voltage = self.__currentToVoltage(set_current)

        self.updateVoltage(voltage, allowNL)

        return True

    def updateVoltage(self, voltage, allowNL=False):
        '''
        Updates the voltage of the currently selected magnet without resetting all the enables.
        Essentially just updates the DAC but cares for the sign of the voltage.
        '''

        if not hasattr(self, 'state_sign'):
            raise AttributeError("Some attributes have not been set. " +
                                 " Run 'selectMagnet()' first")
        if self.state_voltage == None:
            raise AttributeError("Initial voltage has not been set. " +
                                 " Run 'setVoltage()' first")
        if not self.state_isPrimary and self.isArrayMode:
            raise ValueError("Can only set voltages in array mode if magnet has been selected as primary")


        if not allowNL:
            voltage_max = self.max_voltage_linear
        else:
            voltage_max = self.max_voltage

        current_max = voltage_max / self.current_R_row

        if np.abs(voltage) > voltage_max:
            error = "The maximum output is {} V per row/col which is {} mA per magnet".format(str(round(voltage_max,3)), str(round(current_max,3)))
            raise ValueError(error)

        # change enable pins if sign change is necessary
        if (np.sign(voltage) == -1 and self.state_sign == 'positive') or \
                (np.sign(voltage) == 1 and self.state_sign == 'negative'):
            if self.state_posDAC: # only change enables if DAC is set positive
                self.setLow(self.sign_pins['positive'])
                self.setHigh(self.sign_pins['negative'])
                self.state_posDAC = False # update state
        else:
            if not self.state_posDAC: # only change enables if DAC is set negative
                self.setLow(self.sign_pins['negative'])
                self.setHigh(self.sign_pins['positive'])
                self.state_posDAC = True # update state

        # set voltage on DAC
        binary = int(np.abs(voltage) * 2 ** 16 / self.Vcc)
        self.writeDAC(binary)

        self.state_voltage = voltage

    def resetMagnet(self):
        '''
        Removes the magnetization in the currently selected magnet by oscillating an
        exponentially decaying current. DAC finishes at 0V.
        '''

        if not hasattr(self, 'state_sign'):
            raise AttributeError("Some attributes have not been set. " +
                                 " Run 'selectMagnet()' first")
        if self.isArrayMode:
            raise ValueError("Can not reset magnet that has been selected in array mode. Please reselect magnet.")

        # set oscillating and exponentially decaying current through (row, column) magnet
        tt = np.arange(0, 70)
        max_current = round(self.max_voltage_linear[self.state_row] /
                            self.R_row[self.state_row], 4)
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
        if hasattr(self, 'state_row') and hasattr(self, 'state_column') and \
           hasattr(self, 'state_sign'):
            # enforce going through 0
            self.writeDAC(0)
            # disable switches
            self.setLow(self.column_pins[self.state_col])
            self.setLow(self.row_pins[self.state_row])
            self.setLow(self.sign_pins[self.state_sign])
            # switch current sign variable
            if self.state_sign == 'positive':
                self.state_sign = 'negative'
            else:
                self.state_sign = 'positive'
            # enable switches
            self.setHigh(self.column_pins[self.state_col])
            self.setHigh(self.row_pins[self.state_row])
            self.setHigh(self.sign_pins[self.state_sign])
        else:
            raise AttributeError("Some attributes have not been set. " +
                                 " Run 'selectMagnet()' first")

        return True

    def __currentToVoltage(self, current, r):
        '''
        Converts the requested current into a voltage required to be output by the DAC.
        Each row has a large resistor whose value is recorded in R_row.
        '''

        voltage = r * current

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
        self.max_voltage_linear = []
        for voltages in real_voltages:
            self.max_voltage_linear.append(voltages[-1])

        return True

    def __chooseResistor(self, primary):
        if primary:
            return self.R_primary
        if not primary:
            return self.R_auxiliary
