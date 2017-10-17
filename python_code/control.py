from time import sleep
import numpy as np
from arduino import Arduino


class Control(Arduino):
    '''
    Class for controlling the PMM readout. Built on the Arduino class from arduino.py
    '''
    def __init__(self, port, baud_rate=115200):
        # define number of rows and columns
        self.rows = 10
        self.columns = 10

        # define resistor values and max voltage for DAC output
        self.R_row = 356 * np.ones(10)
        self.max_voltage = 5.0

        # pins for enabling row and column switches
        self.row_pins = [32, 33, 36, 37, 40, 41, 44, 45, 48, 49]
        self.column_pins = [34, 35, 38, 39, 42, 43, 46, 47, 50, 51]
        # pins for enabling positive or negative current
        self.sign_pins = {'positive': 52, 'negative': 53}
        # all pins enabled for use
        self.enable_pins = self.row_pins + self.column_pins + self.sign_pins.values()

        # initialize Arduino
        Arduino.__init__(self, port, baud_rate=baud_rate)
        # define output pins
        self.output(self.enable_pins)

    def __str__(self):
        # add digital readout info later
        return "Arduino is on port %s at %d baudrate" % (self.serial.port,
                                                         self.serial.baudrate)

    def selectMagnet(self, row, column, sign='positive'):
        '''
        Selects the magnet in (row,column) by setting the row and column switch to enable
        in either the 'positive' or 'negative' configuration, and all of the other
        switches to disable (open circuit).
        '''
        # check inputs
        if sign != 'positive' and sign != 'negative':
            raise ValueError("parameter 'sign' must be either 'positive' or 'negative'")
        if (not isinstance(row, int)) or row < 0 or row > self.rows - 1:
            raise ValueError("parameter 'row' must be an integer and between 0 and " +
                             str(self.rows - 1))
        if (not isinstance(column, int)) or column < 0 or column > self.columns - 1:
            raise ValueError("parameter 'column' must be an integer and between 0 and " +
                             str(self.columns - 1))

        # disable all switches
        for pin in self.enable_pins:
            self.setLow(pin)

        # set DAC to zero
        self.writeDAC(0)

        # enable switches (row, column)
        self.setHigh(self.column_pins[column])
        self.setHigh(self.row_pins[row])
        self.setHigh(self.sign_pins[sign])

        # record state
        self.current_row = row
        self.current_column = column
        self.current_sign = sign

        return True

    def setCurrent(self, current):
        '''
        Sets a current by selecting an appropriate binary number for the DAC. The current
        flows through whichever magnet has been enabled by running selectMagnet first.
        '''
        if not hasattr(self, 'current_sign'):
            raise AttributeError("Some attributes have not been set. " +
                                 " Run 'selectMagnet()' first")
        # get required voltage
        voltage = self.__currentToVoltage(current)

        # change enable pins if sign change is necessary
        if (np.sign(voltage) == -1 and self.current_sign == 'positive') or \
           (np.sign(voltage) == 1 and self.current_sign == 'negative'):
            self.__changeSign()

        # set voltage on DAC
        binary = int(np.abs(voltage) * (2**16 - 1) / self.max_voltage)
        if binary > 2**16 - 1:
            max_current = round(self.max_voltage / self.R_row[self.current_row], 4)
            raise ValueError('The maximum current allowed for this row is ' +
                             str(max_current)[:5] + ' A')
        self.writeDAC(binary)

        return True

    def setVoltage(self, voltage):
        '''
        Sets a voltage by selecting an appropriate binary number for the DAC. The current
        flows through whichever magnet has been enabled by running selectMagnet first.
        '''
        if not hasattr(self, 'current_sign'):
            raise AttributeError("Some attributes have not been set. " +
                                 " Run 'selectMagnet()' first")

        # change enable pins if sign change is necessary
        if (np.sign(voltage) == -1 and self.current_sign == 'positive') or \
           (np.sign(voltage) == 1 and self.current_sign == 'negative'):
            self.__changeSign()

        # set voltage on DAC
        binary = int(np.abs(voltage) * (2**16 - 1) / self.max_voltage)
        if binary > 2**16 - 1:
            raise ValueError('The maximum voltage allowed is ' +
                             str(self.max_voltage) + ' V')
        self.writeDAC(binary)

        return True

    def resetMagnet(self, row, column, time_step):
        '''
        Removes the magnetization in the magnet in (row, column) by oscillating an
        exponentially decaying current. time_step sets the time to wait before setting
        the next current in the predefined current_list. Magnet selection returned to
        previous state after running. DAC remains off.
        '''
        # record current state so that we can reinitialize after resetting requested
        # magnet
        old_row = self.current_row
        old_column = self.current_column
        old_sign = self.current_sign

        # select requested magnet
        self.selectMagnet(row, column)

        # set oscillating and exponentially decaying current through (row, column) magnet
        tt = np.arange(0, 100)
        voltage_list = np.exp(-tt / 20.0) * np.cos(tt) * self.max_voltage
        voltage_list = np.append(voltage_list, 0)
        for voltage in voltage_list:
            self.setVoltage(voltage)
            sleep(time_step)

        # reselect old magnet
        self.selectMagnet(old_row, old_column, sign=old_sign)

        return True

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

    def __currentToVoltage(self, current):
        '''
        Converts the requested current into a voltage required to be output by the DAC.
        Each row has a large resistor whose value is recorded in R_row.
        '''

        voltage = self.R_row[self.current_row] * current

        return voltage

    def __calibrateMaxVoltage(self):
        '''
        Reads the maximum voltage availible from the DAC and sets the self.max_voltage
        variable. This value can vary depending on the voltage recieved via USB to the
        Arduino.
        '''
        raise NotImplementedError
