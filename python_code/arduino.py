import serial
from time import sleep


class Arduino(object):
    '''
    Class for basic communication to Arduino. Requires serial_control.ino to be loaded
    onto the Arduino board.
    '''

    def __init__(self, port, baud_rate=115200):
        # initiate connection then restart the Arduino so it can be reinitialized
        self.serial = serial.Serial(port, baud_rate)
        self.serial.write(b'99')
        self.serial.setDTR(False)
        sleep(0.022)
        self.serial.flushInput()
        self.serial.setDTR(True)
        self.close()

        # restart the connection
        self.serial = serial.Serial(port, baud_rate)
        self.serial.write(b'99')

        # define DAC power pin
        self.__DAC_PIN_POWER = 13

    def __str__(self):
        return "Arduino is on port %s at %d baudrate" % (self.serial.port,
                                                         self.serial.baudrate)

    def output(self, pin_array, pin_state=False):
        '''
        Configures the pins in pin_array as output pins. This function must be run
        immediately following the instantiation of the class.
        '''
        # check that DAC power pin won't be overridden and add it to the pin_array
        if self.__DAC_PIN_POWER in pin_array:
            return ValueError(
                "can't assign pin {} as output since it powers the DAC".
                format(self.__DAC_PIN_POWER))
        pin_array = list(pin_array) + [self.__DAC_PIN_POWER]

        # set default states
        for each_pin in pin_array:
            if pin_state:
                self.setHigh(each_pin)
            else:
                self.setLow(each_pin)

        # send output pins to the Arduino
        self.__sendData('1')
        self.__sendData(len(pin_array))
        if isinstance(pin_array, list) or isinstance(pin_array, tuple):
            self.__OUTPUT_PINS = pin_array
            for each_pin in pin_array:
                self.__sendData(each_pin)

        # reset DAC to clear memory
        self.resetDAC()

        return True

    def setLow(self, pin):
        '''
        Sets pin to low.
        '''
        self.__sendData('2')
        self.__sendData(pin)
        return True

    def setHigh(self, pin):
        '''
        Sets pin to high
        '''
        self.__sendData('3')
        self.__sendData(pin)
        return True

    def getState(self, pin):
        '''
        Returns the digital pin state, low or high.
        '''
        self.__sendData('4')
        self.__sendData(pin)
        return self.__formatPinState(self.__getData()[0])

    def analogWrite(self, pin, value):
        '''
        Writes an analog PWM wave to pin. The value can be any integer between 0 and 255.
        '''
        self.__sendData('5')
        self.__sendData(pin)
        self.__sendData(value)
        return True

    def analogRead(self, pin):
        '''
        Reads the value from a specified analog pin. The result can be any integer from 0
        to 1023.
        '''
        self.__sendData('6')
        self.__sendData(pin)
        return self.__getData()

    def writeDAC(self, value):
        '''
        Writes a value to the AD5667 DAC (controleverything.com) over the I2C connection.
        Valid values are from 0 to 2^16-1
        '''
        if (not isinstance(value, int)) or value < 0 or value > 2**16 - 1:
            raise ValueError('value must be an integer in [0, 65535]')

        if value < 256:
            data0 = 0
            data1 = value
        else:
            data0 = int(hex(value)[2:4], 16)
            data1 = int(hex(value)[4:], 16)

        self.__sendData('7')
        self.__sendData(data0)
        self.__sendData(data1)
        return True

    def turnOnDAC(self):
        '''
        Turns on the DAC by supplying 5 V to __DAC_PIN_POWER.
        '''
        self.setHigh(self.__DAC_PIN_POWER)
        return True

    def turnOffDAC(self):
        '''
        Turns off the DAC by grounding __DAC_PIN_POWER.
        '''
        self.setLow(self.__DAC_PIN_POWER)
        return True

    def resetDAC(self):
        '''
        Resets the DAC by turning the DAC off and then back on again.
        '''
        self.setLow(self.__DAC_PIN_POWER)
        sleep(0.1)
        self.setHigh(self.__DAC_PIN_POWER)
        return True

    def turnOff(self):
        '''
        Sets all of the output pins to low and turn off DAC.
        '''
        for each_pin in self.__OUTPUT_PINS:
            self.setLow(each_pin)
        return True

    def reset(self):
        '''
        Resets the board and connection.
        '''

        self.close()
        self.__init__(self.serial.port, baud_rate=self.serial.baudrate)

        return True

    def close(self):
        '''
        Closes the instance. Should be run at the end of a code to close the serial
        connection.
        '''
        self.serial.close()
        return True

    def __sendData(self, serial_data):
        '''
        Sends data to the board with the proper formating for serial_control.ino.
        '''
        while self.__getData()[0] != "w":
            pass
        serial_data = str(serial_data).encode('utf-8')
        self.serial.write(serial_data)

    def __getData(self):
        '''
        Gets data from the board with proper formating for serial_control.ino
        '''
        input_string = self.serial.readline()
        input_string = input_string.decode('utf-8')
        return input_string.rstrip('\n')

    def __formatPinState(self, pinValue):
        '''
        Returns the pin state, low or high, for use in returning the digital pin state.
        '''
        if pinValue == '1':
            return True
        else:
            return False
