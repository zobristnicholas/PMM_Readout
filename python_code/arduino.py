import serial
from time   import sleep

class Arduino(object):
    '''
    Class for basic communication to Arduino. 
    Requires serial_control.ino to be loaded onto the Arduino board.
    '''

    def __init__(self, port, baud_rate=115200):
        self.serial = serial.Serial(port, baud_rate)
        self.serial.write(b'99')

    def __str__(self):
        return "Arduino is on port %s at %d baudrate" %(self.serial.port, self.serial.baudrate)

    def output(self, pinArray):
        '''
        Configures the pins in pinArray as output pins. 
        This function must be run immediately following the instantiation of the class.
        '''
        self.__sendData(len(pinArray))

        if(isinstance(pinArray, list) or isinstance(pinArray, tuple)):
            self.__OUTPUT_PINS = pinArray
            for each_pin in pinArray:
                self.__sendData(each_pin)
        return True

    def setLow(self, pin):
        '''
        Sets pin to low.
        '''
        self.__sendData('0')
        self.__sendData(pin)
        return True

    def setHigh(self, pin):
        '''
        Sets pin to high
        '''
        self.__sendData('1')
        self.__sendData(pin)
        return True

    def getState(self, pin):
        '''
        Returns the digital pin state, low or high.
        '''
        self.__sendData('2')
        self.__sendData(pin)
        return self.__formatPinState(self.__getData()[0])

    def analogWrite(self, pin, value):
        '''
        Writes an analog PWM wave to pin. The value can be any integer between 0 and 255.
        '''
        self.__sendData('3')
        self.__sendData(pin)
        self.__sendData(value)
        return True

    def analogRead(self, pin):
        '''
        Reads the value from a specified analog pin. The result can be any integer from 0 to 1023.
        '''
        self.__sendData('4')
        self.__sendData(pin)
        return self.__getData()
        
    def writeDAC(self,value):
        '''
        Writes a value to the AD5667 DAC (controleverything.com) over the I2C connection.
        '''
        self.__sendData('5')
        self.__sendData(value)
        return True
        
    def turnOff(self):
        '''
        Sets all of the output pins to low.
        '''
        for each_pin in self.__OUTPUT_PINS:
            self.setLow(each_pin)
        return True
    
    def reset(self):
        '''
        Resets the board.
        '''
        self.serial.setDTR(True) 
        sleep(0.022)    
        self.serial.setDTR(False)
        return True

    def __sendData(self, serial_data):
        '''
        Sends data to the board with the proper formating for serial_control.ino.
        '''
        while(self.__getData()[0] != "w"):
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

    def close(self):
        '''
        Closes the instance. Should be run at the end of a code to close the serial connection.
        '''
        self.serial.close()
        return True
