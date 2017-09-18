from arduino import Arduino
from numpy import np
from time import sleep

class Control(Arduino):
    '''
    Class for controlling the PMM readout. Built on the Arduino class from arduino.py
    '''
    def __init__(self, port, baud_rate=115200):
        Arduino.__init__(self,port,baud_rate=baud_rate)
        
        #define number of rows and columns
        self.rows=10
        self.columns=10
        
        #pins for enabling row and column switches [[positive enable, negative enable]]
        self.row_pins=[]
        self.column_pins=[]  
        self.enable_pins=[pin for list in self.row_pins for pin in list]+ \
                         [pin for list in self.column_pins for pin in list]
              
        #define output pins
        self.output(self.enable_pins)
        
    def __str__(self):
        #add digital readout info later
        return "Arduino is on port %s at %d baudrate"%(self.serial.port, self.serial.baudrate)          
    
    def selectMagnet(self,row,column,sign_index=0):
        '''
        Selects the magnet in (row,column) by setting the row and column switch to enable in either the
        positive (0) or negative (1) configuration, and all of the other switches to disable (open 
        circuit). 
        '''
        #check inputs
        if sign_index!=0 or sign_index!=1:
            raise ValueError('selectMagnet: parameter sign_index must be either 0 or 1')
        if type(row) != int or row<0 or row>self.rows-1:
            raise ValueError('selectMagnet: parameter row must be an integer and between 0 and ' + \
                             str(self.rows-1))
        if type(column) != int or column<0 or column>self.columns-1:
            raise ValueError('selectMagnet: parameter column must be an integer and between 0 and ' + \
                             str(self.columns-1))        
                                    
        #disable all multiplexing switches 
        for pin in self.enable_pins:    
            self.setLow(pin) 
            
        #set DAC to zero
        self.writeDAC(0)    
        
        #enable switches (row, column)
        self.setHigh(self.column_pins[column][sign_index])
        self.setHigh(self.row_pins[row][sign_index])
        
        #record state
        self.current_row=row
        self.current_column=column
        self.current_sign_index=sign_index
                
        return True
    
    def setCurrent(self, current):
        '''
        Sets a current by selecting an appropriate voltage in the DAC. The current flows through whichever 
        magnet has been enabled by running selectMagnet first.
        '''
        #get required voltage
        voltage=self.__curentToVoltage(current)
        
        #change enable pins if sign change is necessary
        if (np.sign(voltage)==-1 and self.current_sign==0) or \
           (np.sign(voltage)==1 and self.current_sign==1):
            self.writeDAC(0) #enforce going through 0
            self.__changeSign()   
        
        #set voltage on DAC
        self.writeDAC(voltage)     
        
        return True
        
    def resetMagnet(self,row,column,time_step):
        '''
        Removes the magnetization in the magnet in (row, column) by oscillating an exponentially decaying 
        current. time_step sets the time to wait before setting the next current in the predefined 
        current_list. Magnet selection returned to previous state after running. DAC remains off.
        '''
        #record current state so that we can reinitialize after resetting requested magnet
        old_row=self.current_row
        old_column=self.current_column
        old_sign_index=self.current_sign_index
        
        #select requested magnet
        self.selectMagnet(row,column)
        
        #set oscillating and exponentially decaying current through (row, column) magnet
        tt=np.arange(0,100)
        current_list=np.exp(-tt/20.0)*np.cos(tt)*max_current
        current_list=np.append(current_list,0)
        for current in current_list:
            self.setCurrent(current)
            sleep(time_step)
            
        #reselect old magnet
        self.selectMagnet(old_row, old_column,sign_index=old_sign_index)
        
        return True
    
    def findResonances(self,frequency_range):
        '''
        Returns a list of resonance frequencies and IQ sweep data for the frequency range in 
        frequency_range.
        '''
        raise NotImplementedError
        
    def identifyResonance(self,IQ_sweep,resonances,row,column): 
        '''
        Uses the IQ_sweep and resonance list from findResonances to determine the resonance frequency of 
        the resonator at (row, column) and outputs its sensitivity to the magnetic field.
        '''  
        raise NotImplementedError
        
    def alignResonances(self,IQ_sweep,resonances,sensitivities):
        '''
        Uses the IQ_sweep and resonance list from findResonances and the sensitivities list from 
        identifyResonance to compute and set the appropriate magnetizations to align the resonances.
        '''
        raise NotImplementedError                
        
    def __changeSign(self):
        '''
        Changes the sign of the row and column currently selected by selectMagnet. 
        selectMagnet must be run first.
        '''
        if hasattr(self, 'current_row') and hasattr(self,'current_column') and \
           hasattr(self,'current_sign_index'):
            #disable switches
            self.setLow(self.column_pins[self.current_column][self.current_sign_index])
            self.setLow(self.row_pins[self.current_row][self.current_sign_index])
            #0->1 and 1->0
            self.current_sign=1-self.current_sign
            #enable switches 
            self.setHigh(self.column_pins[self.current_column][self.current_sign_index])
            self.setHigh(self.row_pins[self.current_row][self.current_sign_index])
        else:
            ValueError('__changeSign: selectMagnet has not been run')
             
    def __currentToVoltage(self,current):
        '''
        Converts the requested current into a voltage required to be output by the DAC. Each row has a 
        large resistor whose value is recorded in R_row.
        '''
        R_row=[]
        
        voltage=R_row[self.current_row]*current
        
        return voltage