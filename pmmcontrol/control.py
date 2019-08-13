import numpy as np
from pmmcontrol.arduino import Arduino
from time import sleep, time
from scipy.interpolate import interp1d, InterpolatedUnivariateSpline
import matplotlib.pyplot as plt
from scipy import stats
from tqdm import tqdm
import os


class Control(Arduino):
    '''
    Class for controlling the PMM readout. Built on the Arduino class from arduino.py
    '''
    def __init__(self, port, baud_rate=115200, self_test=False):

        # general configuration
        self.data_path = "data/"

        self.setCalibrationData_fname = "dac_set_calibration_data.csv"
        self.readCalibrationData_fname = "analog_calibration_data.csv"
        self.rowPrimData_fname = "row_primary_res_data.csv"
        self.rowAuxData_fname = "row_auxiliary_res_data.csv"
        self.colPrimData_fname = "col_primary_res_data.csv"
        self.colAuxData_fname = "col_auxiliary_res_data.csv"

        # define number of rows and columns
        self.rows = 9
        self.cols = 10

        # initialize resistor values
        self.R_primary = 635
        self.R_auxiliary = 2440

        self.row_primary_res = np.ones(self.rows) * self.R_primary
        self.row_auxiliary_res = np.ones(self.rows) * self.R_auxiliary
        self.col_primary_res = np.ones(self.cols) * self.R_primary
        self.col_auxiliary_res = np.ones(self.cols) * self.R_auxiliary

        # initialize set and read correction
        self.set_correction = lambda x: x
        self.read_correction = lambda x: x

        # value of current sense resistor
        self.R_sense = 0.11556899686977241
        self.nullVoltage_sense = 0.003002132467027
        self.nullCurrent_sense = -0.001147730486291547

        # set voltage limitations
        self.max_voltage = 4.8
        self.max_voltage_linear = 3.5

        self.amp_gain = 3.01561447252416

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

        # state values to be set later
        self.DAC_isPos = None
        self.DAC_voltage = None

        # calibrate vcc reading actual / measured. Use if powered under 5V
        self.vcc_scale = 1

        # let pins equilibriate
        for _ in range(50):
            self.Vcc = self.readVcc()
            self.analogRead(0)
            self.analogRead(1)
        sleep(0.5)

        # calibrate voltage reading
        #try:
        #    calibrationData_read = np.loadtxt(self.data_path + self.readCalibrationData_fname)
        #    self.analogCalibrate_real = calibrationData_read[1]
        #    self.analogCalibrate_measured = calibrationData_read[0]
        #    self.read_correction = interp1d(self.analogCalibrate_measured, self.analogCalibrate_real, bounds_error=False, fill_value='extrapolate')
        #    print("Loaded analogRead() correction data.")
        #except:
        #    print("Failed to load analogRead() calibration data. Proceeding with recalibration...")
        #    self.calibrateAnalogRead()

        # calibrate voltage setting
        try:
            calibrationData_set = np.loadtxt(self.data_path + self.setCalibrationData_fname)
            self.dacCalibrate_real = calibrationData_set[0]
            self.dacCalibrate_requested = calibrationData_set[1]
            self.set_correction = interp1d(self.dacCalibrate_real, self.dacCalibrate_requested, kind="linear", bounds_error=False, fill_value=(self.dacCalibrate_requested[0], self.dacCalibrate_requested[-1]))
            print("Loaded DAC setting correction data.")
        except:
            print("Failed to load DAC calibration data. Proceeding with recalibration...")
            self.calibrateDACSet(30)

        # calibrate sense circuit
        #self.measureSenseResistor()
        self.measureNullVoltage(5)
        print("Calibrated sense circuit.")

        # load resistor values
        res_failed_load = False
        try:
            self.row_primary_res = np.loadtxt(self.data_path + self.rowPrimData_fname)
        except:
            res_failed_load = True
        try:
            self.row_auxiliary_res = np.loadtxt(self.data_path + self.rowAuxData_fname)
        except:
            res_failed_load = True
        try:
            self.col_primary_res = np.loadtxt(self.data_path + self.colPrimData_fname)
        except:
            res_failed_load = True
        try:
            self.col_auxiliary_res = np.loadtxt(self.data_path + self.colAuxData_fname)
        except:
            res_failed_load = True
        if res_failed_load:
            print("WARNING: Failed to load some resistor data. Reverted to defaults. Resistors \
                  can be remeasured using measureResistors()")
        else:
            print("Loaded all resistor values.")

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
        self.maxCurrent_pos = np.zeros((self.rows, self.cols))
        self.maxCurrent_neg = np.zeros((self.rows, self.cols))

        # test each magnet positive and negative
        for row in range(0, self.rows):
            for col in range(2, self.cols):
                # select magnet
                print("Selecting magnet at coordinate {}, {}".format(row, col))
                self.selectMagnet(row, col)

                # set magnet high, low, then off
                self.setVoltage(self.max_voltage_linear)
                self.maxCurrent_pos[row][col] = self.readTotalCurrent()

                self.setVoltage(0)

                self.setVoltage(-self.max_voltage_linear)
                self.maxCurrent_neg[row][col] = self.readTotalCurrent()

                self.setVoltage(0)

        # for debugging purposes
        print("Null current: ", self.null_current)
        print("High test: ", self.maxCurrent_pos)
        print("Low test: ", self.maxCurrent_neg)

        self.saveData("maxCurrent_pos.csv", self.maxCurrent_pos)
        self.saveData("maxCurrent_neg.csv", self.maxCurrent_neg)

        self.maxCurrent_pos_flat = 1000 * np.delete(self.maxCurrent_pos, np.s_[0:2], 1).flatten()
        self.maxCurrent_neg_flat = np.abs(1000 * np.delete(self.maxCurrent_neg, np.s_[0:2], 1).flatten())

        posAvg = np.average(self.maxCurrent_pos_flat)
        negAvg = np.average(self.maxCurrent_neg_flat)

        plt.figure(1)
        plt.plot(self.maxCurrent_pos_flat, linestyle='', marker='x', color='blue', label='Maximum Voltage')
        plt.plot(self.maxCurrent_neg_flat, linestyle='', marker='o', color='green', label='Minimum Voltage')
        plt.axhline(y=posAvg, linestyle='-', color='black', label="Maximum Voltage Average = {}".format(str(round(posAvg, 2))))
        plt.axhline(y=negAvg, linestyle=':', color='black', label="Minimum Voltage Average = {}".format(str(round(negAvg, 2))))
        plt.ylabel("|Current| mA")
        plt.xticks([])
        plt.legend(loc="center right")
        plt.title('Minimum and Maximum Currents')

        plt.figure(2)
        plt.plot(np.delete(self.maxCurrent_pos, np.s_[0:2], 1).flatten(), linestyle='', marker='x')
        plt.title('Minimum Current')

        plt.show()

        self.deselectMagnet()

        return True

    def measureAnalog(self, pin=0, duration=2):
        '''
        Read voltage of analog pin for a duration using self.analogRead() and plot results
        '''

        if not isinstance(pin, int) or pin < 0 or pin > 15:
            raise ValueError("Pin must be an integer between 0 and 15.")

        # array for storing pin 1 measurments
        measurements = np.array([])
        vcc = np.array([])

        # set up timer
        t1 = time()
        t2 = t1

        # keep track of how many measurements are made
        count = 0

        # timed loop
        while t2 - t1 < duration:
            count = count + 1

            # update vcc and take measurement
            self.Vcc = self.readVcc()
            sense_voltage = self.read_correction(self.analogRead(pin))
            measurements = np.append(measurements, sense_voltage)
            vcc = np.append(vcc, self.Vcc)

            t2 = time()

        print("Measured " + str(count) + " times over " + str(t2 - t1) + " seconds.")
        print("Average pin voltage: ", np.mean(measurements))
        print("Average vcc: ", np.mean(vcc))

        plt.figure(1)
        plt.plot(measurements)

        plt.figure(2)
        plt.plot(vcc)

        plt.show()

    def readTotalCurrent(self, seconds=0.1):
        '''
        Uses current sense circuit to measure the total current flowing to ground.
        '''

        # array of voltages above/below the vcc/2 offset voltage
        diff_voltages = np.array([])

        # set up timer
        t1 = time()
        t2 = t1

        # count number of measurements
        count = 0

        while t2 - t1 < seconds:
            count = count + 1

            # update vcc and make measurement considering the offset voltage of vcc/2 and the current
            # sense gain of 50

            sense_voltage = self.read_correction(self.analogRead(self.sense_pin))
            diff_voltages = np.append(diff_voltages, (sense_voltage - (self.readVcc()/2 - self.nullVoltage_sense)) / 50)
            t2 = time()

        #print("Measured current " + str(count) + " times over " + str(t2 - t1) + " seconds.")

        # convert voltages to currents and get average
        current_avg = np.mean(diff_voltages / self.R_sense)

        # apply null-current offset
        return current_avg

    def readTotalCurrent_error(self, seconds=0.1):
        '''
        Uses current sense circuit to measure the total current flowing to ground. Also returns
        standard deviation in current measurement.
        '''

        diff_voltages = np.array([])

        # read voltages as in self.readTotalCurrent()
        t1 = time()
        t2 = t1
        count = 0
        while t2 - t1 < seconds:
            count = count + 1
            sense_voltage = self.read_correction(self.analogRead(self.sense_pin))
            diff_voltages = np.append(diff_voltages, (sense_voltage - (self.readVcc()/2 - self.nullVoltage_sense)) / 50)
            t2 = time()

        #print("Measured current " + str(count) + " times over " + str(t2 - t1) + " seconds.")

        currents = diff_voltages / self.R_sense

        # compute average and standard deviation of current measurments
        current_avg = np.mean(currents)
        current_error = np.std(currents)


        average = current_avg
        error = current_error

        return (average, error)

    def measureOffCurrent(self, seconds=10):
        '''
        Find the current flowing through the current-sense when voltage is off. This will be applied
        when measuring currents in the future.
        '''

        # set DAC to zero
        self.writeDAC(0)

        self.nullCurrent_sense = 0
        self.nullCurrent_sense = self.readTotalCurrent(seconds)

        return True

    def measureNullVoltage(self, duration=10):
        '''
        Find the average difference between vcc/2 and the null-current voltage of the current-sense IC. Ideally, the
        null-current voltage should be at vcc/2, but the voltage divider isn't perfect and there is a small amount of
        leakage current. Usually the offset is around 3mV.con
        '''

        self.nullVoltage_sense = 0

        # array for storing pin measurements
        measurements = np.array([])
        vcc = np.array([])

        # set up timer
        t1 = time()
        t2 = t1

        # keep track of how many measurements are made
        count = 0

        # timed loop
        while t2 - t1 < duration:
            count = count + 1

            # update vcc and take measurement
            self.Vcc = self.readVcc()
            sense_voltage = self.read_correction(self.analogRead(self.sense_pin))
            measurements = np.append(measurements, sense_voltage)
            vcc = np.append(vcc, self.Vcc)

            t2 = time()

        nullVoltages = vcc/2 - measurements


        #print("Measured " + str(count) + " times over " + str(t2 - t1) + " seconds.")

        self.nullVoltage_sense = np.mean(nullVoltages)

        average = self.nullVoltage_sense
        error = np.std(nullVoltages)

        return (average, error)

    def measureSenseResistor(self, steps=10):
        '''
        Measure the sense resistor by setting voltages on an arbitrary row or column and measuring
        the resulting current using a multimeter as well as the voltage on the sense circuit. Resistance
        is found by fitting the slope of the current - vsense plot. The input "steps" is the number of
        voltage steps to iterate through, and thus the number of current measurements that will need to be made.
        '''

        vMin = -self.max_voltage_linear
        vMax = self.max_voltage_linear
        vSteps = steps

        # choose arbitrary row or column to measure on
        row = 1

        set_voltages = np.linspace(vMin, vMax, vSteps)
        current_inputs = np.zeros(vSteps)
        vsense = np.zeros(vSteps)

        # make sure DAC is off
        self.writeDAC(0)

        # disable all switches
        for pin in self.enable_pins:
            self.setLow(pin)

        # configure to measure primary resistor with positive voltage
        self.setHigh(self.row_primary_pins[row])

        # iterate through set voltages
        for idx, v in enumerate(set_voltages):

            # configuration for positive or negative voltage
            if v > 0:
                self.setHigh(self.sign_pins['positive'])
                self.setLow(self.sign_pins['negative'])
            else:
                self.setHigh(self.sign_pins['negative'])
                self.setLow(self.sign_pins['positive'])

            # set voltage on DAC
            binary = int(self.set_correction(np.abs(v)) * (2 ** 16 - 1) / self.readVcc())
            self.writeDAC(binary)
            sleep(.1)

            # record current through user input
            current_inputs[idx] = float(input("Measured current in mA")) / 1000

            # record voltage on across sense resistor using self.analogRead()
            vsense[idx] = ((self.read_correction(self.analogRead(self.sense_pin))) - (self.readVcc()/2 - self.nullVoltage_sense)) / 50

            self.writeDAC(0)

        self.writeDAC(0)

        # disable all switches
        for pin in self.enable_pins:
            self.setLow(pin)

        # save data
        self.current_inputs = current_inputs
        self.vsense_data = vsense
        rsense_fitdata = np.array([self.current_inputs, self.vsense_data])
        self.saveData("rsense_fitdata.csv", rsense_fitdata)

        # fit data
        slope, intercept, r_value, p_value, std_err = stats.linregress(current_inputs, vsense)
        f = lambda x: intercept + slope * x

        # compute resistance from fit
        self.R_sense = slope

        # compute error in fit
        n = np.size(self.vsense_data)
        ssr = np.linalg.norm(self.vsense_data - f(current_inputs))**2
        mse = ssr / (n-2)
        stdx = np.std(self.current_inputs)
        self.R_sense_err = np.sqrt(mse) / stdx

        print("Sense resistance: ", self.R_sense)
        print("Error: ", self.R_sense_err)

        # plot data and fit
        linPoints = np.linspace(np.amin(current_inputs), np.amax(current_inputs), vSteps * 100)
        plt.plot(current_inputs*1000, vsense*1000, marker='x', linestyle='', color='blue')
        plt.plot(linPoints*1000, 1000*f(linPoints),
                 linestyle='--', color='orange', label='Linear Fit: R = {}\nSense Resistance: {} Ohm'.format(str(round(r_value, 4)),str(round(self.R_sense, 4))))
        plt.plot(linPoints*1000, 1000*(intercept + (slope+self.R_sense_err)*linPoints), linestyle='--', color='gray')
        plt.plot(linPoints * 1000, 1000 * (intercept + (slope - self.R_sense_err) * linPoints), linestyle='--',
                 color='gray', label="Slope Bounds")
        plt.ylabel("Differential Sense Voltage [mV]")
        plt.xlabel("Current [mA]")
        plt.title('Total Current vs. Sense Voltage')
        plt.legend()
        plt.grid()
        plt.show()



    def measureVoltageSetting(self, vSteps=10):
        '''
        Utility function with skeleton made to sweep through voltages. Uses and measurements
        made can be altered.
        '''

        vMin = 0
        vMax = self.max_voltage_linear

        vData = np.linspace(vMin, vMax, vSteps)
        #vData = self.dacCalibrate_real[0:3000:100].copy()
        cData = np.array([])
        dacData = np.array([])
        ampData_top = np.array([])
        ampData_bottom = np.array([])

        #TODO measureVoltageSetting() should not need to enable a magnet. Change it so that all switches
        # are off and voltages are set using writeDAC()

        # enable any magnet so we can read current
        self.selectMagnet(3,3)

        for v in tqdm(vData):
            self.setVoltage(v, True)
            #cData = np.append(cData, self.readTotalCurrent(1))
            dacData = np.append(dacData, self.read_correction(self.readDAC()))
            #ampData_top = np.append(ampData_top, float(input("Top opamp output in V: ")))
            #ampData_bottom = np.append(ampData_bottom, float(input("Bottom opamp output in V: ")))
            #input("Voltage at " + str(v) + ", press Enter to continue...")

        self.setVoltage(0)

        # record the data
        self.vData = vData
        #self.cData = cData
        self.dacData = dacData
        #self.ampData_top = ampData_top
        #self.ampData_bottom = ampData_bottom
        #self.gain_top = abs(np.divide(self.ampData_top, self.dacData))
        #self.gain_bottom = abs(np.divide(self.ampData_bottom, self.dacData))
        self.dacErr = self.dacData-abs(self.vData)
        self.dacErr_frac = np.divide(self.dacErr, abs(self.vData))

        #self.amp_gain = np.mean(np.append(self.gain_bottom, self.gain_top))

        plt.figure(1)
        plt.plot(self.vData, self.dacErr_frac * 100, marker='x', linestyle='', color='blue')
        plt.ylabel('Error in DAC Voltage [%]')
        plt.xlabel('Set Voltage [V]')
        plt.title('DAC Error')
        plt.grid(True)

        #plt.figure(2)
        #plt.plot(self.vData, self.gain_top, label="Top Gain", marker='x', linestyle='', color='blue')
        #plt.plot(self.vData, self.gain_bottom, label="Bottom Gain", marker='x', linestyle='', color='orange')
        #plt.axhline(y=3, color='green', linestyle='-')
        #plt.ylabel("|Gain|")
        #plt.xlabel("Set Voltage [V]")
        #plt.title('Opamp Gain vs. Set Voltage')
        #plt.legend()

        plt.show()

        self.deselectMagnet()

        return True

    def measureCurrentSetting(self, cSteps = 15):
        '''
        '''

        cMin = -30
        cMax = 30
        cCutoff = 5

        cStep = np.concatenate((np.linspace(cMin, -cCutoff, int(cSteps / 2)), np.linspace(cCutoff, cMax, int(cSteps / 2))))
        cData = np.array([])
        cData_percent_error = np.array([])
        cData_err = np.array([])

        # enable any magnet so we can read current
        self.selectMagnet(3,3)

        for c in tqdm(cStep, bar_format="{l_bar}{bar}|{n_fmt}/{total_fmt}{postfix}"):
            self.setCurrent(c)
            avg, error = self.readTotalCurrent_error(0.1)
            cData = np.append(cData, avg*1000)
            cData_percent_error = np.append(cData_percent_error, ((avg*1000-c)/c)*100)
            cData_err = np.append(cData_err, error*1000)


        self.setCurrent(0)

        #save the data
        self.cStep = cStep
        self.cData = cData
        self.cData_err = cData_err
        self.cData_percent_error = cData_percent_error

        slope, intercept, r_value, p_value, std_err = stats.linregress(cStep, cData)

        plt.figure(1)
        plt.plot(cStep, cData_percent_error, linestyle='', marker='.', color='blue')
        plt.axvline(x=-cCutoff, linestyle='--', color="grey")
        plt.axvline(x=cCutoff, linestyle='--', color="grey")
        plt.ylabel('Error in Current [%]')
        plt.xlabel('Set Current [mA]')
        plt.title('Error in Current Control')
        plt.grid(True)

        plt.figure(2)
        plt.errorbar(cStep, cData, cData_err, marker='x', linestyle='', color='blue')
        plt.plot(np.linspace(cMin, cMax, cSteps*100), intercept+slope*np.linspace(cMin, cMax, cSteps*100), linestyle='--', color='orange', label='Linear Fit r={}'.format(str(round(r_value,4))))
        plt.ylabel('Measured Current [mA]')
        plt.xlabel('Set Current [mA]')
        plt.title('Current Control')
        plt.grid(True)
        plt.legend()

        plt.show()

        self.deselectMagnet()

        return True

    def measureResistors(self, overwrite=False, manual=False):
        '''
        Measures resistors by setting the voltage and measuring current. Set saveFiles to True to s
        '''

        set_voltage = self.max_voltage_linear

        # set DAC to zero
        self.writeDAC(0)

        # disable all switches
        for pin in self.enable_pins:
            self.setLow(pin)

        if set_voltage >= 0:
            # enable positive voltages
            self.setHigh(self.sign_pins['positive'])
            self.setLow(self.sign_pins['negative'])
        else:
            self.setHigh(self.sign_pins['negative'])
            self.setLow(self.sign_pins['positive'])

        set_voltage=abs(set_voltage)

        print("MEASURING ROW RESISTORS...")

        for row in range(0, self.rows):

            # configure to measure primary resistor
            self.setHigh(self.row_primary_pins[row])

            # set voltage on DAC
            binary = int(self.set_correction(np.abs(set_voltage)) * (2 ** 16 - 1) / self.readVcc())
            self.writeDAC(binary)
            sleep(.1)

            if manual:
                self.row_primary_res[row] = (set_voltage * self.amp_gain) / abs(float(input("Measured Current in mA: "))/1000)
            else:
                self.row_primary_res[row] = (set_voltage * self.amp_gain) / abs(self.readTotalCurrent())

            print("Row #" + str(row) + " primary: " + str(self.row_primary_res[row]) + " Ohm")
            # set DAC to zero
            self.writeDAC(0)

            # configure to measure auxiliary resistor
            self.setLow(self.row_primary_pins[row])
            self.setHigh(self.row_auxiliary_pins[row])

            # set voltage on DAC
            binary = int(self.set_correction(np.abs(set_voltage)) * (2 ** 16 - 1) / self.readVcc())
            self.writeDAC(binary)
            sleep(.1)

            if manual:
                self.row_auxiliary_res[row] = (set_voltage * self.amp_gain) / abs(float(input("Measured Current in mA: "))/1000)
            else:
                self.row_auxiliary_res[row] = (set_voltage * self.amp_gain) / abs(self.readTotalCurrent())

            print("Row #" + str(row) + " auxiliary: " + str(self.row_auxiliary_res[row]) + " Ohm")
            # set DAC to zero
            self.writeDAC(0)

            self.setLow(self.row_auxiliary_pins[row])

        print("MEASURING COLUMN RESISTORS...")

        for col in range(2, self.cols):

            # configure to measure primary resistor with positive voltage
            self.setHigh(self.col_primary_pins[col])

            # set voltage on DAC
            binary = int(self.set_correction(np.abs(set_voltage)) * (2 ** 16 - 1) / self.readVcc())
            self.writeDAC(binary)
            sleep(.1)

            if manual:
                self.col_primary_res[col] = (set_voltage * self.amp_gain) / abs(float(input("Measured Current in mA: "))/1000)
            else:
                self.col_primary_res[col] = (set_voltage * self.amp_gain) / abs(self.readTotalCurrent())

            print("Col #" + str(col) + " primary: " + str(self.col_primary_res[col]) + " Ohm")
            # set DAC to zero
            self.writeDAC(0)

            self.setLow(self.col_primary_pins[col])
            self.setHigh(self.col_auxiliary_pins[col])

            # set voltage on DAC
            binary = int(self.set_correction(np.abs(set_voltage)) * (2 ** 16 - 1) / self.readVcc())
            self.writeDAC(binary)
            sleep(.1)

            if manual:
                self.col_auxiliary_res[col] = (set_voltage * self.amp_gain) / abs(float(input("Measured Current in mA: "))/1000)
            else:
                self.col_auxiliary_res[col] = (set_voltage * self.amp_gain) / abs(self.readTotalCurrent())

            print("Col #" + str(col) + " auxiliary: " + str(self.col_auxiliary_res[col]) + " Ohm")
            # set DAC to zero
            self.writeDAC(0)

            self.setLow(self.col_auxiliary_pins[col])

        if overwrite:
            self.saveData(self.rowPrimData_fname, self.row_primary_res)
            self.saveData(self.rowAuxData_fname, self.row_auxiliary_res)
            self.saveData(self.colPrimData_fname, self.col_primary_res)
            self.saveData(self.colAuxData_fname, self.col_auxiliary_res)

        return True

    def measureLinearity(self, vStep=30, multi=False):
        vMin = -self.max_voltage_linear
        vMax = self.max_voltage_linear
        vCutoff = 0

        vData = np.concatenate((np.linspace(vMin, -vCutoff, int(vStep/2)), np.linspace(vCutoff, vMax, int(vStep/2))))
        rData = np.array([])
        cData = np.array([])
        cData_voltages = np.array([])
        vData_read = np.array([])

        if not multi:
            type = 'Multi'
            self.selectMagnet(3,4, True)
        else:
            row = 1
            type = 'Single'

            # set DAC to zero
            self.writeDAC(0)

            # disable all switches
            for pin in self.enable_pins:
                self.setLow(pin)


        for v in tqdm(vData, bar_format="{l_bar}{bar}|{n_fmt}/{total_fmt}{postfix}"):

            if not multi:
                self.setLow(self.row_primary_pins[row])

                if v >= 0:
                    # enable positive voltages
                    self.setHigh(self.sign_pins['positive'])
                    self.setLow(self.sign_pins['negative'])
                else:
                    self.setHigh(self.sign_pins['negative'])
                    self.setLow(self.sign_pins['positive'])

                self.setHigh(self.row_primary_pins[row])

                # set voltage on DAC
                binary = int(self.set_correction(np.abs(v)) * (2 ** 16 - 1) / self.readVcc())
                self.writeDAC(binary)
                sleep(.1)

            else:
                self.setVoltage(v)

            vRead = np.sign(v) * self.read_correction(self.readDAC())
            vData_read = np.append(vData_read, vRead)

            #current = self.readTotalCurrent(1)

            sense_voltage = self.read_correction(self.analogRead(self.sense_pin))
            current = ((sense_voltage - (self.readVcc() / 2 - self.nullVoltage_sense)) / 50) / self.R_sense
            cData_voltages = np.append(cData_voltages, sense_voltage)


            r = abs((vRead * self.amp_gain) / current)
            rData = np.append(rData, r)
            cData = np.append(cData, current)

            # set DAC to zero
            self.writeDAC(0)

        if not multi:
            self.setLow(self.row_primary_pins[row])
        else:
            self.setVoltage(0)
            self.deselectMagnet()

        # save the data
        self.vData = vData
        self.rData = rData
        self.cData = cData
        self.vData_read = vData_read
        self.cData_voltages = cData_voltages

        plt.figure(1)
        plt.plot(self.amp_gain*vData, rData, marker='.', linestyle='')
        plt.xlabel('Voltage [V]')
        plt.ylabel('Resistance [R]')
        plt.title('Resistor Linearity')
        plt.grid(True)

        plt.figure(2)
        plt.plot(self.amp_gain*vData, cData_voltages, marker='.', linestyle='')
        plt.xlabel("Voltage [V]")
        plt.ylabel('Sense Voltage [mA]')
        plt.title('Sense Voltage Linearity')
        plt.grid(True)

        slope, intercept, r_value, p_value, std_err = stats.linregress(self.amp_gain * vData_read, cData * 1000)
        points = np.linspace(self.amp_gain * vMin, self.amp_gain * vMax, 5000)

        plt.figure(3)
        plt.plot(self.amp_gain * vData_read, cData * 1000, marker='.', linestyle='', label='analogRead() Measurements')
        plt.plot(points, intercept+slope*points, linestyle='--', color='orange', label='Linear Fit: R = {}'.format(str(round(r_value, 4))))
        plt.xlabel("Voltage [V]")
        plt.ylabel('Current [mA]')
        plt.title(type + ' Channel Current Linearity with Reverse Currents')
        plt.legend(loc='upper left')
        plt.grid(True)

        plt.figure(4)
        plt.plot(self.amp_gain*vData, self.amp_gain * vData_read, marker='.', linestyle='')
        plt.xlabel("Voltage [V]")
        plt.ylabel("Read Voltage [V]")
        plt.title('analogRead() Linearity')
        plt.grid(True)

        plt.show()

        return True


    def testSenseConvergence(self, tMin, tMax, tSteps):
        tStep = np.linspace(tMin, tMax, tSteps)
        cAvg = np.array([])
        cErr = np.array([])

        for duration in tqdm(tStep, bar_format="{l_bar}{bar}|{n_fmt}/{total_fmt}{postfix}"):
            avg, err = self.readTotalCurrentError(duration)
            cAvg = np.append(cAvg, avg)
            cErr = np.append(cErr, err)

        plt.plot(tStep, cAvg*1000, marker='x', linestyle='', color='blue')
        plt.ylabel('Measured Current [mA]')
        plt.xlabel('Integration Time [s]')
        plt.title('Current Measurement vs. Integration Time')
        plt.grid(True)

        plt.show()


    def selectMagnet(self, row, column, isArrayMode=False, sign='positive', isPrimary=True):
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

        return True

    def deselectMagnet(self):
        if hasattr(self, 'state_isPrimary'):
            self.setVoltage(0)

        del self.state_row
        del self.state_col
        del self.state_sign
        del self.state_isPrimary
        del self.state_isArrayMode
        del self.state_effective_res

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

        voltage = (set_current * self.state_effective_res) / self.amp_gain

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
            voltage_max = self.set_correction(self.max_voltage_linear)
        else:
            voltage_max = self.set_correction(self.max_voltage)

        current_max = voltage_max * self.state_effective_res

        set_voltage = np.sign(voltage) * self.set_correction(abs(voltage))

        if np.abs(set_voltage) > voltage_max:
            print("Set Voltage: ", set_voltage)
            print("Voltage Max: ", voltage_max)
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
        if (np.sign(set_voltage) == -1 and self.state_sign == 'positive') or \
                (np.sign(set_voltage) == 1 and self.state_sign == 'negative'):
            self.setLow(self.sign_pins['positive'])
            self.setHigh(self.sign_pins['negative'])
            posDAC = False
        else:
            self.setLow(self.sign_pins['negative'])
            self.setHigh(self.sign_pins['positive'])
            posDAC = True

        # set voltage on DAC
        binary = int(np.abs(set_voltage) * (2**16 - 1) / self.readVcc())
        self.writeDAC(binary)
        sleep(0.1)

        self.DAC_isPos = posDAC # keep track of sign of DAC
        self.DAC_voltage = set_voltage # keep track of voltage

    def updateCurrent(self, current, allowNL=False):
        '''
        Updates the current of the currently selected magnet without resetting all the enables.
        Essentially just updates the DAC but cares for the sign of the voltage.
        '''

        if not hasattr(self, 'state_sign'):
            raise AttributeError("Some attributes have not been set. " +
                                 " Run 'selectMagnet()' first")
        if self.DAC_voltage == None:
            raise AttributeError("Initial current has not been set. " +
                                 " Run 'setCurrent()' first")
        if not self.state_isPrimary and self.isArrayMode:
            raise ValueError("Can only set currents in array mode if magnet has been selected as primary")

        set_current = (current / 1000)

        # get required voltage

        voltage = (set_current * self.state_effective_res) / self.amp_gain

        self.updateVoltage(voltage, allowNL)

    def updateVoltage(self, voltage, allowNL=False):
        '''
        Updates the voltage of the currently selected magnet without resetting all the enables.
        Essentially just updates the DAC but cares for the sign of the voltage.
        '''

        if not hasattr(self, 'state_sign'):
            raise AttributeError("Some attributes have not been set. " +
                                 " Run 'selectMagnet()' first")
        if self.DAC_voltage == None:
            raise AttributeError("Initial voltage has not been set. " +
                                 " Run 'setVoltage()' first")
        if not self.state_isPrimary and self.isArrayMode:
            raise ValueError("Can only set voltages in array mode if magnet has been selected as primary")


        if not allowNL:
            voltage_max = self.set_correction(self.max_voltage_linear)
        else:
            voltage_max = self.max_voltage

        current_max = voltage_max * self.state_effective_res

        set_voltage = np.sign(voltage) * self.set_correction(abs(voltage))

        if np.abs(set_voltage) > voltage_max:
            error = "The maximum output is {} V per row/col which is {} mA per magnet".format(str(round(voltage_max,3)), str(round(current_max,3)))
            raise ValueError(error)

        # change enable pins if sign change is necessary
        if (np.sign(set_voltage) == -1 and self.state_sign == 'positive') or \
                (np.sign(set_voltage) == 1 and self.state_sign == 'negative'):
            if self.DAC_isPos: # only change enables if DAC is set positive
                self.setLow(self.sign_pins['positive'])
                self.setHigh(self.sign_pins['negative'])
                self.DAC_isPos = False # update state
        else:
            if not self.DAC_isPos: # only change enables if DAC is set negative
                self.setLow(self.sign_pins['negative'])
                self.setHigh(self.sign_pins['positive'])
                self.DAC_isPos = True # update state

        # set voltage on DAC
        binary = int(np.abs(set_voltage) * (2 ** 16 - 1) / self.readVcc())
        self.writeDAC(binary)

        self.DAC_voltage = set_voltage

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

    def calibrateDACSet(self, steps=10, overwrite=True):
        '''
        Measures the DAC nonlinearity and generates a 'linearity_correction' function for
        each row. This function compensates for the two main sources of nonlinearity:
        voltage errors and current draw errors. Needs to be run in the __init__()
        statement and assumes DAC is on, so run it after 'self.output()'.
        '''

        self.set_correction = lambda x: x

        # turn off DAC
        self.writeDAC(0)

        # ensure all switches are off
        for pin in self.enable_pins:
            self.setLow(pin)

        # write values to the DAC and read the result with the Arduino
        voltage_list = np.linspace(0, self.max_voltage, steps)
        binary_list = np.array(voltage_list * ((2**16 - 1) / self.readVcc()), dtype=int)

        voltages_measured_smoothed = np.array([0])
        voltages_requested_smoothed = np.array([0])

        voltages_requested = np.array([0])
        voltages_measured = np.array([0])

        # number of times we read a repeated voltage
        nRepeated = 0
        nRepeatedSum = 0
        nRepeatedAvg = 0

        nJumps = 0

        bottom = 0

        for bin in tqdm(binary_list, bar_format="{l_bar}{bar}|{n_fmt}/{total_fmt}{postfix}"):
            self.writeDAC(int(bin))
            sleep(0.1)
           # measured = np.mean([self.read_correction(self.readDAC()) for _ in range(5)])

            measured = self.read_correction(self.readDAC())
            requested = bin * (self.readVcc() / (2 ** 16 - 1))

            voltages_measured = np.append(voltages_measured, measured)
            voltages_requested = np.append(voltages_requested, requested)

            vPrev_measured = voltages_measured[-2]
            vCurr_measured = voltages_measured[-1]
            vJump_measured = vCurr_measured - vPrev_measured

            nRepeated = nRepeated + 1

            if vJump_measured > 0.001:

                nJumps = nJumps + 1

                nRepeatedSum = nRepeatedSum + nRepeated
                nRepeated = 0

                top = voltages_requested[-2]

                if top != 0 and bottom != 0:
                    #print("Top " + str(top) + ", Bottom: " + str(bottom) + ", Jump: " + str(vJump_measured) + ".")
                    vAvg_requested = (top + bottom) / 2

                    voltages_measured_smoothed = np.append(voltages_measured_smoothed, vPrev_measured)
                    voltages_requested_smoothed = np.append(voltages_requested_smoothed, vAvg_requested)

                bottom = voltages_requested[-1]

        # return DAC to zero Volts
        self.writeDAC(0)

        print("On average " + str(nRepeatedSum / nJumps) + " data points per jump.")

        # save all data points for reference
        idx = np.argsort(voltages_measured)
        self.dacCalibrate_requested_all = np.array(voltages_requested)[idx]
        self.dacCalibrate_real_all = np.array(voltages_measured)[idx]

        # sort lists
        idx = np.argsort(voltages_measured_smoothed)
        self.dacCalibrate_requested = np.array(voltages_requested_smoothed)[idx]
        self.dacCalibrate_real = np.array(voltages_measured_smoothed)[idx]


        self.set_correction = interp1d(self.dacCalibrate_real, self.dacCalibrate_requested, kind="linear", bounds_error=False, fill_value=(self.dacCalibrate_requested[0], self.dacCalibrate_requested[-1]))

        if overwrite:
            self.saveData(self.setCalibrationData_fname, [self.dacCalibrate_real, self.dacCalibrate_requested])

        linpoints = np.linspace(0, self.max_voltage_linear, 2000)
        plt.plot(linpoints, self.set_correction(linpoints))
        plt.show()

        return True

    def calibrateAnalogRead(self, vSteps=10, overwrite=True):
        '''
        Sets voltages on arbitrary magnet
        '''
        self.read_correction = lambda x: x

        vMin = 0
        vDiv = 1
        vMax = 4

        vSet = np.round(np.concatenate((np.linspace(vMin, vDiv, int((vSteps+1)/3))[:-1], np.linspace(vDiv, vMax, (vSteps+1)-int((vSteps+1)/3)))), 3)
        print(vSet)
        bSet = np.array(vSet * ((2 ** 16 - 1) / self.readVcc()), dtype=int)

        vMeasured = np.array([])
        vReal = np.array([])

        # turn off DAC
        self.writeDAC(0)

        # ensure all switches are off
        for pin in self.enable_pins:
            self.setLow(pin)

        for b, v in zip(bSet, vSet):
            print("Setting voltage to " + str(round(v, 3)) + " volts.")
            self.writeDAC(int(b))
            vReal = np.append(vReal, float(input("DAC voltage: ")))
            vMeasured = np.append(vMeasured, self.readDAC())
            print("Measured voltage: ", round(vMeasured[-1], 4))

        self.writeDAC(0)

        self.analogCalibrate_err = np.divide((self.analogCalibrate_measured - self.analogCalibrate_real), self.analogCalibrate_real)

        # sort lists
        idx = np.argsort(vMeasured)
        self.analogCalibrate_measured = np.array(vMeasured)[idx]
        self.analogCalibrate_real = np.array(vReal)[idx]

        self.read_correction = InterpolatedUnivariateSpline(self.analogCalibrate_measured, self.analogCalibrate_real)

        if overwrite:
            self.saveData(self.readCalibrationData_fname, [self.analogCalibrate_measured, self.analogCalibrate_real])

        plotPoints = np.linspace(vMin, vMax, 2000)

        plt.figure(1)
        plt.plot(self.analogCalibrate_real, self.analogCalibrate_err, marker='x', linestyle='', color='blue')

        plt.figure(2)
        plt.plot(plotPoints, self.read_correction(plotPoints))

        plt.show()

    def saveData(self, fname, data):
        dirname = self.data_path
        filename = fname
        pathname = dirname + filename
        if not os.path.exists(os.path.dirname(dirname)):
            try:
                os.makedirs(os.path.dirname(dirname))
            except:
                print("Could  not save data to directory " + dirname)

        np.savetxt(pathname, data)