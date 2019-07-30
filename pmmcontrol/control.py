import numpy as np
from pmmcontrol.arduino import Arduino
from time import sleep, time
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from scipy import stats
from tqdm import tqdm


class Control(Arduino):
    '''
    Class for controlling the PMM readout. Built on the Arduino class from arduino.py
    '''
    def __init__(self, port, baud_rate=115200, self_test=False):
        # define number of rows and columns
        self.rows = 9
        self.cols = 10

        # define resistor values
        self.R_primary = 635
        self.R_auxiliary = 2440

        self.row_primary_res = np.ones(self.rows) * self.R_primary
        self.row_primary_res = [632.2962870769394,
                                 632.4036765231027,
                                 633.5862607444412,
                                 632.9517433473735,
                                 632.7685036653352,
                                 633.1074352973883,
                                 633.2068520707134,
                                 632.6012473297608,
                                 632.9783127407242]
        self.row_auxiliary_res = np.ones(self.rows) * self.R_auxiliary
        self.row_auxiliary_res = [2410.4635230082313,
                                 2421.209877806157,
                                 2416.6479642492814,
                                 2415.1711800919334,
                                 2411.461300267713,
                                 2420.1299247700613,
                                 2411.5541206372595,
                                 2421.6498871446174,
                                 2433.3276218938236]
        self.col_primary_res = np.ones(self.cols) * self.R_primary
        self.col_primary_res = [620,
                                 620,
                                 637.4304105810651,
                                 633.9105989605471,
                                 635.1373803791279,
                                 634.3898799428237,
                                 633.9818377641924,
                                 633.7661473341418,
                                 635.0383050247008,
                                 634.2186582755699]
        self.col_auxiliary_res = np.ones(self.cols) * self.R_auxiliary
        self.col_auxiliary_res = [2400,
                                 2400,
                                 2420.377915774125,
                                 2424.0900046654942,
                                 2424.930302963896,
                                 2421.7759407239296,
                                 2418.584697017491,
                                 2433.8256938560635,
                                 2422.5800879727162,
                                 2417.0307631229653]

        # value of current sense resistor
        self.R_sense = 0.11427869852558546
        self.sense_offset = -0.001147730486291547

        self.max_voltage = 5
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

        # calibrate max voltage for DAC output
        self.Vcc = self.readVcc()
        sleep(0.5)  # let Vcc equilibriate

        # calibrate analogRead()
        self.analogCalibrate_real =     [0.014, 0.678, 1.342, 2.005, 2.668, 3.331, 3.994]
        self.analogCalibrate_measured = [2.03274648e-06, 6.60656046e-01, 1.32751882e+00, 1.99253746e+00,
                                         2.65305014e+00, 3.31033545e+00, 3.97787913e+00]
        self.read_correction = interp1d(self.analogCalibrate_measured, self.analogCalibrate_real, bounds_error=False, fill_value='extrapolate')
        # ENABLE TO RECALIBRATE
        self.calibrateAnalogRead()

        # voltage set calibration
        self.voltage_correction = lambda x: x
        self.dacCalibrate_requested = [0., 0.12061758, 0.24131088, 0.36200417, 0.48269747,
                                       0.60339077, 0.72408406, 0.84477736, 0.96547066, 1.08616396,
                                       1.20685725, 1.32755055, 1.44824385, 1.56893714, 1.68963044,
                                       1.81032374, 1.93101704, 2.05171033, 2.17240363, 2.29309693,
                                       2.41379022, 2.5344078, 2.6551011, 2.7757944, 2.89648769,
                                       3.01718099, 3.13787429, 3.25856758, 3.37926088, 3.49995418]
        self.dacCalibrate_real = [0.01399796, 0.12160277, 0.24386084, 0.36610753, 0.48737588,
                                  0.60860173, 0.7286054, 0.84959858, 0.9697203, 1.09089837,
                                  1.2121257, 1.33310381, 1.45423645, 1.57545139, 1.69646162,
                                  1.81812531, 1.93894083, 2.06067716, 2.18287933, 2.30343979,
                                  2.42270567, 2.54378928, 2.66577035, 2.78907185, 2.90855339,
                                  3.0293537, 3.15240359, 3.273836, 3.39626474, 3.51735824]
        self.voltage_correction = interp1d(self.dacCalibrate_real, self.dacCalibrate_requested, bounds_error=False,
                                           fill_value='extrapolate')
        # ENABLE TO RECALIBRATE
        # print("CALIBRATING DAC...")
        # self.calibrateDACVoltage(30)

        # calibrate current sense while DAC is at 0
        print("MEASURING OFF CURRENT...")
        self.measureOffCurrent(10)

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
                self.setCurrent(self.max_voltage_linear)
                self.maxCurrent_pos[row][col] = self.readTotalCurrent()

                self.setVoltage(0)

                self.setVoltage(-self.max_voltage_linear)
                self.maxCurrent_neg[row][col] = self.readTotalCurrent()

                self.setVoltage(0)

        # for debugging purposes
        print("Null current: ", self.null_current)
        print("High test: ", self.maxCurrent_pos)
        print("Low test: ", self.maxCurrent_neg)

        np.savetxt("maxCurrent_pos.csv", self.maxCurrent_pos)
        np.savetxt("maxCurrent_neg.csv", self.maxCurrent_neg)

        plt.figure(1)
        plt.plot(self.maxCurrent_pos.flatten())
        plt.title('Maximum Current')

        plt.figure(2)
        plt.plot(self.maxCurrent_neg.flatten())
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
            sense_voltage = self.analogRead(pin)
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

    def readTotalCurrent(self, seconds=10):
        '''
        Uses current sense circuit to measure the total current flowing to ground.
        '''

        # array of voltages above/below the vcc/2 offset voltage
        diff_voltages = np.array([])

        pbar = tqdm(total=100, bar_format="{l_bar}{bar}|{n_fmt}/{total_fmt}{postfix}")

        # set up timer
        t1 = time()
        t2 = t1

        # count number of measurements
        count = 0

        while t2 - t1 < seconds:
            pbar.update(((t2 - t1) / seconds) * 100 - pbar.n)
            count = count + 1

            # update vcc and make measurment considering the offset voltage of vcc/2 and the current
            # sense gain of 50
            self.Vcc = self.readVcc()
            sense_voltage = self.analogRead(self.sense_pin)
            diff_voltages = np.append(diff_voltages, (sense_voltage - (self.Vcc/2)) / 50)
            t2 = time()

        pbar.update(100 - pbar.n)
        pbar.close()

        sleep(.1)

        #print("Measured current " + str(count) + " times over " + str(t2 - t1) + " seconds.")

        # convert voltages to currents and get average
        current_avg = np.mean(diff_voltages / self.R_sense)

        # apply null-current offset
        return current_avg - self.sense_offset

    def readTotalCurrent_error(self, seconds=10):
        '''
        Uses current sense circuit to measure the total current flowing to ground. Also returns
        standard deviation in current measurement.
        '''

        diff_voltages = np.array([])

        pbar = tqdm(total=100, bar_format="{l_bar}{bar}|{n_fmt}/{total_fmt}{postfix}")

        # read voltages as in self.readTotalCurrent()
        t1 = time()
        t2 = t1
        count = 0
        while t2 - t1 < seconds:
            pbar.update(((t2 - t1) / seconds) * 100 - pbar.n)
            count = count + 1

            self.Vcc = self.readVcc()
            sense_voltage = self.analogRead(self.sense_pin)
            diff_voltages = np.append(diff_voltages, (sense_voltage - (self.Vcc/2)) / 50)
            t2 = time()

        pbar.update(100 - pbar.n)
        pbar.close()

        sleep(.1)

        #print("Measured current " + str(count) + " times over " + str(t2 - t1) + " seconds.")

        currents = diff_voltages / self.R_sense

        # compute average and standard deviation of current measurments
        current_avg = np.mean(currents)
        current_error = np.std(currents)


        average = current_avg - self.sense_offset
        error = current_error

        return (average, error)


    def measureOffCurrent(self, seconds=10):
        '''
        Find the current flowing through the current-sense when voltage is off. This will be applied
        when measuring currents in the future.
        '''

        # set DAC to zero
        self.writeDAC(0)

        self.sense_offset = 0
        self.sense_offset = self.readTotalCurrent(seconds)

        return True

    def measureSenseResistor(self, steps=10):
        '''
        Measure the sense resistor by setting voltages on an arbitrary row or column and measuring
        the resulting current using a multimeter as well as the voltage on the sense circuit. Resistance
        is found by fitting the slope of the current - vsense plot. The input "steps" is the number of
        voltage steps to iterate through, and thus the number of current measurements that will need to be made.
        '''

        print("MEASURING SENSE RESISTOR...")

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
            binary = int(self.voltage_correction(np.abs(v)) * (2 ** 16 - 1) / self.Vcc)
            self.writeDAC(binary)
            sleep(.1)

            # record current through user input
            current_inputs[idx] = float(input("Measured current in mA")) / 1000

            # record voltage on across sense resistor using self.analogRead()
            self.Vcc = self.readVcc()
            vsense[idx] = ((self.analogRead(self.sense_pin)) - (self.Vcc/2)) / 50

            self.writeDAC(0)

        self.writeDAC(0)

        # disable all switches
        for pin in self.enable_pins:
            self.setLow(pin)

        # fit data
        slope, intercept, r_value, p_value, std_err = stats.linregress(vsense, current_inputs)

        # compute resistance from fit
        self.R_sense = 1/slope

        # plot data and fit
        linPoints = np.linspace(np.amin(vsense), np.amax(vsense), vSteps * 100)
        plt.plot(vsense*1000, current_inputs*1000, marker='x', linestyle='', color='blue')
        plt.plot(linPoints*1000, 1000*(intercept + slope * linPoints),
                 linestyle='--', color='orange', label='Linear Fit: R = {}\nSense Resistance: {} Ohm'.format(str(round(r_value, 4)),str(round(self.R_sense, 4))))
        plt.xlabel("Differential Sense Voltage [mV]")
        plt.ylabel("Current [mA]")
        plt.title('Total Current vs. Sense Voltage')
        plt.legend()
        plt.grid()
        plt.show()

        print("Sense resistance: ", self.R_sense)


    def measureVoltageSetting(self, vSteps=10):
        '''
        Utility function with skeleton made to sweep through voltages. Uses and measurements
        made can be altered.
        '''

        vMin = -self.max_voltage_linear
        vMax = self.max_voltage_linear

        vData = np.linspace(vMin, vMax, vSteps)
        cData = np.array([])
        dacData = np.array([])
        ampData_top = np.array([])
        ampData_bottom = np.array([])

        # enable any magnet so we can read current
        self.selectMagnet(3,3)

        for v in vData:
            self.setVoltage(v, True)
            #cData = np.append(cData, self.readTotalCurrent(5))
            #dacData = np.append(dacData, float(input("Vdata Measurement in V: ")))
            #ampData_top = np.append(ampData_top, float(input("Top opamp output in V: ")))
            #ampData_bottom = np.append(ampData_bottom, float(input("Bottom opamp output in V: ")))
            input("Voltage at " + str(v) + ", press Enter to continue...")

        self.setVoltage(0)

        # record the data
        self.vData = vData
        self.cData = cData
        self.dacData = dacData
        self.ampData_top = ampData_top
        self.ampData_bottom = ampData_bottom
        self.gain_top = abs(np.divide(self.ampData_top, self.dacData))
        self.gain_bottom = abs(np.divide(self.ampData_bottom, self.dacData))
        self.dacErr = self.dacData-abs(self.vData)
        self.dacErr_frac = np.divide(self.dacErr, abs(self.vData))

        self.amp_gain = np.mean(np.append(self.gain_bottom, self.gain_top))

        plt.figure(1)
        plt.plot(self.vData, self.dacErr_frac, marker='x', linestyle='', color='blue')
        plt.ylabel('Error in DAC Voltage [%]')
        plt.xlabel('Set Voltage [V]')
        plt.title('DAC Error')
        plt.grid(True)

        plt.figure(2)
        plt.plot(self.vData, self.gain_top, label="Top Gain", marker='x', linestyle='', color='blue')
        plt.plot(self.vData, self.gain_bottom, label="Bottom Gain", marker='x', linestyle='', color='orange')
        plt.axhline(y=3, color='green', linestyle='-')
        plt.ylabel("|Gain|")
        plt.xlabel("Set Voltage [V]")
        plt.title('Opamp Gain vs. Set Voltage')
        plt.legend()

        plt.show()

        self.deselectMagnet()

        return True

    def measureCurrentSetting(self, cSteps = 15):
        '''
        '''

        cMin = -30
        cMax = 30

        cStep = np.linspace(cMin, cMax, cSteps)
        cData = np.array([])
        cData_percent_error = np.array([])
        cData_err = np.array([])

        # enable any magnet so we can read current
        self.selectMagnet(3,3)

        for c in tqdm(cStep, bar_format="{l_bar}{bar}|{n_fmt}/{total_fmt}{postfix}"):
            self.setCurrent(c)
            avg, error = self.readTotalCurrentError(5)
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
        plt.plot(cStep, cData_percent_error, linestyle='', marker='o', color='blue')
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

    def measureResistors(self, saveFiles=False, manual=False):
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
            self.Vcc = self.readVcc()
            binary = int(self.voltage_correction(np.abs(set_voltage)) * (2 ** 16 - 1) / self.Vcc)
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
            self.Vcc = self.readVcc()
            binary = int(self.voltage_correction(np.abs(set_voltage)) * (2 ** 16 - 1) / self.Vcc)
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
            self.Vcc = self.readVcc()
            binary = int(self.voltage_correction(np.abs(set_voltage)) * (2 ** 16 - 1) / self.Vcc)
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
            self.Vcc = self.readVcc()
            binary = int(self.voltage_correction(np.abs(set_voltage)) * (2 ** 16 - 1) / self.Vcc)
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

        if saveFiles:
            np.savetxt("row_primary_res.csv", self.row_primary_res)
            np.savetxt("row_auxiliary_res.csv", self.row_auxiliary_res)
            np.savetxt("col_primary_res.csv", self.col_primary_res)
            np.savetxt("col_auxiliary_res.csv", self.col_auxiliary_res)

        return True

    def measureResistorVoltageSweep(self):
        vMin = -self.max_voltage_linear
        vMax = self.max_voltage_linear
        vStep = 30

        vData = np.linspace(vMin, vMax, vStep)
        rData = np.array([])

        row = 1

        # set DAC to zero
        self.writeDAC(0)

        # disable all switches
        for pin in self.enable_pins:
            self.setLow(pin)

        for v in tqdm(vData, bar_format="{l_bar}{bar}|{n_fmt}/{total_fmt}{postfix}"):
            if v >= 0:
                # enable positive voltages
                self.setHigh(self.sign_pins['positive'])
                self.setLow(self.sign_pins['negative'])
            else:
                self.setHigh(self.sign_pins['negative'])
                self.setLow(self.sign_pins['positive'])

            # configure to measure primary resistor
            self.setHigh(self.row_primary_pins[row])

            # set voltage on DAC
            self.Vcc = self.readVcc()
            binary = int(self.voltage_correction(np.abs(v)) * (2 ** 16 - 1) / self.Vcc)
            self.writeDAC(binary)
            sleep(.1)

            r = abs((v * self.amp_gain) / self.readTotalCurrent())
            rData = np.append(rData, r)

            print(r)

            # set DAC to zero
            self.writeDAC(0)

            # configure to measure auxiliary resistor
            self.setLow(self.row_primary_pins[row])

        # save the data
        self.vData = vData
        self.rData = rData

        plt.plot(self.amp_gain*vData, rData)
        plt.xlabel('Voltage [V]')
        plt.ylabel('Resistance [R]')
        plt.title('Resistor Linearity')
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

        return True

    def deselectMagnet(self):
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
            voltage_max = self.max_voltage_linear
        else:
            voltage_max = self.max_voltage

        current_max = voltage_max * self.state_effective_res

        set_voltage = np.sign(voltage) * self.voltage_correction(abs(voltage))

        if np.abs(set_voltage) > voltage_max:
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
        binary = int(np.abs(set_voltage) * (2**16 - 1) / self.Vcc)
        self.writeDAC(binary)
        sleep(1)

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
            voltage_max = self.max_voltage_linear
        else:
            voltage_max = self.max_voltage

        current_max = voltage_max * self.state_effective_res

        set_voltage = np.sign(voltage) * self.voltage_correction(abs(voltage))

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
        binary = int(np.abs(set_voltage) * (2 ** 16 - 1) / self.Vcc)
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

    def calibrateDACVoltage(self, steps=10):
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
        voltage_list = np.linspace(0, self.max_voltage_linear, steps)
        binary_list = np.array(voltage_list * ((2**16 - 1) / self.Vcc), dtype=int)

        voltages_requested = binary_list * (self.Vcc / (2**16 - 1))
        voltages_real = np.zeros(binary_list.size)

        for index, value in enumerate(tqdm(binary_list, bar_format="{l_bar}{bar}|{n_fmt}/{total_fmt}{postfix}")):
            self.writeDAC(int(value))
            sleep(0.1)
            voltages_real[index] = np.mean([self.readDAC() for _ in range(5)])

        # return DAC to zero Volts
        self.writeDAC(0)

        self.voltage_correction = interp1d(voltages_real, voltages_requested, bounds_error=False, fill_value='extrapolate')

        linpoints = np.linspace(0, self.max_voltage_linear, 2000)
        plt.plot(linpoints, self.voltage_correction(linpoints))
        plt.show()

        return True

    def calibrateAnalogRead(self, vSteps=10):
        vMin = 0
        vMax = 4

        vSet = np.linspace(vMin, vMax, vSteps)
        vMeasured = np.array([])
        vReal = np.array([])

        self.selectMagnet(3,3)

        for v in vSet:
            print("Setting voltage to " + str(round(v, 3)) + " volts.")
            self.setVoltage(v, True)
            vReal = np.append(vReal, float(input("DAC voltage: ")))
            vMeasured = np.append(vMeasured, self.readDAC())

        self.setVoltage(0)

        self.analogCalibrate_measured = vMeasured
        self.analogCalibrate_real = vReal
        self.analogCalibrate_err = np.divide((self.analogCalibrate_measured - self.analogCalibrate_real), self.analogCalibrate_real)

        self.read_correction = interp1d(vMeasured, vReal, bounds_error=False, fill_value='extrapolate')

        plotPoints = np.linspace(vMin, vMax, 2000)

        plt.figure(1)
        plt.plot(self.analogCalibrate_real, self.analogCalibrate_err, marker='x', linestyle='', color='blue')

        plt.figure(2)
        plt.plot(plotPoints, self.read_correction(plotPoints))

        plt.show()

    def __chooseResistor(self, primary):
        if primary:
            return self.R_primary
        if not primary:
            return self.R_auxiliary
