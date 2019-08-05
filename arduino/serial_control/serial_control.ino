// Distributed with a free-will license.
// Use it any way you want, profit or free, provided it fits in the licenses of its associated works.
// AD5667
// This code is designed to work with the AD5667_I2CDAC I2C Mini Module available from ControlEverything.com.
// https://www.controleverything.com/content/Digital-Analog?sku=AD5667_I2CDAC#tabs-0-product_tabset-2

#include<Wire.h>
#include<RunningAverage.h>

#ifndef SERIAL_RATE
#define SERIAL_RATE         115200
#endif

#ifndef SERIAL_TIMEOUT
#define SERIAL_TIMEOUT      5
#endif

// AD5667 I2C address is 0x0E(14)
#ifndef DAC_ADDRESS
#define DAC_ADDRESS         0x0E
#endif

int pin = 0;
int value = 0;
int cmd = 0;

//variables for averaging
int num = 30;
float sum = 0, avg;

RunningAverage vcc(30);

void setup()
{
  //initialise I2C communication as Master
  Wire.begin();

  //initialise serial communication, set baud rate = 9600
  Serial.begin(SERIAL_RATE);
  Serial.setTimeout(SERIAL_TIMEOUT);

  //set analog reference voltage
  //analogReference(INTERNAL2V56)

  vcc.clear();

  delay(300);
}

void loop(){

  vcc.addValue(readVcc());
  
  switch (readData()) {
    case 1 :
      //set output pins
      cmd = readData();
      for (int i = 0; i < cmd; i++) {
        pin = readData();
        pinMode(pin, OUTPUT);
      }
      break;
    case 2 :
      //set digital low
      digitalWrite(readData(), LOW); break;
    case 3 :
      //set digital high
      digitalWrite(readData(), HIGH); break;
    case 4 :
      //get digital value
      Serial.println(digitalRead(readData())); break;
    case 5 :
      //set analog value
      pin = readData();
      value = readData();
      analogWrite(pin, value); break;
    case 6 :
        //read analog value
        pin = readData();
        sum = 0;
        for(int i = 0; i < num; i++) {
          sum += analogRead(pin);
          vcc.addValue(readVcc()); //continue averaging vcc during this process
        }
        avg = sum/num;
        Serial.println(avg); break;
    case 7 :
      //set DAC value
      writeDAC(); break;
    case 8 :
      //read Vcc using 1.1 V on board reference
      Serial.println(vcc.getAverage()); break;
    case 99:
      //just dummy to cancel the current read, needed to prevent lock
      //when the PC side dropped the "w" that we sent
      break;
  }
  
}

char readData() {
  Serial.println("w");
  while(1) {
    if(Serial.available() > 0) {
      return Serial.parseInt();
    }
  }
}

void writeDAC() {
  //record data in two pieces
  unsigned int data0 = readData();
  unsigned int data1 = readData();
  //start I2C transmission
  Wire.beginTransmission(DAC_ADDRESS);
  //select DAC and input register
  Wire.write(0x1F);
  //write data
  Wire.write(data0);
  Wire.write(data1);
  //stop I2C transmission
  Wire.endTransmission();
}

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  float scale_constant = 1.1 * 1023 * 1000;

  result = scale_constant / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in milivolts
}
