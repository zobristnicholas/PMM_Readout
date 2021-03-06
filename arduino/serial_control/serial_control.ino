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

RunningAverage vcc(100);
RunningAverage a0(10);
RunningAverage a1(10);

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
  a0.clear();
  a1.clear();

  delay(300);
}

void loop(){
  
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
        if (pin == 0) {
          Serial.println(a0.getAverage()); break;
        }
        else if (pin == 1) {
          Serial.println(a1.getAverage()); break;
        }       
        else {
          Serial.println(analogRead(pin)); break;
        }
        break;
    case 7 :
      //set DAC value
      writeDAC(); break;
    case 8 :
      //read Vcc using 1.1 V on board reference
      Serial.println(vcc.getAverage()); break;
    case 9 :
      writeWave(); break;
    case 99:
      //just dummy to cancel the current read, needed to prevent lock
      //when the PC side dropped the "w" that we sent
      break;
  }
  
}

char readData() {
  Serial.println("w");
  while(1) {
    
    vcc.addValue(readVcc());
    a0.addValue(analogRead(0));
    a1.addValue(analogRead(1));
    
    if(Serial.available() > 0) {
      return Serial.parseInt();
    }
  }
}

uint16_t readInt16() {
  int data0 = (unsigned char)readData();
  int data1 = (unsigned char)readData();

  return (data0 + 256*data1);
}

void writeWave() {
  int pos_pin = readData();
  int neg_pin = readData();
  
  int state_size = readData();
  
  int wave_size = readInt16();
  
  // create pointers
  int *state_list;
  uint8_t *lsb_list;
  uint8_t *msb_list;
  bool *sign_list;
  
  // Allocate memory
  state_list = (int *) malloc(sizeof(int) * state_size);
  lsb_list = (uint8_t *) malloc(sizeof(uint8_t) * wave_size);
  msb_list = (uint8_t *) malloc(sizeof(uint8_t) * wave_size);
  sign_list = (bool *) malloc(sizeof(bool) * wave_size);
  
  if(!state_list || !lsb_list || !msb_list || !sign_list) {
    Serial.println("f");
    return;
  } else {
    Serial.println("s");
  }

  for (int i=0; i<state_size; i++) {
    state_list[i] = readData();
  }

  bool sign = 1;
  uint16_t value = 0;

  uint8_t lsb = 0;
  uint8_t msb = 0;
  
  for (int i=0; i<wave_size; i++) {
    lsb = (unsigned char)readData();
    msb = (unsigned char)readData();
    sign = readData();
    
    lsb_list[i] = lsb;
    msb_list[i] = msb;
    sign_list[i] = sign;
  }

  bool prev_sign = sign_list[0];

  // inital sign config
  unsigned long config_start = millis();
  Wire.beginTransmission(DAC_ADDRESS);
  Wire.write(0x1F);
  Wire.write(0);
  Wire.write(0);
  Wire.endTransmission();
  for (int i=0; i<state_size; i++) {digitalWrite(state_list[i], LOW);}
  if (prev_sign == true) {
    digitalWrite(neg_pin, LOW);
    digitalWrite(pos_pin, HIGH);
  } else {
    digitalWrite(pos_pin, LOW);
    digitalWrite(neg_pin, HIGH);
  }
  for (int i=0; i<state_size; i++) {digitalWrite(state_list[i], HIGH);}
  unsigned long config_end = millis();
  unsigned long config_duration = config_end - config_start;

  uint8_t data0 = 0;
  uint8_t data1 = 0;

  for (int i=0; i<wave_size; i++) {
    data0 = msb_list[i];
    data1 = lsb_list[i];

    // if sign change is necessary
    if (sign_list[i] != prev_sign) {

      // turn off DAC
      Wire.beginTransmission(DAC_ADDRESS);
      Wire.write(0x1F);
      Wire.write(0);
      Wire.write(0);
      Wire.endTransmission();

      // turn off state pins
      for (int i=0; i<state_size; i++) {
        digitalWrite(state_list[i], LOW);
      }

      // update sign
      if (sign_list[i] == true) {
        digitalWrite(neg_pin, LOW);
        digitalWrite(pos_pin, HIGH);
      } else {
        digitalWrite(pos_pin, LOW);
        digitalWrite(neg_pin, HIGH);
      }

      // turn on sign pins
      for (int i=0; i<state_size; i++) {
        digitalWrite(state_list[i], HIGH);
      }
    } else {
      // delay by amount of time config takes to even out steps
      delay(config_duration);
    }

    prev_sign = sign_list[i];

    // set DAC
    Wire.beginTransmission(DAC_ADDRESS);
    Wire.write(0x1F);
    Wire.write(data0);
    Wire.write(data1);
    Wire.endTransmission();
    
  }

  free(state_list);
  free(lsb_list);
  free(msb_list);
  free(sign_list);

  Serial.println(200);
  
  return;
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
