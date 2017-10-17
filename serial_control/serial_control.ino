// Distributed with a free-will license.
// Use it any way you want, profit or free, provided it fits in the licenses of its associated works.
// AD5667
// This code is designed to work with the AD5667_I2CDAC I2C Mini Module available from ControlEverything.com.
// https://www.controleverything.com/content/Digital-Analog?sku=AD5667_I2CDAC#tabs-0-product_tabset-2

#include<Wire.h>

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

void setup()
{
  // Initialise I2C communication as Master
  Wire.begin();
  
  // Initialise serial communication, set baud rate = 9600
  Serial.begin(SERIAL_RATE);
  Serial.setTimeout(SERIAL_TIMEOUT);
  
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
      Serial.println(analogRead(readData())); break;
    case 7 :
      //set DAC value
      writeDAC(); break;
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
  //unsigned int data[2] = {readData(), 0};
  unsigned int data0 = readData();
  unsigned int data1 = readData();
  // Start I2C transmission
  Wire.beginTransmission(DAC_ADDRESS);
  // Select DAC and input register
  Wire.write(0x1F);
  // Write data = 0x8000(32768)
  // data msb = 0x80
  Wire.write(data0);
  // data lsb = 0x00
  Wire.write(data1);
  // Stop I2C transmission
  Wire.endTransmission();

  // Convert the data, Vref = 5 V
  float voltage = (((data[0] * 256) + (data[1])) / 65536.0) * 5.0;

  // Output data to serial monitor
  // Serial.print("Voltage : ");
  // Serial.print(voltage);
  // Serial.println(" V");
  delay(1000);
}
