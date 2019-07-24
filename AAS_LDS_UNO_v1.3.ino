
#define fwVersion "AAS_LDS_UNO_v1.3"
int sampleRate = 2000; //sample rate in mS
#define sensorBaud 9600

// Include libraries for display and communications
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
Adafruit_SSD1306 display(128, 64, 11, 12, 9, 10, 11); //(SCREEN_WIDTH, SCREEN_HEIGHT, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS)
#include "Wire.h"
#include "SoftwareSerial.h"   //arduino software serial

// Initializes the T9602 - temp & humidity sensor
// I2C address of the sensor
#define ADDR_9602  0x28

// Initializes the T6700 - CO2 sensor
// I2C address of the sensor
#define ADDR_6700  0x15

                    //5v Supply, 04L Pin 1 & 2
                    //Ground, 04L Pin 3 & 4
#define sleepPin 4 //D4, Connector Pin 3, 04L Pin 10 Sleep
#define Rx_Pin  5  //D5, Connector Pin 4, 04L Pin 9 Tx
#define Tx_Pin  6  //D6, Connector Pin 5, 04L Pin 7 Rx
SoftwareSerial mySerial(Rx_Pin, Tx_Pin); // RX, TX               // Declare serial

// Initialize global variables
unsigned long sampleTime;         // variable to monitor time of each sample start
int CO2ppmValue;
float temperature;
float humidity;
int PM1value;
int PM25value;
int PM10value;
byte error;
byte Connected = 0; // bit values define sensor presence
//B1 -  T6700, B10 T9602 Temp, B100 T9602 Hum, B1000 Dust

//________________________________setup loop, runs once______________________________
void setup()
{

  // Start Wire I2C based interface
  Wire.begin();

  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH); //puts SM-UART-04L unit active (HIGH), sleep (LOW)

  //opens sensor serial with baud rate, configuration, pins are preset
  mySerial.begin(sensorBaud);  //baud rate, configuration

  display.begin(SSD1306_SWITCHCAPVCC);

  Serial.begin(19200); //Opens the main serial port over USB

  Serial.println(F("Amphenol Advanced Sensors"));
  Serial.println(F(fwVersion));

  display_prepare();
  display.setCursor(10, 30);
  display.print(fwVersion);
  display.display();
  delay(1500);

  display_prepare();
  display.setCursor(15, 40);
  display.print(F("Advanced Sensors"));
  display.setTextSize(2);
  display.setCursor(17, 20);
  display.print(F("Amphenol"));
  display.display();


  Wire.beginTransmission(ADDR_6700); // check if T6700 series is present
  error = Wire.endTransmission();
  if (error == 0)
  {

    Serial.println("T6700 Detected");

    Connected = 1;
  }

  Wire.beginTransmission(ADDR_9602); // check if T9602 series is present
  error = Wire.endTransmission();
  if (error == 0)
  {

    Serial.println("T9602 Detected");

    Connected = Connected + 6;
  }

  // test we have some traffic from dust sensor
  if ( mySerial.available() ) {

    Serial.println("Dust Sensor Detected");

    Connected = Connected + 8;
  }


  Serial.println();
  if ((Connected & B00001000) == 8 )
    Serial.print(F("PM1.0 Standard Smoke (µg/m³), "));
    Serial.print(F("PM2.5 Standard Smoke (µg/m³), "));
    Serial.print(F("PM10 Standard Smoke (µg/m³), "));
  if ((Connected & B00000001) == 1 )
    Serial.print(F("CO2 Value (ppm), "));
  if ((Connected & B00000010) == 2 )
    Serial.print(F(" Temperature (°C), "));
  if ((Connected & B00000100) == 4 )
    Serial.print(F("Humididty (%rH)"));
  Serial.println();

  sampleTime = millis();
}


//_________________________main loop runs continuously__________________________

void loop()
{
  if (millis() >= (sampleRate + sampleTime))
  {
    sampleTime = millis();  //resets timer before printing output
    if ((Connected & B00000001) == 1 ) {
      getT6700data();
    }
    if ((Connected & B00000110) == 6 ) {
      getT9602data();
    }
    if ((Connected & B00001000) == 8 ) {
      getDustValue();
    }

    if ((Connected & B00001000) == 8 ) {
      Serial.print(PM1value); Serial.print(", ");     //PM1.0 Standard Smoke
      Serial.print(PM25value); Serial.print(", ");    //PM2.5 Standard Smoke
      Serial.print(PM10value); Serial.print(", ");    //PM10 Standard Smoke
    } if ((Connected & B00000001) == 1 ) {
      Serial.print(CO2ppmValue); Serial.print(", ");  // CO2 Value
    } if ((Connected & B00000010) == 2 ) {
      Serial.print(temperature, 1); Serial.print(", ");  // Temperature Value
    } if ((Connected & B00000100) == 4 ) {
      Serial.print(humidity, 0); Serial.print(", ");     // HumidityValue
    }   Serial.println();

    displayReading();

  }
}


//___________________________sub routine to read value from sensors_________________
/*
   There are various methods for decoding the serial value from the dust sensor.
   This method used takes a 64 byte string, and then analyses it to find the opening
   0x42, 0x4D bytes, and delivering the values thereafter.
   Other methods include monitoring for the serial break and taking values thereafter,or
   monitoring the stream for opening bytes and recording from there. Limitations in the
   pUNO rocessor disctated this choice.
*/
getDustValue() {
  byte data[32] ; //create an array to store the response
  byte message[64];
  int CRC = 0;
  mySerial.readBytes(message, 64);   //read 64 streamed bytes

  for ( byte i = 0 ; i < 32 ; i++ ) {  //look for ox42, 0x4D sequence and load data from stream
    if ( message[i] == 0x42 && message[i + 1] == 0x4D ) {
      for ( byte j = 0 ; j < 32 ; j++ ) {
        data[j] = message[i];
        i++;
      }
      break;
    }
  }
  //calculate the checksum
  for (int i = 0; i < 30; i++) {
    CRC += data[i];
  }
  if ((data[30] * 256 + data[31]) == CRC) {  //Cyclical Redundancy Check
    PM1value  = ( data[4] & 0x3F ) << 8 | data[5];
    PM25value = ( data[6] & 0x3F ) << 8 | data[7];
    PM10value = ( data[8] & 0x3F ) << 8 | data[9];
  }
  else {
    PM25value = 999;
  }
}

void getT9602data() //gets temp and hum values from T9602 over I2C
{
  Wire.beginTransmission(ADDR_9602);
  Wire.write(0);
  Wire.endTransmission();

  byte data[4];

  Wire.requestFrom(ADDR_9602, 4);
  data[0] = Wire.read();
  data[1] = Wire.read();
  data[2] = Wire.read();
  data[3] = Wire.read();

  // humidity = (rH_High [5:0] x 256 + rH_Low [7:0]) / 16384 x 100
  humidity = (float)(((data[0] & 0x3F ) << 8) + data[1]) / 16384.0 * 100.0;

  // temperature = (Temp_High [7:0] x 64 + Temp_Low [7:2]/4 ) / 16384 x 165 - 40
  temperature = (float)((unsigned)(data[2]  * 64) + (unsigned)(data[3] >> 2 )) / 16384.0 * 165.0 - 40.0;
}


//gets CO2 ppm value from T6700 series over I2C bus
void getT6700data()
{
  byte data[4];
  Wire.beginTransmission(ADDR_6700);
  Wire.write(0x04); Wire.write(0x13); Wire.write(0x8B); Wire.write(0x00); Wire.write(0x01);
  // end transmission
  Wire.endTransmission();

  // read report of current gas measurement in ppm
  delay(5);
  Wire.requestFrom(ADDR_6700, 6);    // request 4 bytes from slave device
  data[0] = Wire.read();
  data[1] = Wire.read();
  data[2] = Wire.read();
  data[3] = Wire.read();
  CO2ppmValue = ((data[2] & 0x3F ) << 8) | data[3];
}



//_________________________________________Display Routines____________________
void display_prepare(void) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
}

void displayReading() {
  display_prepare();
  /*
     Error Byte 0b0ABCDEFG
    A = 1 Laser error
    B = 1 Laser alarm
    C = 1 High temperature alarm
    D = 1 Low temperature alarm
    E = 1 Fan Error
    F = 1 Fan speed compensation start
    G = 1 Fan speed alarm
  */
  display.setCursor(15, 0);  display.print(F("Amphenol Sensors"));
  display.setCursor(13, 55); display.print(F("AQ Evaluation Kit"));


  if ((Connected & B00000001) == 1 || (Connected & B00000010) == 2 ) {
    if ((Connected & B00000001) == 1 ) {
      display.setCursor(3, 12); display.print(F("CO2"));
      display.setCursor(60, 12); display.print(CO2ppmValue);  //CO2 Value
      display.setCursor(90, 12); display.print("ppm");
    } if ((Connected & B00001000) == 8 ) {
      display.setCursor(3, 22); display.print(F("PM2.5"));
      display.setCursor(60, 22); display.print(PM25value);    //PM2.5 Standard Smoke
      display.setCursor(90, 22); display.print("ug/m3");  //display µg/m³
    } if ((Connected & B00000010) == 2 ) {
      display.setCursor(3, 32); display.print(F("Temp"));
      display.setCursor(60, 32); display.print(temperature, 1);  //Temperature Value
      display.setCursor(90, 32); display.write(9); display.print("C");
    } if ((Connected & B00000100) == 4 ) {
      display.setCursor(3, 42); display.print(F("Humidity"));
      display.setCursor(60, 42); display.print(humidity, 0);    //Humidity Value
      display.setCursor(90, 42); display.print("%rH");
    }
  } else {
    display.setCursor(3, 12); display.print(F("PM1.0"));
    display.setCursor(60, 12); display.print(PM1value);    //PM2.5 Standard Smoke
    display.setCursor(90, 12); display.print("ug/m3");  //display µg/m³

    display.setCursor(3, 22); display.print(F("PM2.5"));
    display.setCursor(60, 22); display.print(PM25value);    //PM2.5 Standard Smoke
    display.setCursor(90, 22); display.print("ug/m3");  //display µg/m³

    display.setCursor(3, 32); display.print(F("PM10"));
    display.setCursor(60, 32); display.print(PM10value);    //PM2.5 Standard Smoke
    display.setCursor(90, 32); display.print("ug/m3");  //display µg/m³
  }
  display.display();
}

/*
  Copyright (c) 2019 Amphenol Advanced Sensors
  Permission is hereby granted, free of charge, to any person obtaining a copy of
  this software and associated documentation files (the "Software"), to deal in
  the Software without restriction, including without limitation the rights to
  use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
  the Software, and to permit persons to whom the Software is furnished to do so,
  subject to the following conditions:
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
  FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
  COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
  IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
