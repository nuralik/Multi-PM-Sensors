/***********************************************************************************************************************************************************************************************************                           
************************************************************************************************************************************************************************************************************
▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒█▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀█
▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒█░▒▒▒▒▒▒▒▓▒▒▓▒▒▒▒▒▒▒░█
▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒█░▒▒▓▒▒▒▒▒▒▒▒▒▄▄▒▓▒▒░█░▄▄
▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▄▀▀▄▄█░▒▒▒▒▒▒▓▒▒▒▒█░░▀▄▄▄▄▄▀░░█
▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒█░░░░█░▒▒▒▒▒▒▒▒▒▒▒█░░░░░░░░░░░█                       
▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▀▀▄▄█░▒▒▒▒▓▒▒▒▓▒█░░░█▒░░░░█▒░░█                      
▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒█░▒▓▒▒▒▒▓▒▒▒█░░░░░░░▀░░░░░█                      
▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▄▄█░▒▒▒▓▒▒▒▒▒▒▒█░░█▄▄█▄▄█░░█                      
▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒█░░░█▄▄▄▄▄▄▄▄▄▄█░█▄▄▄▄▄▄▄▄▄█                          
▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒█▄▄█░░█▄▄█░░░░░░█▄▄█░░█▄▄█                      For a sustainable future       

 ********************************************************************************************************************************************************************************************************
 ********************************************************************************************************************************************************************************************************

 This Project Is supported by NYSERDA,USA

 This Project is an open source Project (https://github.com/nuralik/A-smart-mobile-platform-for-air-quality-monitoring)

 All library used in this project is open sourced and from Internet
 
 Nueraili Kuerbanjiang              kuerban@clarkson.edu
 
 ******************************************************************************************************************************************************************************************************** 
 ********************************************************************************************************************************************************************************************************/

#include "Wire.h"
#include "RTClib.h"
#include "Adafruit_HTU21DF.h"
#include "SD.h"
#include "SoftwareSerial.h"
#include "Arduino.h"

File myFile;//SD card
SoftwareSerial hpma(11,63);//Honeywell PM Sensors , TX plug in pin 11, RX plug in pin A9
SoftwareSerial pmsSerial(13, 64 ); //Plantower PM Sensors ignore RX, plug TX in Pin 13
RTC_DS3231 rtc;//Real Time Clock
Adafruit_HTU21DF htu = Adafruit_HTU21DF();//RH Sensors

//Anemometer
double x = 0;
double y = 0;
double a = 0;
double b = 0;
const int sensorPin = 60; //Defines the pin that the anemometer output is connected 
const int numReadings = 1; //Defines number of reading to calculate average windspeed
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int totalWind= 0;                  // the running total
int averageWind = 0;                // the average
int inputPin = 60;
int sensorValue = 0; //Variable stores the value direct from the analog pin
float sensorVoltage = 0; //Variable that stores the voltage (in Volts) from the anemometer being sent to the analog pin
float sensorVoltage2 = 0; //Variable that stores the voltage (in Volts) from the anemometer being sent to the analog pin
float windSpeedex = 0; // Wind speed in meters per second (m/s)
float voltageConversionConstant = .004882814; //This constant maps the value provided from the analog read function, which ranges from 0 to 1023, to actual voltage, which ranges from 0V to 5V
int sensorDelay = 2000; //Delay between sensor readings, measured in milliseconds (ms)
//Anemometer Technical Variables
//The following variables correspond to the anemometer sold by Adafruit, but could be modified to fit other anemometers.
float voltageMin = .4; // Mininum output voltage from anemometer in mV.
float windSpeedMin = 0; // Wind speed in meters/sec corresponding to minimum voltage
float voltageMax = 2.0; // Maximum output voltage from anemometer in mV.
float windSpeedMax = 32; // Wind speed in meters/sec corresponding to maximum voltage


int windRevC_Pin = 54; //plug in pin A1
int windRevC_tmp = 55;//A2

int TMP_Therm_ADunits;  //temp termistor value from wind sensor
float RV_Wind_ADunits;    //RV output from wind sensor 
float RV_Wind_Volts;
unsigned long lastMillis;
int TempCtimes100;
float zeroWind_ADunits;
float zeroWind_volts;
float WindSpeed_MPH;
float WindSpeed_mps;
const float zeroWindAdjustment =  .2;


int PM25, PM10; //Honeywell
int pin_shin1 = 5;
int pin_shin2 = 6;

unsigned long duration1;
unsigned long duration2;
unsigned long starttime;
unsigned long sampletime_ms = 3000;
unsigned long lowpulseoccupancy1 = 0;
unsigned long lowpulseoccupancy2 = 0;
float ratio1 = 0;
float ratio2 = 0;
float concentration1 = 0;
float concentration2 = 0;

void setup() {
  
  //pinMode(windAnemometer_Pin, INPUT);
  //pinMode(windRevC_Pin, INPUT);
  pinMode(pin_shin1, INPUT);
  pinMode(pin_shin2, INPUT);

  starttime = millis(); 
 
  
  Serial.begin(9600);
  
    for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
  // clock begin
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  // RH sensor begin
  if (!htu.begin()) {
    Serial.println("Couldn't find sensor!");
    while (1);
  }
  //SD Card
  pinMode(10, OUTPUT);
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    return;
  }
}
 auto lastRead = millis();
//Plantower
  struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};
struct pms5003data data;

void loop() {
  // Realtime clock data read
  
  DateTime now = rtc.now(); //real time clock data
  Serial.print(now.unixtime()); Serial.print(',');
  Serial.print(htu.readTemperature()); Serial.print(',');
  Serial.print(htu.readHumidity()); Serial.print(',');


  
  int TMP_Therm_ADunits = analogRead(windRevC_tmp);
  int WindRevC_sig = analogRead(windRevC_Pin);
      RV_Wind_Volts = (WindRevC_sig *  0.0048828125);

  TempCtimes100 = (0.005 *((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits)) - (16.862 * (float)TMP_Therm_ADunits) + 9075.4;  
  zeroWind_ADunits = -0.0006*((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits) + 1.0727 * (float)TMP_Therm_ADunits + 47.172;  //  13.0C  553  482.39
  zeroWind_volts = (zeroWind_ADunits * 0.0048828125) - zeroWindAdjustment;  
  WindSpeed_MPH =  pow(((RV_Wind_Volts - zeroWind_volts) /.2300) , 2.7265);
  WindSpeed_mps= WindSpeed_MPH * 0.44704;
  
  sensorValue = analogRead(sensorPin); //Get a value between 0 and 1023 from the analog pin connected to the anemometer
   // subtract the last reading:
  totalWind = totalWind - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = sensorValue;
  // add the reading to the total:
  totalWind = totalWind + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;
 sensorVoltage2 = sensorValue * voltageConversionConstant; //Convert sensor value to actual voltage
  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
    // calculate the average:
  averageWind = totalWind / numReadings;
  sensorVoltage = averageWind * voltageConversionConstant; //Convert sensor value to actual voltage 
  //Convert voltage value to wind speed using range of max and min voltages and wind speed for the anemometer
  }
  if (sensorVoltage <= voltageMin) {
    windSpeedex = 0; //Check if voltage is below minimum value. If so, set wind speed to zero.
  } else {
    windSpeedex = ((sensorVoltage - voltageMin) * windSpeedMax / (voltageMax - voltageMin))*2.232694; //For voltages above minimum value, use the linear relationship to calculate wind speed.
  }
    //Max wind speed calculation
  x = windSpeedex;
  if (x >= y) {
    y = x;
  } else {
    y = y;
  }
//Max voltage calculation
  a = sensorVoltage;
  if (a >= b) {
    b = a;
  } else {
    b = b;
  }
  //Print voltage and windspeed to serial

 // Serial.print("Voltage: ");
 // Serial.print(sensorVoltage);
 // Serial.print("Average: ");
  //Serial.print(averageWind);
  //Serial.print("\t");
 // Serial.print("Wind speed: ");
 // Serial.print(windSpeedex); 
  delay(sensorDelay);
  
  
  Serial.print(WindSpeed_mps);Serial.print(',');
  Serial.print(windSpeedex);Serial.print(',');

    // plantower data
  pmsSerial.begin(9600);
  delay(5000);
  if (readPMSdata(&pmsSerial)) {
     //reading data was successful!
    Serial.print(data.pm10_standard); Serial.print(',');
    Serial.print(data.pm25_standard); Serial.print(',');
    Serial.print(data.pm100_standard); Serial.print(',');
    Serial.print(data.pm10_env); Serial.print(',');
    Serial.print(data.pm25_env); Serial.print(',');
    Serial.print(data.pm100_env); Serial.print(',');
    Serial.print(data.particles_03um); Serial.print(',');
    Serial.print(data.particles_05um); Serial.print(',');
    Serial.print(data.particles_10um); Serial.print(',');
    Serial.print(data.particles_25um); Serial.print(',');
    Serial.print(data.particles_50um); Serial.print(',');
    Serial.print(data.particles_100um); Serial.print(',');
  }
  pmsSerial.end();

  // honeywell data
  hpma.begin(9600);

  stop_autosend();
  start_measurement();
  read_measurement();
  Serial.print(PM25, DEC); Serial.print(',');
  Serial.print(PM10, DEC);   
  hpma.end();

 duration1 = pulseIn(pin_shin1, LOW);
 duration2 = pulseIn(pin_shin2, LOW);
 lowpulseoccupancy1 = lowpulseoccupancy1+duration1;
 lowpulseoccupancy2 = lowpulseoccupancy2+duration2;
 if ((millis()-starttime) > sampletime_ms)//if the sampel time == 30s
 {
 ratio1 = lowpulseoccupancy1/(sampletime_ms*10.0); // Integer percentage 0=>100
 ratio2 = lowpulseoccupancy2/(sampletime_ms*10.0);
 concentration1 = (1.1*pow(ratio1,3)-3.8*pow(ratio1,2)+520*ratio1+0.62)/283.1685; // using spec sheet curve
 concentration2 = (1.1*pow(ratio2,3)-3.8*pow(ratio2,2)+520*ratio2+0.62)/283.1685;
 Serial.print(",");
 Serial.print(concentration1); 
 Serial.print(",");
 Serial.println(concentration2);
 lowpulseoccupancy1 = 0;
 lowpulseoccupancy2 = 0;
 starttime = millis();
 }

  myFile = SD.open("june.txt", FILE_WRITE);
  if (myFile) {

    myFile.print(now.unixtime()); myFile.print(',');
    myFile.print(htu.readTemperature()); myFile.print(',');
    myFile.print(htu.readHumidity()); myFile.print(',');
    myFile.print(WindSpeed_mps); myFile.print(',');
    myFile.print(windSpeedex); myFile.print(',');
    myFile.print(data.pm10_standard); myFile.print(',');
    myFile.print(data.pm25_standard); myFile.print(',');
    myFile.print(data.pm100_standard); myFile.print(',');
    myFile.print(data.pm10_env); myFile.print(',');
    myFile.print(data.pm25_env); myFile.print(',');
    myFile.print(data.pm100_env); myFile.print(',');
    myFile.print(data.particles_03um); myFile.print(',');
    myFile.print(data.particles_05um); myFile.print(',');
    myFile.print(data.particles_10um); myFile.print(',');
    myFile.print(data.particles_25um); myFile.print(',');
    myFile.print(data.particles_50um); myFile.print(',');
    myFile.print(data.particles_100um); myFile.print(',');
    myFile.print(PM25, DEC); myFile.print(',');
    myFile.print(PM10, DEC); myFile.print(',');
    myFile.print(concentration1);myFile.print(',');
    myFile.print(concentration2);
    myFile.println(' ');

  }
  // close the file:
  myFile.close();

}

boolean readPMSdata(Stream *s) {
  if (! s->available()) {
    return false;
  }

  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }

  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }

  uint8_t buffer[32];
  uint16_t sum = 0;
  s->readBytes(buffer, 32);

  // get checksum ready
  for (uint8_t i = 0; i < 30; i++) {
    sum += buffer[i];
  }

  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i = 0; i < 15; i++) {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  }

  // put it into a nice struct :)
  memcpy((void *)&data, (void *)buffer_u16, 30);

  if (sum != data.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
}

bool start_measurement(void)
{
  // First, we send the command
  byte start_measurement[] = {0x68, 0x01, 0x01, 0x96 };
  hpma.write(start_measurement, sizeof(start_measurement));
  //Then we wait for the response
  while (hpma.available() < 2);
  char read1 = hpma.read();
  char read2 = hpma.read();
  // Test the response
  if ((read1 == 0xA5) && (read2 == 0xA5)) {
    // ACK
    return 1;
  }
  else if ((read1 == 0x96) && (read2 == 0x96))
  {
    // NACK
    return 0;
  }
  else return 0;
}

bool stop_measurement(void)
{
  // First, we send the command
  byte stop_measurement[] = {0x68, 0x01, 0x02, 0x95 };
  hpma.write(stop_measurement, sizeof(stop_measurement));
  //Then we wait for the response
  while (hpma.available() < 2);
  char read1 = hpma.read();
  char read2 = hpma.read();
  // Test the response
  if ((read1 == 0xA5) && (read2 == 0xA5)) {
    // ACK
    return 1;
  }
  else if ((read1 == 0x96) && (read2 == 0x96))
  {
    // NACK
    return 0;
  }
  else return 0;
}

bool read_measurement (void)
{
  // Send the command 0x68 0x01 0x04 0x93
  byte read_particle[] = {0x68, 0x01, 0x04, 0x93 };
  hpma.write(read_particle, sizeof(read_particle));
  // A measurement can return 0X9696 for NACK
  // Or can return eight bytes if successful
  // We wait for the first two bytes
  while (hpma.available() < 1);
  byte HEAD = hpma.read();
  while (hpma.available() < 1);
  byte LEN = hpma.read();
  // Test the response
  if ((HEAD == 0x96) && (LEN == 0x96)) {
    // NACK
    Serial.println("NACK");
    return 0;
  }
  else if ((HEAD == 0x40) && (LEN == 0x05))
  {
    // The measuremet is valid, read the rest of the data
    // wait for the next byte
    while (hpma.available() < 1);
    byte COMD = hpma.read();
    while (hpma.available() < 1);
    byte DF1 = hpma.read();
    while (hpma.available() < 1);
    byte DF2 = hpma.read();
    while (hpma.available() < 1);
    byte DF3 = hpma.read();
    while (hpma.available() < 1);
    byte DF4 = hpma.read();
    while (hpma.available() < 1);
    byte CS = hpma.read();
    // Now we shall verify the checksum
    if (((0x10000 - HEAD - LEN - COMD - DF1 - DF2 - DF3 - DF4) % 0XFF) != CS) {
      Serial.println("Checksum fail");
      return 0;
    }
    else
    {
      // Checksum OK, we compute PM2.5 and PM10 values
      PM25 = DF1 * 256 + DF2;
      PM10 = DF3 * 256 + DF4;
      return 1;
    }
  }
}

bool stop_autosend(void)
{
  // Stop auto send
  byte stop_autosend[] = {0x68, 0x01, 0x20, 0x77 };
  hpma.write(stop_autosend, sizeof(stop_autosend));

  //Then we wait for the response
  while (hpma.available() < 2);
  char read1 = hpma.read();
  char read2 = hpma.read();

  // Test the response
  if ((read1 == 0xA5) && (read2 == 0xA5)) {
    // ACK
    return 1;
  }
  else if ((read1 == 0x96) && (read2 == 0x96))
  {
    // NACK
    return 0;
  }
  else return 0;
}

bool start_autosend(void)
{
  // Start auto send
  byte start_autosend[] = {0x68, 0x01, 0x40, 0x57 };
  hpma.write(start_autosend, sizeof(start_autosend));
  //Then we wait for the response
  while (hpma.available() < 2);
  char read1 = hpma.read();
  char read2 = hpma.read();
  // Test the response
  if ((read1 == 0xA5) && (read2 == 0xA5)) {
    // ACK
    return 1;
  }
  else if ((read1 == 0x96) && (read2 == 0x96))
  {
    // NACK
    return 0;
  }
  else return 0;
}
