#include <OneWire.h>

#include "MegunoLink.h"

#include <PID_v1.h>

//OLED Includes//
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET 4 
Adafruit_SSD1306 display(OLED_RESET);

//double setpoint;
//double Input;

const int RELAYTIMER=250;

double pidOutput;  //Pid variables
const double kp=2.5;
const double ki=0.1;
const double kd=20.0;
const int pidSampleTime=25500;

double thermistorRaw=0.0;
double thermistorFilterFactor=0.5;
double thermistorFiltered=0.0;

const int UPBUTTONPIN=9;
const int DOWNBUTTONPIN=6;
const int OFFBUTTONPIN=5;
double temperatureSetpoint=150.0;
const double MAXTEMPERATURE=175;

const int CONTROLPIN=11;

//Timers and Control//

unsigned long buttonPushTime=0;
unsigned long temperatureReadTimer=0;
unsigned long relayTimer=0;
int relayCounter=0;

const int TEMPERATUREREADTIME=1000;
const int DEBOUNCEDELAY=200;
byte pidMode=0;

//Objects//

OneWire  ds(10);

PID temperaturePID(&thermistorFiltered, &pidOutput, &temperatureSetpoint,kp,ki,kd, DIRECT);

TimePlot pidValues;

//Proto Functions//

void relayControl();
void upButton();
void downButton();
void updateDisplay();
float readThermistor();

void setup()
{
    pidValues.SetTitle("PID Values");

    pinMode(UPBUTTONPIN, INPUT_PULLUP);
    pinMode(DOWNBUTTONPIN, INPUT);
    pinMode(OFFBUTTONPIN, INPUT_PULLUP);

    pinMode(CONTROLPIN, OUTPUT);
    
    thermistorFiltered=readThermistor();

    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  

    Serial.begin(9600);
    
    temperaturePID.SetSampleTime(pidSampleTime);
}

void loop()
{

    if ((millis()-temperatureReadTimer)>TEMPERATUREREADTIME)
    {
      thermistorRaw=readThermistor();
      if (thermistorRaw != 0.00)
      {
        thermistorFiltered=(thermistorFilterFactor*thermistorRaw)+((1-thermistorFilterFactor)*thermistorFiltered);
        temperatureReadTimer=millis();
        if (thermistorRaw>=MAXTEMPERATURE)
        {
          digitalWrite(CONTROLPIN, 0);
        }
      }
    }

    if(digitalRead(OFFBUTTONPIN)==0)
    {
      delay(DEBOUNCEDELAY);
      pidMode++;
      if (pidMode>2)
      {
        pidMode=0;
      }
      if (pidMode==1)
      {
        temperaturePID.SetMode(AUTOMATIC);
      }
      if (pidMode==2)
      {
        temperaturePID.SetMode(MANUAL);
      }
    }

    if (temperaturePID.Compute())
    {
    pidValues.SendData("Temperature", thermistorFiltered);
    pidValues.SendData("PID Output", pidOutput);
    pidValues.SendData("Setpoint", temperatureSetpoint);
    }
    
    if (digitalRead(UPBUTTONPIN)==0)
    {
        upButton();
    }

    if (digitalRead(DOWNBUTTONPIN)==0)
    {
        downButton();
    }

    if (pidMode==0)
    {
        digitalWrite(CONTROLPIN, LOW);
    }
    else
    {
       relayControls();
    }
    
    updateDisplay();
}

void relayControls()
{
   if((millis()-relayTimer)>RELAYTIMER)
   {
    if(relayCounter<=pidOutput)
    {
      if (thermistorFiltered<MAXTEMPERATURE)
      {
        digitalWrite(CONTROLPIN, HIGH);
      }
    }
    else
    {
      digitalWrite(CONTROLPIN, LOW);
    }
    Serial.print("relayCounter: ");
    Serial.println(relayCounter);
    relayCounter++;
    if(relayCounter>255)
    {
      relayCounter=0;
    }
    relayTimer=millis();
   }
}

void upButton()
{   
    if (pidMode==1 || pidMode==0)
    { 
      delay(DEBOUNCEDELAY);
      temperatureSetpoint++;
    }
    if (pidMode==2)
    {
      delay(DEBOUNCEDELAY);
      pidOutput++;
      if (pidOutput>255)
      {
        pidOutput=255;
      }
    }
}

void downButton()
{
     if (pidMode==1 || pidMode==0)
    { 
      delay(DEBOUNCEDELAY);
      temperatureSetpoint--;
    }
    if (pidMode==2)
    {
      delay(DEBOUNCEDELAY);
      pidOutput--;
      if (pidOutput<0)
      {
        pidOutput=0;
      }
    }
}

void updateDisplay()
{
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.print("PV: ");
    display.print(thermistorFiltered);
    if(thermistorFiltered>=MAXTEMPERATURE||thermistorRaw>=MAXTEMPERATURE)
    {
      display.print("    HIGH");
    }
    display.println();
    display.print("SP: ");
    display.println(temperatureSetpoint);
    display.print("OUT: ");
    display.println(pidOutput);
    display.print("PID MODE: ");
    if (pidMode==0)
    {
      display.print("DISABLED");
    }
    else if (pidMode==1)
    {
      display.print("AUTO");
    }
    else if (pidMode==2)
    {
      display.print("MANUAL");
    }
    display.display();
}

float readThermistor()
{

    byte i;
    byte present = 0;
    byte type_s;
    byte data[12];
    byte addr[8]={0x28, 0x39, 0x66, 0x9, 0x8, 0x0, 0x0, 0x5E};
    float celsius, fahrenheit;
    /*  
    if ( !ds.search(addr)) {
      Serial.println("No more addresses.");
      Serial.println();
      ds.reset_search();
      delay(250);
      return 0;
      }
  
    Serial.print("ROM =");
    for( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
    }

    if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return 0;
    }
    Serial.println();
 */
 /*
    // the first ROM byte indicates which chip
    switch (addr[0]) {
    case 0x10:
      Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      return 0;
  } 
*/
  ds.reset();
  ds.select(addr);
  ds.write(0x44);        // start conversion, with parasite power on at the end
  
  //delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  Serial.print("  Data = ");
  Serial.print(present, HEX);
  Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" CRC=");
  Serial.print(OneWire::crc8(data, 8), HEX);
  Serial.println();

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;
  Serial.print("  Temperature = ");
  Serial.print(celsius);
  Serial.print(" Celsius, ");
  Serial.print(fahrenheit);
  Serial.println(" Fahrenheit");

    return fahrenheit;

}

