/*-----( Import needed libraries )-----*/
#include <SPI.h>   // Comes with Arduino IDE
#include "RF24.h"  // Download and Install (See above)
#include <LiquidCrystal.h>
#include <OneWire.h>
#include <DallasTemperature.h>
/*-----( Declare Constants and Pin Numbers )-----*/
//None yet
#define ONE_WIRE_BUS 0
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
/*-----( Declare objects )-----*/
// (Create an instance of a radio, specifying the CE and CS pins. )
RF24 myRadio (2, 3); // "myRadio" is the identifier you will use in following methods
/*-----( Declare Variables )-----*/
byte addresses[][6] = {"1Node"}; // Create address for 1 pipe.
int dataTransmitted;  // Data that will be Transmitted from the transmitter



// select the pins used on the LCD panel
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// define some values used by the panel and buttons
int lcd_key     = 0;
int adc_key_in  = 0;
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5
//#define LCDOn() analogWrite(10,127) // turn on backlight
#define LCDOn() analogWrite(10,32+(analogRead(1)*0.1357)) // turn on backlight
//#define LCDOn() digitalWrite(10,1) // turn on backlight
#define LCDOff() analogWrite(10,1+(analogRead(1)*0.03125)) // turn off backlight
//#define LCDOff() digitalWrite(10,0) // turn off backlight
#define MaxTemp 85
#define MinTemp 60

bool changed = 0; //to detect state change for manual LCD brightness purposes.
bool pressed = 0; //to detect buttons pressed for manual LCD brightness purposes.
int HB = 0; // HEAT BLAST
unsigned long HBTime = 0; //Heat Blast start time
int CB = 0; // COOL BLAST
unsigned long CBTime = 0; //Cool Blast start time
//unsigned long BCancel = 0;
int sensorPin = 1;    // The potentiometer is connected to
// analog pin 0

int ledPin = 13;      // The LED is connected to digital pin 13
int heat = 0;
bool lastheat = 0;
int count = 0;
int degreesF;
int TargetTemp = 70 ; //deg F
int LastButton = btnNONE;
unsigned long LastBtnTime = 0;
unsigned long LastTransmit;
//unsigned long start_time

// read the buttons
int read_LCD_buttons()
{
  adc_key_in = analogRead(0);      // read the value from the sensor
  // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
  // we add approx 50 to those values and check to see if we are close
  if (adc_key_in > 1000) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result
  if (adc_key_in < 50)   return btnRIGHT;
  if (adc_key_in < 195)  return btnUP;
  if (adc_key_in < 380)  return btnDOWN;
  if (adc_key_in < 555)  return btnLEFT;
  if (adc_key_in < 790)  return btnSELECT;
  return btnNONE;  // when all others fail, return this...
}

void setup() {
  // put your setup code here, to run once:
  lcd.begin(16, 2);              // start the library
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp|Target|*Off"); // print a simple message
  lcd.setCursor(6, 1);
  lcd.print(TargetTemp);
  sensors.requestTemperatures();
  delay(550); //wait for good first read
  degreesF = sensors.getTempCByIndex(0) * (9.0 / 5.0) + 32.0;
  //degreesF = degreesC * (9.0/5.0) + 32.0;
  lcd.setCursor(1, 1); // "Temp|Target|*Off" is top line
  lcd.print(degreesF);
  LastTransmit = millis();
  // END LCD, BEGIN NRF
  //  Serial.begin(115200);
  //  delay(1000);
  //  Serial.println(F("RF24/Simple Transmit data Test"));
  //  LCDOff();
  myRadio.begin();  // Start up the physical nRF24L01 Radio
  //  digitalWrite(10,0);
  LCDOn();
  myRadio.setChannel(108);  // Above most Wifi Channels
  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  //myRadio.setPALevel(RF24_PA_MIN);
  myRadio.setPALevel(RF24_PA_MAX);  // Uncomment for more power

  myRadio.openWritingPipe( addresses[0]); // Use the first entry in array 'addresses' (Only 1 right now)
  //LCDOn();
  // END NRF

}

void loop() {
  LastButton = read_LCD_buttons();
  if ( (millis() - LastBtnTime) > 200 ) //button noise filter
  {
    switch (LastButton)
    {
      case btnRIGHT:
        {
          //     lcd.print("RIGHT ");
          if (!HB)
            HBTime = millis();
          CB = 0;
          HB++;
          lcd.setCursor(10, 1);
          lcd.print("      ");
          lcd.setCursor(10, 1);
          lcd.print("HB-");
          lcd.print((int)((1 + (HBTime + 600000 * HB - millis()) / 60000)));
          LastBtnTime = millis();
          break;
        }
      case btnLEFT:
        {
          //     lcd.print("LEFT   ");
          if (!CB)
            CBTime = millis();
          HB = 0;
          CB++;
          LastBtnTime = millis();
          lcd.setCursor(10, 1);
          lcd.print("      ");
          lcd.setCursor(10, 1);
          lcd.print("CB-");
          lcd.print((int)((1 + (CBTime + 600000 * CB - millis()) / 60000)));
          break;
        }
      case btnUP:
        {
          //     lcd.print("UP    ");
          if (TargetTemp < MaxTemp)
            TargetTemp++;
          LastBtnTime = millis();
          lcd.setCursor(6, 1);
          lcd.print(TargetTemp);
          break;
        }
      case btnDOWN:
        {
          //     lcd.print("DOWN  ");
          if (TargetTemp > MinTemp)
            TargetTemp--;
          LastBtnTime = millis();
          lcd.setCursor(6, 1);
          lcd.print(TargetTemp);
          break;
        }
      case btnSELECT:
        {
          //     lcd.print("SELECT");
          LastBtnTime = millis();

          break;
        }
      case btnNONE:
        {
          //     lcd.print("NONE  ");
          break;
        }
    } // endif button switch
  } // endif button noise

  //lcd.setCursor(10,1);
  //lcd.print(millis() - LastBtnTime);

  if (millis() - LastBtnTime < 15000 ) // light fade
  {
    LCDOn();
  }
  else
  {
    LCDOff();
  }


  if ( (millis() - LastTransmit) >= 2000 )
  {
    sensors.requestTemperatures();
    degreesF = sensors.getTempCByIndex(0) * (9.0 / 5.0) + 32.0;
    if (CBTime + 600000 * CB <= millis())
      CB = 0;
    if (HBTime + 600000 * HB <= millis())
      HB = 0;

    if ( (TargetTemp <= MinTemp && !HB) || CB  )
      heat = 0;
    else if ( (TargetTemp >= MaxTemp && !CB) || HB )
      heat = 1;
    else // sensor in range or HB on/off
    {
      if ( TargetTemp > degreesF )
        heat = 1;
      else
        heat = 0;
    }


    lcd.setCursor(10, 1);
    lcd.print("      ");
    //    lcd.print("x");
    if (CB)
    {
      lcd.setCursor(10, 1);
      lcd.print("CB-");
      lcd.print((int)((1 + (CBTime + 600000 * CB - millis()) / 60000)));
      if (CB > 6)
        CB = 0;
    }
    else if (HB)
    {
      lcd.setCursor(10, 1);
      lcd.print("HB-");
      lcd.print((int)((1 + (HBTime + 600000 * HB - millis()) / 60000)));
      if (HB > 6)
        HB = 0;
    }
    /*    else // (!HB && !CB)
        {
          lcd.setCursor(10,1);
          lcd.print("      ");
        }  */
    myRadio.write( &heat, sizeof(heat) ); //  Transmit the data
    //    lcd.print("mit");
    LastTransmit = millis();
    //    lcd.setCursor(10,1);
    //    lcd.print("123");
    lcd.setCursor(1, 1); // "Temp|Target|*Off" is top line
    lcd.print(degreesF);
    lcd.setCursor(13, 0);
    if (heat)
      lcd.print("On ");
    else
      lcd.print("Off");
  }

}
