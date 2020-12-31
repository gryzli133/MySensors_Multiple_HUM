/*
Version:
- 1.00 - starting point, old code with SH1106
- 1.01 - code was cleaned, 4th button added - PB Minus, PB Plus, PB Enter, PB Escape
- 1.02 - split the code, add BME280
- 1.03 - add PID controller and ThermoElectricalValve output (with 4 point mapping)
- 1.04 - add PID limits - Limit PID output to smoothly switch between PID and ON/OFF; WiFi OFF
- 1.05 - add MP_Button
- 2.00 - new Board with new Pins
- 2.01 - OTA possible: Press and Hold Enter; after 2s ESP will enter OTA mode and "OTA" will be on display. Press exit button to exit OTA.
- 2.10 - itegration of MySensors with PJON
- 2.11 - add interaction with controller (tested with Domoticz)
- 2.12 - add Humidity and Pressure sensor for Controller. PID output as Dimmer and Humidity (because of trending)
- 2.13 - better device naming for Controller
- 2.14 - second Temp/Hum transmitter -> Temp = Setpoint / Hum = Valve output (0-100%)
- 2.15 - change CHILD_IDs to get right order for Domoticz Grouping

*/
#define SKETCH_NAME "MP_SmartThermostat"
#define SKETCH_VER "0.01"

/////// SELECT WHICH TERMOSTAT YOU WANT TO DOWNLOAD FROM ID_Matrix ///////////
#define TERMOSTAT_TEST
//#include "ID_Matrix.h"

#ifdef TERMOSTAT_TEST
  #define MY_NODE_ID 108
  #define ROOM_NAME "TEST"
  #define OFFSET -1.5
#endif

#include "ESPGateway.h"

///////////////// DON'T FORGET TO SET IF USING PJON //////////////////
//#define MY_PJON
//#define MY_PJON_PIN 14  // GPIO14 = D5

/////////////////           END           //////////////////           

const bool IS_ACK = true; //is to acknowlage

#define MP_MIN_SYNC 2000        // Send data to controller not often than every 2s
#define MP_MAX_SYNC 30000     // Refresh data to controller at least once every 30s

// Enable MySensors debug prints to serial monitor
//#define MY_DEBUG

// Enable MP_DEBUG debug prints to serial monitor
//#define MP_DEBUG

// Enable prints with loop cycle time
//#define MP_DEBUG_CYCLE_TIME

// Use a bit lower baudrate for serial prints on ESP8266 than default in MyConfig.h
//#define MY_BAUD_RATE 9600
//#define MY_BAUD_RATE 115200

#define MY_TRANSPORT_WAIT_READY_MS 1


//#include <ESP8266WiFi.h>
//#include "MP_OTA.h"
#include <MySensors.h>
/*
#include <Arduino.h>
#include <U8g2lib.h>
#include <PID_v1.h>
#include <Bounce2.h>
#include "BME280.h"
#include "ThermoElectricValve.h"
#include "MP_Polygon.h"

#include <MP_Button.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#define SDA 4
#define SCL 5

MP_BME280 tempSensor(0x76, 100); //i2c address; minimal sample time
*/
// The complete list is available here: https://github.com/olikraus/u8g2/wiki/u8g2setupcpp
//U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);

/*
Bounce debouncerUp = Bounce();  
Bounce debouncerDown = Bounce();
Bounce debouncerEnter = Bounce();
Bounce debouncerEsc = Bounce();




int buttonPinUp = 2;
int buttonPinDown = 0;
int buttonPinEnter = 12;
int buttonPinEsc = 16;
int debaunceTime = 20;

MP_Button buttonEnter(buttonPinEnter, true, 20, 300, 300, 800, 2000, 4000);
*/
double SP = 21.0, PV = 22.0, MV = 51.0; // PID Setpoint, Process value, Manipulated Value (output)
/*
//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=4, consKi=0.2, consKd=1;
//double consKp=1, consKi=0.05, consKd=0.25;

PID myPID(&PV, &MV, &SP, consKp, consKi, consKd, DIRECT);
//ThermoElectricValve(int _relayPin, bool invertedRelay)
ThermoElectricValve valve(15, 0); // SSR PIN
//ThermoElectricValve valve(16, 1); // ONBOARD LED FOR TEST
*/
double scaledMV = MV;

int packet;
int voltage;

unsigned long lastMillis, startMillis, buttonMillis, currentMillis, lastUpdateDisplay;
float actTemp = 22.0;
float lastTemp = actTemp;
float setTemp = 23.0;
float storedSP = setTemp * 10;
float offsetTemp = OFFSET;
float MV_minmin = 0.0;    // fix output limit min
float MV_maxmax = 100.0;  // fix output limit max
float MV_min = 0.0;       // variable output limit min
float MV_max = 100.0;     // variable output limit max
float limit0 = 0.10;       // tolerance
float limit1 = 0.25;      // comfort temperature limit - slow PID
float limit2 = 0.50;      // outside comfort temperature limit - ON/OFF regulation
uint8_t actHum = 99;
uint16_t actPressure = 999;

bool lastButtonUp, lastButtonDown, lastButtonEnter, lastButtonEsc;
bool stateButtonUp, stateButtonDown, stateButtonEnter, stateButtonEsc;
bool displayOn=0;
bool OTAactive = 0;
bool boostActive = 0;
double gap = 0.0; // SP-PV error

uint8_t actScreen = 20;
uint8_t requestSync;
uint32_t lastSync;
bool additionalPresentation = false;

uint8_t CHILD_ID_HVAC = 1;
uint8_t CHILD_ID_TEMP = 10;
uint8_t CHILD_ID_HUM = 11;
uint8_t CHILD_ID_PRESSURE = 12;
uint8_t CHILD_ID_SETPOINT_TEMP = 20;
uint8_t CHILD_ID_OUTPUT_HUM = 21;
uint8_t CHILD_ID_BOOST = 30;
uint8_t CHILD_ID_OUTPUT = 31;
uint8_t CHILD_ID_VOLTAGE = 250;

MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgPressure(CHILD_ID_PRESSURE, V_PRESSURE);
MyMessage msgStatus(CHILD_ID_HVAC, V_STATUS);
MyMessage msgSetpoint(CHILD_ID_HVAC, V_HVAC_SETPOINT_HEAT);
MyMessage msgBoost(CHILD_ID_BOOST, V_STATUS);
MyMessage msgOutput(CHILD_ID_OUTPUT, V_STATUS);
MyMessage msgOutputDimmer(CHILD_ID_OUTPUT, V_PERCENTAGE);
MyMessage msgSetpointTemp(CHILD_ID_SETPOINT_TEMP, V_TEMP);
MyMessage msgOutputHum(CHILD_ID_OUTPUT_HUM, V_HUM);




void presentation()  
{   
  // Send the sketch version information to the gateway and Controller
  char newString[80];
  sprintf(newString,"%s %s",SKETCH_NAME,ROOM_NAME);
    sendSketchInfo(newString, SKETCH_VER);
  sprintf(newString,"%s %s","Termostat",ROOM_NAME);
    present(CHILD_ID_HVAC, S_HEATER, newString, IS_ACK);
    present(CHILD_ID_TEMP, S_TEMP, newString, IS_ACK);
    present(CHILD_ID_HUM, S_HUM, newString, IS_ACK);
    present(CHILD_ID_PRESSURE, S_BARO, newString, IS_ACK);
  sprintf(newString,"%s %s","Termostat SP/MV",ROOM_NAME);
    present(CHILD_ID_SETPOINT_TEMP, S_TEMP, newString, IS_ACK);
  sprintf(newString,"%s %s","Termostat output",ROOM_NAME);
    present(CHILD_ID_OUTPUT_HUM, S_HUM, newString, IS_ACK);
    present(CHILD_ID_OUTPUT, S_DIMMER, newString, IS_ACK);
  sprintf(newString,"%s %s","Boost",ROOM_NAME);
    present(CHILD_ID_BOOST, S_DIMMER, newString, IS_ACK);
}

void setup(void) 
{
  /*
  Serial.begin(9600);
  Wire.begin(SDA, SCL);
  u8g2.begin();  
  u8g2.enableUTF8Print();
  tempSensor.Initiate(); 
  pinMode(buttonPinEnter, INPUT_PULLUP);
  pinMode(buttonPinUp, INPUT_PULLUP);
  pinMode(buttonPinDown, INPUT_PULLUP);
  pinMode(buttonPinEsc, INPUT_PULLUP);
  debouncerUp.attach(buttonPinUp);
  debouncerDown.attach(buttonPinDown);
  debouncerEnter.attach(buttonPinEnter);
  debouncerEsc.attach(buttonPinEsc);
  debouncerUp.interval(debaunceTime);
  debouncerDown.interval(debaunceTime);
  debouncerEnter.interval(debaunceTime);
  debouncerEsc.interval(debaunceTime);
  updateDisplay();
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(MV_minmin, MV_maxmax);
  storedSP = loadState(CHILD_ID_HVAC);
  if (storedSP != 0xff && storedSP != 0x00) {
    setTemp = 0.1 * (float)storedSP + 5.0;
  }
  ////////////////////////////// OTA /////////////////////
  OTAactive = !digitalRead(buttonPinEnter);
  if (OTAactive)
  {
    displayOTA();
    setupOTA();
  }
  else
  {
    WiFi.mode( WIFI_OFF );
    WiFi.forceSleepBegin();
    delay( 1 );
  }*/
}

void updateDisplay()
{/*
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_t0_11_tf);
    u8g2.drawStr(0,8,"12:24");
 
    u8g2.setCursor(100, 8);
    u8g2.print(actHum);
    u8g2.print(" %");

    u8g2.setCursor(80, 63);
    u8g2.print(actPressure);
    u8g2.print(" hPa");  
    
    u8g2.setCursor(0, 20);
    u8g2.print("T");
    u8g2.setFont(u8g2_font_u8glib_4_tf);
    u8g2.print("SET ");
    
    u8g2.setFont(u8g2_font_logisoso16_tf);
    u8g2.setCursor(0, 44);
    u8g2.print(setTemp, 1); 

    u8g2.setFont(u8g2_font_t0_11_tf);
    //u8g2.setCursor(44, 8);
    //u8g2.print(lastMillis);
    */
    /*u8g2.print(MV);
    u8g2.print("/");
    u8g2.print(scaledMV);*/
    /*
    u8g2.setCursor(40, 8);
    u8g2.print(MV_min);
    u8g2.print("/");
    u8g2.print(MV_max);
//    u8g2.print(" ms");    
            
    u8g2.drawHLine(0, 10, 130);
    u8g2.drawHLine(0, 50, (int)scaledMV*128/100); //Progress bar - valve output
    u8g2.drawHLine(0, 51, (int)MV*128/100); //Progress bar - valve output    
    u8g2.drawHLine(0, 52, 130);
    u8g2.setFont(u8g2_font_logisoso24_tf);
    u8g2.setCursor(66, 44);
    u8g2.print(actTemp, 1);
    //u8g2.print("°C");    // requires enableUTF8Print()
    
    u8g2.setFont(u8g2_font_t0_11_tf);
    u8g2.setCursor(1, 62);

//    u8g2.setFontMode(1);    // 0=solid, 1=transparent
    if(actTemp>setTemp)
    {
      u8g2.drawBox(28,53,27,11);
      u8g2.setFontMode(1);
      u8g2.setDrawColor(2);
      u8g2.print("Heat Cool");
    }
    else
    {
      u8g2.drawBox(0,53,27,11);
      u8g2.setFontMode(1);
      u8g2.setDrawColor(2);
      u8g2.print("Heat Cool");      
    }
    
  } while ( u8g2.nextPage() );
*/
}

void updateSetpoint()
{/*
  requestSync = 1;
  u8g2.firstPage();
  do
  {
    u8g2.setFont(u8g2_font_logisoso50_tf);
    u8g2.setCursor(7, 57);
    u8g2.print(setTemp, 1); 
  } while ( u8g2.nextPage() );*/
}

void displayOTA()
{
/*  u8g2.firstPage();
  do
  {
    u8g2.setFont(u8g2_font_logisoso50_tf);
    u8g2.setCursor(7, 57);
    u8g2.print("OTA..."); 
  } while ( u8g2.nextPage() );*/
}

void loop(void) 
{
/*  if(OTAactive)
  {
    updateOTA();   
    if (digitalRead(buttonPinEsc) == LOW)
    {
      ESP.restart();
    } 
  }
  else
  {
    // GET DATA FROM SENSOR
    lastTemp = actTemp;
    actTemp = tempSensor.getTemperature();
    actHum = (int)tempSensor.getHumidity();
    actPressure = (int)tempSensor.getPressure();
  
    if(actTemp != actTemp) //NaN
    {
      actTemp = lastTemp;
    }

    actTemp += offsetTemp;
    */
    // PID BEBIN ******************************************************
    PV = actTemp;
    SP = setTemp;
  /*
    gap = absDouble(SP-PV); //distance away from setpoint
    
    if(actTemp < setTemp) // Limit PID output to smoothly switch between PID and ON/OFF
    {
      MV_min = constrain(mapDouble(gap, limit0, limit2, MV_minmin, MV_maxmax), MV_minmin, MV_maxmax-0.001);
      MV_max = MV_maxmax;    
    }
    else
    {
      MV_min = MV_minmin;  
      MV_max = constrain(mapDouble(gap, limit0, limit2, MV_maxmax, MV_minmin), MV_minmin+0.001, MV_maxmax);
    }
    myPID.SetOutputLimits(MV_min, MV_max);
    
    if (gap < limit1)
    {  //we're close to setpoint, use conservative tuning parameters
      myPID.SetTunings(consKp, consKi, consKd);
    }
    else
    {
       //we're far from setpoint, use aggressive tuning parameters
       myPID.SetTunings(aggKp, aggKi, aggKd);
    }
    
  
    myPID.Compute();
    scaledMV = MP_Polygon(MV, 0, 5, 95, 100, 0, 2, 8, 100);
    valve.Update(scaledMV);
    // END PID  ********************************************************
    
    if (debouncerUp.update()) 
    {
      // Get the update value.
      stateButtonUp = debouncerUp.read();
      // Send in the new value.
      if(stateButtonUp == LOW)
      {
        buttonMillis=millis();
        if(displayOn)
        {
          setTemp += 0.5;
          storedSP = (setTemp - 5.0) * 10.0;
          storedSP = constrain(storedSP, 0, 250);
          saveState(CHILD_ID_HVAC, storedSP);
          updateSetpoint();
        }
      }
    }
    if (debouncerDown.update()) 
    {
      // Get the update value.
      stateButtonDown = debouncerDown.read();
      // Send in the new value.
      if(stateButtonDown == LOW)
      {
        buttonMillis=millis();
        if(displayOn)
        {
          setTemp -= 0.5;
          storedSP = (setTemp - 5.0) * 10.0;
          storedSP = constrain(storedSP, 0, 250);
          saveState(CHILD_ID_HVAC, storedSP);
          updateSetpoint();
        }
      }
    }
    buttonEnter.Update();
    if(buttonEnter.Status() == 10)
    {
      ESP.restart();
    }
    if (debouncerEnter.update()) 
    {
      // Get the update value.
      stateButtonEnter = debouncerEnter.read();
      // Send in the new value.
      if(stateButtonEnter == LOW)
      {
        buttonMillis=millis();
      }
    }
  
    if (debouncerEsc.update()) 
    {
      // Get the update value.
      stateButtonEsc = debouncerEsc.read();
      // Send in the new value.
      if(stateButtonEsc == LOW)
      {
        buttonMillis=millis();
      }
    }
    
    startMillis = millis();
    lastMillis = millis() - startMillis;
  
    if(millis() - buttonMillis > 60000)
    {
      if(displayOn)
      {
        u8g2.setPowerSave(1);
        displayOn = 0;
      }
    }
    else
    {
      if(!displayOn)
      {
        updateDisplay();
        lastUpdateDisplay = millis();
        u8g2.setPowerSave(0);
        displayOn = 1;
      }
    }
    if(millis() - buttonMillis > 1000 && millis() - lastUpdateDisplay > 1000 && displayOn)
    {
      updateDisplay();
      lastUpdateDisplay = millis();
    }
  }
  */
  if((requestSync == 1 && millis() - lastSync > MP_MIN_SYNC) ||              // sync first time
   (requestSync == 2 && millis() - lastSync > (MP_MIN_SYNC+MP_MAX_SYNC)/4) || // resync after quarter time
   (requestSync == 3 && millis() - lastSync > (MP_MIN_SYNC+MP_MAX_SYNC)/2) || // resync after half time
   millis() - lastSync > MP_MAX_SYNC)                                      // resync every MP_MAX_SYNC time
   {
    sendState();
  }
  /*if(additionalPresentation == false && millis() - lastSync > 5000 && millis() > 30000)
  {
    presentation();
    additionalPresentation = true;
  }*/
}

void sendState() 
{
  send(msgSetpoint.set(setTemp, 1));  
  send(msgTemp.set(actTemp, 1));
  send(msgHum.set(actHum));
  send(msgPressure.set(actPressure));
  send(msgSetpointTemp.set(setTemp, 1));  
  send(msgOutputHum.set((int)MV));
  send(msgOutputDimmer.set((int)MV));
  send(msgOutput.set(MV > 0.0));
  send(msgStatus.set(MV > 0.0));
  send(msgBoost.set(boostActive));
  requestSync ++;
  lastSync = millis();
}

void sendTemp()
{
  
}

void receive(const MyMessage &message)
{
  if (message.sensor == CHILD_ID_HVAC) 
  {
    switch (message.type) 
    {
      case V_HVAC_SETPOINT_HEAT:
        setTemp = message.getFloat();
        storedSP = (setTemp - 5.0) * 10.0;
        storedSP = constrain(storedSP, 0, 250);
        saveState(CHILD_ID_HVAC, storedSP);
        requestSync = 1;
        break;

      case V_HVAC_SETPOINT_COOL:
        setTemp = message.getFloat();
        storedSP = (setTemp - 5.0) * 10.0;
        storedSP = constrain(storedSP, 0, 250);
        saveState(CHILD_ID_HVAC, storedSP);
        requestSync = 1;
        break;
  
      case V_TEMP:
        // 2check
        break;
  
      case V_STATUS:
        // 2check
        break;
  
      case V_HVAC_FLOW_STATE:
        // 2check
        break;

      case V_HVAC_SPEED:
        // 2check
        break;
    }
    requestSync = 1;
  }
}
