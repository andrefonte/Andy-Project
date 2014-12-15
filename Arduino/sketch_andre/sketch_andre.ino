#include <SPI.h>
#include "Adafruit_BLE_UART.h"

//uncomment this for dev mode
//http://liudr.wordpress.com/2011/02/04/how-to-optimize-your-arduino-memory-usage/
#define DEVMODE 1

// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 3     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9

/* Blink without Delay
 
 Turns on and off a light emitting diode(Led) connected to a digital  
 pin, without using the delay() function.  This means that other code
 can run at the same time without being interrupted by the Led code.
 
 The circuit:
 * Led attached from pin 13 to ground.
 * Note: on most Arduinos, there is already an Led on the board
 that's attached to pin 13, so no hardware is needed for this example.
 
 
 created 2005
 by David A. Mellis
 modified 8 Feb 2010
 by Paul Stoffregen
 
 This example code is in the public domain.

 
 http://www.arduino.cc/en/Tutorial/BlinkWithoutDelay
 */
 
#define MESSAGE_SIZE 2
#define HEADER_INDEX 0
#define MESSAGE_INDEX 1

enum MessageHeader {
  GPS_DATA = 1,
  ACTIVITY_DATA = GPS_DATA + 1,
};

enum LedStatus {
  LED_OFF = 0,
  LED_ON = LED_OFF + 1,
  LED_BLINK = LED_ON + 1,
};

// Request
enum ActivityRequest{
 START_ACTIVITY = 10,
 STOP_ACTIVITY = START_ACTIVITY + 1,
};

enum GPSRequest{
 START_GPS = 20,
 STOP_GPS = START_GPS + 1,
};

// Response
enum ActivityStatus{
 ACTIVITY_STOPPED = 30,
 ACTIVITY_IN_PROGESS = ACTIVITY_STOPPED + 1,
};

enum GPSStatus{
 GPS_TURNED_OFF = 40,
 GPS_TURNED_ON = GPS_TURNED_OFF + 1,
};

// constants won't change. Used here to 
// set pin numbers:
const int gpsButtonPin = 4;          // the number of the pushbutton pin
const int activityButtonPin = 5;     // the number of the pushbutton pin
const int GPSLedPin =  6;            // the number of the Led pin
const int activityLedPin =  8;       // the number of the Led pin
const char * deviceName = "AndyBLE"; // Maximum size: 7 characters

// the follow variables is a long because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
const long interval = 1000;                // interval at which to blink (milliseconds)

// Variables will change:
int gpsButtonState = 0;                // variable for reading the pushbutton status
int activityButtonState = 0;           // variable for reading the pushbutton status
int activityLedState = LOW;            // ledState used to set the Led
int GPSLedState = LOW;                 // ledState used to set the Led
long previousMillisGPS = 0;            // will store last time Led was updated
long previousMillisActivity = 0;       // will store last time Led was updated
int desiredActivityLedState = LED_OFF;
int desiredGPSLedState = LED_BLINK;
int activityStatus = ACTIVITY_STOPPED;
int GPSStatus = GPS_TURNED_OFF;

/**************************************************************************/
/*!    Constantly checks for new events on the nRF8001 */
/**************************************************************************/

aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

// Utility functions and callbacks

void turnONLed(int ledPin, int * ledState)
{
  *ledState = HIGH;
  digitalWrite(ledPin, *ledState);
}

void turnOFFLed(int ledPin, int * ledState)
{
  *ledState = LOW;
  digitalWrite(ledPin, *ledState);
}

void blinkLed(int ledPin, int * ledState, long * previousMillis)
{
  // check to see if it's time to blink the Led; that is, if the 
  // difference between the current time and last time you blinked 
  // the Led is bigger than the interval at which you want to 
  // blink the Led.
  unsigned long currentMillis = millis();
 
  if(currentMillis - *previousMillis > interval) {
    // save the last time you blinked the Led 
    *previousMillis = currentMillis;   

    // if the Led is off turn it on and vice-versa:
    if (*ledState == LOW)
      *ledState = HIGH;
    else
      *ledState = LOW;

    // set the Led with the ledState of the variable:
    digitalWrite(ledPin, *ledState);
  }
}

void checkLedState(int ledPin, int desiredLedState, int * ledState, long * previousMillis)
{
  switch(desiredLedState) {
    case LED_BLINK:
      blinkLed(ledPin, ledState, previousMillis);
      break;
    case LED_OFF:
      if (*ledState == HIGH) {
        turnOFFLed(ledPin, ledState);
      }
      break;
    case LED_ON:
      if (*ledState == LOW) {
        turnONLed(ledPin, ledState);
      }
      break;
  }
}

void checkGPSLedState()
{
  checkLedState(GPSLedPin, desiredGPSLedState, &GPSLedState, &previousMillisGPS);
}

void checkActivityLedState()
{
  checkLedState(activityLedPin, desiredActivityLedState, &activityLedState, &previousMillisActivity);
}

void writeBTLEData(int header, int data)
{
  uint8_t message[MESSAGE_SIZE];
  message[HEADER_INDEX] = header;
  message[MESSAGE_INDEX] = data;
  
  BTLEserial.write(message, MESSAGE_SIZE);
}

int checkPushButtonHigh(int buttonPin) {
  return (digitalRead(buttonPin) == HIGH);
}

// Uses more memory :(
//  if (checkPushButtonHigh(activityButtonPin)) {
//    if (GPSStatus == GPS_TURNED_OFF) {
//      writeBTLEData(START_GPS);
//    }
//    else {
//      writeBTLEData(STOP_GPS);
//    }
//  }

void checkActivityButtonState()
{ 
  int buttonState = digitalRead(activityButtonPin);
  
  if (buttonState == HIGH) {
    if (activityStatus == ACTIVITY_STOPPED) {
      writeBTLEData(ACTIVITY_DATA, START_ACTIVITY);
    }
    else {
      writeBTLEData(ACTIVITY_DATA, STOP_ACTIVITY);
    }
  }
}

void checkGPSButtonState()
{
  int buttonState = digitalRead(gpsButtonPin);
  
  if (buttonState == HIGH) {
    if (GPSStatus == GPS_TURNED_OFF) {
      writeBTLEData(GPS_DATA, START_GPS);
    }
    else {
      writeBTLEData(GPS_DATA, STOP_GPS);
    }
  }
}

void handleGPSResponse(int messageReceived)
{
  switch(messageReceived) {
    case GPS_TURNED_OFF:
      desiredGPSLedState = LED_BLINK;
      break;
    case GPS_TURNED_ON:
      desiredGPSLedState = LED_OFF;
      break;
    default:
      desiredGPSLedState = LED_ON;
      break;
  }
}

void handleActivityResponse(int messageReceived)
{
  switch(messageReceived) {
    case ACTIVITY_STOPPED:
      desiredActivityLedState = LED_OFF;
      break;
    case ACTIVITY_IN_PROGESS:
      desiredActivityLedState = LED_BLINK;
      break;
    default:
      desiredGPSLedState = LED_ON;
      break;
  }
}

/**************************************************************************/
/*!
    This function is called whenever select ACI events happen
*/
/**************************************************************************/
void aciCallback(aci_evt_opcode_t event)
{
  laststatus = event;
#if defined(DEVMODE)
  switch(event)
  {
    case ACI_EVT_DEVICE_STARTED:
      Serial.println(F("Advertising started"));
      break;
    case ACI_EVT_CONNECTED:
      Serial.println(F("Connected!"));
      break;
    case ACI_EVT_DISCONNECTED:
      Serial.println(F("Disconnected or advertising timed out"));
      break;
    default:
      break;
  }
#endif
}

/**************************************************************************/
/*!
    This function is called whenever data arrives on the RX channel
*/
/**************************************************************************/
void rxCallback(uint8_t *buffer, uint8_t len)
{
#if defined(DEVMODE)
  Serial.print(F("Received "));
  Serial.print(len);
#endif

  if(len > 1) {
    int headerReceived = buffer[0];
    int messageReceived = buffer[1];
    
    if(headerReceived == GPS_DATA) {
      handleGPSResponse(messageReceived);
    }
    else if(headerReceived == ACTIVITY_DATA) {
      handleActivityResponse(messageReceived);
    }
  } 
}

/**************************************************************************/
/*!
Configure the Arduino and start advertising with the radio
*/
/**************************************************************************/ 

void setup() {
  // set the digital pin as output:
  pinMode(GPSLedPin, OUTPUT);
  pinMode(activityLedPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(gpsButtonPin, INPUT);
  pinMode(activityButtonPin, INPUT);
  
  #if defined(DEVMODE)
    Serial.begin(9600);
    Serial.println(F("Adafruit Bluefruit Low Energy nRF8001 Print echo demo"));
  #endif
  BTLEserial.setRXcallback(rxCallback);
  BTLEserial.setACIcallback(aciCallback);
  BTLEserial.begin();
  BTLEserial.setDeviceName(deviceName);
}

void loop()
{
  // here is where you'd put code that needs to be running all the time.
  
  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();
  checkGPSButtonState();
  checkActivityButtonState();
  checkGPSLedState();
  checkActivityLedState();
}
