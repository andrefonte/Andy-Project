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
 
 Turns on and off a light emitting diode(LED) connected to a digital  
 pin, without using the delay() function.  This means that other code
 can run at the same time without being interrupted by the LED code.
 
 The circuit:
 * LED attached from pin 13 to ground.
 * Note: on most Arduinos, there is already an LED on the board
 that's attached to pin 13, so no hardware is needed for this example.
 
 
 created 2005
 by David A. Mellis
 modified 8 Feb 2010
 by Paul Stoffregen
 
 This example code is in the public domain.

 
 http://www.arduino.cc/en/Tutorial/BlinkWithoutDelay
 */

// constants won't change. Used here to 
// set pin numbers:
const int ledPin =  13;      // the number of the LED pin
const int buttonPin = 2;     // the number of the pushbutton pin

// Variables will change:
int buttonState = 0;         // variable for reading the pushbutton status 
int ledState = LOW;             // ledState used to set the LED
long previousMillis = 0;        // will store last time LED was updated

// the follow variables is a long because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long interval = 1000;           // interval at which to blink (milliseconds)

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
/**************************************************************************/
/*!
Configure the Arduino and start advertising with the radio
*/
/**************************************************************************/ 

void setup() {
  // set the digital pin as output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT); 
  
  #if defined(DEVMODE)
    Serial.begin(9600);
    Serial.println(F("Adafruit Bluefruit Low Energy nRF8001 Print echo demo"));
  #endif
  BTLEserial.begin();
}

void blinkLED()
{
  // check to see if it's time to blink the LED; that is, if the 
  // difference between the current time and last time you blinked 
  // the LED is bigger than the interval at which you want to 
  // blink the LED.
  unsigned long currentMillis = millis();
 
  if(currentMillis - previousMillis > interval) {
    // save the last time you blinked the LED 
    previousMillis = currentMillis;   

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW)
      ledState = HIGH;
    else
      ledState = LOW;

    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);
  }
}

void checkButtonState()
{
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);
  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    // turn LED on:
    digitalWrite(ledPin, HIGH);
  }
  else {
    // turn LED off:
    digitalWrite(ledPin, LOW);
  }
}

/**************************************************************************/
/*!    Constantly checks for new events on the nRF8001 */
/**************************************************************************/

aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

void updateBLEStatus()
{
  // Ask what is our current status
   aci_evt_opcode_t status = BTLEserial.getState();
   // If the status changed....
   if (status != laststatus) {
     
     #if defined(DEVMODE)
       // print it out!
       if (status == ACI_EVT_DEVICE_STARTED) {
         Serial.println(F("* Advertising started"));
       }
       if (status == ACI_EVT_CONNECTED) {
         Serial.println(F("* Connected!"));
       }
       if (status == ACI_EVT_DISCONNECTED) {
         Serial.println(F("* Disconnected or advertising timed out"));
       }
     #endif
   
     // OK set the last status change to this one
     laststatus = status;
   }
}

void loop()
{
  // here is where you'd put code that needs to be running all the time.
  
  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();
  updateBLEStatus();
  checkButtonState();
  blinkLED();
}
