/* LED strip lighting controller v 0.0.5
   Copyright (C) 2016 by Prajwal Bhattaram
   Created by Prajwal Bhattaram - 29/08/2016
   Updated by Prajwal Bhattaram - 17/02/2018

   This file is part of the LED Strip Lighting Controller repository. This code is for
   controlling LED strips for use as home lighting. This version of the
   code takes user input from a copper tape based membrane LED keypad from Adafruit 
   - https://www.adafruit.com/product/1333 - (refer to schematics
   & BOM) and turns on or off an LED strip connected through a MOSFET. The
   brightness is automatically set through an LDR or a phototransistor

   This code is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This code is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License v3.0
   along with the LED strip lighting controller code.  If not, see
   <http://www.gnu.org/licenses/>.
*/

#include <LowPower.h>
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ (Defines)~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
//#define DEBUG   // Only uncomment this if you need to debug
#define BUTTON_PORT PORTD
#define FET_PORT PORTD
#define BUTTON_DDR DDRD
#define FET_DDR DDRD
#define BUTTONPIN PIND
#define BUTTONLEFT PD2
#define BUTTONRIGHT PD3
#define BUTTONLLED PD4
#define BUTTONRLED PD7
#define DEBOUNCE_TIME 25000    // In microseconds
#define MINLIGHT 50
#define MAXLIGHT 255
#define _LEFT 5   // This is the pin number that the MOSFET for the left strip is connected to
#define _RIGHT 6  // This is the pin number that the MOSFET for the right strip is connected to
#define _BOTH (_LEFT + _RIGHT)
#define _NONE 0
#define FADE true
#define NOFADE false

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ (Pins)~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

const uint8_t LDR = A0;
const uint8_t buttonPins = 0b00001100;
const uint8_t fetPins = 0b01100000;
const uint8_t rButtonLED = 0b10000000; // This is the pin number that the R button's LED is connected to
const uint8_t lButtonLED = 0b00010000;  // This is the pin number that the L button's LED is connected to
const uint8_t buttonLEDPins = 0b10010000;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ (Function defines)~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

#define TURNOFFLIGHTS FET_PORT &= ~(fetPins)
#define TURNONBUTTONLEDS BUTTON_PORT |= buttonLEDPins
#define TURNOFFBUTTONLEDS BUTTON_PORT &= ~(buttonLEDPins)
#define lButtonLEDOff BUTTON_PORT &= ~(lButtonLED)
#define lButtonLEDOn BUTTON_PORT |= lButtonLED
#define rButtonLEDOff BUTTON_PORT &= ~(rButtonLED)
#define rButtonLEDOn BUTTON_PORT |= rButtonLED

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ (Variables)~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
uint8_t prevLightLevel;
struct _state {
  uint8_t left;
  uint8_t right;
};
_state currentButton;
_state prevButton;
int8_t lightStatus;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ (Functions)~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
//                                                     Debounce code for interrupt function                                                     //
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

bool debounce(uint8_t inputreg, uint8_t inputpin) {
  if (bit_is_clear(inputreg, inputpin)) {
    _delay_us(DEBOUNCE_TIME);
    if (bit_is_clear(inputreg, inputpin)) {
      return true;
    }
  }
  return false;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
//                                      Interrupt functions: These functions run when the interrupt is called                                   //
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

void buttonLeft() {
  cli();
  if (debounce(BUTTONPIN, BUTTONLEFT)) {
    currentButton.left = !currentButton.left;
  }
  //Serial.println("Left button clicked ");
  sei();
}

void buttonRight() {
  cli();
  if (debounce(BUTTONPIN, BUTTONRIGHT)) {
    currentButton.right = !currentButton.right;
  }
  //Serial.println("Right button clicked ");
  sei();
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
//                                                    Sets the light level based on the LDR reading                                             //
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

uint8_t lightLevel() {
  uint8_t lightLvl = constrain(map(analogRead(LDR), 0, 255, MINLIGHT, MAXLIGHT), MINLIGHT, MAXLIGHT);
  return lightLvl;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
//                                                              Turns on the light strip                                                        //
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

void lightsOn(uint8_t lightStrip, bool _fade) {

  if (lightStatus == lightStrip) {
    lightStatus = lightStrip;
  }
  else {
    lightStatus += lightStrip;
  }

  prevLightLevel = lightLevel();
  
  if (lightStrip == _LEFT) {
      lButtonLEDOff;
    }
    else if (lightStrip == _RIGHT) {
      rButtonLEDOff;
    }

  if ((lightStrip == _LEFT) || (lightStrip == _RIGHT)) {
    if (lightStatus != _BOTH) {
      if (_fade) {
        for (uint8_t i = 0; i < prevLightLevel; i++) {
          analogWrite(lightStrip, i);
          _delay_ms(20);
        }
      }
      else {
        analogWrite(lightStrip, prevLightLevel);
      }
    }
    else if (lightStatus == _BOTH) {
      if (_fade) {
        for (uint8_t i = 0; i < prevLightLevel; i++) {
          analogWrite(lightStrip, i);
          analogWrite((lightStatus - lightStrip), prevLightLevel);
          _delay_ms(20);
        }
      }
      else {
        analogWrite(lightStrip, prevLightLevel);
        analogWrite((lightStatus - lightStrip), prevLightLevel);
      }
    }
    lightStrip = lightStatus;
  }
  if (lightStrip == _BOTH) {
    TURNOFFBUTTONLEDS;
    analogWrite(_LEFT, prevLightLevel);
    analogWrite(_RIGHT, prevLightLevel);
  }
  
  //Serial.print("Lights on. LightStatus: ");
  //Serial.print(lightStatus);
  //Serial.print(", lightStrip: ");
  //Serial.println(lightStrip);
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
//                                                              Turns off the light strip                                                       //
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

void lightsOff(uint8_t lightStrip) {

  switch (lightStatus) {

    case _BOTH:
      lightStatus -= lightStrip;
      if (lightStatus == _LEFT) {
        rButtonLEDOn;
        for (uint8_t i = prevLightLevel; i <= 1; i--) {
          analogWrite(_RIGHT, i);
          _delay_ms(50);
        }
        digitalWrite(_RIGHT, LOW);
      }
      else if (lightStatus == _RIGHT) {
        lButtonLEDOn;
        for (uint8_t i = prevLightLevel; i <= 1; i--) {
          analogWrite(_LEFT, i);
          _delay_ms(50);
        }
        digitalWrite(_LEFT, LOW);
      }
      break;

    case _NONE:
      lightStatus = _BOTH;
      break;

    default:
      TURNONBUTTONLEDS;
      lightStatus = _NONE; for (uint8_t i = prevLightLevel; i <= 1; i--) {
        analogWrite(_LEFT, i);
        analogWrite(_RIGHT, i);
        _delay_ms(50);
      }
      digitalWrite(_LEFT, LOW);
      digitalWrite(_RIGHT, LOW);
      break;
  }
  //Serial.print("Lights off. LightStatus: ");
  //Serial.print(lightStatus);
  //Serial.print(", lightStrip: ");
  //Serial.println(lightStrip);
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ (Arduino Code)~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

void setup() {
  //Serial.begin(115200);
  delay(1000);
  BUTTON_DDR &= ~(buttonPins); // Sets the button pins to input
  BUTTON_PORT |= buttonPins; // Enables the pull-up resistor
  BUTTON_DDR |= buttonLEDPins; // Sets button LED pins to output
  TURNONBUTTONLEDS; // Turns on the buttons' built-in LEDs to enable easy use
  FET_DDR |= fetPins; //Sets MOSFET pins to output
  TURNOFFLIGHTS; // Turns off MOSFET pins
  currentButton.left = false;
  currentButton.right = false;
  lightStatus = _NONE;

  attachInterrupt(digitalPinToInterrupt(BUTTONLEFT), buttonLeft, LOW);   // Attach interrupt to left button
  attachInterrupt(digitalPinToInterrupt(BUTTONRIGHT), buttonRight, LOW);  // Attach interrupt to right button
}

void loop() {
  if (prevButton.left != currentButton.left) {
  //Serial.println("Left button changed ");
    if (( lightStatus == _NONE) || (lightStatus == _RIGHT)) {
      (lightsOn(_LEFT, FADE));
    }
    else {
      (lightsOff(_LEFT));
    }
    prevButton.left = currentButton.left;
  }

  if (prevButton.right != currentButton.right) {
  //Serial.println("Right button changed ");
    if ((lightStatus == _RIGHT) || (lightStatus == _BOTH)) {
      (lightsOff(_RIGHT));
    }

    else if ((lightStatus == _NONE) || (lightStatus == _LEFT)) {
      (lightsOn(_RIGHT, FADE));
    }
    prevButton.right = currentButton.right;
  }

  switch (lightStatus) {
    case _LEFT:
      lightsOn(_LEFT, NOFADE);
      break;

    case _RIGHT:
      lightsOn(_RIGHT, NOFADE);
      break;

    case _BOTH:
      lightsOn(_BOTH, NOFADE);
      break;

    case _NONE:
      lightsOn(_NONE, NOFADE);
      break;
  }

  if (lightStatus == _NONE) {
    if ((prevButton.left == currentButton.left) || (prevButton.right == currentButton.right)) {
      
      LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    }
  }
}
