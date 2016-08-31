/* LED strip lighting controller v 0.0.3
   Copyright (C) 2016 by Prajwal Bhattaram
   Modified by Prajwal Bhattaram - 29/08/2016

   This file is part of the LED Strip Lighting Controller. This code is for
   controlling LED strips for use as home lighting. This version of the
   code takes user input from a copper tape based capacitive touch button (refer to
   schematics & BOM) and turms on or off an LED strip connected through a MOSFET. The
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
//#define DEBUG
#define BUTTON_PORT PORTD
#define FET_PORT PORTD
#define BUTTON_DDR DDRD
#define FET_DDR DDRD
#define MINLIGHT 50
#define MAXLIGHT 255
#define _LEFT 5   // This is the pin number that the MOSFET for the left strip is connected to
#define _RIGHT 6  // This is the pin number that the MOSFET for the right strip is connected to
#define FADE true
#define NOFADE false

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ (Pins)~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

const uint8_t LDR = A0;
const uint8_t buttonL = 2;
const uint8_t buttonR = 3;
//const uint8_t LFET = 5;
//const uint8_t RFET = 6;
const uint8_t buttonPins = 0x06;
const uint8_t fetPins = 0x60;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ (Variables)~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

uint8_t buttonPush, _lightLevel;
struct _state {
  uint8_t left;
  uint8_t right;
};
_state currentButton;
_state lightStatus;
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ (Debug control)~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
#ifdef DEBUG
#define debug Serial
#endif

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ (Functions)~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

void buttonLeft() {
  cli();
#ifdef DEBUG
  debug.println("Interrupt 0 turned off");
#endif
  buttonPush = _LEFT;
#ifdef DEBUG
  debug.println("Left button pushed");
#endif
}

void buttonRight() {
  cli();
#ifdef DEBUG
  debug.println("Interrupt 1 turned off");
#endif
  buttonPush = _RIGHT;
#ifdef DEBUG
  debug.println("Right button pushed");
#endif
}

uint8_t lightLevel() {
  //uint16_t lightLvl = analogRead(LDR);
  uint8_t lightLvl = constrain(map(analogRead(LDR), 0, 255, MINLIGHT, MAXLIGHT), MINLIGHT, MAXLIGHT);
  //lightLvl = constrain(lightLvl, MINLIGHT, MAXLIGHT);
#ifdef DEBUG
  debug.print("Analog read LDR: ");
  debug.println(analogRead(LDR));
  debug.print("Light level set to: ");
  debug.println(lightLvl);
#endif
  return lightLvl;
}

void lightsOn(uint8_t _lightstrip, bool _fade) {
#ifdef DEBUG
  debug.print("Turning on ");
  (_lightstrip == _LEFT) ? (debug.print("left")) : (_lightstrip == _RIGHT) ? (debug.print("right")) : (debug.print("unknown"));
  debug.println(" light strip");
#endif

  _lightLevel = lightLevel();
  if (_fade) {
    for (uint8_t i = 0; i < _lightLevel; i++) {
      analogWrite(_lightstrip, i);
      _delay_ms(20);
    }
  }
  analogWrite(_lightstrip, _lightLevel);
  (_lightstrip == _LEFT) ? (lightStatus.left = true) : (lightStatus.right = true);
}

void lightsOff(uint8_t _lightstrip) {
#ifdef DEBUG
  debug.print("Turning off ");
  (_lightstrip == _LEFT) ? (debug.print("left")) : (_lightstrip == _RIGHT) ? (debug.print("right")) : (debug.print("unknown"));
  debug.println(" light strip");
#endif

  for (uint8_t i = _lightLevel; i > 0; i--) {
    analogWrite(_lightstrip, i);
    _delay_ms(20);
  }
  digitalWrite(_lightstrip, LOW);
  (_lightstrip == _LEFT) ? (lightStatus.left = false) : (lightStatus.right = false);
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ (Arduino Code)~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

void setup() {
#ifdef DEBUG
  debug.begin(115200);
  debug.println("Initialising sensor");
#endif
  BUTTON_DDR &= ~(buttonPins);
  BUTTON_PORT |= buttonPins;
  attachInterrupt(digitalPinToInterrupt(buttonL), buttonLeft, LOW);
  attachInterrupt(digitalPinToInterrupt(buttonR), buttonRight, LOW);

  currentButton.left = false;
  currentButton.right = false;
  lightStatus.left = false;
  lightStatus.right = false;

  FET_DDR |= fetPins;
  FET_PORT &= ~(fetPins);
}

void loop() {
  uint8_t _status;
  uint32_t _time;

  if (buttonPush) {
    switch (buttonPush) {
      case _LEFT:
        currentButton.left = !currentButton.left;
#ifdef DEBUG
        debug.print("Left button: ");
        (currentButton.left) ? (debug.println("on")) : (debug.println("off"));
#endif
        (currentButton.left) ? (lightsOn(_LEFT, FADE)) : (lightsOff(_LEFT));
        buttonPush = 0;
        sei();
#ifdef DEBUG
        debug.println("Interrupts turned on");
#endif
        break;

      case _RIGHT:
        currentButton.right = !currentButton.right;
#ifdef DEBUG
        debug.print("Right button: ");
        (currentButton.right) ? (debug.println("on")) : (debug.println("off"));
#endif
        (currentButton.right) ? (lightsOn(_RIGHT, FADE)) : (lightsOff(_RIGHT));
        buttonPush = 0;
        sei();
#ifdef DEBUG
        debug.println("Interrupts turned on");
#endif
        break;

      default:
        buttonPush = 0;
        break;
    }
  }
  if (lightStatus.left) {
    if (lightLevel() != _lightLevel) {
      lightsOn(_LEFT, NOFADE);
    }
  }

  else if (lightStatus.right) {
    if (lightLevel() != _lightLevel) {
      lightsOn(_RIGHT, NOFADE);
    }
  }

  else {
    #ifdef DEBUG
    debug.println("Going to sleep.....");
    delay(30);
    #endif
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  }
}

