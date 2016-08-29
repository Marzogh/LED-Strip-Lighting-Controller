/* LED strip lighting controller v 0.0.1
 * Copyright (C) 2016 by Prajwal Bhattaram
 * Modified by Prajwal Bhattaram - 29/08/2016
 *
 * This file is part of the LED Strip Lighting Controller. This code is for
 * controlling LED strips for use as home lighting. This version of the 
 * code takes user input from a copper tape based capacitive touch button (refer to
 * schematics & BOM) and turms on or off an LED strip connected through a MOSFET. The
 * brightness is automatically set through an LDR or a phototransistor
 *
 * This code is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This code is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License v3.0
 * along with the LED strip lighting controller code.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ (Defines)~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
//#define DEBUG
#define BUTTON_PORT PORTD
#define BUTTON_DDR DDRD
#define MINLIGHT 10
#define MAXLIGHT 245
#define _LEFT 5   // This is the pin number that the MOSFET for the left strip is connected to
#define _RIGHT 6  // This is the pin number that the MOSFET for the right strip is connected to

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ (Pins)~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

const uint8_t LDR = A0;
const uint8_t buttonL = 2;
const uint8_t buttonR = 3;
//const uint8_t LFET = 5;
//const uint8_t RFET = 6;
const uint8_t buttonPins = 0x06;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ (Variables)~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

uint8_t buttonPush;
struct _state {
  uint8_t left;
  uint8_t right;
};
_state currentButton;
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ (Debug control)~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
#ifdef DEBUG
#define debug Serial
#endif

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ (Functions)~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

uint8_t lightLevel() {
  //uint16_t lightLvl = analogRead(LDR);
  uint8_t lightLvl = constrain(map(analogRead(LDR), 0, 1023, MINLIGHT, MAXLIGHT), MINLIGHT, MAXLIGHT);
  //lightLvl = constrain(lightLvl, MINLIGHT, MAXLIGHT);
#ifdef DEBUG
  debug.print("Light level set to: ");
  debug.println(lightLvl);
#endif
  return lightLvl;
}

void buttonLeft() {
  buttonPush = _LEFT;
#ifdef DEBUG
  debug.println("Left button pushed");
#endif
}

void buttonRight() {
  buttonPush = _RIGHT;
#ifdef DEBUG
  debug.println("Right button pushed");
#endif
}

void lightsOn(uint8_t _lightstrip) {
#ifdef DEBUG
  debug.print("Turning on ");
  (_lightstrip == _LEFT) ? (debug.print("left")) : (_lightstrip == _RIGHT) ? (debug.print("right")) :;
  debug.println(" light strip");
#endif

  uint8_t _lightLevel = lightLevel();
  for (uint8_t i = 0; i = _lightLevel; i++) {
    analogWrite(_lightstrip, _lightLevel);
    _delay_ms(2);
  }
}

void lightsOff(uint8_t _lightstrip) {
#ifdef DEBUG
  debug.print("Turning on ");
  (_lightstrip == _LEFT) ? (debug.print("left")) : (_lightstrip == _RIGHT) ? (debug.print("right")) :;
  debug.println(" light strip");
#endif

  uint8_t _lightLevel = lightLevel();
  for (uint8_t i = _lightLevel; i = 0; i--) {
    analogWrite(_lightstrip, _lightLevel);
    _delay_ms(2);
  }
  digitalWrite(_lightstrip, LOW);
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
}

void loop() {
  uint8_t _status;
  if (buttonPush) {
    switch (buttonPush) {
      case _LEFT:
        currentButton.left = !currentButton.left;
        (currentButton.left) ? (lightsOn(_LEFT)) : (lightsOff(_LEFT));
        buttonPush = 0;
        break;

      case _RIGHT:
        currentButton.right = !currentButton.right;
        (currentButton.right) ? (lightsOn(_RIGHT)) : (lightsOff(_RIGHT));
        buttonPush = 0;
        break;

      default:
        buttonPush = 0;
        break;
    }
  }
}

