/* Arduino Quectel MC20 Library.
 * See the repository README.md file for author and licensing information.
 *
 * This library is for interfacing to a Quectel MC20 GSM+BT+GNSS module.
 * See the example sketches to learn how to use the library in your code.
 *
 * This is the main code file for the library.
 * ----------------------------------------------------------------------------
 * The header of the original file follows:
 * 
 * MC20_Common.cpp
 * A library for SeeedStudio Wio Tracker
 *
 * Copyright (c) 2017 seeed technology inc.
 * Website    : www.seeed.cc
 * Author     : lawliet zou, lambor
 * Create Time: April 2017
 * Change Log :
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "MC20.h"

#define MC20_CR 0x0D

bool MC20::begin(bool goOnAir) {
    /* If the Arduino API wouldn't be the stinky mess it is, there would be an
     * interface that both SoftwareSerial and HardwareSerial implement and so
     * MC20::_port could be of that type and we could write the elegant code
     * below to test for availability. But it is and so we cannot.
     */
//   if(!this->_port) {
//       /* Serial port implementations on Arduino override bool() to signal
//        * whether their initialization is complete or not.*/
//       return false;
//   }
    if(this->vbat_pin != MC20_VBAT_PIN_HW) {
        pinMode(this->vbat_pin, OUTPUT);
    }
    pinMode(this->pkey_pin, OUTPUT);
    if(this->vbat_pin != MC20_VBAT_PIN_HW) {
        digitalWrite(this->vbat_pin, HIGH);
        /* Datasheet asks for 100ms between VBAT rising and PWRKEY going low. */
        delay(100);
    }
    digitalWrite(this->pkey_pin, HIGH);
    /* Datasheet is unclear on both what is the needed time for sampling PWRKEY
     * on power on, as well as the maximum boot time. It however gives 700ms as
     * the sampling time for PWRKEY on power off, so we use that for power on
     * as well and then we poll the device until it replies with OK.
     */
    delay(700);
    digitalWrite(this->pkey_pin, LOW);

    while(!this->established) {
        this->sendCommand(F("AT"));
    }
    // Somewhere at the end:
    this->established = true;
    return true;
}

void MC20::end(void) {
}

void MC20::sendCommand(const String &command) {
    this->sendCommand(command.c_str());
}

void MC20::sendCommand(const char *command) {
    if(!(this->established && command)) {
        return;
    }
    (void) this->port.write(command);
    (void) this->port.write((uint8_t)MC20_CR);
    this->port.flush();
}

void MC20::sendCommand(const __FlashStringHelper *command) {
}
