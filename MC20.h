/* Arduino Quectel MC20 Library.
 * See the repository README.md file for author and licensing information.
 *
 * This library is for interfacing to a Quectel MC20 GSM+BT+GNSS module.
 * See the example sketches to learn how to use the library in your code.
 *
 * This is the main include file for the library.
 * ----------------------------------------------------------------------------
 * The header of the original file follows:
 * 
 * MC20_Common.h
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

#ifndef _MC20_H_INCLUDED
#define _MC20_H_INCLUDED

#include <stdbool.h>

#include <Stream.h>
#if defined(ARDUINO) && ARDUINO >= 100
# include <Arduino.h>
#else
# include <WProgram.h>
#endif

#define MC20_VBAT_PIN_WIO 7
#define MC20_VBAT_PIN_HW -1
#define MC20_PKEY_PIN_WIO 13

class MC20
{
    public:
        /* Description:
         *   This is the constructor, it creates a new MC20 instance.
         * Parameters:
         *   serialPort - reference to Stream, serial port implementation to
         *                access the MC20 through. This is assumed to be already
         *                configured, opened and ready by the time MC20::begin()
         *                is called.
         *   vbatPin - pin through which to control (turn on and off) power to
         *             the MC20. This is assumed to be active HIGH. Pass
         *             MC20_VBAT_PIN_HW to signal that the MC20 is always
         *             powered in your application.
         *   powerKeyPin - pin which controls the PWRKEY input of the MC20 to
         *                 turn it on.
         */
        MC20(Stream &serialPort, int8_t vbatPin=MC20_VBAT_PIN_WIO, int8_t
             powerKeyPin=MC20_PKEY_PIN_WIO):
             port(serialPort), vbat_pin(vbatPin), pkey_pin(powerKeyPin),
             established(false) {};

        /*
         * Description:
         *   This is the destructor, it simply calls end().
         */
        ~MC20() { end(); };

        /*
         * Description:
         *   Attempts to power on and establish communication with the MC20.
         *   Returns true on success and false otherwise. If communication is
         *   successfully established, it configures the MC20 for use with this
         *   library (i.e. echo off etc.).
         * Parameters:
         *   goOnAir - after establishing communication, tell the MC20 to power
         *             on its GSM radio. Depending on whether a SIM card is
         *             installed, whether it's PIN-locked or not and various
         *             other SIM- and operator-side settings, this may result in
         *             the MC20 registering with the GSM network on power on.
         */
        bool begin(bool goOnAir);

        /*
         * Description:
         *   Tells the MC20 to sign off from the GSM network, then powers it off
         *   if possible. Note that this will also disable any BT and GNSS
         *   functionality that may have previously been enabled.
         */
        void end(void);
    private:
        Stream &port;
        int8_t vbat_pin, pkey_pin;
        bool established;

        /*
         * Description:
         *   Sends the given command string to the MC20 automatically adding the
         *   EOL character (which triggers the command's execution) and blocking
         *   until the entire command line has been sent.
         */
        void sendCommand(const String &command);
        void sendCommand(const char *command);
        void sendCommand(const __FlashStringHelper *command);
};
#endif
