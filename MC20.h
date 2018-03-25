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
#include <stdint.h>

#include <Stream.h>
#if defined(ARDUINO) && ARDUINO >= 100
# include <Arduino.h>
#else
# include <WProgram.h>
#endif

#define MC20_VBAT_PIN_WIO 7
#define MC20_VBAT_PIN_HW -1
#define MC20_PKEY_PIN_WIO 13
#define MC20_DTR_PIN_WIO 9
#define MC20_DTP_PIN_HW -1

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
         *   dtrPin - pin which controls the DTR input of the MC20, used in low
         *            power scenarios to control wake-up. Pass MC20_DTP_PIN_HW
         *            to signal that you don't want this functionality.
         */
        MC20(Stream &serialPort, int8_t vbatPin=MC20_VBAT_PIN_WIO, int8_t
             powerKeyPin=MC20_PKEY_PIN_WIO, int8_t dtrPin=MC20_DTR_PIN_WIO):
             port(serialPort), vbat_pin(vbatPin), pkey_pin(powerKeyPin),
             dtr_pin(dtrPin), established(false) {};

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
         * NOTE: this is NOT equivalent to a hardware RF-Kill switch! The modem
         *       has plenty time to perform RF operations between the moment
         *       it's powered on and until the command corresponding to the
         *       goOnAir value is sent.
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
        int8_t vbat_pin, pkey_pin, dtr_pin;
        bool established;
        char lastLine[256];
        enum RecvRawLineStatus {
            MC20_RRL_COMPLETE,
            MC20_RRL_EMPTY,
            MC20_RRL_PARTIAL,
            MC20_RRL_NOTHING,
            MC20_RRL_OVERFLOW
        };

        /*
         * Description:
         *   Sends the given command string to the MC20 automatically adding the
         *   EOL character (which triggers the command's execution) and blocking
         *   until the entire command line has been sent.
         */
        /* This is Arduino String, not std::string! */
        void sendCommand(const String &command);
        void sendCommand(const char *command);
        void sendCommand(const __FlashStringHelper *command);

        /*
         * Description:
         *   Sends the command execution character and flushes the port.
         */
        void sendEOL(void);

        /*
         * Description:
         *   Sends challenge and then looks for response alone on the first line
         *   of input, optionally preceeded by an echo of challenge and/or URCs.
         *   Returns true if response was found and false otherwise. If the
         *   input is initially empty, it obeys the timeout set with
         *   Stream::setTimeout() on the port. If response was not found as
         *   described anove or the port's timeout was hit, returns false.
         */
        bool challengeResponse(const char *challenge, const char *response);
        bool challengeResponse(const __FlashStringHelper *challenge,
                               const char *response);
        bool challengeResponse(const char *challenge,
                               const __FlashStringHelper *response);
        bool challengeResponse(const __FlashStringHelper *challenge,
                               const __FlashStringHelper *response);

        /*
         * Description:
         *   Reads characters from the input and stores in MC20::lastLine until
         *   "\r\n" is seen. The final "\r\n" is not stored in MC20::lastLine.
         *   If the input is initially empty, it obeys the timeout set with
         *   Stream::setTimeout() on the port. Regardless of the outcome,
         *   MC20::lastLine is properly maintained as a C string.
         * Returns:
         *   MC20::MC20_RRL_COMPLETE if a complete line was found and stored in
         *                           MC20::lastLine.
         *   MC20::MC20_RRL_EMPTY if only a "\r\n" was read.
         *   MC20::MC20_RRL_PARTIAL if at least one character (different from
         *                          '\r') was read within the port's timeout.
         *                          Call the function again with a true argument
         *                          to (attempt to) finish the line.
         *   MC20::MC20_RRL_NOTHING if no characters arrived within the port's
         *                          timeout.
         *   MC20::MC20_RRL_OVERFLOW if available (remaining) space in
         *                           this->lastLine is insufficient to buffer
         *                           the entire line the MC20 sent.
         * NOTE: calling MC20::recvRawLine() a second time with a true argument
         *       and getting a MC20::MC20_RRL_NOTHING return is a good sign the
         *       MC20 is stuck and not coming back without a power cycle.
         */
        MC20::RecvRawLineStatus recvRawLine(bool retry=false);

        /*
         * Description:
         *   Reads complete lines from the input, filtering out (and acting
         *   upon) any URCs encountered. Saves the first non-URC line
         *   encountered in MC20::lastLine and returns true. If no usable
         *   (non-empty and non-URC) line was read within the port's timeout,
         *   returns false.
         */
        bool recvFilteredLine(void);

        /*
         * Description:
         *   Checks MC20::lastLine to see if it's an URC, acts upon it if so and
         *   returns true. Returns false if MC20::lastLine was not an URC.
         */
        bool maybeProcessURC(void);

        /*
         * Description:
         *   Serves as comparison function for bsearch() for PROGMEM-stored
         *   arrays of strings.
         */
        static int flashStringCompare(const void *key, const void *candidate);
};
#endif
