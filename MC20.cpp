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

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include <avr/pgmspace.h>
// Bug in Arduino IDE: having avr-libc installed on the system overrides the
// one shipped with the IDE, which means you may end up linking against an
// ancient one.
#if !defined(pgm_read_ptr)
# if !defined(pgm_read_ptr_near)
#  define pgm_read_ptr_near(address_short) (void*)__LPM_word((uint16_t)(address_short))
# endif
# define pgm_read_ptr(address_short) pgm_read_ptr_near(address_short)
#endif

#include "MC20.h"

const char MC20_URC_AMO[] PROGMEM = "ALARM MODE";
const char MC20_URC_ARG[] PROGMEM = "ALARM RING";
const char MC20_URC_CRY[] PROGMEM = "Call Ready";
const char MC20_URC_MCN[] PROGMEM = "MO CONNECTED";
const char MC20_URC_MRN[] PROGMEM = "MO RING";
const char MC20_URC_NPD[] PROGMEM = "NORMAL POWER DOWN";
const char MC20_URC_OVO[] PROGMEM = "OVER_VOLTAGE POWER DOWN";
const char MC20_URC_OVW[] PROGMEM = "OVER_VOLTAGE WARNING";
const char MC20_URC_RDY[] PROGMEM = "RDY";
const char MC20_URC_RNG[] PROGMEM = "RING";
const char MC20_URC_SRY[] PROGMEM = "SMS Ready";
const char MC20_URC_UVO[] PROGMEM = "UNDER_VOLTAGE POWER DOWN";
const char MC20_URC_UVW[] PROGMEM = "UNDER_VOLTAGE WARNING";

/* Note that "CME ERROR" is not in this list because it's not an *U*RC. */
const char MC20_URC_P_CBCM[] PROGMEM = "CBCM";
const char MC20_URC_P_CBM[] PROGMEM = "CBM";
const char MC20_URC_P_CCINFO[] PROGMEM = "CCINFO";
const char MC20_URC_P_CCWA[] PROGMEM = "CCWA";
const char MC20_URC_P_CDS[] PROGMEM = "CDS";
const char MC20_URC_P_CFUN[] PROGMEM = "CFUN";
const char MC20_URC_P_CGEV[] PROGMEM = "CGEV";
const char MC20_URC_P_CGREG[] PROGMEM = "CGREG";
const char MC20_URC_P_CLIP[] PROGMEM = "CLIP";
const char MC20_URC_P_CMWT[] PROGMEM = "CMWT";
const char MC20_URC_P_CMT[] PROGMEM = "CMT";
const char MC20_URC_P_CMTI[] PROGMEM = "CMTI";
const char MC20_URC_P_COLP[] PROGMEM = "COLP";
const char MC20_URC_P_CPIN[] PROGMEM = "CPIN";
const char MC20_URC_P_CREG[] PROGMEM = "CREG";
const char MC20_URC_P_CRING[] PROGMEM = "CRING";
const char MC20_URC_P_CSQN[] PROGMEM = "CSQN";
const char MC20_URC_P_FPLMN[] PROGMEM = "FPLMN";
const char MC20_URC_P_QBAND[] PROGMEM = "QBAND";
const char MC20_URC_P_QCGTIND[] PROGMEM = "QCGTIND";
const char MC20_URC_P_QGURC[] PROGMEM = "QGURC";
const char MC20_URC_P_TSMSINFO[] PROGMEM = "TSMSINFO";

const char MC20_CT_ERROR[] PROGMEM = "ERROR";
const char MC20_CT_OK[] PROGMEM = "OK";
const char MC20_CT_P_CMEE[] PROGMEM = "CME ERROR";
const char MC20_CT_P_CMSE[] PROGMEM = "CMS ERROR";


/* Keep these two sorted in strcmp() order. It's easier to achieve that if
 * they're already listed in sorted order above.
 */
PGM_P const MC20_Simple_URCs[] PROGMEM = {
    MC20_URC_AMO,
    MC20_URC_ARG,
    MC20_URC_CRY,
    MC20_URC_MCN,
    MC20_URC_MRN,
    MC20_URC_NPD,
    MC20_URC_OVO,
    MC20_URC_OVW,
    MC20_URC_RDY,
    MC20_URC_RNG,
    MC20_URC_SRY,
    MC20_URC_UVO,
    MC20_URC_UVW
};

PGM_P const MC20_Plus_URCs[] PROGMEM = {
    MC20_URC_P_CBCM,
    MC20_URC_P_CBM,
    MC20_URC_P_CCINFO,
    MC20_URC_P_CCWA,
    MC20_URC_P_CDS,
    MC20_URC_P_CFUN,
    MC20_URC_P_CGEV,
    MC20_URC_P_CGREG,
    MC20_URC_P_CLIP,
    MC20_URC_P_CMWT,
    MC20_URC_P_CMT,
    MC20_URC_P_CMTI,
    MC20_URC_P_COLP,
    MC20_URC_P_CPIN,
    MC20_URC_P_CREG,
    MC20_URC_P_CRING,
    MC20_URC_P_CSQN,
    MC20_URC_P_FPLMN,
    MC20_URC_P_QBAND,
    MC20_URC_P_QCGTIND,
    MC20_URC_P_QGURC,
    MC20_URC_P_TSMSINFO
};

PGM_P const MC20_Simple_CmdTerms[] PROGMEM = {
    MC20_CT_ERROR,
    MC20_CT_OK
};

PGM_P const MC20_Plus_CmdTerms[] PROGMEM = {
    MC20_CT_P_CMEE,
    MC20_CT_P_CMSE
};

bool MC20::begin(bool goOnAir) {
    /* If the Arduino API wouldn't be the stinky mess it is, there would be an
     * interface that both SoftwareSerial and HardwareSerial implement and so
     * MC20::port could be of that type and we could write the elegant code
     * below to test for availability. But it is and so we cannot.
     */
//   if(!this->_port) {
//       /* Serial port implementations on Arduino override bool() to signal
//        * whether their initialization is complete or not.*/
//       return false;
//   }
    /* In general, we expect the modem to reply in less than 300ms, according to
     * the datasheet. Allowing three times that to account for stuff like
     * multiprocessing-related interrupts.
     */
    this->port.setTimeout(1000);
    /* Turn the modem power on. */
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

    while(!(this->established = this->challengeResponse("AT", "OK")));
//TODO: configure
//TODO: obey goOnAir
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
    this->sendEOL();
}

void MC20::sendCommand(const __FlashStringHelper *command) {
    /* command is a pointer to EEPROM so a value of NULL is actually valid. */
    if(!this->established) {
        return;
    }
    PGM_P cmd = reinterpret_cast<PGM_P>(command);
    while(uint8_t c = pgm_read_byte(cmd++)) {
        (void) this->port.write(c);
    }
    this->sendEOL();
}

void MC20::sendEOL(void) {
    /* Datasheet says CR alone will suffice. */
    (void) this->port.write((uint8_t)'\r');
    this->port.flush();
}

bool MC20::challengeResponse(const char *challenge, const char *response) {
    this->sendCommand(challenge);
    /* Possibly wait for and read the first line in. */
    if(!this->recvFilteredLine()) {
        /* Timeout ocurred, report failure. */
        return false;
    }
    if(!strcmp(this->lastLine, response)) {
        /* The first line read was the response, we're done. */
        return true;
    } else if(!strcmp(this->lastLine, challenge)) {
        /* The first line was the challenge echoed back, go on. */
        if(!this->recvFilteredLine()) {
            /* Timeout ocurred, report failure. */
            return false;
        }
        if(!strcmp(this->lastLine, response)) {
            return true;
        } else {
            return false;
        }
    } else {
        /* The first line was neither the challenge, nor the response. */
        return false;
    }
}

bool MC20::recvFilteredLine(void) {
    while(true) {
        MC20::RecvRawLineStatus result = this->recvRawLine();
        switch(result) {
            case MC20::MC20_RRL_COMPLETE:
                if(this->maybeProcessURC()) {
                    /* This was an URC, we need the next line. */
                    continue;
                } else {
                    return true;
                }
                break;
            case MC20::MC20_RRL_EMPTY:
                continue;
                break;
            case MC20::MC20_RRL_PARTIAL:
                if(this->recvRawLine(true) != MC20::MC20_RRL_COMPLETE) {
                    /* We cannot get a complete line in 2 attempts, bail. */
                    return false;
                }
                break;
            case MC20::MC20_RRL_NOTHING:
                /* Timeout ocurred, report failure. */
                return false;
                break;
        }
    }
}

MC20::RecvRawLineStatus MC20::recvRawLine(bool retry) {
    char *start = this->lastLine;
    size_t bufSize = sizeof(this->lastLine) / sizeof(this->lastLine[0]) - 1;
    size_t lastSize = strlen(this->lastLine);

    if(retry && lastSize) {
        start += lastSize;
        bufSize -= lastSize;
    }
    size_t lineLen = this->port.readBytesUntil('\r', start, bufSize);
    /* Stream::readBytesUntil() is not well-behaved in terms of C-strings. */
    start[lineLen] = '\0';
    /* If this was a well-formed line, the next character in the serial buffer
     * would have to be a '\n'.
     */
    if(this->port.peek() == '\n') {
        (void) this->port.read();
        if(lineLen) {
            /* At least one character was read before EOL. */
            return MC20::MC20_RRL_COMPLETE;
        } else {
            return MC20::MC20_RRL_EMPTY;
        }
    } else {
        if(this->port.peek() == -1) {
            if(lineLen) {
                /* Timeout ocurred mid-line. */
                return MC20::MC20_RRL_PARTIAL;
            } else {
                return MC20::MC20_RRL_NOTHING;
            }
        } else {
            /* We've exceeded our available buffer. */
            return MC20::MC20_RRL_OVERFLOW;
        }
    }
}

int MC20::flashStringCompare(const void *key, const void *candidate) {
    /* Extract candidate first ... */
    PGM_VOID_P candidateValue = pgm_read_ptr(candidate);
    /* ... then do the comparison. */
    return strcmp_P((const char *)key, (const char *)candidateValue);
}

bool MC20::maybeProcessURC(void) {
    PGM_VOID_P urcMatch;

    if(this->lastLine[0] == '+') {
        char *urcEnd = strchr(this->lastLine, ':');
        if(!urcEnd) {
            /* Begins with + but doesn't have a : down the line, not an URC. */
            return false;
        }
        urcEnd[0] = '\0';
        /* Note how having to have the strings declared and thus stored before
         * the arrays of pointers to them in EEPROM saves us from bsearch()
         * returning NULL to mean "the first element in the array matches".
         */
        urcMatch = bsearch(&lastLine[1], MC20_Plus_URCs,
                           sizeof(MC20_Plus_URCs) / sizeof(MC20_Plus_URCs[0]),
                           sizeof(MC20_Plus_URCs[0]), MC20::flashStringCompare);
        urcEnd[0] = ':';
    } else {
        urcMatch = bsearch(
            lastLine, MC20_Simple_URCs,
            sizeof(MC20_Simple_URCs) / sizeof(MC20_Simple_URCs[0]),
            sizeof(MC20_Simple_URCs[0]), MC20::flashStringCompare);
    }

    if(urcMatch) {
        //TODO: process URC.
        return true;
    } else {
        return false;
    }
}
