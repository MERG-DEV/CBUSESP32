
/*

  Copyright (C) Duncan Greenwood 2017 (duncan_greenwood@hotmail.com)

  This work is licensed under the:
      Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   To view a copy of this license, visit:
      http://creativecommons.org/licenses/by-nc-sa/4.0/
   or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

   License summary:
    You are free to:
      Share, copy and redistribute the material in any medium or format
      Adapt, remix, transform, and build upon the material

    The licensor cannot revoke these freedoms as long as you follow the license terms.

    Attribution : You must give appropriate credit, provide a link to the license,
                  and indicate if changes were made. You may do so in any reasonable manner,
                  but not in any way that suggests the licensor endorses you or your use.

    NonCommercial : You may not use the material for commercial purposes. **(see note below)

    ShareAlike : If you remix, transform, or build upon the material, you must distribute
                 your contributions under the same license as the original.

    No additional restrictions : You may not apply legal terms or technological measures that
                                 legally restrict others from doing anything the license permits.

   ** For commercial use, please contact the original copyright holder(s) to agree licensing terms

    This software is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE

*/

#pragma once

// header files

#include <CBUS.h>               // abstract base class
#include <CBUSconfig.h>         // CBUS config class
#include <CBUSLED.h>            // CBUS LED class
#include <CBUSswitch.h>         // CBUS switch class

// constants

static const byte ESP32_TXPIN = 16;                          // CAN TX pin
static const byte ESP32_RXPIN = 17;                          // CAN RX pin
static const byte NUM_BUFFS = 20;                            // default value

//
/// an implementation of the abstract base CBUS class
/// to support the ESP32 CAN controller peripheral
//

class CBUSESP32 : public CBUS {

  public:

    CBUSESP32();

    // these methods are declared virtual in the base class and must be implemented by the derived class
    bool begin(void);
    bool available(void);
    CANFrame getNextMessage(void);
    bool sendMessage(CANFrame *msg, bool rtr = false, bool ext = false);    // note default arguments
    void reset(void);
    void setNumBuffers(byte num);
    void setPins(byte rxpin, byte rxPin);

    // these methods are specific to this implementation
    // they are not declared or implemented by the base CBUS class
    void printStatus(void);

private:
    byte _txPin, _rxPin;
    byte numbuffers;

};
