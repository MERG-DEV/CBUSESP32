
//
///
//

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

#include <WiFi.h>
#include <Streaming.h>

// CBUS library header files
#include <CBUSESP32.h>              // CAN controller and CBUS class
#include <CBUSswitch.h>             // pushbutton switch
#include <CBUSLED.h>                // CBUS LEDs
#include <CBUSconfig.h>             // module configuration
#include <CBUSParams.h>             // module parameters
#include <cbusdefs.h>               // MERG CBUS constants

// local header
#include "defs.h"

// CBUS objects
CBUSConfig myconfig;                // configuration object
CBUSESP32 CBUS(&myconfig);          // CBUS object
CBUSLED ledGrn, ledYlw;             // two LED objects
CBUSSwitch pb_switch;               // switch object

// module name
unsigned char mname[7] = { 'E', 'M', 'P', 'T', 'Y', ' ', ' ' };

// forward function declarations
void eventhandler(byte index, byte opc);
void framehandler(CANFrame *msg);

//
/// setup - runs once at power on
//

void setup() {

  Serial.begin (115200);
  Serial << "> ** CBUS empty test module v1 **" << endl;

  // set myconfig layout parameters
  myconfig.EE_NVS_START = 10;
  myconfig.EE_NUM_NVS = 10;
  myconfig.EE_EVENTS_START = 50;
  myconfig.EE_MAX_EVENTS = 64;
  myconfig.EE_NUM_EVS = 1;
  myconfig.EE_BYTES_PER_EVENT = (myconfig.EE_NUM_EVS + 4);

  // initialise and load myconfiguration
  myconfig.setEEPROMtype(EEPROM_INTERNAL);
  myconfig.begin();

  Serial << "> mode = " << (myconfig.FLiM ? "FLiM" : "SLiM") << "> NN = " << myconfig.nodeNum << endl;

  CBUSParams params(myconfig);
  params.setVersion(VER_MAJ, VER_MIN, VER_BETA);
  params.setModuleId(MODULE_ID);
  params.setFlags(PF_FLiM | PF_COMBI);

  // assign to CBUS
  CBUS.setParams(params.getParams());
  CBUS.setName(mname);

  // initialise CBUS LEDs
  ledGrn.setPin(LED_GRN);
  ledYlw.setPin(LED_YLW);

  // initialise CBUS switch
  pb_switch.setPin(SWITCH0, LOW);

  // module reset - if switch is depressed at startup and module is in SLiM mode
  pb_switch.run();

  if (pb_switch.isPressed() && !myconfig.FLiM) {
    Serial << "> switch was pressed at startup in SLiM mode" << endl;
    myconfig.resetModule(ledGrn, ledYlw, pb_switch);
  }

  // register our CBUS event handler, to receive event messages of learned accessory events
  CBUS.setEventHandler(eventhandler);

  // register our CAN frame handler, to receive *every* CAN frame
  CBUS.setFrameHandler(framehandler);

  // assign switch and LEDs to CBUS
  CBUS.setLEDs(ledGrn, ledYlw);
  CBUS.setSwitch(pb_switch);

  // set CBUS LEDs to indicate the current mode
  CBUS.indicateMode(myconfig.FLiM);

  // start CAN bus and CBUS message processing
  CBUS.setPins(16, 17);
  CBUS.setNumBuffers(64, 64);

  if (!CBUS.begin()) {
    Serial << "> error starting CBUS" << endl;
  }

  // end of setup
  Serial << "> ready" << endl;
}

//
/// loop - runs forever
//

void loop() {

  //
  /// do CBUS message, switch and LED processing
  //

  CBUS.process();

}

//
/// user-defined event processing function
/// called from the CBUS library when a learned event is received
/// it receives the event table index and the CAN frame
//

void eventhandler(byte index, CANFrame *msg) {

  // as an example, display the opcode and the first EV of this event

  Serial << "> event handler: index = " << index << ", opcode = 0x" << _HEX(msg->data[0]) << endl;
  return;
}

//
/// user-defined frame processing function
/// called from the CBUS library for selected CAN frame received
/// it receives a pointer to the received CAN frame
//

void framehandler(CANFrame *msg) {

  // as an example, format and display the received frame
  char fbuff[40], dbuff[8];

  sprintf(fbuff, "[%03lu] [%d] [ ", (msg->id & 0x7f), msg->len);

  for (byte d = 0; d < msg->len; d++) {
    sprintf(dbuff, "%02x ", msg->data[d]);
    strcat(fbuff, dbuff);
  }

  strcat(fbuff, "]");
  Serial << fbuff << endl;
  return;
}
