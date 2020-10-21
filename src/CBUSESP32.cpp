
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


#include <WiFi.h>               // main ESP32 header
#include "driver/can.h"         // ESP32 CAN driver header

// 3rd party libraries
#include <Streaming.h>

// CBUS ESP32 library
#include <CBUSESP32.h>

// CBUS configuration object, declared externally
extern CBUSConfig config;

// buffered message from the RX queue
can_message_t buffered_message;
bool have_buffered_message = false;

//
/// constructor
//

CBUSESP32::CBUSESP32() {
  numbuffers = NUM_BUFFS;
  eventhandler = NULL;
  framehandler = NULL;
  _txPin = ESP32_TXPIN;
  _rxPin = ESP32_RXPIN;
}

//
/// initialise the CAN controller and buffers, and attach the ISR
//

bool CBUSESP32::begin(void) {

  _numMsgsSent = 0;
  _numMsgsRcvd = 0;

  // configure CAN bus driver
  can_general_config_t g_config;

  g_config.mode = CAN_MODE_NORMAL;
  g_config.tx_io = (gpio_num_t)_txPin;
  g_config.rx_io = (gpio_num_t)_rxPin;
  g_config.clkout_io = (gpio_num_t)CAN_IO_UNUSED;
  g_config.bus_off_io = (gpio_num_t)CAN_IO_UNUSED;
  g_config.tx_queue_len = numbuffers;
  g_config.rx_queue_len = numbuffers;
  g_config.alerts_enabled = CAN_ALERT_ALL;
  g_config.clkout_divider = 0;

  const can_timing_config_t t_config = CAN_TIMING_CONFIG_125KBITS();
  const can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();

  // install CAN driver
  esp_err_t iret = can_driver_install(&g_config, &t_config, &f_config);

  if (iret != ESP_OK) {
    Serial << F("Error installing CAN driver, ret = ") << iret << endl;
    return false;
  }

  // start CAN driver
  iret = can_start();

  if (iret != ESP_OK) {
    Serial << F("Error starting CAN driver, ret = ") << iret << endl;
    return false;
  }

  return true;
}

//
/// check for unprocessed messages in the receive queue
//

bool CBUSESP32::available(void) {

  /// the ESP32 CAN API has no check/peek function, so we need to receive the next message and buffer it

  can_message_t message;      // ESP32 CAN message type

  // buffered message still there from last check, so don't check again
  if (have_buffered_message) {
    return have_buffered_message;
  }

  // no buffered message, check the driver receive queue, with no blocking
  esp_err_t ret = can_receive(&message, (TickType_t)0);

  // got one, buffer it
  if (ret == ESP_OK) {
    memcpy(&buffered_message, &message, sizeof(can_message_t));
    have_buffered_message = true;
  } else {
    // queue is empty or receive error
    have_buffered_message = false;
  }

  return have_buffered_message;
}

//
/// get next unprocessed message from the buffer
//

CANFrame CBUSESP32::getNextMessage(void) {

  // there should always be a saved message, if available() has been called first as expected
  if (have_buffered_message) {
    _msg.id = buffered_message.identifier;
    _msg.len = buffered_message.data_length_code;
    _msg.rtr = buffered_message.flags & CAN_MSG_FLAG_RTR;
    _msg.ext = buffered_message.flags & CAN_MSG_FLAG_EXTD;
    memcpy(_msg.data, buffered_message.data, buffered_message.data_length_code);
    ++_numMsgsRcvd;
    have_buffered_message = false;  // we've taken it
  }

  // if available() hasn't been called and there is no buffered message, this returns an empty message
  // not ideal, but won't confuse the CBUS processing
  return _msg;
}

//
/// send a CBUS message
//

bool CBUSESP32::sendMessage(CANFrame *msg, bool rtr, bool ext) {

  // caller must populate the message data
  // this method will create the correct frame header (CAN ID and priority bits)
  // rtr and ext default to false unless arguments are supplied - see method definition in .h

  can_message_t message;                // ESP32 CAN message type

  makeHeader(msg);                      // set the CBUS header - CANID and priority bits
  message.identifier = msg->id;
  message.data_length_code = msg->len;
  
  if (rtr) {
    message.flags |= CAN_MSG_FLAG_RTR;
  }

  if (ext) {
    message.flags |= CAN_MSG_FLAG_EXTD;
  }

  memcpy(message.data, msg->data, msg->len);

  esp_err_t ret = can_transmit(&message, (TickType_t)0);

  if (ret == ESP_OK) {
    return true;
  } else {
    return false;
  }

}

//
/// display the CAN bus status instrumentation
//

void CBUSESP32::printStatus(void) {

  // !! todo
  return;
}

//
/// reset the CAN driver
//

void CBUSESP32::reset(void) {
  can_stop();
  can_driver_uninstall();
  begin();
}

//
/// set the TX and RX pins
//

void CBUSESP32::setPins(byte txPin, byte rxPin) {
  _txPin = txPin;
  _rxPin = rxPin;
}

//
/// set the depth of the TX and RX queues
//

void CBUSESP32::setNumBuffers(byte num) {
  numbuffers = num;
}
