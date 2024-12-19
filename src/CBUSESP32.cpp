
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

// CBUS ESP32 library
#include <CBUSESP32.h>

// CBUS configuration object, declared externally
// extern CBUSConfig config;

// buffered message from the RX queue
twai_message_t buffered_message;
bool have_buffered_message = false;

void format_message(CANFrame *msg);

//
/// constructor
//

CBUSESP32::CBUSESP32() {
  _num_rx_buffers = NUM_BUFFS;
  _num_tx_buffers = NUM_BUFFS;
  _txPin = ESP32_TXPIN;
  _rxPin = ESP32_RXPIN;

  eventhandler = NULL;
  framehandler = NULL;
}

CBUSESP32::CBUSESP32(CBUSConfig *the_config) : CBUSbase(the_config) {
  _num_rx_buffers = NUM_BUFFS;
  _num_tx_buffers = NUM_BUFFS;
  _txPin = ESP32_TXPIN;
  _rxPin = ESP32_RXPIN;

  eventhandler = NULL;
  framehandler = NULL;
}

//
/// initialise the CAN controller and buffers, and attach the ISR
//

bool CBUSESP32::begin(bool poll, SPIClass spi) {

  esp_err_t iret;

  _numMsgsSent = 0;
  _numMsgsRcvd = 0;

  // configure CAN bus driver
  twai_general_config_t g_config;

  g_config.mode = TWAI_MODE_NORMAL;
  g_config.tx_io = (gpio_num_t)_txPin;
  g_config.rx_io = (gpio_num_t)_rxPin;
  g_config.clkout_io = (gpio_num_t)TWAI_IO_UNUSED;
  g_config.bus_off_io = (gpio_num_t)TWAI_IO_UNUSED;
  g_config.tx_queue_len = _num_tx_buffers;
  g_config.rx_queue_len = _num_rx_buffers;
  g_config.alerts_enabled = TWAI_ALERT_ALL;
  g_config.clkout_divider = 0;

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // install CAN driver
  iret = twai_driver_install(&g_config, &t_config, &f_config);

  if (iret != ESP_OK) {
    Serial << F("> Error installing TWAI driver, ret = ") << iret << endl;

    switch (iret) {
    case ESP_ERR_INVALID_ARG:
      Serial << F("> Arguments are invalid") << endl;
      break;
    case ESP_ERR_NO_MEM:
      Serial << F("> Insufficient memory") << endl;
      break;
    case ESP_ERR_INVALID_STATE:
      Serial << F("> Driver is already installed") << endl;
      break;
    default:
      Serial << F("> Unknown error") << endl;
      break;
    }

    return false;
  }

  // start CAN driver
  iret = twai_start();

  if (iret != ESP_OK) {
    Serial << F("> Error starting TWAI driver, ret = ") << iret << endl;

    switch (iret) {
    case ESP_ERR_INVALID_STATE:
      Serial << F("> Driver is not in running state, or is not installed") << endl;
      break;
    default:
      Serial << F("> Unknown error") << endl;
      break;
    }

    return false;
  }

  if (iret == ESP_OK) {
    Serial << F("> TWAI driver installed and started ok") << endl;
  }

  return true;
}

//
/// check for unprocessed messages in the receive queue
//

bool CBUSESP32::available(void) {

  /// the ESP32 CAN API has no check/peek function, so we need to receive the next message and buffer it

  twai_message_t message;      // ESP32 TWAI message type

  // buffered message still there from last check, so don't check again
  if (have_buffered_message) {
    return have_buffered_message;
  }

  // no buffered message, check the driver receive queue, with no blocking
  esp_err_t ret = twai_receive(&message, (TickType_t)0);

  // got one, buffer it
  if (ret == ESP_OK) {
    memcpy(&buffered_message, &message, sizeof(twai_message_t));
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
    _msg.rtr = buffered_message.flags & TWAI_MSG_FLAG_RTR;
    _msg.ext = buffered_message.flags & TWAI_MSG_FLAG_EXTD;
    memcpy(_msg.data, buffered_message.data, buffered_message.data_length_code);

    ++_numMsgsRcvd;
    have_buffered_message = false;  // we've taken it
  }

  // format_message(&_msg);

  // if available() hasn't been called and there is no buffered message, this returns an empty message
  // not ideal, but won't confuse the CBUS processing
  return _msg;
}

//
/// send a CBUS message
//

bool CBUSESP32::sendMessage(CANFrame *msg, bool rtr, bool ext, byte priority) {

  // caller must populate the message data
  // this method will create the correct frame header (CAN ID and priority bits)
  // rtr and ext default to false unless arguments are supplied - see method definition in .h

  bool ok;

  msg->rtr = rtr;
  msg->ext = ext;
  makeHeader(msg, priority);                      // set the CBUS header - CANID and priority bits

  ok = sendMessageNoUpdate(msg);

  // call user transmit handler
  if (transmithandler != nullptr) {
    (void)(*transmithandler)(msg);
  }

  return ok;
}

//
///
//

bool CBUSESP32::sendMessageNoUpdate(CANFrame *msg) {

  twai_message_t message;                // ESP32 CAN message type

  message.identifier = msg->id;
  message.data_length_code = msg->len;
  memcpy(message.data, msg->data, msg->len);

  if (msg->rtr) {
    message.flags |= TWAI_MSG_FLAG_RTR;
  }

  if (msg->ext) {
    message.flags |= TWAI_MSG_FLAG_EXTD;
  }

  esp_err_t ret = twai_transmit(&message, (TickType_t)0);

  if (ret == ESP_OK) {
    // Serial << F("> sent TWAI message ok") << endl;
    return true;
  } else {
    // Serial << F("> error sending TWAI message") << endl;
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
  twai_stop();
  twai_driver_uninstall();
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

void CBUSESP32::setNumBuffers(byte num_rx_buffers, byte num_tx_buffers) {
  _num_rx_buffers = num_rx_buffers;
  _num_tx_buffers = num_tx_buffers;
}

//
/// utility
//

void format_message(CANFrame *msg) {

  char mbuff[80], dbuff[8];

  sprintf(mbuff, "[%03d] [%d] [", (msg->id & 0x7f), msg->len);

  for (byte i = 0; i < msg->len; i++) {
    sprintf(dbuff, " %02x", msg->data[i]);
    strcat(mbuff, dbuff);
  }

  strcat(mbuff, " ]");
  Serial << F("> ") << mbuff << endl;

  return;
}
