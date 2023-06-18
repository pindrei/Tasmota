/*
  xdrv_36_keeloq.ino - Jarolift Keeloq shutter support for Tasmota

  Copyright (C) 2021  he-so

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_RF_868
/*********************************************************************************************\
 * Keeloq shutter support
 *
 * Uses hardware SPI and two user configurable GPIO's (CC1101 GDO0 and CC1101 GDO2)
 *
 * Considering the implementation these two user GPIO's are fake.
 * Only CC1101 GDO0 is used and must always be GPIO05 dictated by the used CC1101 library.
\*********************************************************************************************/

#define XDRV_100 100


//#include "cc1101.h"
//#include <RadioLib.h>
#define USE_CC1101
#define PIN_RECEIVER_CS   15

// CC1101: GDO0 / RFM95W/SX127x: G0
#define PIN_RECEIVER_IRQ  4

// CC1101: GDO2 / RFM95W/SX127x: G1
#define PIN_RECEIVER_GPIO 5

// RFM95W/SX127x - GPIOxx / CC1101 - RADIOLIB_NC
#define PIN_RECEIVER_RST  2
#define NUM_SENSORS 1
//#include "WeatherSensorCfg.h"

#include "cc1101.h"
#define SYNC_WORD 199

#define Lowpulse         400
#define Highpulse        800

const char kJaroliftCommands[] PROGMEM = "Keeloq|" // prefix
  "SendRaw|SendButton|Set";

void (* const jaroliftCommand[])(void) PROGMEM = {
  &CmndSendRaw, &CmdSendButton, &CmdSet};

CC1101 cc1101;


struct JAROLIFT_DEVICE {
  int device_key_msb       = 0x0; // stores cryptkey MSB
  int device_key_lsb       = 0x0; // stores cryptkey LSB
  uint64_t button          = 0x0; // 1000=0x8 up, 0100=0x4 stop, 0010=0x2 down, 0001=0x1 learning
  int disc                 = 0x0100; // 0x0100 for single channel remote
  uint32_t enc             = 0x0;   // stores the 32Bit encrypted code
  uint64_t pack            = 0;   // Contains data to send.
  int count                = 0;
  uint32_t serial          = 0x0;
  int8_t port_tx;
  int8_t port_rx;
} jaroliftDevice;

void init_RF868(){
  jaroliftDevice.port_tx = Pin(GPIO_CC1101_GDO2);              // Output port for transmission
  jaroliftDevice.port_rx = Pin(GPIO_CC1101_GDO0);              // Input port for reception

  DEBUG_DRIVER_LOG(PSTR("RF868: cc1101.init()"));
  delay(100);
  cc1101.init();
  AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("RF868: CC1101 done"));
  cc1101.setSyncWord(SYNC_WORD, false);
  cc1101.setCarrierFreq(CFREQ_433);
  cc1101.disableAddressCheck();

  pinMode(jaroliftDevice.port_tx, OUTPUT);
  pinMode(jaroliftDevice.port_rx, INPUT_PULLUP);

  jaroliftDevice.serial = Settings->keeloq_serial;
  jaroliftDevice.count = Settings->keeloq_count;
  //GenerateDeviceCryptKey();
}
void loop_RF868(){
 
}
/*********************************************************************************************\
 * Interface
\*********************************************************************************************/
bool Xdrv100(uint32_t function)
{
  
  
 // if (!PinUsed(GPIO_CC1101_GDO0) || !PinUsed(GPIO_CC1101_GDO2)) { return false; }

  bool result = false;

  switch (function) {
    case FUNC_COMMAND:
      AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("RF868: calling command"));
      result = 0;// DecodeCommand(kJaroliftCommands, jaroliftCommand);
      break;
    case FUNC_INIT:
   //   KeeloqInit();
      
      init_RF868();
      
     
      break;
    case FUNC_WEB_SENSOR:
    #ifdef USE_WEBSERVER
       //   TTGO_WebShow(0);
    #endif
      break;
    case FUNC_JSON_APPEND:
     // TTGO_WebShow(1);
      break;
    case FUNC_LOOP:
    loop_RF868();
     // TTGO_loop(1);
      break;  
  }

  return result;
}

#endif  // USE_RF_868
