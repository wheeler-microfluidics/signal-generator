#ifndef _SIGNAL_GENERATOR_H_
#define _SIGNAL_GENERATOR_H_

#include "Arduino.h"
#include <stdint.h>

/* [`LTC6903`][1]: 1kHz – 68MHz Serial Port Programmable Oscillator.
 *
 * [1]: http://www.linear.com/product/LTC6903 */

// Arduino digital pin 6 (MOSI) to LTC6903 pin 2
// Arduino digital pin 5 (SCK) to LTC6903 pin 3
// Arduino digital pin 2 to LTC6903 pin 4

namespace signal_generator {

inline void shiftOutFast(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder,
                         uint8_t val) {
  uint8_t cnt;
  uint8_t bitData, bitNotData;
  uint8_t bitClock, bitNotClock;
  volatile uint8_t *outData;
  volatile uint8_t *outClock;

  outData = portOutputRegister(digitalPinToPort(dataPin));
  outClock = portOutputRegister(digitalPinToPort(clockPin));
  bitData = digitalPinToBitMask(dataPin);
  bitClock = digitalPinToBitMask(clockPin);

  bitNotClock = bitClock;
  bitNotClock ^= 0x0ff;

  bitNotData = bitData;
  bitNotData ^= 0x0ff;

  cnt = 8;
  if (bitOrder == LSBFIRST) {
    do {
      if ( val & 1 ) {
        *outData |= bitData;
      } else {
        *outData &= bitNotData;
      }

      *outClock |= bitClock;
      *outClock &= bitNotClock;
      val >>= 1;
      cnt--;
    } while( cnt != 0 );
  } else {
    do {
      if ( val & 128 ) {
        *outData |= bitData;
      } else {
        *outData &= bitNotData;
      }

      *outClock |= bitClock;
      *outClock &= bitNotClock;
      val <<= 1;
      cnt--;
    } while( cnt != 0 );
  }
}


class SignalGeneratorClass {
public:
  // reserved return codes
  static const int8_t RETURN_OK                   = 0;
  static const int8_t RETURN_GENERAL_ERROR        = -1;
  static const int8_t RETURN_TIMEOUT              = -2;
  static const int8_t RETURN_NOT_CONNECTED        = -3;
  static const int8_t RETURN_BAD_INDEX            = -4;
  static const int8_t RETURN_BAD_PACKET_SIZE      = -5;
  static const int8_t RETURN_BAD_CRC              = -6;
  static const int8_t RETURN_BAD_VALUE            = -7;
  static const int8_t RETURN_MAX_PAYLOAD_EXCEEDED = -8;

  static const uint16_t MAX_PAYLOAD_LENGTH = 48;
  static const uint32_t BAUD_RATE          = 115200;
  static const float F_MIN                 = 100.0;
  static const float F_MAX                 = 10e3;
  static const float R_MAX                 = 100e3;
  static const float C1                    = 47e-9;
  static const float C2                    = 6.8e-9;
  static const int POT_COUNT               = 6;
  static const float LOG_F_STEP;
  static const char R1_INDEX[] PROGMEM;
  static const char R2_INDEX[] PROGMEM;
  static const char R4_INDEX[] PROGMEM;
  static const char R5_INDEX[] PROGMEM;

  void begin() {}
};

}  // namespace signal_generator

extern signal_generator::SignalGeneratorClass SignalGenerator;

#endif  // #ifndef _SIGNAL_GENERATOR_H_
