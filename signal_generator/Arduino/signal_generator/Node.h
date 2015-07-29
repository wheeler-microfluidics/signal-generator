#ifndef ___NODE__H___
#define ___NODE__H___

#include <stdint.h>
#include <Arduino.h>
#include <NadaMQ.h>
#include <BaseNodeRpc.h>
#include <BaseNodeEeprom.h>
#include <BaseNodeI2c.h>
#include <BaseNodeConfig.h>
#include <BaseNodeState.h>
#include <BaseNodeSerialHandler.h>
#include <BaseNodeI2cHandler.h>
#include <Array.h>
#include <I2cHandler.h>
#include <SerialHandler.h>
#include <pb_validate.h>
#include <pb_eeprom.h>
#include "SignalGenerator.h"
#include "signal_generator_config_validate.h"
#include "signal_generator_state_validate.h"
#include <SignalGenerator/config_pb.h>


namespace signal_generator {
const size_t FRAME_SIZE = (3 * sizeof(uint8_t)  // Frame boundary
                           - sizeof(uint16_t)  // UUID
                           - sizeof(uint16_t)  // Payload length
                           - sizeof(uint16_t));  // CRC

class Node;

typedef nanopb::EepromMessage<signal_generator_Config,
                              config_validate::Validator<Node> > config_t;
typedef nanopb::Message<signal_generator_State,
                        state_validate::Validator<Node> > state_t;

class Node :
  public BaseNode,
  public BaseNodeEeprom,
  public BaseNodeI2c,
  public BaseNodeConfig<config_t>,
  public BaseNodeState<state_t>,
#ifndef DISABLE_SERIAL
  public BaseNodeSerialHandler,
#endif  // #ifndef DISABLE_SERIAL
  public BaseNodeI2cHandler<base_node_rpc::i2c_handler_t> {
public:
  typedef PacketParser<FixedPacket> parser_t;

  static const uint16_t BUFFER_SIZE = 128;  // >= longest property string

  uint8_t buffer_[BUFFER_SIZE];

  Node() : BaseNode(), BaseNodeConfig<config_t>(signal_generator_Config_fields),
           BaseNodeState<state_t>(signal_generator_State_fields) {}

  UInt8Array get_buffer() { return UInt8Array(sizeof(buffer_), buffer_); }
  /* This is a required method to provide a temporary buffer to the
   * `BaseNode...` classes. */

  void begin();
  void set_i2c_address(uint8_t value);  // Override to validate i2c address

  bool on_state_voltage_changed(float new_value) {
    /* This method is triggered whenever a voltage is included in a state
     * update. */
    if ((0 <= new_value) && (new_value <= config_._.max_waveform_voltage)) {
      apply_waveform_voltage(new_value);
      return true;
    } else {
      return false;
    }
  }

  bool on_state_frequency_changed(float new_value) {
    /* This method is triggered whenever a frequency is included in a state
     * update. */
    if ((config_._.min_waveform_frequency <= new_value) &&
        (new_value <= config_._.max_waveform_frequency)) {
      apply_waveform_frequency(new_value);
      return true;
    } else {
      return false;
    }
  }

  void set_pots();  // Initialize potentiometer settings from config
  void reset_pots();  // Reset potentiometer settings to defaults
  float vout_pk_pk();
  void apply_waveform_voltage(float vrms);
  bool on_config_hf_amplitude_correction_changed(float correction) {
    config_._.hf_amplitude_correction = correction;
    /* Reset the frequency to update amplitude based on the new correction
     * factor. */
    apply_waveform_frequency(state_._.frequency);
    return true;
  }

  void set_pot(uint8_t index, uint8_t value);
  void apply_waveform_frequency(float freq);
  float max_waveform_voltage() const { return config_._.max_waveform_voltage; }
  float min_waveform_frequency() const { return config_._.min_waveform_frequency; }
  float max_waveform_frequency() const { return config_._.max_waveform_frequency; }
  uint8_t get_pot(uint8_t index) const { return config_._.pot[index]; }
  uint16_t get_voltage_lookup_index(float vrms) const {
    return round(vrms / config_._.max_waveform_voltage * 1023);
  }
  uint8_t get_resistor_value(uint8_t R_index, uint16_t index) const {
    switch (R_index) {
      case 1:
        return pgm_read_byte_near(SignalGenerator.R1_INDEX + index);
      case 2:
        return pgm_read_byte_near(SignalGenerator.R2_INDEX + index);
      case 4:
        return pgm_read_byte_near(SignalGenerator.R4_INDEX + index);
      case 5:
        return pgm_read_byte_near(SignalGenerator.R5_INDEX + index);
      default:
        return 0;
    }
  }
};

}  // namespace signal_generator


#endif  // #ifndef ___NODE__H___
