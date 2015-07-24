#include "Node.h"

namespace signal_generator {

void Node::begin() {
  config_.set_buffer(get_buffer());
  config_.validator_.set_node(*this);
  config_.reset();
  config_.load();
  state_.set_buffer(get_buffer());
  state_.validator_.set_node(*this);
  state_._.voltage = 0;
  state_._.frequency = config_._.min_waveform_frequency;
  state_._.has_voltage = true;
  state_._.has_frequency = true;
  // Start Serial after loading config to set baud rate.
#if !defined(DISABLE_SERIAL)
  //Serial.begin(config_._.baud_rate);
  Serial.begin(115200);
#endif  // #ifndef DISABLE_SERIAL
  // Set i2c clock-rate to 400kHz.
  TWBR = 12;
  SignalGenerator.begin();

  pinMode(config_._.ltc6903_ss_pin, OUTPUT);
  pinMode(config_._.ad5206_ss_pin, OUTPUT);
  pinMode(config_._.freq_range_pin, OUTPUT);
  pinMode(config_._.spi_sck_pin, OUTPUT);
  pinMode(config_._.spi_mosi_pin, OUTPUT);

  set_pots();  // Initialize potentiometer settings from config
  on_state_voltage_changed(state_._.voltage);
  on_state_frequency_changed(state_._.frequency);
}


void Node::set_i2c_address(uint8_t value) {
  // Validator expects `uint32_t` by reference.
  uint32_t address = value;
  /* Validate address and update the active `Wire` configuration if the
    * address is valid. */
  config_.validator_.i2c_address_(address, config_._.i2c_address);
  config_._.i2c_address = address;
}


void Node::set_pots() {
  if (config_._.pot_count < sizeof(config_._.pot) / sizeof(uint32_t)) {
    reset_pots();
  }
  for(uint8_t i = 0; i < config_._.pot_count; i++) {
    set_pot(i, config_._.pot[i]);
  }
}


void Node::reset_pots() {
  config_._.pot_count = sizeof(config_._.pot) / sizeof(uint32_t);
  for(uint8_t i = 0; i < config_._.pot_count; i++) {
    set_pot(i, 128);
  }
}


// TODO: Refactor into `on_pot_changed`
void Node::set_pot(uint8_t index, uint8_t value) {
  if (config_._.pot_count < index + 1) {
    config_._.pot_count = index + 1;
    for(uint8_t i = 0; i < config_._.pot_count - 1; i++) {
      config_._.pot[i] = 128;
    }
  }
  config_._.pot[index] = value;
  // Set the SS pin low to select the chip.
  digitalWrite(config_._.ad5206_ss_pin, LOW);
  shiftOutFast(config_._.spi_mosi_pin, config_._.spi_sck_pin, MSBFIRST, index);
  shiftOutFast(config_._.spi_mosi_pin, config_._.spi_sck_pin, MSBFIRST, value);
  // Set the SS pin high to de-select the chip.
  digitalWrite(config_._.ad5206_ss_pin, HIGH);
}


void Node::apply_waveform_voltage(float vrms) {
  const uint16_t index = get_voltage_lookup_index(vrms);
  const uint8_t R4 = get_resistor_value(4, index);
  const uint8_t R5 = get_resistor_value(5, index);

  /* Only set pots if they have changed. */
  if(config_._.pot[4] != R4 || config_._.pot[5] != R5) {
    /* If pot 4 is increasing, set pot 5 first. */
    if(config_._.pot[4] < R4) {
      set_pot(5, R5);
      set_pot(4, R4);
    } else {
      set_pot(4, R4);
      set_pot(5, R5);
    }
  }
}


void Node::apply_waveform_frequency(float freq) {
  float scaling = 32;
  /* $frequency = 2 ^ oct * 2078 / (2 - dac / 1024)$ */
  uint8_t oct = 3.322 * log(freq * scaling / 1039) / log(10);
  uint16_t dac = round(2048 - (2078 * pow(2, 10 + oct)) / (freq * scaling));
  uint8_t cnf = 2; // CLK on, /CLK off
  // msb = OCT3 OCT2 OCT1 OCT0 DAC9 DAC8 DAC7 DAC6
  uint8_t msb = (oct << 4) | (dac >> 6);
  // lsb =  DAC5 DAC4 DAC3 DAC2 DAC1 DAC0 CNF1 CNF0
  uint8_t lsb = (dac << 2) | cnf;

  digitalWrite(config_._.ltc6903_ss_pin, LOW);
  shiftOutFast(config_._.spi_mosi_pin, config_._.spi_sck_pin, MSBFIRST, msb);
  shiftOutFast(config_._.spi_mosi_pin, config_._.spi_sck_pin, MSBFIRST, lsb);
  digitalWrite(config_._.ltc6903_ss_pin, HIGH);

  float FSF;
  float amplitude_correction;
  if(freq <= 1e3) {
    FSF = 1;
    amplitude_correction = 1.0;
    digitalWrite(config_._.freq_range_pin, HIGH);
  } else {
    FSF = 10;
    amplitude_correction = config_._.hf_amplitude_correction;
    digitalWrite(config_._.freq_range_pin, LOW);
  }

  uint8_t index = ((log10(freq) - log10(SignalGenerator.F_MIN * FSF)) /
                   SignalGenerator.LOG_F_STEP);
  set_pot(1, get_resistor_value(1, index));
  set_pot(2, get_resistor_value(2, index));

  const float R1 = ((float)get_resistor_value(1, index) /
              255.0 * SignalGenerator.R_MAX);
  const float R2 = ((float)get_resistor_value(2, index) /
              255.0 * SignalGenerator.R_MAX);
  float gain = 1 / sqrt(pow(1 - pow(2 * PI * freq / FSF, 2) * R1 * R2 *
                            SignalGenerator.C1 * SignalGenerator.C2, 2)
                        + pow(2 * PI * freq * SignalGenerator.C2 / FSF *
                              (R1 + R2), 2));
  set_pot(0, round(128 / gain * amplitude_correction));
}


float Node::vout_pk_pk() {
  uint16_t min_v = 1023;
  uint16_t max_v = 0;
  uint16_t v;

  for(int i = 0; i < 1000; i++) {
    v = analogRead(0);
    if(v < min_v) {
      min_v = v;
    }
    if(v > max_v) {
      max_v = v;
    }
  }
  float vout = (float)(max_v - min_v) * (5.0 / 1023.0);
  return vout;
}

}  // namespace signal_generator
