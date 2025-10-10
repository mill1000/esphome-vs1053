#pragma once

#include "esphome/components/spi/spi.h"
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace vs1053 {

static constexpr const uint8_t VS1053_VERSION = 4;  // Per datasheet, VS1053/VS8053 SS_VER = 4

// VS1053 has two SPI interfaces, a command (SCI) and data (SDI) interface
// XCS is the command select, XDCS is the data select
// Both interfaces are Mode 0 MSB First
class VS1053_SCI_SPIDevice : spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW, spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_200KHZ> {};
class VS1053_SDI_SPIDevice : spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW, spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_8MHZ> {};

class VS1053Component : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_reset_pin(GPIOPin* pin) { this->reset_pin_ = pin; }
  void set_sci_device(VS1053_SCI_SPIDevice* spi) { this->sci_spi_ = spi; }
  void set_sdi_device(VS1053_SDI_SPIDevice* spi) { this->sdi_spi_ = spi; }

  void set_volume(uint9_t left, uint8_t right);
  void play_test_sine(uint16_t ms, uint32_t freq_hz = 1000, uint32_t sample_rate_hz = 44100);

 protected:
  GPIOPin* reset_pin_;
  VS1053_SCI_SPIDevice sci_spi_;
  VS1053_SDI_SPIDevice sdi_spi_;

  void init_(void);
  void hard_reset_(void);
  void soft_reset(void);

  void command_write_(uint8_t addr, uint16_t data);
  uint16_t command_read_(uint8_t addr);
};

}  // namespace vs1053
}  // namespace esphome
