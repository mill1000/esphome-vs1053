#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esphome/components/spi/spi.h"
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/hal.h"
#include "vs1053_reg.h"

namespace esphome {
namespace vs1053 {

static constexpr const uint32_t VS1053_MAX_LOOP_RUNTIME = 8000;  // 8 ms

static constexpr const size_t VS1053_FIFO_LENGTH = 2048;
static constexpr const size_t VS1053_TRANSFER_SIZE = 32;  // DREQ must be checked at least every 32 bytes

static constexpr const size_t VS1053_FILL_LENGTH = 2052;

static constexpr const uint8_t VS1053_VERSION = 4;  // Per datasheet, VS1053/VS8053 SS_VER = 4

// VS1053 has two SPI interfaces, a command (SCI) and data (SDI) interface
// XCS is the command select, XDCS is the data select
// Both interfaces are Mode 0 MSB First
class VS1053_SCI_SPIDevice : public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW, spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_200KHZ> {};
class VS1053_SDI_SPIDevice : public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW, spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_8MHZ> {};

class VS1053Component : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  // float get_setup_priority() const override { return setup_priority::DATA; }

  void set_reset_pin(GPIOPin* pin) { this->reset_pin_ = pin; }
  void set_dreq_pin(GPIOPin* pin) { this->dreq_pin_ = pin; }
  void set_sci_device(VS1053_SCI_SPIDevice* spi) { this->sci_spi_ = spi; }
  void set_sdi_device(VS1053_SDI_SPIDevice* spi) { this->sdi_spi_ = spi; }

  void set_volume(uint8_t left, uint8_t right);
  void play_test_sine(uint16_t ms, uint32_t freq_hz = 1000, uint32_t sample_rate_hz = 44100);
  void play_test_sine_sdi(uint16_t ms);
  void play_file();

 protected:
  GPIOPin* reset_pin_ = nullptr;
  GPIOPin* dreq_pin_ = nullptr;
  VS1053_SCI_SPIDevice* sci_spi_ = nullptr;
  VS1053_SDI_SPIDevice* sdi_spi_ = nullptr;

  enum class PlaybackState {
    Idle,
    Playing,
    Stop,
    StopFault,
  };

  PlaybackState state_ = PlaybackState::Idle;
  const uint8_t* buffer_ = nullptr;
  const uint8_t* buffer_end_ = nullptr;
  uint8_t fill_buffer_[VS1053_TRANSFER_SIZE];
  size_t fill_remaining_ = 0;

  bool init_(bool soft_reset = false);

  bool wait_data_ready_(uint32_t timeout_us);
  bool data_ready_() { return this->dreq_pin_->digital_read(); }

  void data_write_(const uint8_t* buffer, size_t length);
  uint16_t command_transfer_(uint8_t instruction, uint8_t addr, uint16_t data);
  void command_write_(uint8_t addr, uint16_t data) { this->command_transfer_(SCI_CMD_WRITE, addr, data); }
  uint16_t command_read_(uint8_t addr) { return this->command_transfer_(SCI_CMD_READ, addr, 0); }
};

}  // namespace vs1053
}  // namespace esphome
