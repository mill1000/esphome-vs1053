
#include "vs1053.h"

#include "esphome/core/log.h"

namespace esphome {
namespace vs1053 {

static const char *TAG = "VS1053";

void VS1053Component::setup() {
  // Setup pins
  this->reset_pin_->setup();

  // Initialize SPI device
  this->spi_setup();

  // Hard reset device via pin
  this->hard_reset();
  delay(100); // TODO Delays in ms? This is quite long?

  // Soft reset via registers
  this->soft_reset_();
  delay(100);

  // TODO taken from SPI example
  // this->enable(); // TODO CS?
  // this->write_byte();
  // this->read_byte();

  // If failure
  // ESP_LOGE(TAG, "Initialization failed; response: %d", response);
  // this->mark_failed(); // Mark the component as failed
  // return;

  // Appears that VS1053 hs "two" SPI devices. A command bus and a data bus which must(can?) operate at different data rates?
  // Adafruit lib uses one to control DCS and the other to control CS
  // SCI 250 kHz
  // SDI 8 MHz

  // Adafruit reset
  //      // TODO:
  //   // http://www.vlsi.fi/player_vs1011_1002_1003/modularplayer/vs10xx_8c.html#a3
  //   // hardware reset
  //   if (_reset >= 0) {
  //     digitalWrite(_reset, LOW);
  //     delay(100);
  //     digitalWrite(_reset, HIGH);
  //   }

  //   delay(100);
  //   softReset();
  //   delay(100);

  //   sciWrite(VS1053_REG_CLOCKF, 0x6000);

  //   setVolume(40, 40);
}

void VS1053Component::loop() {
}

void VS1053Component::dump_config() {
  ESP_LOGCONFIG(TAG, "VS1053");
}

void VS1053Component::hard_reset_() {
  this->reset_pin_->digital_write(false);
  delay(100);
  this->reset_pin_->digital_write(true);
}

}  // namespace vs1053
}  // namespace esphome