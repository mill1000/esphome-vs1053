
#include "vs1053.h"

#include "esphome/core/log.h"
#include "vs1053_reg.h"

namespace esphome {
namespace vs1053 {

static const char *TAG = "VS1053";

void VS1053Component::setup() {
  // Setup pins
  this->reset_pin_->setup();

  // Initialize SPI devices
  this->sci_spi_->spi_setup();
  this->sdi_spi_->spi_setup();

  // Reset and configure device
  this->init_();

  // Read status register
  uint16_t status = this->command_read_(SCI_REG_STATUS);

  // Validate device version
  uint8_t version = (status >> 4) & 0x0F;
  if (version != VS1053_VERSION) {
    ESP_LOGE(TAG, "Initialization failed. SS_VER %d != %d", version, VS1053_VERSION);
    this->mark_failed();  // Mark the component as failed
    return;
  }
}

void VS1053Component::loop() {
}

void VS1053Component::dump_config() {
  ESP_LOGCONFIG(TAG, "VS1053:");
  LOG_PIN("  RESET Pin: ", this->reset_pin_);
  LOG_PIN("  DREQ Pin: ", this->dreq_pin_);
  // TODO LOG SPI devices?
  // ESP_LOGCONFIG(TAG, "  SCI SPI ", this->address_);
}

void VS1053Component::set_volume(uint8_t left, uint8_t right) {
  // Convert incoming volume to attenuation
  uint8_t convert = [](uint8_t v) {
    return 0xFF - v;
  };

  // Register sets attenuation in 0.5 dB steps
  // Maximum volume 0x0000, minimum 0xFEFE, analog powerdown 0xFFFF
  uint16_t vol = convert(left) << 8 | convert(right);
  this->command_write_(SCI_REG_VOLUME, vol);
}

void VS1053Component::play_test_sine(uint16_t ms, uint32_t freq_hz, uint32_t sample_rate_hz) {
  // Re-init device
  this->init_();

  // // Enable test modes -> Old sine test
  // uint16_t mode = this->command_read_(SCI_REG_MODE);
  // mode |= MODE_SM_TESTS;
  // this->command_write_(SCI_REG_MODE);

  // New sine test
  // AICTRLn = Fsin X 65536 / Fs
  // AICTRLn max value 0x8000
  // AICTRLn 3 LSB should be zero for best SNR

  // Set sample rate TODO??
  this->command_write_(SCI_REG_AUDATA, sample_rate_hz & 0xFFFE);

  // Configure sine frequency
  uint16_t aictrl = std::min((freq_hz * 65536) / sample_rate_hz, 0x8000) & ~0x0007;
  this->command_write_(SCI_REG_AICTRL0, aictrl);  // Left channel
  this->command_write_(SCI_REG_AICTRL1, aictrl);  // Right channel

  this->command_write_(SCI_REG_AIADDR, 0x4020);  // Start test
  delay(ms);                                     // TODO Units?
  this->command_write_(SCI_REG_AIADDR, 0);       // TODO Stop test? Nothing is docs

  // this->soft_reset_(); // TODO?
}

void VS1053Component::init_() {
  // Hard reset device via pin
  this->hard_reset();
  delay(100);  // TODO Delays in ms? This is quite long? // TODO datasheet says 1.8 ms @ 12.288 MHz

  // Soft reset via registers
  this->soft_reset_();
  delay(100);

  // Configure clocks
  // CLKI = 3x XTALI, XTALI = 12.288 MHz
  // TOOD data sheet recommends 3.5x 0x9800
  this->command_write_(SCI_REG_CLOCKF, 0x6000);

  // Set minimum volume
  // TODO or set analog power down?
  this->set_volume_(1, 1);
}

void VS1053Component::hard_reset_() {
  this->reset_pin_->digital_write(false);
  delay(100);
  this->reset_pin_->digital_write(true);
}

void VS1053Component::soft_reset_() {
  this->command_write_(SCI_REG_MODE, MODE_SM_SDINEW | MODE_SM_RESET);
}

void VS1053Component::command_write_(uint8_t addr, uint16_t data) {
  // Assert XCS
  this->sci_spi_->enable();

  uint8_t buffer[4] = {
      SCI_CMD_WRITE,
      addr,
      data >> 8,
      data & 0xFF,
  };
  this->sci_spi_->write_array(buffer, sizeof(buffer));

  // Deassert XCS
  this->sci_spi->disable();
}

// TODO could probably combine read/write to a single function
uint16_t VS1053Component::command_read_(uint8_t addr) {
  // Assert XCS
  this->sci_spi_->enable();

  uint8_t buffer[4] = {
      SCI_CMD_READ,
      addr,
      0,
      0,
  };
  this->sci_spi_->transfer_array(buffer, sizeof(buffer));

  // Deassert XCS
  this->sci_spi->disable();

  return buffer[2] << 8 | buffer[3];
}

}  // namespace vs1053
}  // namespace esphome