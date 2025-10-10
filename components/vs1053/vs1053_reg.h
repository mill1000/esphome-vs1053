#pragma once

#include <stdint.h>

namespace esphome {
namespace vs1053 {

static constexpr const uint8_t SCI_CMD_READ = 0x03;   // Serial command read
static constexpr const uint8_t SCI_CMD_WRITE = 0x02;  // Serial command write

static constexpr const uint8_t SCI_REG_MODE = 0x00;        // Mode control
static constexpr const uint8_t SCI_REG_STATUS = 0x01;      // Status of VS1053b
static constexpr const uint8_t SCI_REG_BASS = 0x02;        // Built-in bass/treble control
static constexpr const uint8_t SCI_REG_CLOCKF = 0x03;      // Clock frequency + multiplier
static constexpr const uint8_t SCI_REG_DECODETIME = 0x04;  // Decode time in seconds
static constexpr const uint8_t SCI_REG_AUDATA = 0x05;      // Misc. audio data
static constexpr const uint8_t SCI_REG_WRAM = 0x06;        // RAM write/read
static constexpr const uint8_t SCI_REG_WRAMADDR = 0x07;    // Base address for RAM write/read
static constexpr const uint8_t SCI_REG_HDAT0 = 0x08;       // Stream header data 0
static constexpr const uint8_t SCI_REG_HDAT1 = 0x09;       // Stream header data 1
static constexpr const uint8_t SCI_REG_AIADDR = 0x0A;      // Start address of application
static constexpr const uint8_t SCI_REG_VOLUME = 0x0B;      // Volume control
static constexpr const uint8_t SCI_REG_AICTRL0 = 0x0C;     // Application control register 0
static constexpr const uint8_t SCI_REG_AICTRL1 = 0x0D;     // Application control register 1
static constexpr const uint8_t SCI_REG_AICTRL2 = 0x0E;     // Application control register 2
static constexpr const uint8_t SCI_REG_AICTRL3 = 0x0F;     // Application control register 3

static constexpr const uint8_t GPIO_DDR = 0xC017;    // Direction
static constexpr const uint8_t GPIO_IDATA = 0xC018;  // Values read from pins
static constexpr const uint8_t GPIO_ODATA = 0xC019;  // Values set to the pins

static constexpr const uint8_t INT_ENABLE = 0xC01A;  // Interrupt enable

static constexpr const uint8_t MODE_SM_DIFF = 0x0001;      // Differential, 0: normal in-phase audio, 1: left channel inverted
static constexpr const uint8_t MODE_SM_LAYER12 = 0x0002;   // Allow MPEG layers I & II
static constexpr const uint8_t MODE_SM_RESET = 0x0004;     // Soft reset
static constexpr const uint8_t MODE_SM_CANCEL = 0x0008;    // Cancel decoding current file
static constexpr const uint8_t MODE_SM_EARSPKLO = 0x0010;  // EarSpeaker low setting
static constexpr const uint8_t MODE_SM_TESTS = 0x0020;     // Allow SDI tests
static constexpr const uint8_t MODE_SM_STREAM = 0x0040;    // Stream mode
static constexpr const uint8_t MODE_SM_SDINEW = 0x0800;    // VS1002 native SPI modes
static constexpr const uint8_t MODE_SM_ADPCM = 0x1000;     // PCM/ADPCM recording active
static constexpr const uint8_t MODE_SM_LINE1 = 0x4000;     // MIC/LINE1 selector, 0: MICP, 1: LINE1
static constexpr const uint8_t MODE_SM_CLKRANGE = 0x8000;  // Input clock range, 0: 12..13 MHz, 1: 24..26 MHz


}  // namespace vs1053
}  // namespace esphome