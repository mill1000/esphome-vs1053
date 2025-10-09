#pragma once

#include "esphome/components/spi/spi.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace vs1053 {

class VS1053 : public Component,
                public spi::SPIDevice<
                    // TODO
                    spi::BIT_ORDER_MSB_FIRST,
                    spi::CLOCK_POLARITY_LOW,
                    spi::CLOCK_PHASE_TRAILING,
                    spi::DATA_RATE_1MHZ> {
}

}  // namespace vs1053
}  // namespace esphome