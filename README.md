# esphome-vs0153
An minimal ESPHome component for the VS1053 Audio Codec.

## Features
- Playback of file data stored in memory
- Playback cancellation
- Volume control
- Hardware or software reset


## Configuration
Below is a configuration snippet for the Adafruit ESP32 V2 Feather and Adafruit Music Maker FeatherWing.
```yaml
external_components:
  - source:
      type: local
      path: /vs1053/components
    components: [ vs1053 ]

spi:
  clk_pin: GPIO5
  mosi_pin: GPIO19
  miso_pin: GPIO21

vs1053:
  id: player
  # reset_pin: GPIO22 # Optional
  dreq_pin: GPIO15
  sci_spi:
    cs_pin: GPIO32
  sdi_spi:
    cs_pin: GPIO33
```
