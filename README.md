# esphome-vs0153
An minimal ESPHome component for the VS1053 Audio Codec.

## Features
- Playback of file data stored in memory
- Playback cancellation
- Volume control
- Hardware or software reset


## Configuration

```yaml
external_components:
  - source: github://mill1000/esphome-vs1053@main
    components: [vs1053]

vs1053:
  dreq_pin: GPIOX
  reset_pin: GPIOX # Optional HW reset
  sdi_spi:
    cs_pin: GPIOX
  sci_spi:
    cs_pin: GPIOX
```
