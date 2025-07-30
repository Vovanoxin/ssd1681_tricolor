# SSD1681 Tricolor E-Paper Display Driver for ZMK

This is a standalone ZMK module that provides support for the SSD1681 tricolor e-paper display controller. The driver supports both monochrome and tricolor display modes.

## Features

- **Tricolor Support**: Full support for black, white, and red pixel display
- **Monochrome Compatibility**: Backward compatible with monochrome display modes
- **ZMK Integration**: Designed specifically for ZMK keyboard firmware
- **Partial Refresh**: Support for partial display updates to improve performance

## Installation

1. Add this module to your ZMK configuration:
   ```yaml
   # In your west.yml or as a git submodule
   ```

2. Enable the driver in your board configuration:
   ```kconfig
   CONFIG_SSD1681_TRICOLOR=y
   ```

3. Configure your device tree:
   ```dts
   &spi1 {
       status = "okay";
       // SPI configuration...
       
       epd: display@0 {
           compatible = "solomon,ssd1681";
           reg = <0>;
           width = <200>;
           height = <200>;
           dc-gpios = <&gpio1 2 GPIO_ACTIVE_LOW>;
           reset-gpios = <&gpio1 4 GPIO_ACTIVE_LOW>;
           busy-gpios = <&gpio1 10 GPIO_ACTIVE_HIGH>;
           spi-max-frequency = <2000000>;
       };
   }
   ```

## Usage

The driver automatically detects the pixel format and routes to the appropriate rendering function:
- `PIXEL_FORMAT_MONO10`: Monochrome mode
- `PIXEL_FORMAT_RGB_565`: Tricolor mode (uses specific RGB values for red/black detection)

## Tricolor Pixel Mapping

In tricolor mode (`PIXEL_FORMAT_RGB_565`):
- `0x0000` (Black RGB): Black pixel
- `0xF800` (Full Red RGB): Red pixel  
- Any other value: White pixel

## Changes Made

This driver is based on the Zephyr mainline `ssd16xx` driver but includes:
- Simplified focus on SSD1681 controllers
- Extended tricolor rendering capabilities
- ZMK-specific optimizations and configurations
- Proper memory management for dual-plane rendering

## License

Licensed under the [Apache 2.0 License](https://www.apache.org/licenses/LICENSE-2.0), maintaining compatibility with the original Zephyr driver.
