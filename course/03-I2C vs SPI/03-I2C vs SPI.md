# I2C vs SPI

## Communication Type

| Feature | I2C | SPI |
|-----------|-----------|-----------|
| Type   | Multi-master, multi-slave | Single-master or multi-slave |
| Connections | Two-wire (SDA, SCL) | Four-wire (MOSI, MISO, SCLK, CS) |

- I2C uses fewer pins, making it more suitable for simpler designs with |multiple devices on the same bus.
- SPI requires more pins but offers higher performance.

## Speed

|Feature | I2C | SPI |
|-----------|-----------|-----------|
|Typical Speed | Up to 400 kHz (standard mode), 3.4 MHz (high-speed mode) | Up to 80 MHz (ESP32 supports high clock speeds) |

- I2C is slower, making it suitable for low-speed peripherals like sensors.
- SPI is significantly faster, ideal for high-speed peripherals like displays or memory chips.

## Data Transfer
|Feature | I2C | SPI
|-----------|-----------|-----------|
|Data Transfer | Half-duplex | Full-duplex |

- I2C is half-duplex, meaning data transfer occurs in one direction at a time.
- SPI supports full-duplex communication, enabling simultaneous data transfer in both directions.

## Device Addressing

| Feature | I2C | SPI |
|-----------|-----------|-----------|
|Device Selection | Uses 7-bit or 10-bit addresses | Uses chip select (CS) lines |

- I2C relies on device addresses for communication, so fewer pins are required.
- SPI uses dedicated CS lines, requiring one CS pin per device.

## Power Consumption
| Feature | I2C | SPI |
|-----------|-----------|-----------|
| Power Usage | Lower (less active circuitry) | Higher (higher clock rates) |

- I2C typically consumes less power due to lower speeds.
- SPI may consume more power at high speeds but allows more efficient data transfer.

## Complexity

| Feature | I2C | SPI |
|-----------|-----------|-----------|
| Complexity | Simpler to wire, but requires software handling for addressing and bus arbitration | More pins and wiring complexity but straightforward data transfer protocol |

## Use Cases
| Feature | I2C | SPI |
|-----------|-----------|-----------|
| Best For | Low-speed devices like sensors, RTCs, small EEPROMs | High-speed devices like LCDs, flash memory, ADCs, DACs |

## Summary Table
| Feature | I2C | SPI |
|-----------|-----------|-----------|
| Pins Required | 2 | 4 (minimum) |
| Speed | Slower | Faster |
| Duplex | Half-duplex | Full-duplex |
| Device Count | Many devices (addressable) | Limited by CS pins |
| Best For | Low-speed peripherals | High-speed peripherals |
