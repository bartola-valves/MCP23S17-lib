# MCP23S17 C++ Library for Raspberry Pi Pico

A modern C++ library for interfacing with the MCP23S17 16-bit I/O Expander using SPI communication on the Raspberry Pi Pico with the Pico SDK.

## Features

- **Modern C++ API**: Clean, object-oriented interface with RAII principles
- **Multiple Device Support**: Support for up to 8 devices on the same SPI bus using hardware addressing
- **Comprehensive I/O Operations**: Individual pin control, port-level operations, and 16-bit operations
- **Hardware Features**: Pull-up resistors, input polarity inversion, configurable I/O directions
- **Built-in Patterns**: Running lights, binary counter, and other useful patterns
- **Error Handling**: Boolean return values for easy error checking
- **Pico SDK Integration**: Uses hardware SPI and GPIO libraries

## Hardware Setup

### Basic Connections

Connect the MCP23S17 to your Raspberry Pi Pico as follows:

| MCP23S17 Pin | Pico Pin | Function |
|--------------|----------|----------|
| VDD | 3.3V | Power |
| VSS | GND | Ground |
| SI (MOSI) | GPIO 19 | SPI Data In |
| SO (MISO) | GPIO 16 | SPI Data Out |
| SCK | GPIO 18 | SPI Clock |
| CS | GPIO 17 | Chip Select |
| A0, A1, A2 | GND or 3.3V | Device Address |
| RESET | 3.3V | Reset (active low) |

### Device Addressing

The MCP23S17 supports up to 8 devices on the same SPI bus using hardware addressing pins A0, A1, A2:

| A2 | A1 | A0 | Address |
|----|----|----|---------|
| 0  | 0  | 0  | 0x00 |
| 0  | 0  | 1  | 0x01 |
| 0  | 1  | 0  | 0x02 |
| ... | ... | ... | ... |
| 1  | 1  | 1  | 0x07 |

## Installation

1. Copy `mcp23s17.hpp` and `mcp23s17.cpp` to your project directory
2. Update your `CMakeLists.txt` to include the library:

```cmake
target_sources(your_project PRIVATE
    mcp23s17.cpp
    # your other source files
)

target_link_libraries(your_project
    pico_stdlib
    hardware_spi
    hardware_gpio
)
```

## Quick Start

### Basic Usage

```cpp
#include "mcp23s17.hpp"

int main() {
    stdio_init_all();
    
    // Initialize SPI
    spi_init(spi0, 1000000);  // 1 MHz
    gpio_set_function(16, GPIO_FUNC_SPI);  // MISO
    gpio_set_function(18, GPIO_FUNC_SPI);  // SCK  
    gpio_set_function(19, GPIO_FUNC_SPI);  // MOSI
    
    // Create and initialize MCP23S17
    MCP23S17 mcp(spi0, 17, 0);  // SPI, CS pin, address
    if (!mcp.begin()) {
        printf("Failed to initialize MCP23S17!\n");
        return -1;
    }
    
    // Configure ports
    mcp.setupLEDPort(MCP23S17_Port::A);      // Port A for LEDs
    mcp.setupButtonPort(MCP23S17_Port::B);   // Port B for buttons
    
    while (true) {
        // Blink LEDs on Port A
        for (int i = 0; i < 8; i++) {
            mcp.writePort(MCP23S17_Port::A, 1 << i);
            sleep_ms(200);
        }
    }
}
```

## API Reference

### Class Construction and Initialization

#### `MCP23S17(spi_inst_t* spi_inst, uint8_t cs_pin, uint8_t device_addr = 0)`
Create a new MCP23S17 instance.

#### `bool begin()`
Initialize the MCP23S17 chip. Returns `true` on success.

#### `bool isInitialized()`
Check if the device has been successfully initialized.

### Configuration Methods

#### `bool setupLEDPort(MCP23S17_Port port)`
Configure a port for LED output (all pins as outputs, pull-ups disabled).

#### `bool setupButtonPort(MCP23S17_Port port)`
Configure a port for button input (all pins as inputs with pull-ups enabled).

#### `bool setPortDirection(MCP23S17_Port port, uint8_t direction)`
Set I/O direction for an entire port (0x00 = all outputs, 0xFF = all inputs).

#### `bool setPinDirection(MCP23S17_Port port, uint8_t pin, bool is_input)`
Set I/O direction for an individual pin.

#### `bool setPortPullups(MCP23S17_Port port, uint8_t pullup_mask)`
Enable/disable internal pull-up resistors for a port.

#### `bool setPinPullup(MCP23S17_Port port, uint8_t pin, bool enable)`
Enable/disable pull-up for an individual pin.

### I/O Operations

#### Port-Level Operations
```cpp
uint8_t readPort(MCP23S17_Port port);
bool writePort(MCP23S17_Port port, uint8_t value);
```

#### Individual Pin Operations
```cpp
bool readPin(MCP23S17_Port port, uint8_t pin);
bool writePin(MCP23S17_Port port, uint8_t pin, bool state);
bool togglePin(MCP23S17_Port port, uint8_t pin);
```

#### 16-bit Operations
```cpp
uint16_t readGPIO();
bool writeGPIO(uint16_t value);
```

### Pattern Methods

#### `bool runningLights(MCP23S17_Port port, uint32_t delay_ms, int cycles = 1)`
Run a LED chase pattern on a port.

#### `bool binaryCounter(MCP23S17_Port port, uint32_t delay_ms, uint8_t max_count = 255)`
Run a binary counter pattern on a port.

### Register Access

#### `uint8_t readRegister(uint8_t reg)`
Read directly from a register.

#### `bool writeRegister(uint8_t reg, uint8_t value)`
Write directly to a register.

## Port and Pin Enumerations

### Ports
```cpp
enum class MCP23S17_Port {
    A = 0,  // Port A (GPIO0-GPIO7)
    B = 1   // Port B (GPIO8-GPIO15)
};
```

### Pin Constants
```cpp
enum class MCP23S17_Pin {
    PIN_0 = 0x01,
    PIN_1 = 0x02,
    // ... up to PIN_7 = 0x80
    PIN_ALL = 0xFF
};
```

## Examples

### LED Chaser
```cpp
// Configure Port A for LEDs
mcp.setupLEDPort(MCP23S17_Port::A);

// Run chase pattern
mcp.runningLights(MCP23S17_Port::A, 150, 3);  // 150ms delay, 3 cycles
```

### Input Reading
```cpp
// Configure Port B for buttons
mcp.setupButtonPort(MCP23S17_Port::B);

// Read button states
uint8_t buttons = mcp.readPort(MCP23S17_Port::B);
if (mcp.readPin(MCP23S17_Port::B, 0)) {
    printf("Button 0 pressed!\\n");
}
```

### Multiple Devices
```cpp
MCP23S17 device1(spi0, 17, 0);  // Address 0
MCP23S17 device2(spi0, 18, 1);  // Address 1

device1.begin();
device2.begin();

// Control them independently
device1.writePort(MCP23S17_Port::A, 0xAA);
device2.writePort(MCP23S17_Port::A, 0x55);
```

### 16-bit Operations
```cpp
// Configure both ports as outputs
mcp.setupLEDPort(MCP23S17_Port::A);
mcp.setupLEDPort(MCP23S17_Port::B);

// Write 16-bit patterns
uint16_t patterns[] = {0x0001, 0x0002, 0x0004, 0x0008};
for (auto pattern : patterns) {
    mcp.writeGPIO(pattern);
    sleep_ms(200);
}
```

## Advanced Usage

### Direct Register Access
```cpp
// Read IOCON register
uint8_t iocon = mcp.readRegister(MCP23S17_IOCON_A);

// Set custom configuration
mcp.writeRegister(MCP23S17_IOCON_A, 0x08);  // Enable hardware addressing
```

### Custom Pin Configurations
```cpp
// Configure specific pins
mcp.setPinDirection(MCP23S17_Port::A, 0, false);  // Pin 0 as output
mcp.setPinDirection(MCP23S17_Port::A, 1, true);   // Pin 1 as input
mcp.setPinPullup(MCP23S17_Port::A, 1, true);      // Enable pull-up on pin 1
```

## Error Handling

All methods return boolean values (`true` for success, `false` for failure) or appropriate data values. Always check return values for critical operations:

```cpp
if (!mcp.begin()) {
    printf("Failed to initialize MCP23S17!\\n");
    return -1;
}

if (!mcp.writePort(MCP23S17_Port::A, 0xFF)) {
    printf("Failed to write to port A!\\n");
}
```

## Register Map

The library defines all MCP23S17 registers when IOCON.BANK = 0:

| Register | Address | Description |
|----------|---------|-------------|
| IODIR_A/B | 0x00/0x01 | I/O Direction |
| IPOL_A/B | 0x02/0x03 | Input Polarity |
| GPINTEN_A/B | 0x04/0x05 | Interrupt Enable |
| DEFVAL_A/B | 0x06/0x07 | Default Compare |
| INTCON_A/B | 0x08/0x09 | Interrupt Control |
| IOCON_A/B | 0x0A/0x0B | Configuration |
| GPPU_A/B | 0x0C/0x0D | Pull-up Enable |
| INTF_A/B | 0x0E/0x0F | Interrupt Flag |
| INTCAP_A/B | 0x10/0x11 | Interrupt Capture |
| GPIO_A/B | 0x12/0x13 | Port Register |
| OLAT_A/B | 0x14/0x15 | Output Latch |

## Performance Notes

- SPI speed up to 10 MHz is supported
- Hardware addressing allows up to 8 devices per SPI bus
- 16-bit operations are atomic (both ports updated together)
- Built-in CS pin management for clean SPI transactions

## Contributing

Contributions are welcome! Please feel free to submit issues, feature requests, or pull requests.

## License

This library is provided under the MIT License. See the source files for full license text.

## Credits

Based on the original work by Mathias Yde (https://github.com/MathiasYde/pico-mcp23s17) with significant enhancements and a complete C++ rewrite.

## Features

- **Modern C API**: Clean, well-documented function interface
- **Multiple Device Support**: Support for up to 8 devices on the same SPI bus using hardware addressing
- **Comprehensive I/O Operations**: Individual pin control, port-level operations, and 16-bit operations
- **Hardware Features**: Pull-up resistors, input polarity inversion, configurable I/O directions
- **Legacy Compatibility**: Backward-compatible functions for existing code
- **Error Handling**: Proper error codes and validation
- **Pico SDK Integration**: Uses hardware SPI and GPIO libraries

## Hardware Setup

### Basic Connections

Connect the MCP23S17 to your Raspberry Pi Pico as follows:

| MCP23S17 Pin | Pico Pin | Function |
|--------------|----------|----------|
| VDD | 3.3V | Power |
| VSS | GND | Ground |
| SI (MOSI) | GPIO 19 | SPI Data In |
| SO (MISO) | GPIO 16 | SPI Data Out |
| SCK | GPIO 18 | SPI Clock |
| CS | GPIO 17 | Chip Select |
| A0, A1, A2 | GND or 3.3V | Device Address |
| RESET | 3.3V | Reset (active low) |

### Device Addressing

The MCP23S17 supports up to 8 devices on the same SPI bus using hardware addressing pins A0, A1, A2:

| A2 | A1 | A0 | Address |
|----|----|----|---------|
| 0  | 0  | 0  | 0x00 |
| 0  | 0  | 1  | 0x01 |
| 0  | 1  | 0  | 0x02 |
| ... | ... | ... | ... |
| 1  | 1  | 1  | 0x07 |

## Installation

1. Copy `mcp23s17.h` and `mcp23s17.c` to your project directory
2. Update your `CMakeLists.txt` to include the library:

```cmake
target_sources(your_project PRIVATE
    mcp23s17.c
    # your other source files
)

target_link_libraries(your_project
    pico_stdlib
    hardware_spi
    hardware_gpio
)
```

## Quick Start

### Basic Usage

```c
#include "mcp23s17.h"

int main() {
    stdio_init_all();
    
    // Initialize SPI
    spi_init(spi0, 1000000);  // 1 MHz
    gpio_set_function(16, GPIO_FUNC_SPI);  // MISO
    gpio_set_function(18, GPIO_FUNC_SPI);  // SCK  
    gpio_set_function(19, GPIO_FUNC_SPI);  // MOSI
    
    // Initialize MCP23S17 device
    mcp23s17_device_t device;
    mcp23s17_init_device(&device, spi0, 17, 0);  // SPI, CS pin, address
    mcp23s17_init(&device);
    
    // Configure ports
    mcp23s17_set_direction(&device, MCP23S17_PORT_A, MCP23S17_OUTPUT);
    mcp23s17_set_direction(&device, MCP23S17_PORT_B, MCP23S17_INPUT);
    mcp23s17_set_pullup(&device, MCP23S17_PORT_B, MCP23S17_PULLUP_ENABLE);
    
    while (true) {
        // Blink LEDs on Port A
        for (int i = 0; i < 8; i++) {
            mcp23s17_write_port(&device, MCP23S17_PORT_A, 1 << i);
            sleep_ms(200);
        }
    }
}
```

## API Reference

### Device Management

#### `mcp23s17_init_device()`
```c
int mcp23s17_init_device(mcp23s17_device_t* device, spi_inst_t* spi_inst, 
                        uint8_t cs_pin, uint8_t device_addr);
```
Initialize a device structure.

#### `mcp23s17_init()`
```c
int mcp23s17_init(mcp23s17_device_t* device);
```
Initialize the MCP23S17 chip with default configuration.

### I/O Configuration

#### `mcp23s17_set_direction()`
```c
int mcp23s17_set_direction(mcp23s17_device_t* device, mcp23s17_port_t port, uint8_t direction);
```
Set I/O direction for an entire port (0x00 = all outputs, 0xFF = all inputs).

#### `mcp23s17_set_pin_direction()`
```c
int mcp23s17_set_pin_direction(mcp23s17_device_t* device, mcp23s17_port_t port, 
                              uint8_t pin, bool is_input);
```
Set I/O direction for an individual pin.

#### `mcp23s17_set_pullup()`
```c
int mcp23s17_set_pullup(mcp23s17_device_t* device, mcp23s17_port_t port, uint8_t pullup_mask);
```
Enable/disable internal pull-up resistors.

### I/O Operations

#### Port-Level Operations
```c
int mcp23s17_read_port(mcp23s17_device_t* device, mcp23s17_port_t port, uint8_t* value);
int mcp23s17_write_port(mcp23s17_device_t* device, mcp23s17_port_t port, uint8_t value);
```

#### Individual Pin Operations
```c
int mcp23s17_read_pin(mcp23s17_device_t* device, mcp23s17_port_t port, uint8_t pin, bool* state);
int mcp23s17_write_pin(mcp23s17_device_t* device, mcp23s17_port_t port, uint8_t pin, bool state);
int mcp23s17_toggle_pin(mcp23s17_device_t* device, mcp23s17_port_t port, uint8_t pin);
```

#### 16-bit Operations
```c
int mcp23s17_read_gpio(mcp23s17_device_t* device, uint16_t* value);
int mcp23s17_write_gpio(mcp23s17_device_t* device, uint16_t value);
```

### Register Access

#### Direct Register Access
```c
int mcp23s17_read_byte(mcp23s17_device_t* device, uint8_t reg, uint8_t* value);
int mcp23s17_write_byte(mcp23s17_device_t* device, uint8_t reg, uint8_t value);
```

## Constants and Enums

### Ports
```c
typedef enum {
    MCP23S17_PORT_A = 0,
    MCP23S17_PORT_B = 1
} mcp23s17_port_t;
```

### Pin Masks
```c
typedef enum {
    MCP23S17_PIN_0 = 0x01,
    MCP23S17_PIN_1 = 0x02,
    // ... up to PIN_7 = 0x80
    MCP23S17_PIN_ALL = 0xFF
} mcp23s17_pin_t;
```

### Configuration Constants
```c
#define MCP23S17_INPUT   0xFF    // Configure as inputs
#define MCP23S17_OUTPUT  0x00    // Configure as outputs

#define MCP23S17_PULLUP_ENABLE  0xFF  // Enable pull-ups
#define MCP23S17_PULLUP_DISABLE 0x00  // Disable pull-ups

#define MCP23S17_POLARITY_NORMAL   0x00  // Normal polarity
#define MCP23S17_POLARITY_INVERTED 0xFF  // Inverted polarity
```

## Examples

### LED Chaser
```c
// Configure Port A as outputs
mcp23s17_set_direction(&device, MCP23S17_PORT_A, MCP23S17_OUTPUT);

// Chase pattern
while (true) {
    for (int i = 0; i < 8; i++) {
        mcp23s17_write_port(&device, MCP23S17_PORT_A, 1 << i);
        sleep_ms(100);
    }
}
```

### Input Reading with Debouncing
```c
// Configure Port B as inputs with pull-ups
mcp23s17_set_direction(&device, MCP23S17_PORT_B, MCP23S17_INPUT);
mcp23s17_set_pullup(&device, MCP23S17_PORT_B, MCP23S17_PULLUP_ENABLE);

uint8_t last_state = 0xFF;
while (true) {
    uint8_t current_state;
    mcp23s17_read_port(&device, MCP23S17_PORT_B, &current_state);
    
    if (current_state != last_state) {
        printf("Input changed: 0x%02X\\n", current_state);
        last_state = current_state;
    }
    sleep_ms(10);
}
```

### Multiple Devices
```c
mcp23s17_device_t device1, device2;

// Initialize two devices with different addresses
mcp23s17_init_device(&device1, spi0, 17, 0);  // Address 0
mcp23s17_init_device(&device2, spi0, 18, 1);  // Address 1

mcp23s17_init(&device1);
mcp23s17_init(&device2);

// Control them independently
mcp23s17_write_port(&device1, MCP23S17_PORT_A, 0xAA);
mcp23s17_write_port(&device2, MCP23S17_PORT_A, 0x55);
```

## Legacy Compatibility

The library includes legacy compatibility functions for code based on the original pico-mcp23s17 library:

```c
// Legacy function calls (no device structure needed)
mcp23s17_legacy_write_byte(spi0, cs_pin, MCP23S17_GPIO_A, value);
mcp23s17_legacy_read_byte(spi0, cs_pin, MCP23S17_GPIO_B, &value);
```

## Error Handling

All functions return `PICO_OK` on success or an appropriate error code:
- `PICO_ERROR_INVALID_ARG`: Invalid argument passed
- Other Pico SDK error codes as appropriate

## Register Map

The library defines all MCP23S17 registers when IOCON.BANK = 0:

| Register | Address | Description |
|----------|---------|-------------|
| IODIR_A/B | 0x00/0x01 | I/O Direction |
| IPOL_A/B | 0x02/0x03 | Input Polarity |
| GPINTEN_A/B | 0x04/0x05 | Interrupt Enable |
| DEFVAL_A/B | 0x06/0x07 | Default Compare |
| INTCON_A/B | 0x08/0x09 | Interrupt Control |
| IOCON_A/B | 0x0A/0x0B | Configuration |
| GPPU_A/B | 0x0C/0x0D | Pull-up Enable |
| INTF_A/B | 0x0E/0x0F | Interrupt Flag |
| INTCAP_A/B | 0x10/0x11 | Interrupt Capture |
| GPIO_A/B | 0x12/0x13 | Port Register |
| OLAT_A/B | 0x14/0x15 | Output Latch |

## Contributing

Contributions are welcome! Please feel free to submit issues, feature requests, or pull requests.

## License

This library is provided under the MIT License. See the source files for full license text.

## Credits

Based on the original work by Mathias Yde (https://github.com/MathiasYde/pico-mcp23s17) with significant enhancements and improvements.
