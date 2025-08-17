#ifndef MCP23S17_H
#define MCP23S17_H

#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#ifdef __cplusplus
extern "C"
{
#endif

// MCP23S17 Register Addresses (IOCON.BANK = 0)
#define MCP23S17_IODIR_A 0x00   // I/O Direction Register A
#define MCP23S17_IODIR_B 0x01   // I/O Direction Register B
#define MCP23S17_IPOL_A 0x02    // Input Polarity Register A
#define MCP23S17_IPOL_B 0x03    // Input Polarity Register B
#define MCP23S17_GPINTEN_A 0x04 // Interrupt-on-Change Enable Register A
#define MCP23S17_GPINTEN_B 0x05 // Interrupt-on-Change Enable Register B
#define MCP23S17_DEFVAL_A 0x06  // Default Compare Register for Interrupt A
#define MCP23S17_DEFVAL_B 0x07  // Default Compare Register for Interrupt B
#define MCP23S17_INTCON_A 0x08  // Interrupt Control Register A
#define MCP23S17_INTCON_B 0x09  // Interrupt Control Register B
#define MCP23S17_IOCON_A 0x0A   // Configuration Register A
#define MCP23S17_IOCON_B 0x0B   // Configuration Register B
#define MCP23S17_GPPU_A 0x0C    // Pull-up Resistor Register A
#define MCP23S17_GPPU_B 0x0D    // Pull-up Resistor Register B
#define MCP23S17_INTF_A 0x0E    // Interrupt Flag Register A
#define MCP23S17_INTF_B 0x0F    // Interrupt Flag Register B
#define MCP23S17_INTCAP_A 0x10  // Interrupt Capture Register A
#define MCP23S17_INTCAP_B 0x11  // Interrupt Capture Register B
#define MCP23S17_GPIO_A 0x12    // General Purpose I/O Port Register A
#define MCP23S17_GPIO_B 0x13    // General Purpose I/O Port Register B
#define MCP23S17_OLAT_A 0x14    // Output Latch Register A
#define MCP23S17_OLAT_B 0x15    // Output Latch Register B

// SPI Opcodes
#define MCP23S17_OPCODE_WRITE 0x40
#define MCP23S17_OPCODE_READ 0x41

// Configuration Constants
#define MCP23S17_MAX_SPEED 10000000    // 10 MHz max SPI speed
#define MCP23S17_DEFAULT_SPEED 1000000 // 1 MHz default SPI speed

// Direction Constants
#define MCP23S17_INPUT 0xFF
#define MCP23S17_OUTPUT 0x00

// Pull-up Constants
#define MCP23S17_PULLUP_ENABLE 0xFF
#define MCP23S17_PULLUP_DISABLE 0x00

// Polarity Constants
#define MCP23S17_POLARITY_NORMAL 0x00
#define MCP23S17_POLARITY_INVERTED 0xFF

    // Port Constants
    typedef enum
    {
        MCP23S17_PORT_A = 0,
        MCP23S17_PORT_B = 1
    } mcp23s17_port_t;

    // Pin Constants
    typedef enum
    {
        MCP23S17_PIN_0 = 0x01,
        MCP23S17_PIN_1 = 0x02,
        MCP23S17_PIN_2 = 0x04,
        MCP23S17_PIN_3 = 0x08,
        MCP23S17_PIN_4 = 0x10,
        MCP23S17_PIN_5 = 0x20,
        MCP23S17_PIN_6 = 0x40,
        MCP23S17_PIN_7 = 0x80,
        MCP23S17_PIN_ALL = 0xFF
    } mcp23s17_pin_t;

    // Device structure for multiple device support
    typedef struct
    {
        spi_inst_t *spi_inst;
        uint8_t cs_pin;
        uint8_t device_addr; // Hardware address (A2, A1, A0 pins)
        bool initialized;
    } mcp23s17_device_t;

    // Function Prototypes

    /**
     * @brief Initialize a CS pin for MCP23S17
     * @param cs_pin GPIO pin number to use as chip select (active low)
     * @return PICO_OK on success, error code on failure
     */
    int mcp23s17_init_cs_pin(uint8_t cs_pin);

    /**
     * @brief Initialize multiple CS pins for MCP23S17
     * @param cs_pins Array of GPIO pin numbers to use as chip select
     * @param pin_count Number of pins in the array
     * @return PICO_OK on success, error code on failure
     */
    int mcp23s17_init_cs_pins(const uint8_t cs_pins[], uint8_t pin_count);

    /**
     * @brief Initialize MCP23S17 device structure
     * @param device Pointer to device structure
     * @param spi_inst SPI instance to use
     * @param cs_pin Chip select pin
     * @param device_addr Hardware address (0-7, based on A2,A1,A0 pins)
     * @return PICO_OK on success, error code on failure
     */
    int mcp23s17_init_device(mcp23s17_device_t *device, spi_inst_t *spi_inst,
                             uint8_t cs_pin, uint8_t device_addr);

    /**
     * @brief Initialize MCP23S17 chip with default configuration
     * @param device Pointer to device structure
     * @return PICO_OK on success, error code on failure
     */
    int mcp23s17_init(mcp23s17_device_t *device);

    /**
     * @brief Read a byte from a register
     * @param device Pointer to device structure
     * @param reg Register address to read from
     * @param value Pointer to store the read value
     * @return PICO_OK on success, error code on failure
     */
    int mcp23s17_read_byte(mcp23s17_device_t *device, uint8_t reg, uint8_t *value);

    /**
     * @brief Write a byte to a register
     * @param device Pointer to device structure
     * @param reg Register address to write to
     * @param value Value to write
     * @return PICO_OK on success, error code on failure
     */
    int mcp23s17_write_byte(mcp23s17_device_t *device, uint8_t reg, uint8_t value);

    /**
     * @brief Read 16-bit word from paired registers
     * @param device Pointer to device structure
     * @param low_reg Lower register address (A port)
     * @param high_reg Higher register address (B port)
     * @param value Pointer to store the 16-bit value
     * @return PICO_OK on success, error code on failure
     */
    int mcp23s17_read_word(mcp23s17_device_t *device, uint8_t low_reg,
                           uint8_t high_reg, uint16_t *value);

    /**
     * @brief Write 16-bit word to paired registers
     * @param device Pointer to device structure
     * @param low_reg Lower register address (A port)
     * @param high_reg Higher register address (B port)
     * @param value 16-bit value to write
     * @return PICO_OK on success, error code on failure
     */
    int mcp23s17_write_word(mcp23s17_device_t *device, uint8_t low_reg,
                            uint8_t high_reg, uint16_t value);

    /**
     * @brief Set pin direction for a port
     * @param device Pointer to device structure
     * @param port Port to configure (MCP23S17_PORT_A or MCP23S17_PORT_B)
     * @param direction Pin directions (1=input, 0=output)
     * @return PICO_OK on success, error code on failure
     */
    int mcp23s17_set_direction(mcp23s17_device_t *device, mcp23s17_port_t port, uint8_t direction);

    /**
     * @brief Set individual pin direction
     * @param device Pointer to device structure
     * @param port Port to configure (MCP23S17_PORT_A or MCP23S17_PORT_B)
     * @param pin Pin number (0-7)
     * @param is_input true for input, false for output
     * @return PICO_OK on success, error code on failure
     */
    int mcp23s17_set_pin_direction(mcp23s17_device_t *device, mcp23s17_port_t port,
                                   uint8_t pin, bool is_input);

    /**
     * @brief Enable/disable pull-up resistors for a port
     * @param device Pointer to device structure
     * @param port Port to configure (MCP23S17_PORT_A or MCP23S17_PORT_B)
     * @param pullup_mask Pull-up enable mask (1=enable, 0=disable)
     * @return PICO_OK on success, error code on failure
     */
    int mcp23s17_set_pullup(mcp23s17_device_t *device, mcp23s17_port_t port, uint8_t pullup_mask);

    /**
     * @brief Read GPIO values from a port
     * @param device Pointer to device structure
     * @param port Port to read from (MCP23S17_PORT_A or MCP23S17_PORT_B)
     * @param value Pointer to store the port value
     * @return PICO_OK on success, error code on failure
     */
    int mcp23s17_read_port(mcp23s17_device_t *device, mcp23s17_port_t port, uint8_t *value);

    /**
     * @brief Write GPIO values to a port
     * @param device Pointer to device structure
     * @param port Port to write to (MCP23S17_PORT_A or MCP23S17_PORT_B)
     * @param value Value to write to the port
     * @return PICO_OK on success, error code on failure
     */
    int mcp23s17_write_port(mcp23s17_device_t *device, mcp23s17_port_t port, uint8_t value);

    /**
     * @brief Read both GPIO ports as 16-bit value
     * @param device Pointer to device structure
     * @param value Pointer to store the 16-bit value (A port = lower 8 bits)
     * @return PICO_OK on success, error code on failure
     */
    int mcp23s17_read_gpio(mcp23s17_device_t *device, uint16_t *value);

    /**
     * @brief Write both GPIO ports as 16-bit value
     * @param device Pointer to device structure
     * @param value 16-bit value to write (A port = lower 8 bits)
     * @return PICO_OK on success, error code on failure
     */
    int mcp23s17_write_gpio(mcp23s17_device_t *device, uint16_t value);

    /**
     * @brief Read individual pin state
     * @param device Pointer to device structure
     * @param port Port to read from (MCP23S17_PORT_A or MCP23S17_PORT_B)
     * @param pin Pin number (0-7)
     * @param state Pointer to store pin state (true=high, false=low)
     * @return PICO_OK on success, error code on failure
     */
    int mcp23s17_read_pin(mcp23s17_device_t *device, mcp23s17_port_t port,
                          uint8_t pin, bool *state);

    /**
     * @brief Write individual pin state
     * @param device Pointer to device structure
     * @param port Port to write to (MCP23S17_PORT_A or MCP23S17_PORT_B)
     * @param pin Pin number (0-7)
     * @param state Pin state to set (true=high, false=low)
     * @return PICO_OK on success, error code on failure
     */
    int mcp23s17_write_pin(mcp23s17_device_t *device, mcp23s17_port_t port,
                           uint8_t pin, bool state);

    /**
     * @brief Toggle individual pin state
     * @param device Pointer to device structure
     * @param port Port to toggle (MCP23S17_PORT_A or MCP23S17_PORT_B)
     * @param pin Pin number (0-7)
     * @return PICO_OK on success, error code on failure
     */
    int mcp23s17_toggle_pin(mcp23s17_device_t *device, mcp23s17_port_t port, uint8_t pin);

    /**
     * @brief Configure input polarity for a port
     * @param device Pointer to device structure
     * @param port Port to configure (MCP23S17_PORT_A or MCP23S17_PORT_B)
     * @param polarity_mask Polarity mask (1=inverted, 0=normal)
     * @return PICO_OK on success, error code on failure
     */
    int mcp23s17_set_polarity(mcp23s17_device_t *device, mcp23s17_port_t port, uint8_t polarity_mask);

    // Convenience functions for backward compatibility with the original library
    int mcp23s17_legacy_init_gpio(uint8_t pins[], uint8_t pin_count);
    int mcp23s17_legacy_init(spi_inst_t *spi_inst, uint8_t cs_pin);
    int mcp23s17_legacy_read_byte(spi_inst_t *spi_inst, uint8_t cs_pin, uint8_t reg, uint8_t *value);
    int mcp23s17_legacy_write_byte(spi_inst_t *spi_inst, uint8_t cs_pin, uint8_t reg, uint8_t value);
    int mcp23s17_legacy_read_word(spi_inst_t *spi_inst, uint8_t cs_pin, uint8_t low_reg, uint8_t high_reg, uint16_t *value);

#ifdef __cplusplus
}
#endif

#endif // MCP23S17_H
