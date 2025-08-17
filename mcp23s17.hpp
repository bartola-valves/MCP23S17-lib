#ifndef MCP23S17_HPP
#define MCP23S17_HPP

#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

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

// Port Enumeration
enum class MCP23S17_Port
{
    A = 0,
    B = 1
};

// Pin Constants
enum class MCP23S17_Pin
{
    PIN_0 = 0x01,
    PIN_1 = 0x02,
    PIN_2 = 0x04,
    PIN_3 = 0x08,
    PIN_4 = 0x10,
    PIN_5 = 0x20,
    PIN_6 = 0x40,
    PIN_7 = 0x80,
    PIN_ALL = 0xFF
};

/**
 * @brief MCP23S17 16-bit I/O Expander Class
 *
 * This class provides a C++ interface to the MCP23S17 SPI I/O expander.
 * Features include:
 * - Individual pin control
 * - Port-level operations
 * - 16-bit operations across both ports
 * - Configurable pull-ups and polarity
 * - Multiple device support via hardware addressing
 */
class MCP23S17
{
private:
    spi_inst_t *spi_inst_;
    uint8_t cs_pin_;
    uint8_t device_addr_;
    bool initialized_;

    /**
     * @brief Perform SPI transaction with the MCP23S17
     * @param opcode SPI opcode (read/write)
     * @param reg Register address
     * @param data Pointer to data buffer
     * @param data_len Length of data
     * @param is_read True for read operation, false for write
     * @return PICO_OK on success, error code on failure
     */
    int spiTransaction(uint8_t opcode, uint8_t reg, uint8_t *data, uint8_t data_len, bool is_read);

public:
    /**
     * @brief Construct a new MCP23S17 object
     * @param spi_inst SPI instance to use (spi0 or spi1)
     * @param cs_pin GPIO pin to use as chip select
     * @param device_addr Hardware address (0-7, set by A0, A1, A2 pins)
     */
    MCP23S17(spi_inst_t *spi_inst, uint8_t cs_pin, uint8_t device_addr = 0);

    /**
     * @brief Initialize the MCP23S17 device
     * @return true on success, false on failure
     */
    bool begin();

    /**
     * @brief Check if the device is initialized
     * @return true if initialized, false otherwise
     */
    bool isInitialized() const { return initialized_; }

    // Register Access Methods

    /**
     * @brief Read a byte from a register
     * @param reg Register address
     * @return Register value, or 0xFF on error
     */
    uint8_t readRegister(uint8_t reg);

    /**
     * @brief Write a byte to a register
     * @param reg Register address
     * @param value Value to write
     * @return true on success, false on failure
     */
    bool writeRegister(uint8_t reg, uint8_t value);

    /**
     * @brief Read 16-bit value from paired registers
     * @param low_reg Lower register (Port A)
     * @param high_reg Higher register (Port B)
     * @return 16-bit value (high byte = Port B, low byte = Port A)
     */
    uint16_t readRegisterPair(uint8_t low_reg, uint8_t high_reg);

    /**
     * @brief Write 16-bit value to paired registers
     * @param low_reg Lower register (Port A)
     * @param high_reg Higher register (Port B)
     * @param value 16-bit value to write
     * @return true on success, false on failure
     */
    bool writeRegisterPair(uint8_t low_reg, uint8_t high_reg, uint16_t value);

    // Configuration Methods

    /**
     * @brief Set I/O direction for a port
     * @param port Port to configure
     * @param direction Direction mask (1=input, 0=output)
     * @return true on success, false on failure
     */
    bool setPortDirection(MCP23S17_Port port, uint8_t direction);

    /**
     * @brief Set I/O direction for individual pin
     * @param port Port containing the pin
     * @param pin Pin number (0-7)
     * @param is_input true for input, false for output
     * @return true on success, false on failure
     */
    bool setPinDirection(MCP23S17_Port port, uint8_t pin, bool is_input);

    /**
     * @brief Configure pull-up resistors for a port
     * @param port Port to configure
     * @param pullup_mask Pull-up mask (1=enable, 0=disable)
     * @return true on success, false on failure
     */
    bool setPortPullups(MCP23S17_Port port, uint8_t pullup_mask);

    /**
     * @brief Configure pull-up for individual pin
     * @param port Port containing the pin
     * @param pin Pin number (0-7)
     * @param enable true to enable pull-up, false to disable
     * @return true on success, false on failure
     */
    bool setPinPullup(MCP23S17_Port port, uint8_t pin, bool enable);

    /**
     * @brief Configure input polarity for a port
     * @param port Port to configure
     * @param polarity_mask Polarity mask (1=inverted, 0=normal)
     * @return true on success, false on failure
     */
    bool setPortPolarity(MCP23S17_Port port, uint8_t polarity_mask);

    // I/O Methods

    /**
     * @brief Read all pins from a port
     * @param port Port to read
     * @return Port value (0-255)
     */
    uint8_t readPort(MCP23S17_Port port);

    /**
     * @brief Write all pins on a port
     * @param port Port to write
     * @param value Value to write (0-255)
     * @return true on success, false on failure
     */
    bool writePort(MCP23S17_Port port, uint8_t value);

    /**
     * @brief Read both ports as 16-bit value
     * @return 16-bit value (Port B = upper 8 bits, Port A = lower 8 bits)
     */
    uint16_t readGPIO();

    /**
     * @brief Write both ports as 16-bit value
     * @param value 16-bit value (Port B = upper 8 bits, Port A = lower 8 bits)
     * @return true on success, false on failure
     */
    bool writeGPIO(uint16_t value);

    /**
     * @brief Read individual pin state
     * @param port Port containing the pin
     * @param pin Pin number (0-7)
     * @return true if pin is high, false if low or on error
     */
    bool readPin(MCP23S17_Port port, uint8_t pin);

    /**
     * @brief Write individual pin state
     * @param port Port containing the pin
     * @param pin Pin number (0-7)
     * @param state Pin state (true=high, false=low)
     * @return true on success, false on failure
     */
    bool writePin(MCP23S17_Port port, uint8_t pin, bool state);

    /**
     * @brief Toggle individual pin state
     * @param port Port containing the pin
     * @param pin Pin number (0-7)
     * @return true on success, false on failure
     */
    bool togglePin(MCP23S17_Port port, uint8_t pin);

    // Utility Methods

    /**
     * @brief Initialize CS pin for use
     * @param cs_pin GPIO pin number
     */
    static void initCSPin(uint8_t cs_pin);

    /**
     * @brief Configure a port for LED output (all outputs, no pull-ups)
     * @param port Port to configure
     * @return true on success, false on failure
     */
    bool setupLEDPort(MCP23S17_Port port);

    /**
     * @brief Configure a port for button input (all inputs with pull-ups)
     * @param port Port to configure
     * @return true on success, false on failure
     */
    bool setupButtonPort(MCP23S17_Port port);

    /**
     * @brief Run LED chase pattern on a port
     * @param port Port to use
     * @param delay_ms Delay between steps
     * @param cycles Number of complete cycles
     * @return true on success, false on failure
     */
    bool runningLights(MCP23S17_Port port, uint32_t delay_ms, int cycles = 1);

    /**
     * @brief Run binary counter on a port
     * @param port Port to use
     * @param delay_ms Delay between counts
     * @param max_count Maximum count (0-255)
     * @return true on success, false on failure
     */
    bool binaryCounter(MCP23S17_Port port, uint32_t delay_ms, uint8_t max_count = 255);

    // Legacy C-style functions for compatibility

    /**
     * @brief Legacy read byte function (for compatibility)
     * @param reg Register address
     * @param value Pointer to store result
     * @return PICO_OK on success, error on failure
     */
    int legacy_read_byte(uint8_t reg, uint8_t *value);

    /**
     * @brief Legacy write byte function (for compatibility)
     * @param reg Register address
     * @param value Value to write
     * @return PICO_OK on success, error on failure
     */
    int legacy_write_byte(uint8_t reg, uint8_t value);
};

#endif // MCP23S17_HPP
