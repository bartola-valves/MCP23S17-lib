#ifndef MCP23S17_UTILS_H
#define MCP23S17_UTILS_H

#include "mcp23s17.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Common SPI pin configurations for Pico
     */
    typedef struct
    {
        uint8_t miso_pin;
        uint8_t cs_pin;
        uint8_t sck_pin;
        uint8_t mosi_pin;
    } mcp23s17_spi_pins_t;

    // Predefined pin configurations
    extern const mcp23s17_spi_pins_t MCP23S17_DEFAULT_PINS;
    extern const mcp23s17_spi_pins_t MCP23S17_ALT_PINS;

    /**
     * @brief Complete setup for MCP23S17 with SPI initialization
     * @param device Pointer to device structure
     * @param spi_inst SPI instance (spi0 or spi1)
     * @param pins Pin configuration structure
     * @param device_addr Hardware address (0-7)
     * @param baudrate SPI baudrate (recommended: 1000000 for 1MHz)
     * @return PICO_OK on success, error code on failure
     */
    int mcp23s17_setup_complete(mcp23s17_device_t *device, spi_inst_t *spi_inst,
                                const mcp23s17_spi_pins_t *pins, uint8_t device_addr,
                                uint32_t baudrate);

    /**
     * @brief Quick setup with default pins and 1MHz SPI
     * @param device Pointer to device structure
     * @param device_addr Hardware address (0-7)
     * @return PICO_OK on success, error code on failure
     */
    int mcp23s17_quick_setup(mcp23s17_device_t *device, uint8_t device_addr);

    /**
     * @brief Configure a port for LED output (all pins as outputs, no pull-ups)
     * @param device Pointer to device structure
     * @param port Port to configure
     * @return PICO_OK on success, error code on failure
     */
    int mcp23s17_setup_led_port(mcp23s17_device_t *device, mcp23s17_port_t port);

    /**
     * @brief Configure a port for button/switch input (all pins as inputs with pull-ups)
     * @param device Pointer to device structure
     * @param port Port to configure
     * @param enable_pullups Enable internal pull-up resistors
     * @return PICO_OK on success, error code on failure
     */
    int mcp23s17_setup_input_port(mcp23s17_device_t *device, mcp23s17_port_t port, bool enable_pullups);

    /**
     * @brief Set multiple pins at once using a mask
     * @param device Pointer to device structure
     * @param port Port to modify
     * @param pin_mask Mask of pins to set (1 = set high)
     * @param clear_others If true, clear all other pins; if false, only modify specified pins
     * @return PICO_OK on success, error code on failure
     */
    int mcp23s17_set_pins_masked(mcp23s17_device_t *device, mcp23s17_port_t port,
                                 uint8_t pin_mask, bool clear_others);

    /**
     * @brief Clear multiple pins at once using a mask
     * @param device Pointer to device structure
     * @param port Port to modify
     * @param pin_mask Mask of pins to clear (1 = clear to low)
     * @return PICO_OK on success, error code on failure
     */
    int mcp23s17_clear_pins_masked(mcp23s17_device_t *device, mcp23s17_port_t port, uint8_t pin_mask);

    /**
     * @brief Running light pattern on a port
     * @param device Pointer to device structure
     * @param port Port to use for pattern
     * @param delay_ms Delay between steps in milliseconds
     * @param cycles Number of complete cycles to run
     * @return PICO_OK on success, error code on failure
     */
    int mcp23s17_running_lights(mcp23s17_device_t *device, mcp23s17_port_t port,
                                uint32_t delay_ms, int cycles);

    /**
     * @brief Binary counter pattern on a port
     * @param device Pointer to device structure
     * @param port Port to use for pattern
     * @param delay_ms Delay between counts in milliseconds
     * @param max_count Maximum count value (0-255)
     * @return PICO_OK on success, error code on failure
     */
    int mcp23s17_binary_counter(mcp23s17_device_t *device, mcp23s17_port_t port,
                                uint32_t delay_ms, uint8_t max_count);

    /**
     * @brief Read port with debouncing
     * @param device Pointer to device structure
     * @param port Port to read
     * @param value Pointer to store debounced value
     * @param debounce_ms Debounce time in milliseconds
     * @return PICO_OK on success, error code on failure
     */
    int mcp23s17_read_port_debounced(mcp23s17_device_t *device, mcp23s17_port_t port,
                                     uint8_t *value, uint32_t debounce_ms);

    /**
     * @brief Test all pins on both ports (diagnostic function)
     * @param device Pointer to device structure
     * @param delay_ms Delay between test steps
     * @return PICO_OK on success, error code on failure
     */
    int mcp23s17_test_all_pins(mcp23s17_device_t *device, uint32_t delay_ms);

#ifdef __cplusplus
}
#endif

#endif // MCP23S17_UTILS_H
