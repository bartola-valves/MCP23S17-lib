#include "mcp23s17_utils.h"
#include "pico/stdlib.h"

// Predefined pin configurations
const mcp23s17_spi_pins_t MCP23S17_DEFAULT_PINS = {
    .miso_pin = 16,
    .cs_pin = 17,
    .sck_pin = 18,
    .mosi_pin = 19};

const mcp23s17_spi_pins_t MCP23S17_ALT_PINS = {
    .miso_pin = 12,
    .cs_pin = 13,
    .sck_pin = 14,
    .mosi_pin = 15};

int mcp23s17_setup_complete(mcp23s17_device_t *device, spi_inst_t *spi_inst,
                            const mcp23s17_spi_pins_t *pins, uint8_t device_addr,
                            uint32_t baudrate)
{
    if (!device || !pins || !spi_inst)
    {
        return PICO_ERROR_INVALID_ARG;
    }

    // Initialize SPI
    spi_init(spi_inst, baudrate);

    // Configure SPI pins
    gpio_set_function(pins->miso_pin, GPIO_FUNC_SPI);
    gpio_set_function(pins->sck_pin, GPIO_FUNC_SPI);
    gpio_set_function(pins->mosi_pin, GPIO_FUNC_SPI);

    // Initialize device
    int result = mcp23s17_init_device(device, spi_inst, pins->cs_pin, device_addr);
    if (result != PICO_OK)
    {
        return result;
    }

    // Initialize MCP23S17
    return mcp23s17_init(device);
}

int mcp23s17_quick_setup(mcp23s17_device_t *device, uint8_t device_addr)
{
    return mcp23s17_setup_complete(device, spi0, &MCP23S17_DEFAULT_PINS,
                                   device_addr, 1000000);
}

int mcp23s17_setup_led_port(mcp23s17_device_t *device, mcp23s17_port_t port)
{
    if (!device)
    {
        return PICO_ERROR_INVALID_ARG;
    }

    // Configure as outputs
    int result = mcp23s17_set_direction(device, port, MCP23S17_OUTPUT);
    if (result != PICO_OK)
    {
        return result;
    }

    // Disable pull-ups (not needed for outputs)
    result = mcp23s17_set_pullup(device, port, MCP23S17_PULLUP_DISABLE);
    if (result != PICO_OK)
    {
        return result;
    }

    // Set initial state to all off
    return mcp23s17_write_port(device, port, 0x00);
}

int mcp23s17_setup_input_port(mcp23s17_device_t *device, mcp23s17_port_t port, bool enable_pullups)
{
    if (!device)
    {
        return PICO_ERROR_INVALID_ARG;
    }

    // Configure as inputs
    int result = mcp23s17_set_direction(device, port, MCP23S17_INPUT);
    if (result != PICO_OK)
    {
        return result;
    }

    // Configure pull-ups
    uint8_t pullup_setting = enable_pullups ? MCP23S17_PULLUP_ENABLE : MCP23S17_PULLUP_DISABLE;
    return mcp23s17_set_pullup(device, port, pullup_setting);
}

int mcp23s17_set_pins_masked(mcp23s17_device_t *device, mcp23s17_port_t port,
                             uint8_t pin_mask, bool clear_others)
{
    if (!device)
    {
        return PICO_ERROR_INVALID_ARG;
    }

    if (clear_others)
    {
        // Simply write the mask directly
        return mcp23s17_write_port(device, port, pin_mask);
    }
    else
    {
        // Read current state, set specified pins, write back
        uint8_t current_value;
        int result = mcp23s17_read_port(device, port, &current_value);
        if (result != PICO_OK)
        {
            return result;
        }

        current_value |= pin_mask; // Set bits
        return mcp23s17_write_port(device, port, current_value);
    }
}

int mcp23s17_clear_pins_masked(mcp23s17_device_t *device, mcp23s17_port_t port, uint8_t pin_mask)
{
    if (!device)
    {
        return PICO_ERROR_INVALID_ARG;
    }

    // Read current state, clear specified pins, write back
    uint8_t current_value;
    int result = mcp23s17_read_port(device, port, &current_value);
    if (result != PICO_OK)
    {
        return result;
    }

    current_value &= ~pin_mask; // Clear bits
    return mcp23s17_write_port(device, port, current_value);
}

int mcp23s17_running_lights(mcp23s17_device_t *device, mcp23s17_port_t port,
                            uint32_t delay_ms, int cycles)
{
    if (!device || cycles < 0)
    {
        return PICO_ERROR_INVALID_ARG;
    }

    for (int cycle = 0; cycle < cycles; cycle++)
    {
        // Forward direction
        for (int i = 0; i < 8; i++)
        {
            int result = mcp23s17_write_port(device, port, 1 << i);
            if (result != PICO_OK)
            {
                return result;
            }
            sleep_ms(delay_ms);
        }

        // Backward direction (skip last position to avoid double-flash)
        for (int i = 6; i >= 1; i--)
        {
            int result = mcp23s17_write_port(device, port, 1 << i);
            if (result != PICO_OK)
            {
                return result;
            }
            sleep_ms(delay_ms);
        }
    }

    // Turn off all LEDs
    return mcp23s17_write_port(device, port, 0x00);
}

int mcp23s17_binary_counter(mcp23s17_device_t *device, mcp23s17_port_t port,
                            uint32_t delay_ms, uint8_t max_count)
{
    if (!device)
    {
        return PICO_ERROR_INVALID_ARG;
    }

    for (uint8_t count = 0; count <= max_count; count++)
    {
        int result = mcp23s17_write_port(device, port, count);
        if (result != PICO_OK)
        {
            return result;
        }
        sleep_ms(delay_ms);
    }

    return PICO_OK;
}

int mcp23s17_read_port_debounced(mcp23s17_device_t *device, mcp23s17_port_t port,
                                 uint8_t *value, uint32_t debounce_ms)
{
    if (!device || !value)
    {
        return PICO_ERROR_INVALID_ARG;
    }

    uint8_t reading1, reading2;

    // First reading
    int result = mcp23s17_read_port(device, port, &reading1);
    if (result != PICO_OK)
    {
        return result;
    }

    // Wait debounce time
    sleep_ms(debounce_ms);

    // Second reading
    result = mcp23s17_read_port(device, port, &reading2);
    if (result != PICO_OK)
    {
        return result;
    }

    // If readings are stable, return the value
    if (reading1 == reading2)
    {
        *value = reading1;
        return PICO_OK;
    }

    // If readings differ, wait a bit more and try again
    sleep_ms(debounce_ms);
    return mcp23s17_read_port(device, port, value);
}

int mcp23s17_test_all_pins(mcp23s17_device_t *device, uint32_t delay_ms)
{
    if (!device)
    {
        return PICO_ERROR_INVALID_ARG;
    }

    // Configure both ports as outputs
    int result = mcp23s17_set_direction(device, MCP23S17_PORT_A, MCP23S17_OUTPUT);
    if (result != PICO_OK)
        return result;

    result = mcp23s17_set_direction(device, MCP23S17_PORT_B, MCP23S17_OUTPUT);
    if (result != PICO_OK)
        return result;

    // Test all pins high
    mcp23s17_write_gpio(device, 0xFFFF);
    sleep_ms(delay_ms);

    // Test all pins low
    mcp23s17_write_gpio(device, 0x0000);
    sleep_ms(delay_ms);

    // Test individual pins
    for (int pin = 0; pin < 16; pin++)
    {
        uint16_t pattern = 1 << pin;
        mcp23s17_write_gpio(device, pattern);
        sleep_ms(delay_ms);
    }

    // Test alternating patterns
    uint16_t patterns[] = {0x5555, 0xAAAA, 0x3333, 0xCCCC, 0x0F0F, 0xF0F0};
    for (size_t i = 0; i < sizeof(patterns) / sizeof(patterns[0]); i++)
    {
        mcp23s17_write_gpio(device, patterns[i]);
        sleep_ms(delay_ms);
    }

    // Clear all pins
    mcp23s17_write_gpio(device, 0x0000);

    return PICO_OK;
}
