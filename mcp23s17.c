#include "mcp23s17.h"

// Static helper functions
static int mcp23s17_spi_transaction(mcp23s17_device_t *device, uint8_t opcode,
                                    uint8_t reg, uint8_t *data, uint8_t data_len, bool is_read)
{
    if (!device || !device->initialized)
    {
        return PICO_ERROR_INVALID_ARG;
    }

    // Calculate full opcode with device address
    uint8_t full_opcode = opcode | ((device->device_addr & 0x07) << 1);

    // Select device (CS active low)
    gpio_put(device->cs_pin, false);

    // Send opcode and register address
    uint8_t command[2] = {full_opcode, reg};
    int result = spi_write_blocking(device->spi_inst, command, 2);
    if (result < 0)
    {
        gpio_put(device->cs_pin, true);
        return result;
    }

    // Handle data transfer
    if (is_read)
    {
        result = spi_read_blocking(device->spi_inst, 0x00, data, data_len);
    }
    else
    {
        result = spi_write_blocking(device->spi_inst, data, data_len);
    }

    // Deselect device
    gpio_put(device->cs_pin, true);

    return (result < 0) ? result : PICO_OK;
}

// Public API implementation

int mcp23s17_init_cs_pin(uint8_t cs_pin)
{
    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_put(cs_pin, true); // Deselect (active low)
    return PICO_OK;
}

int mcp23s17_init_cs_pins(const uint8_t cs_pins[], uint8_t pin_count)
{
    for (uint8_t i = 0; i < pin_count; i++)
    {
        int result = mcp23s17_init_cs_pin(cs_pins[i]);
        if (result != PICO_OK)
        {
            return result;
        }
    }
    return PICO_OK;
}

int mcp23s17_init_device(mcp23s17_device_t *device, spi_inst_t *spi_inst,
                         uint8_t cs_pin, uint8_t device_addr)
{
    if (!device || !spi_inst || device_addr > 7)
    {
        return PICO_ERROR_INVALID_ARG;
    }

    device->spi_inst = spi_inst;
    device->cs_pin = cs_pin;
    device->device_addr = device_addr;
    device->initialized = false;

    // Initialize CS pin
    int result = mcp23s17_init_cs_pin(cs_pin);
    if (result != PICO_OK)
    {
        return result;
    }

    device->initialized = true;
    return PICO_OK;
}

int mcp23s17_init(mcp23s17_device_t *device)
{
    if (!device || !device->initialized)
    {
        return PICO_ERROR_INVALID_ARG;
    }

    // Configure IOCON register for default operation
    // BANK=0, MIRROR=0, SEQOP=0, DISSLW=0, HAEN=1, ODR=0, INTPOL=0
    uint8_t iocon_value = 0x08; // Enable hardware addressing (HAEN=1)

    int result = mcp23s17_write_byte(device, MCP23S17_IOCON_A, iocon_value);
    if (result != PICO_OK)
    {
        return result;
    }

    // Write to IOCON_B as well to ensure consistency
    result = mcp23s17_write_byte(device, MCP23S17_IOCON_B, iocon_value);

    return result;
}

int mcp23s17_read_byte(mcp23s17_device_t *device, uint8_t reg, uint8_t *value)
{
    if (!value)
    {
        return PICO_ERROR_INVALID_ARG;
    }

    return mcp23s17_spi_transaction(device, MCP23S17_OPCODE_READ, reg, value, 1, true);
}

int mcp23s17_write_byte(mcp23s17_device_t *device, uint8_t reg, uint8_t value)
{
    return mcp23s17_spi_transaction(device, MCP23S17_OPCODE_WRITE, reg, &value, 1, false);
}

int mcp23s17_read_word(mcp23s17_device_t *device, uint8_t low_reg,
                       uint8_t high_reg, uint16_t *value)
{
    if (!value)
    {
        return PICO_ERROR_INVALID_ARG;
    }

    uint8_t low_byte, high_byte;

    int result = mcp23s17_read_byte(device, low_reg, &low_byte);
    if (result != PICO_OK)
    {
        return result;
    }

    result = mcp23s17_read_byte(device, high_reg, &high_byte);
    if (result != PICO_OK)
    {
        return result;
    }

    *value = ((uint16_t)high_byte << 8) | low_byte;
    return PICO_OK;
}

int mcp23s17_write_word(mcp23s17_device_t *device, uint8_t low_reg,
                        uint8_t high_reg, uint16_t value)
{
    int result = mcp23s17_write_byte(device, low_reg, (uint8_t)(value & 0xFF));
    if (result != PICO_OK)
    {
        return result;
    }

    result = mcp23s17_write_byte(device, high_reg, (uint8_t)(value >> 8));
    return result;
}

int mcp23s17_set_direction(mcp23s17_device_t *device, mcp23s17_port_t port, uint8_t direction)
{
    uint8_t reg = (port == MCP23S17_PORT_A) ? MCP23S17_IODIR_A : MCP23S17_IODIR_B;
    return mcp23s17_write_byte(device, reg, direction);
}

int mcp23s17_set_pin_direction(mcp23s17_device_t *device, mcp23s17_port_t port,
                               uint8_t pin, bool is_input)
{
    if (pin > 7)
    {
        return PICO_ERROR_INVALID_ARG;
    }

    uint8_t reg = (port == MCP23S17_PORT_A) ? MCP23S17_IODIR_A : MCP23S17_IODIR_B;
    uint8_t current_value;

    int result = mcp23s17_read_byte(device, reg, &current_value);
    if (result != PICO_OK)
    {
        return result;
    }

    if (is_input)
    {
        current_value |= (1 << pin); // Set bit for input
    }
    else
    {
        current_value &= ~(1 << pin); // Clear bit for output
    }

    return mcp23s17_write_byte(device, reg, current_value);
}

int mcp23s17_set_pullup(mcp23s17_device_t *device, mcp23s17_port_t port, uint8_t pullup_mask)
{
    uint8_t reg = (port == MCP23S17_PORT_A) ? MCP23S17_GPPU_A : MCP23S17_GPPU_B;
    return mcp23s17_write_byte(device, reg, pullup_mask);
}

int mcp23s17_read_port(mcp23s17_device_t *device, mcp23s17_port_t port, uint8_t *value)
{
    uint8_t reg = (port == MCP23S17_PORT_A) ? MCP23S17_GPIO_A : MCP23S17_GPIO_B;
    return mcp23s17_read_byte(device, reg, value);
}

int mcp23s17_write_port(mcp23s17_device_t *device, mcp23s17_port_t port, uint8_t value)
{
    uint8_t reg = (port == MCP23S17_PORT_A) ? MCP23S17_GPIO_A : MCP23S17_GPIO_B;
    return mcp23s17_write_byte(device, reg, value);
}

int mcp23s17_read_gpio(mcp23s17_device_t *device, uint16_t *value)
{
    return mcp23s17_read_word(device, MCP23S17_GPIO_A, MCP23S17_GPIO_B, value);
}

int mcp23s17_write_gpio(mcp23s17_device_t *device, uint16_t value)
{
    return mcp23s17_write_word(device, MCP23S17_GPIO_A, MCP23S17_GPIO_B, value);
}

int mcp23s17_read_pin(mcp23s17_device_t *device, mcp23s17_port_t port,
                      uint8_t pin, bool *state)
{
    if (pin > 7 || !state)
    {
        return PICO_ERROR_INVALID_ARG;
    }

    uint8_t port_value;
    int result = mcp23s17_read_port(device, port, &port_value);
    if (result == PICO_OK)
    {
        *state = (port_value & (1 << pin)) != 0;
    }

    return result;
}

int mcp23s17_write_pin(mcp23s17_device_t *device, mcp23s17_port_t port,
                       uint8_t pin, bool state)
{
    if (pin > 7)
    {
        return PICO_ERROR_INVALID_ARG;
    }

    uint8_t reg = (port == MCP23S17_PORT_A) ? MCP23S17_GPIO_A : MCP23S17_GPIO_B;
    uint8_t current_value;

    int result = mcp23s17_read_byte(device, reg, &current_value);
    if (result != PICO_OK)
    {
        return result;
    }

    if (state)
    {
        current_value |= (1 << pin); // Set bit
    }
    else
    {
        current_value &= ~(1 << pin); // Clear bit
    }

    return mcp23s17_write_byte(device, reg, current_value);
}

int mcp23s17_toggle_pin(mcp23s17_device_t *device, mcp23s17_port_t port, uint8_t pin)
{
    if (pin > 7)
    {
        return PICO_ERROR_INVALID_ARG;
    }

    uint8_t reg = (port == MCP23S17_PORT_A) ? MCP23S17_GPIO_A : MCP23S17_GPIO_B;
    uint8_t current_value;

    int result = mcp23s17_read_byte(device, reg, &current_value);
    if (result != PICO_OK)
    {
        return result;
    }

    current_value ^= (1 << pin); // Toggle bit

    return mcp23s17_write_byte(device, reg, current_value);
}

int mcp23s17_set_polarity(mcp23s17_device_t *device, mcp23s17_port_t port, uint8_t polarity_mask)
{
    uint8_t reg = (port == MCP23S17_PORT_A) ? MCP23S17_IPOL_A : MCP23S17_IPOL_B;
    return mcp23s17_write_byte(device, reg, polarity_mask);
}

// Legacy compatibility functions
int mcp23s17_legacy_init_gpio(uint8_t pins[], uint8_t pin_count)
{
    return mcp23s17_init_cs_pins(pins, pin_count);
}

int mcp23s17_legacy_init(spi_inst_t *spi_inst, uint8_t cs_pin)
{
    // Simple initialization without device address (assumes address 0)
    gpio_put(cs_pin, false);
    uint8_t command[] = {MCP23S17_OPCODE_WRITE, MCP23S17_IOCON_A, 0x08}; // HAEN=1
    spi_write_blocking(spi_inst, command, 3);
    gpio_put(cs_pin, true);
    return PICO_OK;
}

int mcp23s17_legacy_read_byte(spi_inst_t *spi_inst, uint8_t cs_pin, uint8_t reg, uint8_t *value)
{
    if (!value)
    {
        return PICO_ERROR_INVALID_ARG;
    }

    uint8_t command[] = {MCP23S17_OPCODE_READ, reg};
    gpio_put(cs_pin, false);
    spi_write_blocking(spi_inst, command, 2);
    spi_read_blocking(spi_inst, 0x00, value, 1);
    gpio_put(cs_pin, true);

    return PICO_OK;
}

int mcp23s17_legacy_write_byte(spi_inst_t *spi_inst, uint8_t cs_pin, uint8_t reg, uint8_t value)
{
    uint8_t command[] = {MCP23S17_OPCODE_WRITE, reg, value};
    gpio_put(cs_pin, false);
    spi_write_blocking(spi_inst, command, 3);
    gpio_put(cs_pin, true);

    return PICO_OK;
}

int mcp23s17_legacy_read_word(spi_inst_t *spi_inst, uint8_t cs_pin,
                              uint8_t low_reg, uint8_t high_reg, uint16_t *value)
{
    if (!value)
    {
        return PICO_ERROR_INVALID_ARG;
    }

    uint8_t low_byte, high_byte;

    int result = mcp23s17_legacy_read_byte(spi_inst, cs_pin, low_reg, &low_byte);
    if (result != PICO_OK)
    {
        return result;
    }

    result = mcp23s17_legacy_read_byte(spi_inst, cs_pin, high_reg, &high_byte);
    if (result != PICO_OK)
    {
        return result;
    }

    *value = ((uint16_t)high_byte << 8) | low_byte;
    return PICO_OK;
}
