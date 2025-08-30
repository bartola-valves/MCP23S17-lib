#include "mcp23s17.hpp"

MCP23S17::MCP23S17(spi_inst_t *spi_inst, uint8_t cs_pin, uint8_t device_addr)
    : spi_inst_(spi_inst), cs_pin_(cs_pin), device_addr_(device_addr), initialized_(false)
{
    if (device_addr > 7)
    {
        device_addr_ = 0; // Clamp to valid range
    }
    initCSPin(cs_pin_);
}

bool MCP23S17::begin()
{
    if (!spi_inst_)
    {
        return false;
    }

    // Configure IOCON register for default operation
    // BANK=0, MIRROR=0, SEQOP=0, DISSLW=0, HAEN=1, ODR=0, INTPOL=0
    uint8_t iocon_value = 0x08; // Enable hardware addressing (HAEN=1)

    if (!writeRegister(MCP23S17_IOCON_A, iocon_value))
    {
        return false;
    }

    // Write to IOCON_B as well to ensure consistency
    if (!writeRegister(MCP23S17_IOCON_B, iocon_value))
    {
        return false;
    }

    initialized_ = true;
    return true;
}

int MCP23S17::spiTransaction(uint8_t opcode, uint8_t reg, uint8_t *data, uint8_t data_len, bool is_read)
{
    if (!data)
    {
        return PICO_ERROR_INVALID_ARG;
    }

    // Calculate full opcode with device address
    uint8_t full_opcode = opcode | ((device_addr_ & 0x07) << 1);

    // Select device (CS active low)
    gpio_put(cs_pin_, false);

    // Send opcode and register address
    uint8_t command[2] = {full_opcode, reg};
    int result = spi_write_blocking(spi_inst_, command, 2);
    if (result < 0)
    {
        gpio_put(cs_pin_, true);
        return result;
    }

    // Handle data transfer
    if (is_read)
    {
        result = spi_read_blocking(spi_inst_, 0x00, data, data_len);
    }
    else
    {
        result = spi_write_blocking(spi_inst_, data, data_len);
    }

    // Deselect device
    gpio_put(cs_pin_, true);

    return (result < 0) ? result : PICO_OK;
}

uint8_t MCP23S17::readRegister(uint8_t reg)
{
    uint8_t value = 0xFF;
    spiTransaction(MCP23S17_OPCODE_READ, reg, &value, 1, true);
    return value;
}

bool MCP23S17::writeRegister(uint8_t reg, uint8_t value)
{
    return spiTransaction(MCP23S17_OPCODE_WRITE, reg, &value, 1, false) == PICO_OK;
}

uint16_t MCP23S17::readRegisterPair(uint8_t low_reg, uint8_t high_reg)
{
    uint8_t low_byte = readRegister(low_reg);
    uint8_t high_byte = readRegister(high_reg);
    return ((uint16_t)high_byte << 8) | low_byte;
}

bool MCP23S17::writeRegisterPair(uint8_t low_reg, uint8_t high_reg, uint16_t value)
{
    bool success = writeRegister(low_reg, (uint8_t)(value & 0xFF));
    success &= writeRegister(high_reg, (uint8_t)(value >> 8));
    return success;
}

bool MCP23S17::setPortDirection(MCP23S17_Port port, uint8_t direction)
{
    uint8_t reg = (port == MCP23S17_Port::A) ? MCP23S17_IODIR_A : MCP23S17_IODIR_B;
    return writeRegister(reg, direction);
}

bool MCP23S17::setPinDirection(MCP23S17_Port port, uint8_t pin, bool is_input)
{
    if (pin > 7)
    {
        return false;
    }

    uint8_t reg = (port == MCP23S17_Port::A) ? MCP23S17_IODIR_A : MCP23S17_IODIR_B;
    uint8_t current_value = readRegister(reg);

    if (is_input)
    {
        current_value |= (1 << pin); // Set bit for input
    }
    else
    {
        current_value &= ~(1 << pin); // Clear bit for output
    }

    return writeRegister(reg, current_value);
}

bool MCP23S17::setPortPullups(MCP23S17_Port port, uint8_t pullup_mask)
{
    uint8_t reg = (port == MCP23S17_Port::A) ? MCP23S17_GPPU_A : MCP23S17_GPPU_B;
    return writeRegister(reg, pullup_mask);
}

bool MCP23S17::setPinPullup(MCP23S17_Port port, uint8_t pin, bool enable)
{
    if (pin > 7)
    {
        return false;
    }

    uint8_t reg = (port == MCP23S17_Port::A) ? MCP23S17_GPPU_A : MCP23S17_GPPU_B;
    uint8_t current_value = readRegister(reg);

    if (enable)
    {
        current_value |= (1 << pin); // Set bit to enable pull-up
    }
    else
    {
        current_value &= ~(1 << pin); // Clear bit to disable pull-up
    }

    return writeRegister(reg, current_value);
}

bool MCP23S17::setPortPolarity(MCP23S17_Port port, uint8_t polarity_mask)
{
    uint8_t reg = (port == MCP23S17_Port::A) ? MCP23S17_IPOL_A : MCP23S17_IPOL_B;
    return writeRegister(reg, polarity_mask);
}

uint8_t MCP23S17::readPort(MCP23S17_Port port)
{
    uint8_t reg = (port == MCP23S17_Port::A) ? MCP23S17_GPIO_A : MCP23S17_GPIO_B;
    return readRegister(reg);
}

bool MCP23S17::writePort(MCP23S17_Port port, uint8_t value)
{
    uint8_t reg = (port == MCP23S17_Port::A) ? MCP23S17_GPIO_A : MCP23S17_GPIO_B;
    return writeRegister(reg, value);
}

uint16_t MCP23S17::readGPIO()
{
    return readRegisterPair(MCP23S17_GPIO_A, MCP23S17_GPIO_B);
}

bool MCP23S17::writeGPIO(uint16_t value)
{
    return writeRegisterPair(MCP23S17_GPIO_A, MCP23S17_GPIO_B, value);
}

bool MCP23S17::readPin(MCP23S17_Port port, uint8_t pin)
{
    if (pin > 7)
    {
        return false;
    }

    uint8_t port_value = readPort(port);
    return (port_value & (1 << pin)) != 0;
}

bool MCP23S17::writePin(MCP23S17_Port port, uint8_t pin, bool state)
{
    if (pin > 7)
    {
        return false;
    }

    uint8_t reg = (port == MCP23S17_Port::A) ? MCP23S17_GPIO_A : MCP23S17_GPIO_B;
    uint8_t current_value = readRegister(reg);

    if (state)
    {
        current_value |= (1 << pin); // Set bit
    }
    else
    {
        current_value &= ~(1 << pin); // Clear bit
    }

    return writeRegister(reg, current_value);
}

bool MCP23S17::togglePin(MCP23S17_Port port, uint8_t pin)
{
    if (pin > 7)
    {
        return false;
    }

    uint8_t reg = (port == MCP23S17_Port::A) ? MCP23S17_GPIO_A : MCP23S17_GPIO_B;
    uint8_t current_value = readRegister(reg);

    current_value ^= (1 << pin); // Toggle bit

    return writeRegister(reg, current_value);
}

void MCP23S17::initCSPin(uint8_t cs_pin)
{
    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_put(cs_pin, true); // Deselect (active low)
}

bool MCP23S17::setupLEDPort(MCP23S17_Port port)
{
    // Configure as outputs
    if (!setPortDirection(port, MCP23S17_OUTPUT))
    {
        return false;
    }

    // Disable pull-ups (not needed for outputs)
    if (!setPortPullups(port, MCP23S17_PULLUP_DISABLE))
    {
        return false;
    }

    // Set initial state to all off
    return writePort(port, 0x00);
}

bool MCP23S17::setupButtonPort(MCP23S17_Port port)
{
    // Configure as inputs
    if (!setPortDirection(port, MCP23S17_INPUT))
    {
        return false;
    }

    // Enable pull-ups
    return setPortPullups(port, MCP23S17_PULLUP_ENABLE);
}

bool MCP23S17::runningLights(MCP23S17_Port port, uint32_t delay_ms, int cycles)
{
    if (cycles < 0)
    {
        return false;
    }

    for (int cycle = 0; cycle < cycles; cycle++)
    {
        // Forward direction
        for (int i = 0; i < 8; i++)
        {
            if (!writePort(port, 1 << i))
            {
                return false;
            }
            sleep_ms(delay_ms);
        }

        // Backward direction (skip last position to avoid double-flash)
        for (int i = 6; i >= 1; i--)
        {
            if (!writePort(port, 1 << i))
            {
                return false;
            }
            sleep_ms(delay_ms);
        }
    }

    // Turn off all LEDs
    return writePort(port, 0x00);
}

bool MCP23S17::binaryCounter(MCP23S17_Port port, uint32_t delay_ms, uint8_t max_count)
{
    for (uint8_t count = 0; count <= max_count; count++)
    {
        if (!writePort(port, count))
        {
            return false;
        }
        sleep_ms(delay_ms);
    }

    return true;
}

// Legacy compatibility functions
int MCP23S17::legacy_read_byte(uint8_t reg, uint8_t *value)
{
    if (!value)
    {
        return PICO_ERROR_INVALID_ARG;
    }

    *value = readRegister(reg);
    return PICO_OK;
}

int MCP23S17::legacy_write_byte(uint8_t reg, uint8_t value)
{
    return writeRegister(reg, value) ? PICO_OK : PICO_ERROR_GENERIC;
}
