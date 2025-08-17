#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "mcp23s17.hpp"

// Simple example showing basic MCP23S17 usage
// This example assumes the following connections:
// - SPI0 MISO: GPIO 16
// - SPI0 CS:   GPIO 17
// - SPI0 SCK:  GPIO 18
// - SPI0 MOSI: GPIO 19
// - MCP23S17 A2, A1, A0 pins tied to GND (device address = 0)

#define SPI_PORT spi0
#define PIN_CS 17

int main()
{
    stdio_init_all();

    // Initialize SPI
    spi_init(SPI_PORT, 1000000);          // 1 MHz
    gpio_set_function(16, GPIO_FUNC_SPI); // MISO
    gpio_set_function(18, GPIO_FUNC_SPI); // SCK
    gpio_set_function(19, GPIO_FUNC_SPI); // MOSI

    // Create and initialize MCP23S17
    MCP23S17 mcp(SPI_PORT, PIN_CS, 0); // Address 0
    if (!mcp.begin())
    {
        printf("Failed to initialize MCP23S17!\n");
        return -1;
    }

    // Configure Port A as LED outputs, Port B as button inputs
    mcp.setupLEDPort(MCP23S17_Port::A);
    mcp.setupButtonPort(MCP23S17_Port::B);

    printf("MCP23S17 Simple Example\n");
    printf("Port A: LED outputs, Port B: Button inputs\n");

    while (true)
    {
        // Blink LEDs on Port A
        for (int i = 0; i < 8; i++)
        {
            uint8_t pattern = 1 << i;
            mcp.writePort(MCP23S17_Port::A, pattern);

            // Read Port B
            uint8_t input_value = mcp.readPort(MCP23S17_Port::B);
            printf("LED: 0x%02X, Buttons: 0x%02X\n", pattern, input_value);

            sleep_ms(250);
        }
    }

    return 0;
}
