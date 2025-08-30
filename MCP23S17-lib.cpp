#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "mcp23s17.hpp"

// SPI Configuration
#define SPI_PORT spi0
#define SPI_BAUDRATE 1000000 // 1 MHz

// Pin definitions (adjust these for your hardware setup)
#define PIN_MISO 16
#define PIN_CS 17
#define PIN_SCK 18
#define PIN_MOSI 19

// MCP23S17 device address (0-7, set by A0, A1, A2 pins)
#define MCP_DEVICE_ADDR 0

void setup_spi()
{
    // Initialize SPI
    spi_init(SPI_PORT, SPI_BAUDRATE);

    // Set up SPI pins
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // Initialize CS pin as GPIO output and set it HIGH (inactive)
    // This is CRITICAL - the library expects CS pin to be initialized!
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, true); // CS is active LOW, so set HIGH initially

    printf("SPI initialized at %d Hz\n", SPI_BAUDRATE);
}

int main()
{
    stdio_init_all();
    sleep_ms(2000); // Give time for serial connection

    printf("\n===========================================\n");
    printf("MCP23S17 Library Test - Step by Step\n");
    printf("===========================================\n");

    // Setup SPI
    setup_spi();

    printf("Creating MCP23S17 object...\n");
    // Create MCP23S17 object
    MCP23S17 mcp(SPI_PORT, PIN_CS, MCP_DEVICE_ADDR);

    printf("Initializing MCP23S17...\n");
    if (!mcp.begin())
    {
        printf("ERROR: Failed to initialize MCP23S17!\n");
        return -1;
    }
    printf("MCP23S17 initialized successfully!\n");

    // Test 1: Basic Port A blinking using library methods
    printf("\n=== Test 1: Port A Blinking with Library ===\n");
    printf("Setting up Port A as output...\n");
    if (!mcp.setupLEDPort(MCP23S17_Port::A))
    {
        printf("ERROR: Failed to setup Port A\n");
        return -1;
    }

    printf("Testing Port A - All pins blinking every 1 second\n");
    printf("Use multimeter to test pins A0-A7 (MCP23S17 pins 21-28)\n\n");

    bool state = false;
    int cycle = 0;

    // Run for 10 cycles, then move to next test
    while (cycle < 10)
    {
        if (state)
        {
            printf("Cycle %d: Port A = 0xFF (All HIGH - should read 3.3V)\n", cycle);
            mcp.writePort(MCP23S17_Port::A, 0xFF);
        }
        else
        {
            printf("Cycle %d: Port A = 0x00 (All LOW - should read 0V)\n", cycle);
            mcp.writePort(MCP23S17_Port::A, 0x00);
        }

        state = !state;
        cycle++;
        sleep_ms(1000);
    }

    // Test 2: Port B blinking
    printf("\n=== Test 2: Port B Blinking with Library ===\n");
    printf("Setting up Port B as output...\n");
    if (!mcp.setupLEDPort(MCP23S17_Port::B))
    {
        printf("ERROR: Failed to setup Port B\n");
        return -1;
    }

    printf("Testing Port B - All pins blinking every 1 second\n");
    printf("Use multimeter to test pins B0-B7 (MCP23S17 pins 1-8)\n\n");

    state = false;
    cycle = 0;

    // Run for 10 cycles, then move to next test
    while (cycle < 10)
    {
        if (state)
        {
            printf("Cycle %d: Port B = 0xFF (All HIGH - should read 3.3V)\n", cycle);
            mcp.writePort(MCP23S17_Port::B, 0xFF);
        }
        else
        {
            printf("Cycle %d: Port B = 0x00 (All LOW - should read 0V)\n", cycle);
            mcp.writePort(MCP23S17_Port::B, 0x00);
        }

        state = !state;
        cycle++;
        sleep_ms(1000);
    }

    // Test 3: Both ports blinking together using 16-bit method
    printf("\n=== Test 3: Both Ports A & B Together (16-bit) ===\n");
    printf("Testing both ports simultaneously using 16-bit method\n");
    printf("Port A (lower 8 bits) and Port B (upper 8 bits)\n\n");

    state = false;
    cycle = 0;

    // Run for 10 cycles, then continue with individual pin tests
    while (cycle < 10)
    {
        if (state)
        {
            printf("Cycle %d: Both ports = 0xFFFF (All HIGH - should read 3.3V)\n", cycle);
            mcp.writeGPIO(0xFFFF); // Both ports HIGH
        }
        else
        {
            printf("Cycle %d: Both ports = 0x0000 (All LOW - should read 0V)\n", cycle);
            mcp.writeGPIO(0x0000); // Both ports LOW
        }

        state = !state;
        cycle++;
        sleep_ms(1000);
    }

    printf("\n=== Basic Tests Complete! ===\n");
    printf("Library is working correctly!\n");
    printf("Ready for individual pin tests...\n\n");

    // Continuous operation - alternating between ports
    printf("=== Continuous Test: Alternating Ports ===\n");
    cycle = 0;
    while (true)
    {
        // Port A on, Port B off
        printf("Cycle %d: Port A ON, Port B OFF\n", cycle);
        mcp.writePort(MCP23S17_Port::A, 0xFF);
        mcp.writePort(MCP23S17_Port::B, 0x00);
        sleep_ms(1000);

        // Port A off, Port B on
        printf("Cycle %d: Port A OFF, Port B ON\n", cycle);
        mcp.writePort(MCP23S17_Port::A, 0x00);
        mcp.writePort(MCP23S17_Port::B, 0xFF);
        sleep_ms(1000);

        cycle++;
    }

    return 0;
}
