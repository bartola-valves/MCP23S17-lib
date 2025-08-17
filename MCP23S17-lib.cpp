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

// MCP23S17 device address (based on A2, A1, A0 hardware pins)
#define MCP23S17_ADDR 0x00 // All address pins connected to GND

void setup_spi()
{
    // Initialize SPI
    spi_init(SPI_PORT, SPI_BAUDRATE);

    // Set up SPI pins
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    printf("SPI initialized at %d Hz\n", SPI_BAUDRATE);
}

void demo_basic_operations(MCP23S17 &mcp)
{
    printf("\n=== Basic I/O Operations Demo ===\n");

    // Configure Port A as outputs and Port B as inputs with pull-ups
    printf("Configuring Port A as LED outputs, Port B as button inputs...\n");

    mcp.setupLEDPort(MCP23S17_Port::A);
    mcp.setupButtonPort(MCP23S17_Port::B);

    // Test pattern on Port A
    printf("Running LED pattern on Port A...\n");
    for (int pattern = 0; pattern < 3; pattern++)
    {
        for (int i = 0; i < 8; i++)
        {
            uint8_t value = 1 << i; // Shift bit pattern
            mcp.writePort(MCP23S17_Port::A, value);
            printf("Port A: 0x%02X\n", value);

            // Read Port B
            uint8_t port_b_value = mcp.readPort(MCP23S17_Port::B);
            printf("Port B: 0x%02X\n", port_b_value);

            sleep_ms(200);
        }
    }

    // Turn off all LEDs
    mcp.writePort(MCP23S17_Port::A, 0x00);
}

void demo_individual_pins(MCP23S17 &mcp)
{
    printf("\n=== Individual Pin Control Demo ===\n");

    // Test individual pin control
    printf("Testing individual pin control on Port A...\n");

    for (int pin = 0; pin < 8; pin++)
    {
        printf("Setting pin %d high\n", pin);
        mcp.writePin(MCP23S17_Port::A, pin, true);
        sleep_ms(300);

        printf("Setting pin %d low\n", pin);
        mcp.writePin(MCP23S17_Port::A, pin, false);
        sleep_ms(100);
    }

    // Test pin toggling
    printf("Testing pin toggling...\n");
    for (int i = 0; i < 10; i++)
    {
        mcp.togglePin(MCP23S17_Port::A, 0);
        mcp.togglePin(MCP23S17_Port::A, 7);
        sleep_ms(200);
    }

    // Turn off pins
    mcp.writePin(MCP23S17_Port::A, 0, false);
    mcp.writePin(MCP23S17_Port::A, 7, false);
}

void demo_16bit_operations(MCP23S17 &mcp)
{
    printf("\n=== 16-bit Operations Demo ===\n");

    // Configure both ports as outputs
    mcp.setupLEDPort(MCP23S17_Port::A);
    mcp.setupLEDPort(MCP23S17_Port::B);

    // Test 16-bit patterns
    uint16_t patterns[] = {
        0x0001, 0x0002, 0x0004, 0x0008,
        0x0010, 0x0020, 0x0040, 0x0080,
        0x0100, 0x0200, 0x0400, 0x0800,
        0x1000, 0x2000, 0x4000, 0x8000,
        0xFFFF, 0x0000, 0xAAAA, 0x5555};

    printf("Running 16-bit patterns...\n");
    for (size_t i = 0; i < sizeof(patterns) / sizeof(patterns[0]); i++)
    {
        printf("Pattern: 0x%04X\n", patterns[i]);
        mcp.writeGPIO(patterns[i]);

        // Read back and verify
        uint16_t readback = mcp.readGPIO();
        printf("Readback: 0x%04X %s\n", readback,
               (readback == patterns[i]) ? "✓" : "✗");

        sleep_ms(500);
    }

    // Clear all outputs
    mcp.writeGPIO(0x0000);
}

void demo_running_patterns(MCP23S17 &mcp)
{
    printf("\n=== Pattern Demo ===\n");

    // Setup Port A for LEDs
    mcp.setupLEDPort(MCP23S17_Port::A);

    printf("Running lights pattern...\n");
    mcp.runningLights(MCP23S17_Port::A, 150, 2);

    sleep_ms(1000);

    printf("Binary counter pattern...\n");
    mcp.binaryCounter(MCP23S17_Port::A, 100, 31); // Count to 31

    // Clear
    mcp.writePort(MCP23S17_Port::A, 0x00);
}

void demo_input_monitoring(MCP23S17 &mcp)
{
    printf("\n=== Input Monitoring Demo ===\n");
    printf("Configure Port B as inputs and monitor for changes...\n");
    printf("Connect switches or jumpers to Port B pins for testing.\n");

    // Configure Port B as inputs with pull-ups
    mcp.setupButtonPort(MCP23S17_Port::B);

    uint8_t last_value = 0xFF; // Assume all pins high initially

    // Monitor for 10 seconds
    absolute_time_t end_time = make_timeout_time_ms(10000);

    while (!time_reached(end_time))
    {
        uint8_t current_value = mcp.readPort(MCP23S17_Port::B);

        if (current_value != last_value)
        {
            printf("Port B changed: 0x%02X -> 0x%02X\n", last_value, current_value);

            // Show individual pin states
            for (int pin = 0; pin < 8; pin++)
            {
                bool current_state = (current_value & (1 << pin)) != 0;
                bool last_state = (last_value & (1 << pin)) != 0;

                if (current_state != last_state)
                {
                    printf("  Pin %d: %s -> %s\n", pin,
                           last_state ? "HIGH" : "LOW",
                           current_state ? "HIGH" : "LOW");
                }
            }

            last_value = current_value;
        }

        sleep_ms(50); // Check every 50ms
    }

    printf("Input monitoring finished.\n");
}

int main()
{
    stdio_init_all();

    printf("\n==================================\n");
    printf("MCP23S17 C++ Library Demo for Pico\n");
    printf("==================================\n");

    // Setup SPI
    setup_spi();

    // Create MCP23S17 instance
    printf("Creating MCP23S17 instance...\n");
    MCP23S17 mcp(SPI_PORT, PIN_CS, MCP23S17_ADDR);

    // Initialize the device
    printf("Initializing MCP23S17...\n");
    if (!mcp.begin())
    {
        printf("Failed to initialize MCP23S17!\n");
        return -1;
    }

    printf("MCP23S17 initialized successfully!\n");

    while (true)
    {
        printf("\nStarting demo sequence...\n");

        // Run demonstration sequence
        demo_basic_operations(mcp);
        sleep_ms(1000);

        demo_individual_pins(mcp);
        sleep_ms(1000);

        demo_16bit_operations(mcp);
        sleep_ms(1000);

        demo_running_patterns(mcp);
        sleep_ms(1000);

        demo_input_monitoring(mcp);
        sleep_ms(2000);

        printf("\nDemo sequence complete. Repeating in 3 seconds...\n");
        sleep_ms(3000);
    }

    return 0;
}
