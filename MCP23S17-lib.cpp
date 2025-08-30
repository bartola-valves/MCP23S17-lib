#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

// SPI Configuration
#define SPI_PORT spi0
#define SPI_BAUDRATE 1000000 // 1 MHz

// Pin definitions (adjust these for your hardware setup)
#define PIN_MISO 16
#define PIN_CS 17
#define PIN_SCK 18
#define PIN_MOSI 19

void setup_spi()
{
    // Initialize SPI
    spi_init(SPI_PORT, SPI_BAUDRATE);

    // Set up SPI pins
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // Initialize CS pin as GPIO output and set it HIGH (inactive)
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, true); // CS is active LOW, so set HIGH initially

    printf("SPI initialized at %d Hz\n", SPI_BAUDRATE);
}

int main()
{
    stdio_init_all();
    sleep_ms(2000); // Give time for serial connection

    printf("\n=================================\n");
    printf("MCP23S17 Simple Multimeter Test\n");
    printf("=================================\n");

    // Setup SPI
    setup_spi();

    printf("Initializing MCP23S17...\n");
    sleep_ms(1000); // Give MCP23S17 time to power up completely

    // Enable HAEN (Hardware Address Enable) bit via direct SPI
    gpio_put(PIN_CS, false);
    uint8_t iocon_cmd[] = {0x40, 0x0A, 0x28}; // Enable HAEN bit
    spi_write_blocking(SPI_PORT, iocon_cmd, 3);
    gpio_put(PIN_CS, true);
    sleep_ms(10);

    // Set Port A as all outputs via direct SPI
    gpio_put(PIN_CS, false);
    uint8_t set_outputs[] = {0x40, 0x00, 0x00}; // IODIR_A = 0x00 (all outputs)
    spi_write_blocking(SPI_PORT, set_outputs, 3);
    gpio_put(PIN_CS, true);
    sleep_ms(10);

    printf("MCP23S17 initialized successfully!\n");
    printf("Starting Port A blink test...\n");
    printf("All pins will toggle every 1 second.\n");
    printf("Use multimeter to test pins A0-A7 (MCP23S17 pins 21-28).\n\n");

    bool state = false;
    int cycle = 0;

    while (true)
    {
        if (state)
        {
            printf("Cycle %d: Port A = 0xFF (All HIGH - should read 3.3V)\n", cycle);
            gpio_put(PIN_CS, false);
            uint8_t high_cmd[] = {0x40, 0x12, 0xFF}; // GPIO_A = 0xFF
            spi_write_blocking(SPI_PORT, high_cmd, 3);
            gpio_put(PIN_CS, true);
        }
        else
        {
            printf("Cycle %d: Port A = 0x00 (All LOW - should read 0V)\n", cycle);
            gpio_put(PIN_CS, false);
            uint8_t low_cmd[] = {0x40, 0x12, 0x00}; // GPIO_A = 0x00
            spi_write_blocking(SPI_PORT, low_cmd, 3);
            gpio_put(PIN_CS, true);
        }

        state = !state; // Toggle state
        cycle++;
        sleep_ms(1000); // Wait 1 second
    }

    return 0;
}
