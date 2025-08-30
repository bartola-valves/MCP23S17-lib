#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "mcp23s17.hpp"

// SPI Configuration
#define SPI_PORT spi0
#define SPI_BAUDRATE 1000000 // 1 MHz

// Pin definitions (adjust these for your hardware setup)
#define PIN_MISO 16
#define PIN_CS 17
#define PIN_SCK 18
#define PIN_MOSI 19

// Interrupt pins - connect these to MCP23S17 INTA and INTB
#define PIN_INTA 20 // Connect to MCP23S17 INTA (pin 19)
#define PIN_INTB 21 // Connect to MCP23S17 INTB (pin 20)

// MCP23S17 device address (0-7, set by A0, A1, A2 pins)
#define MCP_DEVICE_ADDR 0

// Global variables for interrupt handling
volatile bool interrupt_occurred = false;
volatile bool inta_triggered = false;
volatile bool intb_triggered = false;
MCP23S17 *global_mcp = nullptr;

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

// INTA interrupt handler
void inta_interrupt_handler(uint gpio, uint32_t events)
{
    if (gpio == PIN_INTA && (events & GPIO_IRQ_EDGE_FALL))
    {
        inta_triggered = true;
        interrupt_occurred = true;
    }
}

// INTB interrupt handler
void intb_interrupt_handler(uint gpio, uint32_t events)
{
    if (gpio == PIN_INTB && (events & GPIO_IRQ_EDGE_FALL))
    {
        intb_triggered = true;
        interrupt_occurred = true;
    }
}

void setup_interrupt_pins()
{
    // Setup INTA pin (Port A interrupts)
    gpio_init(PIN_INTA);
    gpio_set_dir(PIN_INTA, GPIO_IN);
    gpio_pull_up(PIN_INTA); // Pull-up because MCP23S17 INTA is open-drain

    // Setup INTB pin (Port B interrupts)
    gpio_init(PIN_INTB);
    gpio_set_dir(PIN_INTB, GPIO_IN);
    gpio_pull_up(PIN_INTB); // Pull-up because MCP23S17 INTB is open-drain

    // Enable interrupts on falling edge (active low)
    gpio_set_irq_enabled_with_callback(PIN_INTA, GPIO_IRQ_EDGE_FALL, true, &inta_interrupt_handler);
    gpio_set_irq_enabled_with_callback(PIN_INTB, GPIO_IRQ_EDGE_FALL, true, &intb_interrupt_handler);

    printf("Interrupt pins configured:\n");
    printf("  INTA: GPIO %d (connect to MCP23S17 pin 19)\n", PIN_INTA);
    printf("  INTB: GPIO %d (connect to MCP23S17 pin 20)\n", PIN_INTB);
}

int main()
{
    stdio_init_all();
    sleep_ms(2000); // Give time for serial connection

    printf("\n===========================================\n");
    printf("MCP23S17 Interrupt Test - INTA/INTB Demo\n");
    printf("===========================================\n");

    // Setup SPI and interrupt pins
    setup_spi();
    setup_interrupt_pins();

    printf("Creating MCP23S17 object...\n");
    // Create MCP23S17 object
    MCP23S17 mcp(SPI_PORT, PIN_CS, MCP_DEVICE_ADDR);
    global_mcp = &mcp; // Store global reference

    printf("Initializing MCP23S17...\n");
    if (!mcp.begin())
    {
        printf("ERROR: Failed to initialize MCP23S17!\n");
        return -1;
    }
    printf("MCP23S17 initialized successfully!\n");

    // Configure interrupt output (active low, separate INTA/INTB)
    printf("Configuring interrupt output...\n");
    if (!mcp.configureInterruptOutput(false, false))
    { // No mirroring, active low
        printf("ERROR: Failed to configure interrupt output!\n");
        return -1;
    }

    // Setup Port A as outputs for LEDs (to show interrupt feedback)
    printf("Setting up Port A as outputs for LED feedback...\n");
    if (!mcp.setupLEDPort(MCP23S17_Port::A))
    {
        printf("ERROR: Failed to setup Port A\n");
        return -1;
    }

    // Setup Port B as inputs with pull-ups for switches
    printf("Setting up Port B as inputs with pull-ups...\n");
    if (!mcp.setupButtonPort(MCP23S17_Port::B))
    {
        printf("ERROR: Failed to setup Port B\n");
        return -1;
    }

    // Enable interrupt on GPB0 (Port B, pin 0)
    printf("Enabling interrupt on Port B pin 0 (GPB0)...\n");
    if (!mcp.enablePinInterrupt(MCP23S17_Port::B, 0, true))
    {
        printf("ERROR: Failed to enable interrupt on GPB0\n");
        return -1;
    }

    // Set interrupt to trigger on any change (not comparison mode)
    printf("Configuring interrupt for change detection...\n");
    if (!mcp.setInterruptControl(MCP23S17_Port::B, 0x00))
    { // 0=change from previous
        printf("ERROR: Failed to set interrupt control\n");
        return -1;
    }

    // Clear any existing interrupts and read initial state
    printf("Clearing any existing interrupts...\n");
    mcp.clearInterrupts(MCP23S17_Port::A);
    mcp.clearInterrupts(MCP23S17_Port::B);

    // Read initial port state to establish baseline
    uint8_t initial_state = mcp.readPort(MCP23S17_Port::B);
    printf("Initial Port B state: 0x%02X\n", initial_state);

    printf("\n=== Interrupt Test Ready ===\n");
    printf("Hardware setup required:\n");
    printf("1. Connect MCP23S17 pin 19 (INTA) to Pico GPIO %d\n", PIN_INTA);
    printf("2. Connect MCP23S17 pin 20 (INTB) to Pico GPIO %d\n", PIN_INTB);
    printf("3. Connect a switch between GPB0 (MCP23S17 pin 1) and GND\n");
    printf("4. Leave other Port B pins unconnected (they have pull-ups)\n");
    printf("5. Press the switch to trigger interrupts!\n\n");

    printf("Monitoring interrupts... (Press GPB0 switch)\n");

    int interrupt_count = 0;
    bool led_state = false;

    while (true)
    {
        // Check if interrupt occurred
        if (interrupt_occurred)
        {
            interrupt_occurred = false; // Clear flag
            interrupt_count++;

            printf("\n*** INTERRUPT #%d DETECTED! ***\n", interrupt_count);

            // Check which interrupt triggered
            if (intb_triggered)
            {
                intb_triggered = false;
                printf("INTB triggered (Port B interrupt)\n");

                // Read interrupt flags to see which pin caused it
                uint8_t int_flags = mcp.getInterruptFlags(MCP23S17_Port::B);
                printf("Interrupt flags (Port B): 0x%02X\n", int_flags);

                // Only process if we have valid interrupt flags
                if (int_flags != 0x00)
                {
                    // Read captured values at time of interrupt
                    uint8_t int_capture = mcp.getInterruptCapture(MCP23S17_Port::B);
                    printf("Captured values (Port B): 0x%02X\n", int_capture);

                    // Check if GPB0 caused the interrupt
                    if (int_flags & 0x01)
                    {
                        bool pin_state = (int_capture & 0x01) == 0;
                        printf("GPB0 switch %s (pin state: %s)\n",
                               pin_state ? "PRESSED" : "RELEASED",
                               pin_state ? "LOW" : "HIGH");

                        // Toggle LED on Port A as visual feedback
                        led_state = !led_state;
                        mcp.writePin(MCP23S17_Port::A, 0, led_state);
                        printf("LED on GPA0 turned %s\n", led_state ? "ON" : "OFF");
                    }

                    // Check for other pins (noise detection)
                    if (int_flags & 0xFE) // Any other pins besides GPB0
                    {
                        printf("WARNING: Other pins triggered interrupt (noise?): 0x%02X\n",
                               int_flags & 0xFE);
                    }
                }
                else
                {
                    printf("WARNING: Spurious interrupt (flags = 0x00) - likely noise or timing issue\n");
                }

                // Clear interrupts
                mcp.clearInterrupts(MCP23S17_Port::B);
                printf("Interrupts cleared\n");
            }

            if (inta_triggered)
            {
                inta_triggered = false;
                printf("INTA triggered (Port A interrupt)\n");
                mcp.clearInterrupts(MCP23S17_Port::A);
            }

            printf("Ready for next interrupt...\n");
        }

        // Small delay to prevent busy waiting
        sleep_ms(10);

        // Optional: Show current port states every 5 seconds
        static uint32_t last_status_time = 0;
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        if (current_time - last_status_time > 5000)
        {
            last_status_time = current_time;
            uint8_t portb_state = mcp.readPort(MCP23S17_Port::B);
            printf("Status: Port B = 0x%02X, Interrupts received: %d\n",
                   portb_state, interrupt_count);
        }
    }

    return 0;
}
