/*
 * Simple ADC/Protothreads demo integrated with Keypad
 *
 * Scans a 3x4 matrix keypad with debouncing.
 * When key '0' is pressed and held, plays a sine wave tone via DAC (SPI).
 * Sine wave frequency is dynamically updated via ADC input on GPIO 26
 * (defaulting to 440 Hz if ADC is 0V/unconnected).
 */

#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include "hardware/sync.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// ==========================================
// === protothreads globals
// ==========================================
#include "pt_cornell_rp2040_v1_4.h"

// Pin definitions
#define LED_PIN         25
#define ADC_PIN         26
#define ADC_MUX         0

// Low-level alarm infrastructure
#define ALARM_NUM       0
#define ALARM_IRQ       timer_hardware_alarm_get_irq_num(timer_hw, ALARM_NUM)

// DDS parameters
#define two32           4294967296.0 // 2^32 
#define Fs              50000        // Sample rate (50 kHz)
#define DELAY           20           // 1/Fs in microseconds

// DDS units:
volatile unsigned int phase_accum_main = 0;
volatile unsigned int phase_incr_main = 0;
volatile bool play_tone = false;
volatile unsigned int current_adc_val = 0;
volatile unsigned int current_freq = 440;

// SPI data and DAC configurations (MCP4822 / MCP4802)
uint16_t DAC_data;
#define DAC_config_chan_A 0b0011000000000000 // A-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000 // B-channel, 1x, active

// SPI configurations
#define PIN_MISO        4
#define PIN_CS          5
#define PIN_SCK         6
#define PIN_MOSI        7
#define SPI_PORT        spi0

// GPIO for timing the ISR
#define ISR_GPIO        2

// DDS sine table
#define sine_table_size 256
volatile int sin_table[sine_table_size];

// ==================================================
// === Keypad Configuration
// ==================================================
// Keypad connections:
// GPIO 9-12: Rows 1-4 (outputs)
// GPIO 13-15: Cols 1-3 (inputs with pull-ups)
#define BASE_KEYPAD_PIN 9
#define KEYROWS         4
#define NUMKEYS         12

// Keycodes mapped to key values (index 0 is key '0', 1-9 are '1'-'9', 10 is '*', 11 is '#')
const unsigned int keycodes[NUMKEYS] = {
    0x57, // '0' (Row 4, Col 2)
    0x6E, // '1' (Row 1, Col 1)
    0x5E, // '2' (Row 1, Col 2)
    0x3E, // '3' (Row 1, Col 3)
    0x6D, // '4' (Row 2, Col 1)
    0x5D, // '5' (Row 2, Col 2)
    0x3D, // '6' (Row 2, Col 3)
    0x6B, // '7' (Row 3, Col 1)
    0x5B, // '8' (Row 3, Col 2)
    0x3B, // '9' (Row 3, Col 3)
    0x67, // '*' (Row 4, Col 1)
    0x37  // '#' (Row 4, Col 3)
};

const unsigned int scancodes[KEYROWS] = { 0xE, 0xD, 0xB, 0x7 };
const unsigned int button_mask = 0x70; // Column bits mask (pins 13, 14, 15)

// Debouncing state machine states
enum debounce_state {
    STATE_NOT_PRESSED = 0,
    STATE_MAYBE_PRESSED,
    STATE_PRESSED,
    STATE_MAYBE_NOT_PRESSED
};

// Scan the 3x4 keypad matrix
static int scan_keypad(void) {
    int row, key_idx;
    uint32_t keypad_state = 0;

    for (row = 0; row < KEYROWS; row++) {
        // Set one row low, others high
        gpio_put_masked((0xF << BASE_KEYPAD_PIN), (scancodes[row] << BASE_KEYPAD_PIN));
        // Small delay for line capacitance to settle
        sleep_us(1);
        // Read the 7 keypad pins (bits 0-3 rows, bits 4-6 columns)
        keypad_state = ((gpio_get_all() >> BASE_KEYPAD_PIN) & 0x7F);

        // Break if any column is pulled low in this row
        if ((~keypad_state) & button_mask) {
            break;
        }
    }

    // Reset all rows back to high
    gpio_put_masked((0xF << BASE_KEYPAD_PIN), (0xF << BASE_KEYPAD_PIN));

    // If a button press was detected on any column
    if ((~keypad_state) & button_mask) {
        for (key_idx = 0; key_idx < NUMKEYS; key_idx++) {
            if (keypad_state == keycodes[key_idx]) {
                return key_idx;
            }
        }
        return -1; // Multiple keys / invalid code
    }

    return -1; // No button pressed
}

// ==================================================
// === ADC Protothread
// ==================================================
static PT_THREAD (protothread_adc(struct pt *pt))
{
    PT_BEGIN(pt);

    static unsigned int adc_val;

    while(1) {
        // Toggle onboard LED as system heartbeat
        gpio_put(LED_PIN, !gpio_get(LED_PIN));

        // Read the ADC
        adc_val = adc_read();
        current_adc_val = adc_val;

        // Determine frequency: use ADC reading or default to 440 Hz if 0
        current_freq = (adc_val > 0) ? adc_val : 440;
        phase_incr_main = (unsigned int)((current_freq * two32) / Fs);

        // Periodic yield (100 ms)
        PT_YIELD_usec(100000);
    }

    PT_END(pt);
}

// ==================================================
// === Keypad Protothread (with debouncing)
// ==================================================
static PT_THREAD (protothread_keypad(struct pt *pt))
{
    PT_BEGIN(pt);

    static int scanned_key;
    static int possible = -1;
    static enum debounce_state state = STATE_NOT_PRESSED;

    while(1) {
        scanned_key = scan_keypad();

        switch(state) {
            case STATE_NOT_PRESSED:
                if (scanned_key != -1) {
                    possible = scanned_key;
                    state = STATE_MAYBE_PRESSED;
                }
                break;

            case STATE_MAYBE_PRESSED:
                if (scanned_key == possible) {
                    state = STATE_PRESSED;
                    // Confirmed key press
                    if (possible == 0) {
                        play_tone = true;
                    } 

                } else {
                    state = STATE_NOT_PRESSED;
                }

                break;

            case STATE_PRESSED:
                if (scanned_key == possible) {
                    // Key is continuously held down
                    if (possible == 0) {
                        play_tone = true;
                    }
                } else {
                    state = STATE_MAYBE_NOT_PRESSED;
                }
                break;

            case STATE_MAYBE_NOT_PRESSED:
                if (scanned_key == possible) {
                    // Bounced back to pressed
                    state = STATE_PRESSED;
                    if (possible == 0) {
                        play_tone = true;
                    }
                } else {
                    state = STATE_NOT_PRESSED;
                    // Confirmed key release
                    if (possible == 0) {
                        play_tone = false;
                    }
                    possible = -1;
                }
                break;
        }

        // Scan every 30 ms
        PT_YIELD_usec(30000);
    }

    PT_END(pt);
}

// ==================================================
// === Alarm ISR (DDS Sample Output to DAC)
// ==================================================
static void alarm_irq(void) {
    // Assert timing GPIO
    gpio_put(ISR_GPIO, 1);

    // Clear alarm interrupt
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);

    // Reset alarm register
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

    if (play_tone) {
        // DDS phase accumulation and sine table lookup
        phase_accum_main += phase_incr_main;
        DAC_data = (DAC_config_chan_B | ((sin_table[phase_accum_main >> 24] + 2048) & 0xffff));
    } else {
        // Silence: hold phase at 0 and output midscale 2048 
        phase_accum_main = 0;
        DAC_data = (DAC_config_chan_B | 2048);
    }

    // Write to DAC via SPI
    spi_write16_blocking(SPI_PORT, &DAC_data, 1);

    // De-assert timing GPIO
    gpio_put(ISR_GPIO, 0);
}

// ==================================================
// === Main
// ==================================================
int main(void) {
    // Optional overclock to 150 MHz for RP2040 / default RP2350
    set_sys_clock_khz(150000, true);

    // Initialize stdio
    stdio_init_all();
    printf("\n\rProtothreads RP2040/RP2350 v1.4\n\r");
    printf("ADC + Keypad DDS Tone Generator\n\r");
    printf("Press and hold '0' on the keypad to play the sine wave tone.\n\r");

    // Initialize ADC
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(ADC_MUX);

    // Initialize LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, true);

    // Initialize SPI for DAC
    spi_init(SPI_PORT, 20000000);
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Setup ISR-timing GPIO
    gpio_init(ISR_GPIO);
    gpio_set_dir(ISR_GPIO, GPIO_OUT);
    gpio_put(ISR_GPIO, 0);

    // Map SPI signals to GPIO ports
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);

    // Initialize DDS sine table
    int ii;
    for (ii = 0; ii < sine_table_size; ii++) {
        sin_table[ii] = (int)(2047.0 * sin((double)ii * 6.283185307179586 / (double)sine_table_size));
    }

    // Default DDS frequency (440 Hz) and initial silence
    phase_accum_main = 0;
    phase_incr_main = (unsigned int)((440.0 * two32) / Fs);
    play_tone = false;

    // Initialize Keypad GPIOs
    gpio_init_mask((0x7F << BASE_KEYPAD_PIN));
    // Columns as inputs
    gpio_set_dir((BASE_KEYPAD_PIN + 4), GPIO_IN);
    gpio_set_dir((BASE_KEYPAD_PIN + 5), GPIO_IN);
    gpio_set_dir((BASE_KEYPAD_PIN + 6), GPIO_IN);
    // Rows as outputs
    gpio_set_dir_out_masked((0xF << BASE_KEYPAD_PIN));
    // Set all rows high initially
    gpio_put_masked((0xF << BASE_KEYPAD_PIN), (0xF << BASE_KEYPAD_PIN));
    // Enable pullup resistors for columns
    gpio_pull_up((BASE_KEYPAD_PIN + 4));
    gpio_pull_up((BASE_KEYPAD_PIN + 5));
    gpio_pull_up((BASE_KEYPAD_PIN + 6));

    // Enable hardware timer alarm interrupt (Alarm 0)
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq);
    irq_set_enabled(ALARM_IRQ, true);
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

    // Register Protothreads
    pt_add_thread(protothread_adc);
    pt_add_thread(protothread_keypad);

    // Start Protothreads scheduler
    pt_schedule_start;

    return 0;
}
