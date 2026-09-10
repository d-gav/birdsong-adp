/*
 * ADC/Protothreads Demo with Keypad Record & Playback
 *
 * Requirements:
 * - Boots in "Play Mode" (default).
 * - Pressing/releasing key '0' toggles the live potentiometer tone generator ON/OFF.
 * - Pressing/releasing the asterisk '*' button toggles "Record Mode".
 * - In "Record Mode", pushing and holding any key 1-9 records the frequency of the
 *   potentiometer at 100 Hz for as long as that button is held. Sound is played live.
 * - Releasing the button stops recording and saves the frequency sequence (up to 10 seconds).
 * - Pressing/releasing the asterisk '*' button returns to "Play Mode".
 * - In "Play Mode", pressing/releasing that key plays back the recorded frequency sequence at 100 Hz.
 * - Pressing '#' puts the device into Compose Mode (recording keystrokes).
 * - Pressing '#' again puts the device into Playback Mode (playing keystroke sequence).
 * - Storing frequencies (uint16_t in Hz), not raw waveforms, at 100 Hz.
 * - Only 2 protothreads on Core 0:
 *     1) protothread_keypad: Keypad scanning, debouncing, mode toggling, button events.
 *     2) protothread_adc: 100 Hz audio control engine (ADC sampling, recording, and playback).
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
// === Protothreads headers & definitions
// ==========================================
#include "pt_cornell_rp2040_v1_4.h"

// Pin configurations
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

#define VOLUME_SWITCH_GPIO 16

// GPIO for timing the ISR (measured with oscilloscope)
#define ISR_GPIO        2

// DDS sine table
#define sine_table_size 256
volatile int sin_table[sine_table_size];

// ==================================================
// === Recording & Playback Parameters (100 Hz)
// ==================================================
#define RECORD_SECONDS      10
#define RECORD_RATE_HZ      100
#define MAX_RECORD_SAMPLES  (RECORD_SECONDS * RECORD_RATE_HZ) // 1,000 samples = 10 seconds
#define MAX_RECORD_KEYSTROKES 128
#define NUM_RECORD_KEYS     10                               // Keys 0..9 (1..9 used for sounds)

typedef enum {
    VOLUME_CONTROL_OFF,
    VOLUME_CONTROL_ON,
} volume_mode_t;

volatile volume_mode_t current_volume_mode = VOLUME_CONTROL_OFF;
volatile uint16_t volume_value = 1;


typedef enum {
    MODE_PLAY,
    MODE_RECORD,
    MODE_COMPOSE,
    MODE_PLAYBACK
} system_mode_t;

typedef struct {
    uint16_t freq[MAX_RECORD_SAMPLES]; // Frequency stored in Hz (100 Hz sampling rate)
    uint16_t count;                    // Number of recorded samples
} sound_recording_t;


typedef struct {
    uint16_t keystrokes[MAX_RECORD_KEYSTROKES]; 
    uint16_t count;                    
} keystroke_recording_t;

// Frequency recordings for keys 1 through 9
sound_recording_t recordings[NUM_RECORD_KEYS];

// Keystroke recordings for all keys
keystroke_recording_t keystrokes; 

// System mode and state flags
volatile system_mode_t current_mode = MODE_PLAY;
volatile bool live_tone_on = false;       // Toggled by key '0'

volatile bool is_recording = false;       // Active while holding a key 1-9 in Record Mode
volatile int  recording_key = -1;

volatile bool is_playing = false;         // Active while playing back a recorded sequence
volatile int  playback_key = -1;
volatile uint16_t playback_idx = 0;
volatile uint16_t global_key_idx = 0;     // Current keystroke index during sequence playback


// ==================================================
// === Keypad Configuration
// ==================================================
#define BASE_KEYPAD_PIN 9
#define KEYROWS         4
#define NUMKEYS         12

// Keycodes mapped to key values:
// Index 0: '0'
// Indices 1..9: '1'..'9'
// Index 10: '*'
// Index 11: '#'
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
const unsigned int button_mask = 0x70; // Columns mask (pins 13, 14, 15)

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
        sleep_us(1);
        keypad_state = ((gpio_get_all() >> BASE_KEYPAD_PIN) & 0x7F);

        if ((~keypad_state) & button_mask) {
            break;
        }
    }

    // Reset all rows high
    gpio_put_masked((0xF << BASE_KEYPAD_PIN), (0xF << BASE_KEYPAD_PIN));

    if ((~keypad_state) & button_mask) {
        for (key_idx = 0; key_idx < NUMKEYS; key_idx++) {
            if (keypad_state == keycodes[key_idx]) {
                return key_idx;
            }
        }
        return -1; // Invalid keycode
    }

    return -1; // No key pressed
}

//
// Thread 1: Volume Switch 
//
static PT_THREAD (protothread_volume_switch(struct pt *pt))
{
    PT_BEGIN(pt);

    while(1) {
        if (gpio_get(VOLUME_SWITCH_GPIO) == 1) {
            current_volume_mode = VOLUME_CONTROL_ON;
        } else {
            current_volume_mode = VOLUME_CONTROL_OFF;
        }
        PT_YIELD_usec(10000);
    }
    PT_END(pt);
}


// ==================================================
// === Thread 1: Keypad Protothread
// ==================================================
// Handles debouncing, button press/release events, mode toggling
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
                    // --- BUTTON PRESS EVENT CONFIRMED ---

                    if (current_mode == MODE_RECORD) {
                        // In Record Mode: pushing and holding a key 1-9 starts recording
                        if (possible >= 1 && possible <= 9) {
                            recording_key = possible;
                            recordings[recording_key].count = 0;
                            is_recording = true;
                            printf("[RECORD] Key %d recording started (hold down to record, release to finish)...\n", recording_key);
                        }
                    } else if (current_mode == MODE_PLAY) {
                        // In Play Mode
                        if (possible >= 1 && possible <= 9) {
                            // Pressing key 1-9 triggers playback of stored frequency sequence
                            if (recordings[possible].count > 0) {
                                playback_key = possible;
                                playback_idx = 0;
                                is_playing = true;
                            } 

                        } else if (possible == 0) {
                            // Pressing '0' toggles live potentiometer tone generator ON/OFF
                            live_tone_on = !live_tone_on;
                            if (!live_tone_on && !is_playing && !is_recording) {
                                play_tone = false;
                            }
                            printf("[LIVE] Live tone generator %s\n", live_tone_on ? "ON" : "OFF");
                        }
                    } else if (current_mode == MODE_COMPOSE) {
                    } else if (current_mode == MODE_PLAYBACK) {
                    }
                } else {
                    state = STATE_NOT_PRESSED;
                }
                break;

            case STATE_PRESSED:
                if (scanned_key != possible) {
                    state = STATE_MAYBE_NOT_PRESSED;
                }
                break;

            case STATE_MAYBE_NOT_PRESSED:
                if (scanned_key == possible) {
                    // Glitch, back to pressed
                    state = STATE_PRESSED;
                } else {
                    state = STATE_NOT_PRESSED;
                    // --- BUTTON RELEASE EVENT CONFIRMED ---
                    int released_key = possible;

                    // Asterisk key ('*', index 10): Return to Play Mode or Toggle Record Mode
                    if (released_key == 10) {
                        // Stop any ongoing recording or playback when switching modes
                        if (is_recording && recording_key >= 1 && recording_key <= 9) {
                            is_recording = false;
                            recording_key = -1;
                        }
                        is_playing = false;
                        if (!live_tone_on) {
                            play_tone = false;
                        }
                        global_key_idx = 0;
                        playback_idx = 0;

                        // Toggle mode (if in COMPOSE or PLAYBACK, returns to PLAY)
                        current_mode = (current_mode == MODE_PLAY) ? MODE_RECORD : MODE_PLAY;
                        printf("\n========================================\n");
                        if (current_mode == MODE_RECORD) {
                            printf(">>> MODE: RECORD MODE <<<\n");
                            printf("Push & hold any key 1-9 to record slide pot frequencies.\n");
                            printf("Press '*' again to return to Play Mode.\n");
                        } else {
                            printf(">>> MODE: PLAY MODE <<<\n");
                            printf("Press any key 1-9 to play back recorded sounds.\n");
                            printf("Press '0' to toggle live tone.\n");
                            printf("Press '#' to enter Compose Mode.\n");
                        }
                        printf("========================================\n\n");
                    }
                    // Hash key ('#', index 11): Transition into Compose Mode or Playback Mode
                    else if (released_key == 11) {
                        // Stop any ongoing recording or playback when switching modes
                        if (is_recording && recording_key >= 1 && recording_key <= 9) {
                            is_recording = false;
                            recording_key = -1;
                        }
                        is_playing = false;
                        if (!live_tone_on) {
                            play_tone = false;
                        }
                        global_key_idx = 0;
                        playback_idx = 0;

                        if (current_mode == MODE_COMPOSE) {
                            current_mode = MODE_PLAYBACK;
                            printf("\n========================================\n");
                            printf(">>> MODE: PLAYBACK MODE <<<\n");
                            printf("Playing back keystroke sequence (%d keystrokes)...\n", keystrokes.count);
                            printf("Press '#' to return to Compose Mode.\n");
                            printf("Press '*' to return to Play Mode.\n");
                            printf("========================================\n\n");
                        } else {
                            current_mode = MODE_COMPOSE;
                            printf("\n========================================\n");
                            printf(">>> MODE: COMPOSE MODE <<<\n");
                            printf("Record keystroke sequence.\n");
                            printf("Press '#' again to enter Playback Mode.\n");
                            printf("Press '*' to return to Play Mode.\n");
                            printf("========================================\n\n");
                        }
                    }
                    // Releasing a key 1-9 in Record Mode stops recording
                    else if (current_mode == MODE_RECORD && is_recording && released_key == recording_key) {
                        is_recording = false;
                        if (!live_tone_on) {
                            play_tone = false;
                        }
                        printf("[RECORD] Key %d saved: %d samples (%.2f seconds).\n",
                               recording_key, recordings[recording_key].count,
                               (float)recordings[recording_key].count / (float)RECORD_RATE_HZ);
                        recording_key = -1;

                        // Recording is done, so drop straight back into Play Mode. 
                        current_mode = MODE_PLAY;
                    }

                    else if (current_mode == MODE_COMPOSE) {
                        keystrokes.keystrokes[keystrokes.count++] = released_key;
                    } 

                    possible = -1;
                }
                break;
        }

        // Debounce scan cadence: 30 ms
        PT_YIELD_usec(30000);
    }

    PT_END(pt);
}

// ==================================================
// === Thread 2: ADC & Audio Control Protothread (100 Hz)
// ==================================================
// Runs at exactly 100 Hz (10 ms period).
// Responsible for:
//   1. Sampling ADC and storing frequencies into memory when recording
//   2. Stepping through recorded frequencies when playing back
//   3. Live tone potentiometer frequency tracking when live tone is enabled
static PT_THREAD (protothread_adc(struct pt *pt))
{
    PT_BEGIN(pt);

    static unsigned int adc_val;
    static uint16_t current_freq;
    static int tick_counter = 0;

    while(1) {
        // --- 1. RECORDING AT 100 Hz ---
        if (is_recording && recording_key >= 1 && recording_key <= 9) {
            adc_val = adc_read();
            // Scale 12-bit ADC (0-4095) to 0-10,000 Hz range
            if (current_volume_mode == VOLUME_CONTROL_OFF) {
                current_freq = (uint16_t)((adc_val * 5000UL) / 4095UL);
                volume_value = 1;
            } else {
                volume_value = (uint16_t)(adc_val / 2048); 
            }
            if (current_freq < 100) current_freq = 100; // Minimum audible threshold

            // Store frequency into key buffer if space remains
            if (recordings[recording_key].count < MAX_RECORD_SAMPLES) {
                recordings[recording_key].freq[recordings[recording_key].count++] = current_freq * 8;
            } else {
                // Buffer full warning (10 seconds reached)
                static bool warned = false;
                if (!warned) {
                    printf("[RECORD] Key %d buffer full (max %d seconds reached)!\n", recording_key, RECORD_SECONDS);
                    warned = true;
                }
            }

            // Output sound live so user hears what they are recording
            phase_incr_main = (unsigned int)((current_freq * two32) / Fs);
            play_tone = true;
        }

        // --- 2. PLAYBACK AT 100 Hz ---
        else if (is_playing && playback_key >= 1 && playback_key <= 9) {
            if (playback_idx < recordings[playback_key].count) {
                current_freq = recordings[playback_key].freq[playback_idx++];
                phase_incr_main = (unsigned int)((current_freq * two32) / Fs);
                play_tone = true;
            } else {
                // Playback finished
                is_playing = false;
                if (!live_tone_on) {
                    play_tone = false;
                }
                printf("[PLAY] Key %d playback complete.\n", playback_key);
                playback_key = -1;
            }
        }

        // --- 3. KEYSTROKE SEQUENCE PLAYBACK AT 100 Hz (Playback Mode) ---
        else if (current_mode == MODE_PLAYBACK) {
            // Check global key index and that key's playback index; advance if key is finished
            while (global_key_idx < keystrokes.count) {
                uint16_t curr_key = keystrokes.keystrokes[global_key_idx];
                if (curr_key >= 1 && curr_key <= 9 && playback_idx < recordings[curr_key].count) {
                    // Valid key and valid sample index found
                    break;
                }
                // Key has finished playing or has no samples, advance to next keystroke
                playback_idx = 0;
                global_key_idx++;
            }

            // If still within keystroke sequence, play the current sample
            if (global_key_idx < keystrokes.count) {
                uint16_t curr_key = keystrokes.keystrokes[global_key_idx];
                current_freq = recordings[curr_key].freq[playback_idx++];
                phase_incr_main = (unsigned int)((current_freq * two32) / Fs);
                play_tone = true;
            } else {
                // Entire keystroke sequence playback finished
                if (!live_tone_on) {
                    play_tone = false;
                }
                printf("[PLAYBACK] Keystroke sequence complete (%d keystrokes played).\n", keystrokes.count);
                global_key_idx = 0;
                playback_idx = 0;
                current_mode = MODE_PLAY;
                printf(">>> MODE: PLAY MODE <<<\n\n");
            }
        }

        // --- 4. LIVE TONE GENERATOR (Key 0 toggle) ---
        else if (live_tone_on) {
            adc_val = adc_read();
            current_freq = (uint16_t)((adc_val * 5000UL) / 4095UL);
            if (current_freq < 100) current_freq = 100;
            phase_incr_main = (unsigned int)((current_freq * two32) / Fs);
            play_tone = true;
        }

        // --- 4. IDLE / SILENCE ---
        else {
            play_tone = false;
        }

        // Heartbeat LED toggle every 500 ms (50 ticks @ 10 ms)
        if (++tick_counter >= 50) {
            tick_counter = 0;
            gpio_put(LED_PIN, !gpio_get(LED_PIN));
        }

        // Yield for 10 ms -> Exactly 100 Hz sample rate
        PT_YIELD_usec(10000);
    }

    PT_END(pt);
}

// ==================================================
// === Alarm ISR: Direct Digital Synthesis (50 kHz)
// ==================================================
static void alarm_irq(void) {
    // Assert timing GPIO (to measure ISR execution time with oscilloscope)
    gpio_put(ISR_GPIO, 1);

    // Clear alarm interrupt
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);

    // Re-arm alarm register for DELAY microseconds (20 us -> 50 kHz)
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

    if (play_tone) {
        // DDS phase accumulation and sine table lookup
        phase_accum_main += phase_incr_main;
        DAC_data = (DAC_config_chan_B | ((sin_table[phase_accum_main >> 24] + 2048) & 0xffff)) * volume_value;
    } else {
        // Silence: hold phase at 0 and output midscale 2048 (0V AC)
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
    // Overclock to 150 MHz for RP2040 / default for RP2350
    set_sys_clock_khz(150000, true);

    // Initialize stdio
    stdio_init_all();

    // Initialize ADC on GPIO 26
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(ADC_MUX);

    // Initialize onboard LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, true);

    // Initialize SPI for DAC (MCP4822 / MCP4802)
    spi_init(SPI_PORT, 20000000);
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Setup ISR-timing GPIO (GPIO 2)
    gpio_init(ISR_GPIO);
    gpio_set_dir(ISR_GPIO, GPIO_OUT);
    gpio_put(ISR_GPIO, 0);

    // Map SPI signals to GPIO ports
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);


    // Initialize Volume Switch GPIO (0 for off, 1 for on)
    gpio_init(VOLUME_SWITCH_GPIO);
    gpio_set_dir(VOLUME_SWITCH_GPIO, GPIO_IN);

    // Initialize DDS sine table (256 entries, amplitude +/- 2047)
    int ii;
    for (ii = 0; ii < sine_table_size; ii++) {
        sin_table[ii] = (int)(2047.0 * sin((double)ii * 6.283185307179586 / (double)sine_table_size));
    }

    // Default DDS values
    phase_accum_main = 0;
    phase_incr_main = (unsigned int)((440.0 * two32) / Fs);
    play_tone = false;

    // Clear all sound recordings
    for (ii = 0; ii < NUM_RECORD_KEYS; ii++) {
        recordings[ii].count = 0;
    }
    keystrokes.count = 0;

    // Initialize Keypad GPIOs (pins 9-15)
    gpio_init_mask((0x7F << BASE_KEYPAD_PIN));
    // Columns (GPIO 13, 14, 15) as inputs with pull-ups
    gpio_set_dir((BASE_KEYPAD_PIN + 4), GPIO_IN);
    gpio_set_dir((BASE_KEYPAD_PIN + 5), GPIO_IN);
    gpio_set_dir((BASE_KEYPAD_PIN + 6), GPIO_IN);
    gpio_pull_up((BASE_KEYPAD_PIN + 4));
    gpio_pull_up((BASE_KEYPAD_PIN + 5));
    gpio_pull_up((BASE_KEYPAD_PIN + 6));

    // Rows (GPIO 9, 10, 11, 12) as outputs, initially set high
    gpio_set_dir_out_masked((0xF << BASE_KEYPAD_PIN));
    gpio_put_masked((0xF << BASE_KEYPAD_PIN), (0xF << BASE_KEYPAD_PIN));

    // Enable hardware timer alarm interrupt (Alarm 0 @ 50 kHz)
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq);
    irq_set_enabled(ALARM_IRQ, true);
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

    // Register Protothreads (only 2 threads on Core 0)
    pt_add_thread(protothread_adc);
    pt_add_thread(protothread_keypad);

    // Start Protothreads scheduler
    pt_schedule_start;

    return 0;
}
