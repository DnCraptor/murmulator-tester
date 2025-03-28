#include <cstdlib>
#include <cstring>
#include <math.h>
#include <pico.h>
#include <hardware/vreg.h>
#include <hardware/clocks.h>
#include <hardware/flash.h>
#include <hardware/watchdog.h>
#include <pico/bootrom.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <hardware/pwm.h>

#include "graphics.h"

extern "C" volatile bool SELECT_VGA = true;

#ifndef MURM20
#include "psram_spi.h"
#endif
#include "nespad.h"
#include "ff.h"

static void goutf(int outline, bool err, const char *__restrict str, ...) {
    va_list ap;
    char buf[80];
    va_start(ap, str);
    vsnprintf(buf, 80, str, ap);
    va_end(ap);
    printf("%s\n", buf);
    draw_text(buf, 0, outline, err ? 12 : 7, 0);
}

#if PICO_RP2040
volatile static bool no_butterbod = true;
#else
volatile static bool no_butterbod = false;
#endif
volatile static bool ctrlPressed = false;
volatile static bool altPressed = false;
volatile static bool Spressed = false;
volatile static bool Lpressed = false;
volatile static bool Rpressed = false;
volatile static bool Ipressed = false;
volatile static bool i2s_1nit = false;
volatile static uint32_t cpu = 0;
volatile static uint32_t vol = 0;
const int samples = 64;
static int16_t samplesL[samples][2];
static int16_t samplesR[samples][2];
static int16_t samplesLR[samples][2];

inline bool isSpeaker() {
    nespad_read();
    return Spressed || (nespad_state & DPAD_A) || (nespad_state2 & DPAD_A);
}

inline bool isI2S() {
    return Ipressed || (nespad_state & DPAD_B) || (nespad_state2 & DPAD_B);
}

inline bool isInterrupted() {
    return isSpeaker() || isI2S();
}

#if !PICO_RP2040
#include <hardware/structs/qmi.h>
#include <hardware/structs/xip.h>
volatile uint8_t * PSRAM_DATA = (uint8_t*)0x11000000;
void __no_inline_not_in_flash_func(psram_init)(uint cs_pin) {
    gpio_set_function(cs_pin, GPIO_FUNC_XIP_CS1);

    // Enable direct mode, PSRAM CS, clkdiv of 10.
    qmi_hw->direct_csr = 10 << QMI_DIRECT_CSR_CLKDIV_LSB | \
                               QMI_DIRECT_CSR_EN_BITS | \
                               QMI_DIRECT_CSR_AUTO_CS1N_BITS;
    while (qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS)
        ;

    // Enable QPI mode on the PSRAM
    const uint CMD_QPI_EN = 0x35;
    qmi_hw->direct_tx = QMI_DIRECT_TX_NOPUSH_BITS | CMD_QPI_EN;

    while (qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS)
        ;

    // Set PSRAM timing for APS6404
    //
    // Using an rxdelay equal to the divisor isn't enough when running the APS6404 close to 133MHz.
    // So: don't allow running at divisor 1 above 100MHz (because delay of 2 would be too late),
    // and add an extra 1 to the rxdelay if the divided clock is > 100MHz (i.e. sys clock > 200MHz).
    const int max_psram_freq = 166000000;
    const int clock_hz = clock_get_hz(clk_sys);
    int divisor = (clock_hz + max_psram_freq - 1) / max_psram_freq;
    if (divisor == 1 && clock_hz > 100000000) {
        divisor = 2;
    }
    int rxdelay = divisor;
    if (clock_hz / divisor > 100000000) {
        rxdelay += 1;
    }

    // - Max select must be <= 8us.  The value is given in multiples of 64 system clocks.
    // - Min deselect must be >= 18ns.  The value is given in system clock cycles - ceil(divisor / 2).
    const int clock_period_fs = 1000000000000000ll / clock_hz;
    const int max_select = (125 * 1000000) / clock_period_fs;  // 125 = 8000ns / 64
    const int min_deselect = (18 * 1000000 + (clock_period_fs - 1)) / clock_period_fs - (divisor + 1) / 2;

    qmi_hw->m[1].timing = 1 << QMI_M1_TIMING_COOLDOWN_LSB |
                          QMI_M1_TIMING_PAGEBREAK_VALUE_1024 << QMI_M1_TIMING_PAGEBREAK_LSB |
                          max_select << QMI_M1_TIMING_MAX_SELECT_LSB |
                          min_deselect << QMI_M1_TIMING_MIN_DESELECT_LSB |
                          rxdelay << QMI_M1_TIMING_RXDELAY_LSB |
                          divisor << QMI_M1_TIMING_CLKDIV_LSB;

    // Set PSRAM commands and formats
    qmi_hw->m[1].rfmt =
        QMI_M0_RFMT_PREFIX_WIDTH_VALUE_Q << QMI_M0_RFMT_PREFIX_WIDTH_LSB |\
        QMI_M0_RFMT_ADDR_WIDTH_VALUE_Q   << QMI_M0_RFMT_ADDR_WIDTH_LSB |\
        QMI_M0_RFMT_SUFFIX_WIDTH_VALUE_Q << QMI_M0_RFMT_SUFFIX_WIDTH_LSB |\
        QMI_M0_RFMT_DUMMY_WIDTH_VALUE_Q  << QMI_M0_RFMT_DUMMY_WIDTH_LSB |\
        QMI_M0_RFMT_DATA_WIDTH_VALUE_Q   << QMI_M0_RFMT_DATA_WIDTH_LSB |\
        QMI_M0_RFMT_PREFIX_LEN_VALUE_8   << QMI_M0_RFMT_PREFIX_LEN_LSB |\
        6                                << QMI_M0_RFMT_DUMMY_LEN_LSB;

    qmi_hw->m[1].rcmd = 0xEB;

    qmi_hw->m[1].wfmt =
        QMI_M0_WFMT_PREFIX_WIDTH_VALUE_Q << QMI_M0_WFMT_PREFIX_WIDTH_LSB |\
        QMI_M0_WFMT_ADDR_WIDTH_VALUE_Q   << QMI_M0_WFMT_ADDR_WIDTH_LSB |\
        QMI_M0_WFMT_SUFFIX_WIDTH_VALUE_Q << QMI_M0_WFMT_SUFFIX_WIDTH_LSB |\
        QMI_M0_WFMT_DUMMY_WIDTH_VALUE_Q  << QMI_M0_WFMT_DUMMY_WIDTH_LSB |\
        QMI_M0_WFMT_DATA_WIDTH_VALUE_Q   << QMI_M0_WFMT_DATA_WIDTH_LSB |\
        QMI_M0_WFMT_PREFIX_LEN_VALUE_8   << QMI_M0_WFMT_PREFIX_LEN_LSB;

    qmi_hw->m[1].wcmd = 0x38;

    // Disable direct mode
    qmi_hw->direct_csr = 0;

    // Enable writes to PSRAM
    hw_set_bits(&xip_ctrl_hw->ctrl, XIP_CTRL_WRITABLE_M1_BITS);
}
#endif

extern "C" {
    #include "audio.h"
    #include "ps2.h"
    bool __time_critical_func(handleScancode)(const uint32_t ps2scancode) {
        goutf(TEXTMODE_ROWS - 3, false, "Last scancode: %04Xh  ", ps2scancode);
        if (ps2scancode == 0x1F) { // S
            Spressed = true;
        }
        else if (ps2scancode == 0x26) { // L
            Lpressed = true;
        }
        else if (ps2scancode == 0x13) { // R
            Rpressed = true;
        }
        else if (ps2scancode == 0x17) { // I
            Ipressed = true;
        }
        else if (ps2scancode == 0x9F) { // S up
            Spressed = false;
        }
        else if (ps2scancode == 0xA6) { // L up
            Lpressed = false;
        }
        else if (ps2scancode == 0x93) { // R up
            Rpressed = false;
        }
        else if (ps2scancode == 0x97) { // I up
            Ipressed = false;
        }
        else if (ps2scancode == 0x1D) {
            ctrlPressed = true;
        }
        else if (ps2scancode == 0x9D) {
            ctrlPressed = false;
        }
        else if (ps2scancode == 0x38) {
            altPressed = true;
        }
        else if (ps2scancode == 0xB8) {
            altPressed = false;
        }
        else if (ps2scancode == 0x53 && altPressed && ctrlPressed) {
            watchdog_hw->scratch[0] = cpu | (vol << 22);
            watchdog_enable(1, 0);
        }
        else if (ps2scancode == 0x4E) { // +
            watchdog_hw->scratch[0] = (cpu + 4) | (vol << 22);
            watchdog_enable(1, 0);
        }
        else if (ps2scancode == 0x4A) { // -
            watchdog_hw->scratch[0] = (cpu - 4) | (vol << 22);
            watchdog_enable(1, 0);
        }
        else if (ps2scancode == 0x52) { // Ins
            watchdog_hw->scratch[0] = (cpu + 40) | (vol << 22);
            watchdog_enable(1, 0);
        }
        else if (ps2scancode == 0x53) { // Del
            watchdog_hw->scratch[0] = (cpu - 40) | (vol << 22);
            watchdog_enable(1, 0);
        }
        else if (ps2scancode == 0x49) { // PageUp
            watchdog_hw->scratch[0] = cpu | ((vol + 1) << 22);
            watchdog_enable(1, 0);
        }
        else if (ps2scancode == 0x51) { // PageDown
            watchdog_hw->scratch[0] = cpu | ((vol - 1) << 22);
            watchdog_enable(1, 0);
        }
        return true;
    }
}
///static uint16_t dma_buffer[22050 / 60];
static i2s_config_t i2s_config = {
    .sample_freq = 22050,
    .channel_count = 2,
    .data_pin = AUDIO_DATA_PIN,
    .clock_pin_base = AUDIO_CLOCK_PIN,
    .pio = pio1,
    .sm = 0,
    .dma_channel = 0,
    .dma_trans_count = 0, ///sizeof(dma_buffer) / sizeof(uint32_t), // Number of 32 bits words to transfer
    .dma_buf = 0,
    .volume = 0, // 16 - is 0
};
    
static semaphore vga_start_semaphore;
static uint16_t SCREEN[TEXTMODE_ROWS][80];

void __time_critical_func(render_core)() {
    multicore_lockout_victim_init();
    graphics_init();
    const auto buffer = (uint8_t *)SCREEN;
    graphics_set_bgcolor(0x000000);
    graphics_set_offset(0, 0);
    graphics_set_flashmode(false, false);
    graphics_set_mode(TEXTMODE_DEFAULT);
    graphics_set_buffer(buffer, TEXTMODE_COLS, TEXTMODE_ROWS);
    graphics_set_textbuffer(buffer);
    clrScr(0);
    sem_acquire_blocking(&vga_start_semaphore);

    // 60 FPS loop
#define frame_tick (16666)
    uint64_t tick = time_us_64();
    uint64_t last_frame_tick = tick;
    static uint32_t frame_no = 0;
    while (true) {
        if (tick >= last_frame_tick + frame_tick) {
            frame_no++;
#ifdef TFT
            refresh_lcd();
#endif
            /**
            if (i2s_1nit && (Lpressed || Rpressed)) {
                uint16_t out = (frame_no & 1) ? 0xFFFF : 0;
                for (int i = 0; i < (sizeof(dma_buffer) / sizeof(uint16_t)); ) {
                    dma_buffer[i++] = Lpressed ? out : 0;
                    dma_buffer[i++] = Rpressed ? out : 0;
                }
                i2s_dma_write(&i2s_config, dma_buffer);
            }
            */
            last_frame_tick = tick;
        }
        tick = time_us_64();
        tight_loop_contents();
    }
    __unreachable();
}

static bool __not_in_flash_func(write_flash)(void) {
    static uint8_t buffer[FLASH_SECTOR_SIZE];
    memcpy(buffer, (const void*)XIP_BASE, FLASH_SECTOR_SIZE);
    uint32_t flash_target_offset = 0x20000;

    multicore_lockout_start_blocking();
    const uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(flash_target_offset, FLASH_SECTOR_SIZE);
    flash_range_program(flash_target_offset, buffer, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);
    multicore_lockout_end_blocking();

    return memcmp((const void*)XIP_BASE, (const void*)XIP_BASE + flash_target_offset, FLASH_SECTOR_SIZE) == 0;
}

static void blink(uint32_t pin) {
    for (uint32_t i = 0; i < pin + 1; ++i) {
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(650);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
        sleep_ms(550);
    }
    sleep_ms(1000);
}

// connection is possible 00->00 (external pull down)
static int test_0000_case(uint32_t pin0, uint32_t pin1, int res) {
    gpio_init(pin0);
    gpio_set_dir(pin0, GPIO_OUT);
    sleep_ms(33);
    gpio_put(pin0, 1);

    gpio_init(pin1);
    gpio_set_dir(pin1, GPIO_IN);
    gpio_pull_down(pin1); /// external pulled down (so, just to ensure)
    sleep_ms(33);
    if ( gpio_get(pin1) ) { // 1 -> 1, looks really connected
        res |= (1 << 5) | 1;
    }
    gpio_deinit(pin0);
    gpio_deinit(pin1);
    return res;
}

// connection is possible 01->01 (no external pull up/down)
static int test_0101_case(uint32_t pin0, uint32_t pin1, int res) {
    gpio_init(pin0);
    gpio_set_dir(pin0, GPIO_OUT);
    sleep_ms(33);
    gpio_put(pin0, 1);

    gpio_init(pin1);
    gpio_set_dir(pin1, GPIO_IN);
    gpio_pull_down(pin1);
    sleep_ms(33);
    if ( gpio_get(pin1) ) { // 1 -> 1, looks really connected
        res |= (1 << 5) | 1;
    }
    gpio_deinit(pin0);
    gpio_deinit(pin1);
    return res;
}

// connection is possible 11->11 (externally pulled up)
static int test_1111_case(uint32_t pin0, uint32_t pin1, int res) {
    gpio_init(pin0);
    gpio_set_dir(pin0, GPIO_OUT);
    sleep_ms(33);
    gpio_put(pin0, 0);

    gpio_init(pin1);
    gpio_set_dir(pin1, GPIO_IN);
    gpio_pull_up(pin1); /// external pulled up (so, just to ensure)
    sleep_ms(33);
    if ( !gpio_get(pin1) ) { // 0 -> 0, looks really connected
        res |= 1;
    }
    gpio_deinit(pin0);
    gpio_deinit(pin1);
    return res;
}

static int testPins(uint32_t pin0, uint32_t pin1) {
    int res = 0b000000;
#if PICO_RP2350
    /// do not try to test butter psram this way
    if (pin0 == BUTTER_PSRAM_GPIO || pin1 == BUTTER_PSRAM_GPIO) return res;
    if (pin0 == 23 || pin1 == 23) return res; // SMPS Power
    if (pin0 == 24 || pin1 == 24) return res; // VBus sense
    if (pin0 == 25 || pin1 == 25) return res; // LED
#endif
    // try pull down case (passive)
    gpio_init(pin0);
    gpio_set_dir(pin0, GPIO_IN);
    gpio_pull_down(pin0);
    gpio_init(pin1);
    gpio_set_dir(pin1, GPIO_IN);
    gpio_pull_down(pin1);
    sleep_ms(33);
    int pin0vPD = gpio_get(pin0);
    int pin1vPD = gpio_get(pin1);
    gpio_deinit(pin0);
    gpio_deinit(pin1);
    /// the same for pull_down state, try pull up case (passive)
    gpio_init(pin0);
    gpio_set_dir(pin0, GPIO_IN);
    gpio_pull_up(pin0);
    gpio_init(pin1);
    gpio_set_dir(pin1, GPIO_IN);
    gpio_pull_up(pin1);
    sleep_ms(33);
    int pin0vPU = gpio_get(pin0);
    int pin1vPU = gpio_get(pin1);
    gpio_deinit(pin0);
    gpio_deinit(pin1);

    res = (pin0vPD << 4) | (pin0vPU << 3) | (pin1vPD << 2) | (pin1vPU << 1);

    if (pin0vPD == 1) {
        if (pin0vPU == 1) { // pin0vPD == 1 && pin0vPU == 1
            if (pin1vPD == 1) { // pin0vPD == 1 && pin0vPU == 1 && pin1vPD == 1
                if (pin1vPU == 1) { // pin0vPD == 1 && pin0vPU == 1 && pin1vPD == 1 && pin1vPU == 1
                    // connection is possible 11->11 (externally pulled up)
                    return test_1111_case(pin0, pin1, res);
                } else { // pin0vPD == 1 && pin0vPU == 1 && pin1vPD == 1 && pin1vPU == 0
                    // connection is impossible
                    return res;
                }
            } else { // pin0vPD == 1 && pin0vPU == 1 && pin1vPD == 0
                if (pin1vPU == 1) { // pin0vPD == 1 && pin0vPU == 1 && pin1vPD == 0 && pin1vPU == 1
                    // connection is impossible
                    return res;
                } else { // pin0vPD == 1 && pin0vPU == 1 && pin1vPD == 0 && pin1vPU == 0
                    // connection is impossible
                    return res;
                }
            }
        } else {  // pin0vPD == 1 && pin0vPU == 0
            if (pin1vPD == 1) { // pin0vPD == 1 && pin0vPU == 0 && pin1vPD == 1
                if (pin1vPU == 1) { // pin0vPD == 1 && pin0vPU == 0 && pin1vPD == 1 && pin1vPU == 1
                    // connection is impossible
                    return res;
                } else { // pin0vPD == 1 && pin0vPU == 0 && pin1vPD == 1 && pin1vPU == 0
                    // connection is possible 10->10 (pulled up on down, and pulled down on up?)
                    return res |= (1 << 5) | 1; /// NOT SURE IT IS POSSIBLE TO TEST SUCH CASE (TODO: think about real cases)
                }
            } else { // pin0vPD == 1 && pin0vPU == 0 && pin1vPD == 0
                if (pin1vPU == 1) { // pin0vPD == 1 && pin0vPU == 0 && pin1vPD == 0 && pin1vPU == 1
                    // connection is impossible
                    return res;
                } else { // pin0vPD == 1 && pin0vPU == 0 && pin1vPD == 0 && pin1vPU == 0
                    // connection is impossible
                    return res;
                }
            }
        }
    } else { // pin0vPD == 0
        if (pin0vPU == 1) { // pin0vPD == 0 && pin0vPU == 1
            if (pin1vPD == 1) { // pin0vPD == 0 && pin0vPU == 1 && pin1vPD == 1
                if (pin1vPU == 1) { // pin0vPD == 0 && pin0vPU == 1 && pin1vPD == 1 && pin1vPU == 1
                    // connection is impossible
                    return res;
                } else { // pin0vPD == 0 && pin0vPU == 1 && pin1vPD == 1 && pin1vPU == 0
                    // connection is impossible
                    return res;
                }
            } else { // pin0vPD == 0 && pin0vPU == 1 && pin1vPD == 0
                if (pin1vPU == 1) { // pin0vPD == 0 && pin0vPU == 1 && pin1vPD == 0 && pin1vPU == 1
                    // connection is possible 01->01 (no external pull up/down)
                    return test_0101_case(pin0, pin1, res);
                } else { // pin0vPD == 0 && pin0vPU == 1 && pin1vPD == 0 && pin1vPU == 0
                    // connection is impossible
                    return res;
                }
            }
        } else {  // pin0vPD == 0 && pin0vPU == 0
            if (pin1vPD == 1) { // pin0vPD == 0 && pin0vPU == 0 && pin1vPD == 1
                if (pin1vPU == 1) { // pin0vPD == 0 && pin0vPU == 0 && pin1vPD == 1 && pin1vPU == 1
                    // connection is impossible
                    return res;
                } else { // pin0vPD == 0 && pin0vPU == 0 && pin1vPD == 1 && pin1vPU == 0
                    // connection is impossible
                    return res;
                }
            } else { // pin0vPD == 0 && pin0vPU == 0 && pin1vPD == 0
                if (pin1vPU == 1) { // pin0vPD == 0 && pin0vPU == 0 && pin1vPD == 0 && pin1vPU == 1
                    // connection is impossible
                    return res;
                } else { // pin0vPD == 0 && pin0vPU == 0 && pin1vPD == 0 && pin1vPU == 0
                    // connection is possible 00->00 (externally pulled down)
                    return test_0000_case(pin0, pin1, res);
                }
            }
        }
    }
    return res;
}

static void get_cpu_flash_jedec_id(uint8_t _rx[4]) {
    static uint8_t rx[4] = {0};
    if (rx[0] == 0) {
        uint8_t tx[4] = {0x9f};
        multicore_lockout_start_blocking();
        const uint32_t ints = save_and_disable_interrupts();
        flash_do_cmd(tx, rx, 4);
        restore_interrupts(ints);
        multicore_lockout_end_blocking();
    }
    *(unsigned*)_rx = *(unsigned*)rx;
}

static pwm_config config = pwm_get_default_config();
static void PWM_init_pin(uint8_t pinN, uint16_t max_lvl) {
    gpio_set_function(pinN, GPIO_FUNC_PWM);
    pwm_config_set_clkdiv(&config, 1.0);
    pwm_config_set_wrap(&config, max_lvl); // MAX PWM value
    pwm_init(pwm_gpio_to_slice_num(pinN), &config, true);
}

#define short_light 100

static const char* get_volt() {
    const char* volt = (const char*)"1.3 V";
    switch(vol) {
#if !PICO_RP2040
        case VREG_VOLTAGE_0_60: volt = "0.6 V"; break;
        case VREG_VOLTAGE_0_65: volt = "0.65V"; break;
        case VREG_VOLTAGE_0_70: volt = "0.7 V"; break;
        case VREG_VOLTAGE_0_75: volt = "0.75V"; break;
        case VREG_VOLTAGE_0_80: volt = "0.8 V"; break;
#endif
        case VREG_VOLTAGE_0_85: volt = "0.85V"; break;
        case VREG_VOLTAGE_0_90: volt = "0.9 V"; break;
        case VREG_VOLTAGE_0_95: volt = "0.95V"; break;
        case VREG_VOLTAGE_1_00: volt = "1.0 V"; break;
        case VREG_VOLTAGE_1_05: volt = "1.05V"; break;
        case VREG_VOLTAGE_1_10: volt = "1.1 V"; break;
        case VREG_VOLTAGE_1_15: volt = "1.15V"; break;
        case VREG_VOLTAGE_1_20: volt = "1.2 V"; break;
        case VREG_VOLTAGE_1_25: volt = "1.25V"; break;
        case VREG_VOLTAGE_1_30: volt = "1.3 V"; break;
#if !PICO_RP2040
        // Above this point you will need to set POWMAN_VREG_CTRL_DISABLE_VOLTAGE_LIMIT
        case VREG_VOLTAGE_1_35: volt = "1.35V"; break;
        case VREG_VOLTAGE_1_40: volt = "1.4 V"; break;
        case VREG_VOLTAGE_1_50: volt = "1.5 V"; break;
        case VREG_VOLTAGE_1_60: volt = "1.6 V"; break;
        case VREG_VOLTAGE_1_65: volt = "1.65V"; break;
        case VREG_VOLTAGE_1_70: volt = "1.7 V"; break;
        case VREG_VOLTAGE_1_80: volt = "1.8 V"; break;
        case VREG_VOLTAGE_1_90: volt = "1.9 V"; break;
        case VREG_VOLTAGE_2_00: volt = "2.0 V"; break;
        case VREG_VOLTAGE_2_35: volt = "2.35V"; break;
        case VREG_VOLTAGE_2_50: volt = "2.5 V"; break;
        case VREG_VOLTAGE_2_65: volt = "2.65V"; break;
        case VREG_VOLTAGE_2_80: volt = "2.8 V"; break;
        case VREG_VOLTAGE_3_00: volt = "3.0 V"; break;
        case VREG_VOLTAGE_3_15: volt = "3.15V"; break;
        case VREG_VOLTAGE_3_30: volt = "3.3 V"; break;
#endif
    }
    return volt;
}

#include <hardware/exception.h>

void sigbus(void) {
    printf("SIGBUS exception caught...\n");
    // reset_usb_boot(0, 0);
}
void __attribute__((naked, noreturn)) __printflike(1, 0) dummy_panic(__unused const char *fmt, ...) {
    printf("*** PANIC ***");
    if (fmt)
        printf(fmt);
}

#ifdef MURM20
static const bool critical[] = {
    0, // 00 -> 01 PS2 mouse
    0, // 01 -> 02 PS2 mouse
    0, // 02 -> 03 PS2 kbd
    1, // 03 -> 04 PS2 kbd
    1, // 04 -> 05 SD
    1, // 05 -> 06 SD
    1, // 06 -> 07 SD
    1, // 07 -> 08 SD
    0, // 08 -> 09 PSRAM+
    0, // 09 -> 10 beeper/i2s data
    0, // 10 -> 11 r/slc
    0, // 11 -> 12 l/lrck
    0, // 12 -> 13 vga/hdmi
    1, // 13 -> 14 vga/hdmi
    0, // 14 -> 15 vga/hdmi
    1, // 15 -> 16 vga/hdmi
    0, // 16 -> 17 vga/hdmi
    1, // 17 -> 18 vga/hdmi
    1, // 18 -> 19 vga/hdmi
    0, // 19 -> 20 nes
    0, // 20 -> 21 nes
    0, // 21 -> 22 load in
    0, // 22 -> 23 smps power
    0, // 23 -> 24 vbus sence
    0, // 24 -> 25 LED
    0, // 25 -> 26 NES
    0, // 26 -> 27 NES
    0, // 27 -> 28 ADC2
    0, // 28 -> 29 ADC3
};
static const char* const desc[] = {
    "MOUSE", // 00 -> 01
    "MOUSE KBD", // 01 -> 02
    "KBD", // 02 -> 03
    "KBD SD", // 03 -> 04
    "SD", // 04 -> 05
    "SD", // 05 -> 06
    "SD", // 06 -> 07
    "SD", // 07 -> 08
    "PSRAM+", // 08 -> 09
    "I2S?", // 09 -> 10
    "I2S?", // 10 -> 11
    "I2S VGA", // 11 -> 12
    "VGA", // 12 -> 13
    "VIDEO", // 13 -> 14
    "VGA", // 14 -> 15
    "VIDEO", // 15 -> 16
    "VGA", // 16 -> 17
    "VIDEO", // 17 -> 18
    "HDMI?", // 18 -> 19
    "VIDEO NES", // 19 -> 20
    "NES LOADIN", // 20 -> 21
    "LOADIN", // 21 -> 22
    "", // 22 -> 23
    "", // 23 -> 24
    "", // 24 -> 25
    "NES", // 25 -> 26
    "NES", // 26 -> 27
    "", // 27 -> 28
    "", // 28 -> 29
};

#else
static const bool critical[] = {
    1, // 00 -> 01 PS/2 kbd clk -> data
    1, // 01 -> 02 PS/2 kbd data -> sd clk
    1, // 02 -> 03 sd clk -> sd mosi
    1, // 03 -> 04 sd mosi -> sd miso
    1, // 04 -> 05 sd miso -> sd cs
    1, // 05 -> 06 sd cs -> vga 0
    0, // 06 -> 07 vga 0 -> vga 1
    1, // 07 -> 08
    0, // 08 -> 09 vga 2 -> vga 3
    1, // 09 -> 10
    0, // 10 -> 11 vga 4 -> vga 5
    1, // 11 -> 12
    0, // 12 -> 13
    1, // 13 -> 14 vga vs -> nes clk
    1, // 14 -> 15
    1, // 15 -> 16
    1, // 16 -> 17
    1, // 17 -> 18
    1, // 18 -> 19
    1, // 19 -> 20
    0, // 20 -> 21 psram
    1, // 21 -> 22
    1, // 22 -> 23
    1, // 23 -> 24
    1, // 24 -> 25
    1, // 25 -> 26
    1, // 26 -> 27
    1, // 27 -> 28
    1, // 28 -> 29
};
static const char* const desc[] = {
    "KBD", // 00 -> 01 PS/2 kbd clk -> data
    "KBD SD", // 01 -> 02 PS/2 kbd data -> sd clk
    "SD", // 02 -> 03 sd clk -> sd mosi
    "SD", // 03 -> 04 sd mosi -> sd miso
    "SD", // 04 -> 05 sd miso -> sd cs
    "SD VIDEO", // 05 -> 06 sd cs -> vga 0
    "VGA B", // 06 -> 07 vga 0 -> vga 1
    "VIDEO", // 07 -> 08
    "VGA G", // 08 -> 09 vga 2 -> vga 3
    "VIDEO", // 09 -> 10
    "VGA R", // 10 -> 11 vga 4 -> vga 5
    "VIDEO", // 11 -> 12
    "HDMI?", // 12 -> 13
    "VIDEO NES", // 13 -> 14 vga vs -> nes clk
    "NES", // 14 -> 15
    "NES", // 15 -> 16
    "NES", // 16 -> 17
    "NES PSRAM", // 17 -> 18
    "PSRAM", // 18 -> 19
    "PSRAM", // 19 -> 20
    "PSRAM", // 20 -> 21 psram
    "PSRAM LOADIN", // 21 -> 22
    "", // 22 -> 23
    "", // 23 -> 24
    "", // 24 -> 25
    "", // 25 -> 26
    "I2S", // 26 -> 27
    "I2S", // 27 -> 28
    "I2S", // 28 -> 29
};
#endif

int main() {
    auto scratch0 = watchdog_hw->scratch[0];
    cpu = scratch0 & 0x03FF;
    vol = (scratch0 & 0xFC00) >> 22;
    if (!cpu) cpu = 252;
    if (!vol) vol = VREG_VOLTAGE_1_30;

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    stdio_init_all();

    printf("%d MHz %s\n", cpu, get_volt());
    if (vol > VREG_VOLTAGE_1_30)
#if PICO_RP2040
        vol = VREG_VOLTAGE_1_30;
#else
        vreg_disable_voltage_limit();
#endif
    vreg_set_voltage((vreg_voltage)vol);
    vol = vreg_get_voltage();
    sleep_ms(33);
    uint vco, postdiv1, postdiv2;
    if (check_sys_clock_khz(cpu * KHZ, &vco, &postdiv1, &postdiv2)) {
        set_sys_clock_pll(vco, postdiv1, postdiv2);
    } else {
        cpu = clock_get_hz(clk_sys) / (KHZ * KHZ);
    }
    printf("%d (%d:%d:%d) MHz %s\n", cpu, vco, postdiv1, postdiv2, get_volt());

#if PICO_RP2350
#ifdef BUTTER_PSRAM_GPIO
    psram_init(BUTTER_PSRAM_GPIO);
#endif
#endif

    /// startup signal
    for (int i = 0; i < 2; i++) {
        sleep_ms(short_light);
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(short_light);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
    }
    sleep_ms(1000);

    int links[27] = { false };
    for(uint32_t pin = 0; pin < 27; ++pin) {
        links[pin] = testPins(pin, pin + 1);
    }
    SELECT_VGA = (links[VGA_BASE_PIN] == 0) || (links[VGA_BASE_PIN] == 0x1F);
    for(uint32_t pin = VGA_BASE_PIN; pin < VGA_BASE_PIN + 7; ++pin) {
        if ((links[pin] & 0b000001) && (!SELECT_VGA || critical[pin])) {
            blink(pin);
        }
    }

    sleep_ms(1000);
    /// main test DONE signal
    for (int i = 0; i < 4; i++) {
        sleep_ms(short_light);
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(short_light);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
    }
    
    sem_init(&vga_start_semaphore, 0, 1);
    multicore_launch_core1(render_core);
    sem_release(&vga_start_semaphore);

    sleep_ms(250);

    int y = 0;
    goutf(y++, false, "Started with %d MHz (%d:%d:%d) %s", cpu, vco / (KHZ * KHZ), postdiv1, postdiv2, get_volt());
/*
    static const int vga_disconnected[7]    = { 0x1F, 0x1A, 0x1F, 0x1A, 0x1F, 0x1A, 0x1A };
    static const int vga_connected[7]       = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    static const int frank_disconnected[7]  = { 0x1F, 0x1A, 0x1F, 0x1A, 0x1F, 0x1A, 0x1A };

    static const int frank_hdmi_vga[7]      = { 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E };
    static const int hdmi_connected[7]      = { 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x18, 0x06? };
    static const int frank_hdmi_connected[] = { 0x21, 0x00, 0x21, 0x00, 0x21, 0x00, 0x21 };
*/
    for(uint32_t pin = 0; pin < 27; ++pin) {
#if DEBUG
        if (links[pin]) {
            bool cs      = !!(links[pin] & 0b100000);
            bool pin0vPD = !!(links[pin] & 0b010000);
            bool pin0vPU = !!(links[pin] & 0b001000);
            bool pin1vPD = !!(links[pin] & 0b000100);
            bool pin1vPU = !!(links[pin] & 0b000010);
            bool connect = !!(links[pin] & 0b000001);
            if (connect)
                goutf(y++, true, "GPIO %02d connected to %02d %d[%d%d->%d%d]", pin, pin + 1, cs, pin0vPD, pin0vPU, pin1vPD, pin1vPU);
            else
                goutf(y++, false, "GPIO %02d not connected to %02d %d[%d%d->%d%d]", pin, pin + 1, cs, pin0vPD, pin0vPU, pin1vPD, pin1vPU);
        }
#else
        if (links[pin] & 1) {
            goutf(y++, critical[pin], "GPIO %02d connected to %02d (%d) %s", pin, pin + 1, !!(links[pin] & 0b100000), desc[pin]);
        }
#endif
    }

    FATFS fs;
    if (f_mount(&fs, "SD", 1) == FR_OK) {
        goutf(y++, false, "SDCARD %d FATs; %d free clusters (%d KB each)", fs.n_fats, f_getfree32(&fs), fs.csize >> 1);
    } else {
        draw_text("SDCARD not connected", 0, y++, 12, 0);
    }

    keyboard_init();

#if PICO_RP2350
    #ifdef BUTTER_PSRAM_GPIO
    {
        exception_set_exclusive_handler(HARDFAULT_EXCEPTION, sigbus);
        goutf(y++, false, "Try Butter-PSRAM test (on GPIO-%d)", BUTTER_PSRAM_GPIO);
        uint32_t psram32 = 4 << 12;
        uint32_t a = 0;
        uint32_t elapsed;
        uint32_t begin = time_us_32();
        double d = 1.0;
        double speedw, speedr;
        for (; a < 4 << 12; ++a) {
            PSRAM_DATA[a] =  a & 0xFF;
        }
        elapsed = time_us_32() - begin;
        speedw = d * a / elapsed;
        begin = time_us_32();
        for (a = 0; a < psram32; ++a) {
            if ((a & 0xFF) != PSRAM_DATA[a]) {
                goutf(y++, false, "Butter-PSRAM read failed at %ph", PSRAM_DATA+a);
                no_butterbod = true;
                break;
            }
        }
        elapsed = time_us_32() - begin;
        speedr = d * a / elapsed;
        if (!no_butterbod) {
            goutf(y++, false, "8-bit line write speed: %f MBps", speedw);
            goutf(y++, false, "8-bit line read speed: %f MBps", speedr);
        }
        if (no_butterbod) {
            goutf(y++, false, "Butter-PSRAM on GPIO-%d not found", BUTTER_PSRAM_GPIO);
        }
    }
    #endif
#endif
    nespad_begin(clock_get_hz(clk_sys) / 1000, NES_GPIO_CLK, NES_GPIO_DATA, NES_GPIO_LAT);

    if (!isInterrupted()) {
        uint8_t rx[4];
        get_cpu_flash_jedec_id(rx);
        uint32_t flash_size = (1 << rx[3]);
        goutf(y++, false, "FLASH %d MB; JEDEC ID: %02X-%02X-%02X-%02X",
                 flash_size >> 20, rx[0], rx[1], rx[2], rx[3]
        );
    
        if (!isInterrupted()) {
            printf("Test flash write ... ");
            if (write_flash()) {
                printf("Test write to FLASH - passed\n");
                draw_text("Test write to FLASH - passed", 0, y++, 7, 0);
            } else {
                printf("Test write to FLASH - failed\n");
                draw_text("Test write to FLASH - failed", 0, y++, 12, 0);
            }
            }
    }
#if !MURM20
    if (!isInterrupted() && no_butterbod) {
        init_psram();
        uint32_t psram32 = psram_size();
        uint8_t rx8[8];
        psram_id(rx8);
        if (psram32) {
            goutf(y++, false, "PSRAM %d MB; MF ID: %02x; KGD: %02x; EID: %02X%02X-%02X%02X-%02X%02X",
                              psram32 >> 20, rx8[0], rx8[1], rx8[2], rx8[3], rx8[4], rx8[5], rx8[6], rx8[7]);
    
            uint32_t a = 0;
            uint32_t elapsed;
            uint32_t begin = time_us_32();
            double d = 1.0;
            double speed;
            for (; a < psram32; ++a) {
                if (isInterrupted()) goto skip_it;
                write8psram(a, a & 0xFF);
            }
            elapsed = time_us_32() - begin;
            speed = d * a / elapsed;
            goutf(y++, false, "8-bit line write speed: %f MBps", speed);
            begin = time_us_32();
            for (a = 0; a < psram32; ++a) {
                if (isInterrupted()) goto skip_it;
                if ((a & 0xFF) != read8psram(a)) {
                    goutf(y++, true, "8-bit read failed at %ph", a);
                    break;
                }
            }
            elapsed = time_us_32() - begin;
            speed = d * a / elapsed;
            goutf(y++, false, "8-bit line read speed: %f MBps", speed);
        
            begin = time_us_32();
            for (a = 0; a < psram32; a += 2) {
                if (isInterrupted()) goto skip_it;
                write16psram(a, a & 0xFFFF);
            }
            elapsed = time_us_32() - begin;
            speed = d * a / elapsed;
            goutf(y++, false, "16-bit line write speed: %f MBps", speed);
       
            begin = time_us_32();
            for (a = 0; a < psram32; a += 2) {
                if (isInterrupted()) goto skip_it;
                if ((a & 0xFFFF) != read16psram(a)) {
                    goutf(y++, true, "16-bit read failed at %ph", a);
                    break;
                }
            }
            elapsed = time_us_32() - begin;
            speed = d * a / elapsed;
            goutf(y++, false, "16-bit line read speed: %f MBps", speed);
        
            begin = time_us_32();
            for (a = 0; a < psram32; a += 4) {
                if (isInterrupted()) goto skip_it;
                write32psram(a, a);
            }
            elapsed = time_us_32() - begin;
            speed = d * a / elapsed;
            goutf(y++, false, "32-bit line write speed: %f MBps", speed);
        
            begin = time_us_32();
            for (a = 0; a < psram32; a += 4) {
                if (isInterrupted()) goto skip_it;
                if (a != read32psram(a)) {
                    goutf(y++, true, "32-bit read failed at %ph", a);
                    break;
                }
            }
            elapsed = time_us_32() - begin;
            speed = d * a / elapsed;
            goutf(y++, false, "32-bit line read speed: %f MBps", speed);
        } else {
            goutf(y++, false, "No PSRAM detected");
        }
    }
#endif
    goutf(y++, false, "DONE");
skip_it:
    draw_text("S(A) - try PWM, L(SELECT) - left, R(START) - right", 0, y++, 7, 0);
    draw_text("I(B) - try i2s sound (+L/R)", 0, y, 7, 0);

    for (int i = 0; i < 8; i++) {
        sleep_ms(short_light);
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(short_light);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
    }

    bool pwm_init = false;
    while(true) {
        uint32_t nstate = nespad_state;
        uint32_t nstate2 = nespad_state2;
        bool S = isSpeaker();
        bool I = isI2S();
        bool L = Lpressed || (nespad_state & DPAD_SELECT) || (nespad_state2 & DPAD_SELECT);
        bool R = Rpressed || (nespad_state & DPAD_START) || (nespad_state2 & DPAD_START);
        if (!i2s_1nit && (S || R || L)) {
            if (!pwm_init) {
                PWM_init_pin(BEEPER_PIN, (1 << 12) - 1);
                PWM_init_pin(PWM_PIN0  , (1 << 12) - 1);
                PWM_init_pin(PWM_PIN1  , (1 << 12) - 1);
                draw_text(" (Ctrl+Alt+Del - restart to test i2s) ", 0, y, 7, 0);
                pwm_init = true;
            }
            if (S) pwm_set_gpio_level(BEEPER_PIN, (1 << 12) - 1);
            if (R) pwm_set_gpio_level(PWM_PIN0  , (1 << 12) - 1);
            if (L) pwm_set_gpio_level(PWM_PIN1  , (1 << 12) - 1);
            sleep_ms(1);
            if (S) pwm_set_gpio_level(BEEPER_PIN, 0);
            if (R) pwm_set_gpio_level(PWM_PIN0  , 0);
            if (L) pwm_set_gpio_level(PWM_PIN1  , 0);
            sleep_ms(1);
        }
        else if (!pwm_init && I) {
            if (!i2s_1nit) {
                draw_text(" (Ctrl+Alt+Del - restart to test PWM)                         ", 0, y - 1, 7, 0);
                i2s_config.dma_trans_count = samples >> 1;
                i2s_init(&i2s_config);
                for (int i = 0; i < samples; ++i) {
                    int16_t v = std::sin(2 * 3.1415296 * i / samples) * 32767;
            
                    samplesL[i][0] = v;
                    samplesL[i][1] = 0;
            
                    samplesR[i][0] = 0;
                    samplesR[i][1] = v;
            
                    samplesLR[i][0] = v;
                    samplesLR[i][1] = v;
                }
                i2s_1nit = true;
            }
        }
        else if (i2s_1nit && (L || R)) {
            i2s_dma_write(
                &i2s_config,
                (int16_t*)(L && R ? samplesLR : (L ? samplesL : samplesR))
            );
        }
        else {
            sleep_ms(100);
        }
        if (nstate != nespad_state || nstate2 != nespad_state2)
            goutf(TEXTMODE_ROWS - 2, false, "NES PAD: %04Xh %04Xh ", nespad_state, nespad_state2);
    }
    draw_text("Volage - PageUp/PageDown; Freq. - NumPad +/-; Ins/Del", 0, TEXTMODE_ROWS - 3, 7, 0);
    __unreachable();
}
