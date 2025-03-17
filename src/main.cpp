#include <cstdlib>
#include <cstring>
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
#include "psram_spi.h"
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

volatile static bool ctrlPressed = false;
volatile static bool altPressed = false;
volatile static bool Spressed = false;
volatile static bool Lpressed = false;
volatile static bool Rpressed = false;
volatile static bool Ipressed = false;
volatile static bool i2s_1nit = false;

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
            watchdog_reboot(0, 0, 0);
        }
        return true;
    }
}
static uint16_t dma_buffer[22050 / 60];
static i2s_config_t i2s_config = {
    .sample_freq = 22050,
    .channel_count = 2,
    .data_pin = AUDIO_DATA_PIN,
    .clock_pin_base = AUDIO_CLOCK_PIN,
    .pio = pio1,
    .sm = 0,
    .dma_channel = 0,
    .dma_trans_count = sizeof(dma_buffer) / sizeof(uint32_t), // Number of 32 bits words to transfer
    .dma_buf = 0,
    .volume = 0, // 16 - is 0
};
    
static semaphore vga_start_semaphore;
static uint16_t SCREEN[TEXTMODE_ROWS][TEXTMODE_COLS];

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
        sleep_ms(250);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
        sleep_ms(150);
    }
}

static bool testPinPlus1(uint32_t pin, const char* msg) {
    bool res = false;
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    sleep_ms(33);
    gpio_put(pin, false);

    gpio_init(pin + 1);
    gpio_set_dir(pin + 1, GPIO_IN);
    gpio_pull_down(pin + 1);

    gpio_put(pin, true);
    sleep_ms(33);
    res = gpio_get(pin + 1);
    if (res) {
        if (msg)
            printf("GPIO %d connected to %d (%s)\n", pin, pin + 1, msg);
        else {
            printf("GPIO %d connected to %d\n", pin, pin + 1);
            blink(pin);
        }
    }
    gpio_put(pin, false);
    sleep_ms(33);

    gpio_deinit(pin);
    gpio_deinit(pin + 1);
    sleep_ms(1000);
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

int main() {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    printf("Starting...\n");

    set_sys_clock_khz(252 * KHZ, 0);

    printf("252 MHz\n");

    /// startup signal
    for (int i = 0; i < 2; i++) {
        sleep_ms(50);
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(50);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
    }

    bool links[32] = { false };
    bool videoLinks = false;
    bool frenkLinks = false;
    for(uint32_t pin = VGA_BASE_PIN; pin < VGA_BASE_PIN + 7; ++pin) {
        if (pin == VGA_BASE_PIN || pin == VGA_BASE_PIN + 2 || pin == VGA_BASE_PIN + 4) {
            #ifdef VGA
            continue;
            #endif
            links[pin] = testPinPlus1(pin, 0);
            if (links[pin]) frenkLinks = true;
        } else {
            links[pin] = testPinPlus1(pin, 0);
            if (links[pin]) videoLinks = true;
        }
    }

    /// vga-done startup signal
    for (int i = 0; i < 4; i++) {
        sleep_ms(50);
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(50);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
    }

    sem_init(&vga_start_semaphore, 0, 1);
    multicore_launch_core1(render_core);
    sem_release(&vga_start_semaphore);

    sleep_ms(250);

    int y = 2;
    draw_text("252 MHz board was started, without overvoltage", 0, y++, 7, 0);
    if (videoLinks) {
        draw_text("Video out was tested with issues", 0, y++, 12, 0);
        draw_text(" (try disconnect video cable)", 0, y++, 12, 0);
        for(uint32_t pin = VGA_BASE_PIN; pin < VGA_BASE_PIN + 7; ++pin) {
            if (links[pin]) {
                goutf(y++, false, "GPIO %d connected to %d", pin, pin + 1);
            }
        }
    } else {
        if (frenkLinks) {
            draw_text("Video out was tested with issues, may be Frank board", 0, y++, 7, 0);
            for(uint32_t pin = VGA_BASE_PIN; pin < VGA_BASE_PIN + 7; ++pin) {
                if (links[pin]) {
                    goutf(y++, false, "GPIO %d connected to %d", pin, pin + 1);
                }
            }
        } else {
            draw_text("Video out was started", 0, y++, 7, 0);
        }
    }

    bool sdcardLinks = false;
    for(uint32_t pin = SDCARD_PIN_SPI0_SCK; pin < SDCARD_PIN_SPI0_SCK + 4; ++pin) {
        links[pin] = testPinPlus1(pin, "SDCARD inserted?");
        if (links[pin]) {
            sdcardLinks = true;
            goutf(y++, false, "GPIO %d connected to %d (SDCARD inserted?)", pin, pin + 1);
        }
    }
    if ( sdcardLinks ) {
        FATFS fs;
        if (f_mount(&fs, "SD", 1) == FR_OK) {
            goutf(y++, false, "SDCARD %d FATs; %d free clusters (%d KB each)", fs.n_fats, f_getfree32(&fs), fs.csize >> 1);
        } else {
            draw_text("SDCARD port looks NOK", 0, y++, 12, 0);
        }
    } else {
        draw_text("SDCARD port looks ok (no card installed)", 0, y++, 7, 0);
    }

    bool tryKeyboard = false;
    for(uint32_t pin = 0; pin < 2; ++pin) {
        links[pin] = testPinPlus1(pin, "keyboard?");
        if (links[pin]) {
            tryKeyboard = true;
            goutf(y++, false, "GPIO %d connected to %d (keyboard?)", pin, pin + 1);
        }
    }
    if (tryKeyboard) {
        keyboard_init();
    }
    bool tryNespad = false;
    for(uint32_t pin = NES_GPIO_CLK; pin < 21; ++pin) {
        links[pin] = testPinPlus1(pin, "NES PAD attached?");
        if (links[pin]) {
            tryNespad = true;
            goutf(y++, false, "GPIO %d connected to %d (NES PAD attached?)", pin, pin + 1);
        }
    }
    if (tryNespad) {
        nespad_begin(clock_get_hz(clk_sys) / 1000, NES_GPIO_CLK, NES_GPIO_DATA, NES_GPIO_LAT);
    }
    for(uint32_t pin = 26; pin < 27; ++pin) {
        links[pin] = testPinPlus1(pin, "?");
        if (links[pin]) {
            goutf(y++, false, "GPIO %d connected to %d (?)", pin, pin + 1);
        }
    }

    uint8_t rx[4];
    get_cpu_flash_jedec_id(rx);
    uint32_t flash_size = (1 << rx[3]);
    goutf(y++, false, "FLASH %d MB; JEDEC ID: %02X-%02X-%02X-%02X",
             flash_size >> 20, rx[0], rx[1], rx[2], rx[3]
    );

    printf("Test flash write ... ");
    if (write_flash()) {
        printf("Test write to FLASH - passed\n");
        draw_text("Test write to FLASH - passed", 0, y++, 7, 0);
    } else {
        printf("Test write to FLASH - failed\n");
        draw_text("Test write to FLASH - failed", 0, y++, 12, 0);
    }

    init_psram();
    uint32_t psram32 = psram_size();
    uint8_t rx8[8];
    psram_id(rx8);
    if (psram32) {
        goutf(y++, false, "PSRAM %d MB; MF ID: %02x; KGD: %02x; EID: %02X%02X-%02X%02X-%02X%02X",
                          psram32 >> 20, rx8[0], rx8[1], rx8[2], rx8[3], rx8[4], rx8[5], rx8[6], rx8[7]);

        uint32_t a = 0;
        uint32_t begin = time_us_32();
        for (; a < psram32; ++a) {
            write8psram(a, a & 0xFF);
        }
        uint32_t elapsed = time_us_32() - begin;
        double d = 1.0;
        double speed = d * a / elapsed;
        goutf(y++, false, "8-bit line write speed: %f MBps", speed);
        begin = time_us_32();
        for (a = 0; a < psram32; ++a) {
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
            write16psram(a, a & 0xFFFF);
        }
        elapsed = time_us_32() - begin;
        speed = d * a / elapsed;
        goutf(y++, false, "16-bit line write speed: %f MBps", speed);
   
        begin = time_us_32();
        for (a = 0; a < psram32; a += 2) {
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
            write32psram(a, a);
        }
        elapsed = time_us_32() - begin;
        speed = d * a / elapsed;
        goutf(y++, false, "32-bit line write speed: %f MBps", speed);
    
        begin = time_us_32();
        for (a = 0; a < psram32; a += 4) {
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

    goutf(y++, false, "DONE");

    draw_text("S - try speaker, L - try left PWM, R - try right PWM", 0, y++, 7, 0);
    draw_text("I - try i2s sound (+L/R)", 0, y, 7, 0);

    for (int i = 0; i < 8; i++) {
        sleep_ms(33);
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(33);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
    }

    bool pwm_init = false;
    while(true) {
        if (!i2s_1nit && (Spressed || Rpressed || Lpressed)) {
            if (!pwm_init) {
                PWM_init_pin(BEEPER_PIN, (1 << 12) - 1);
                PWM_init_pin(PWM_PIN0  , (1 << 12) - 1);
                PWM_init_pin(PWM_PIN1  , (1 << 12) - 1);
                draw_text(" (restart to test i2s)   ", 0, y, 7, 0);
                pwm_init = true;
            }
            if (Spressed) pwm_set_gpio_level(BEEPER_PIN, (1 << 12) - 1);
            if (Lpressed) pwm_set_gpio_level(PWM_PIN0  , (1 << 12) - 1);
            if (Rpressed) pwm_set_gpio_level(PWM_PIN1  , (1 << 12) - 1);
            sleep_ms(1);
            if (Spressed) pwm_set_gpio_level(BEEPER_PIN, 0);
            if (Lpressed) pwm_set_gpio_level(PWM_PIN0  , 0);
            if (Rpressed) pwm_set_gpio_level(PWM_PIN1  , 0);
            sleep_ms(1);
        }
        else if (!pwm_init && Ipressed) {
            if (!i2s_1nit) {
                draw_text(" (restart to test PWM)                               ", 0, y - 1, 7, 0);
                i2s_init(&i2s_config);
                i2s_1nit = true;
            }
        }
        else if (i2s_1nit && (Lpressed || Rpressed)) {
            uint16_t samples[] = {
                               0x0000    ,            0x0000    ,
                    Lpressed ? 0x0FFF : 0, Rpressed ? 0x0FFF : 0,
                    Lpressed ? 0x1F00 : 0, Rpressed ? 0x1F00 : 0,
                    Lpressed ? 0x1FFF : 0, Rpressed ? 0x1FFF : 0,
                    Lpressed ? 0x3FFF : 0, Rpressed ? 0x3FFF : 0,
                    Lpressed ? 0x5FFF : 0, Rpressed ? 0x5FFF : 0,
                    Lpressed ? 0x7FFF : 0, Rpressed ? 0x7FFF : 0,
                    Lpressed ? 0xAFFF : 0, Rpressed ? 0xaFFF : 0,
                    Lpressed ? 0xFFFF : 0, Rpressed ? 0xFFFF : 0,
                    Lpressed ? 0xAFFF : 0, Rpressed ? 0xaFFF : 0,
                    Lpressed ? 0x7FFF : 0, Rpressed ? 0x7FFF : 0,
                    Lpressed ? 0x5FFF : 0, Rpressed ? 0x5FFF : 0,
                    Lpressed ? 0x3FFF : 0, Rpressed ? 0x3FFF : 0,
                    Lpressed ? 0x1F00 : 0, Rpressed ? 0x1F00 : 0,
                    Lpressed ? 0x1FFF : 0, Rpressed ? 0x1FFF : 0,
                    Lpressed ? 0x0FFF : 0, Rpressed ? 0x0FFF : 0,
            };
            i2s_write(&i2s_config, samples, sizeof(samples) / sizeof(uint32_t));
        }
        else {
            sleep_ms(100);
            if (tryNespad) {
                nespad_read();
                goutf(TEXTMODE_ROWS - 2, false, "NES PAD: %02Xh %02Xh ", nespad_state, nespad_state2);
            }
        }
    }

    __unreachable();
}
