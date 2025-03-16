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

#include "graphics.h"
#include "psram_spi.h"
#include "ff.h"

extern "C" {
    #include "ps2.h"
    bool __time_critical_func(handleScancode)(const uint32_t ps2scancode) {
        char tmp[80];
        snprintf(tmp, 80, "Last scancode: %04Xh", ps2scancode);
        printf("%s\n", tmp);
        draw_text(tmp, 0, TEXTMODE_ROWS - 2, 7, 0);
        return true;
    }
}
        
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
    while (true) {
        if (tick >= last_frame_tick + frame_tick) {
#ifdef TFT
            refresh_lcd();
#endif
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
    char tmp[80];
    if (videoLinks) {
        draw_text("Video out was tested with issues", 0, y++, 12, 0);
        draw_text(" (try disconnect video cable)", 0, y++, 12, 0);
        for(uint32_t pin = VGA_BASE_PIN; pin < VGA_BASE_PIN + 7; ++pin) {
            if (links[pin]) {
                snprintf(tmp, 80, "GPIO %d connected to %d", pin, pin + 1);
                draw_text(tmp, 0, y++, 7, 0);
            }
        }
    } else {
        if (frenkLinks) {
            draw_text("Video out was tested with issues, may be Frank board", 0, y++, 7, 0);
            for(uint32_t pin = VGA_BASE_PIN; pin < VGA_BASE_PIN + 7; ++pin) {
                if (links[pin]) {
                    snprintf(tmp, 80, "GPIO %d connected to %d", pin, pin + 1);
                    draw_text(tmp, 0, y++, 7, 0);
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
            snprintf(tmp, 80, "GPIO %d connected to %d (SDCARD inserted?)", pin, pin + 1);
            draw_text(tmp, 0, y++, 7, 0);
        }
    }
    if ( sdcardLinks ) {
        FATFS fs;
        if (f_mount(&fs, "SD", 1) == FR_OK) {
            snprintf(tmp, 80, "SDCARD %d FATs; %d free clusters (%d KB each)", fs.n_fats, f_getfree32(&fs), fs.csize >> 1);
            printf("%s\n", tmp);
            draw_text(tmp, 0, y++, 7, 0);
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
            snprintf(tmp, 80, "GPIO %d connected to %d (keyboard?)", pin, pin + 1);
            draw_text(tmp, 0, y++, 7, 0);
        }
    }
    if (tryKeyboard) {
        keyboard_init();
    }
    for(uint32_t pin = NES_GPIO_CLK; pin < 21; ++pin) {
        links[pin] = testPinPlus1(pin, "NES PAD attached?");
        if (links[pin]) {
            snprintf(tmp, 80, "GPIO %d connected to %d (NES PAD attached?)", pin, pin + 1);
            draw_text(tmp, 0, y++, 7, 0);
        }
    }
    for(uint32_t pin = 26; pin < 27; ++pin) {
        links[pin] = testPinPlus1(pin, "?");
        if (links[pin]) {
            snprintf(tmp, 80, "GPIO %d connected to %d (?)", pin, pin + 1);
            draw_text(tmp, 0, y++, 7, 0);
        }
    }

    uint8_t rx[4];
    get_cpu_flash_jedec_id(rx);
    uint32_t flash_size = (1 << rx[3]);
    snprintf(tmp, 80, "FLASH %d MB; JEDEC ID: %02X-%02X-%02X-%02X",
             flash_size >> 20, rx[0], rx[1], rx[2], rx[3]
    );
    draw_text(tmp, 0, y++, 7, 0);
    printf("%s\n", tmp);

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
        snprintf(tmp, 80, "PSRAM %d MB; MF ID: %02x; KGD: %02x; EID: %02X%02X-%02X%02X-%02X%02X",
                 psram32 >> 20, rx8[0], rx8[1], rx8[2], rx8[3], rx8[4], rx8[5], rx8[6], rx8[7]);
        draw_text(tmp, 0, y++, 7, 0);
        printf("%s\n", tmp);

        uint32_t a = 0;
        uint32_t begin = time_us_32();
        for (; a < psram32; ++a) {
            write8psram(a, a & 0xFF);
        }
        uint32_t elapsed = time_us_32() - begin;
        double d = 1.0;
        double speed = d * a / elapsed;
        snprintf(tmp, 80, "8-bit line write speed: %f MBps", speed);
        draw_text(tmp, 0, y++, 7, 0);
        printf("%s\n", tmp);
        begin = time_us_32();
        for (a = 0; a < psram32; ++a) {
            if ((a & 0xFF) != read8psram(a)) {
                snprintf(tmp, 80, "8-bit read failed at %ph", a);
                draw_text(tmp, 0, y++, 12, 0);
                printf("%s\n", tmp);
                break;
            }
        }
        elapsed = time_us_32() - begin;
        speed = d * a / elapsed;
        snprintf(tmp, 80, "8-bit line read speed: %f MBps", speed);
        draw_text(tmp, 0, y++, 7, 0);
        printf("%s\n", tmp);
    
        begin = time_us_32();
        for (a = 0; a < psram32; a += 2) {
            write16psram(a, a & 0xFFFF);
        }
        elapsed = time_us_32() - begin;
        speed = d * a / elapsed;
        snprintf(tmp, 80, "16-bit line write speed: %f MBps", speed);
        draw_text(tmp, 0, y++, 7, 0);
        printf("%s\n", tmp);
    
        begin = time_us_32();
        for (a = 0; a < psram32; a += 2) {
            if ((a & 0xFFFF) != read16psram(a)) {
                snprintf(tmp, 80, "16-bit read failed at %ph", a);
                draw_text(tmp, 0, y++, 12, 0);
                printf("%s\n", tmp);
                break;
            }
        }
        elapsed = time_us_32() - begin;
        speed = d * a / elapsed;
        snprintf(tmp, 80, "16-bit line read speed: %f MBps", speed);
        draw_text(tmp, 0, y++, 7, 0);
        printf("%s\n", tmp);
    
        begin = time_us_32();
        for (a = 0; a < psram32; a += 4) {
            write32psram(a, a);
        }
        elapsed = time_us_32() - begin;
        speed = d * a / elapsed;
        snprintf(tmp, 80, "32-bit line write speed: %f MBps", speed);
        draw_text(tmp, 0, y++, 7, 0);
        printf("%s\n", tmp);
    
        begin = time_us_32();
        for (a = 0; a < psram32; a += 4) {
            if (a != read32psram(a)) {
                snprintf(tmp, 80, "32-bit read failed at %ph", a);
                draw_text(tmp, 0, y++, 12, 0);
                printf("%s\n", tmp);
                break;
            }
        }
        elapsed = time_us_32() - begin;
        speed = d * a / elapsed;
        snprintf(tmp, 80, "32-bit line read speed: %f MBps", speed);
        draw_text(tmp, 0, y++, 7, 0);
        printf("%s\n", tmp);
    } else {
        draw_text("No PSRAM detected", 0, y++, 7, 0);
        printf("No PSRAM detected\n");
    }

    printf("DONE\n");
    draw_text("DONE", 0, y, 7, 0);

    for (int i = 0; i < 8; i++) {
        sleep_ms(33);
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(33);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
    }

    while(1) sleep_ms(1);
    __unreachable();
}
