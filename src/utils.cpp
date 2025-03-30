#include <stdint.h>
#include <pico/multicore.h>
#include <hardware/flash.h>
#include "psram_spi.h"
extern "C" {
    #include "sd_util.h"
    extern volatile int y;
}

inline static void _flash_do_cmd(const uint8_t *tx, uint8_t *rx, size_t count) {
    multicore_lockout_start_blocking();
    const uint32_t ints = save_and_disable_interrupts();
    flash_do_cmd(tx, rx, count);
    restore_interrupts(ints);
    multicore_lockout_end_blocking();
}

void get_cpu_flash_jedec_id(uint8_t _rx[4]) {
    static uint8_t rx[4] = {0};
    if (rx[0] == 0) {
        uint8_t tx[4] = {0x9f};
        _flash_do_cmd(tx, rx, 4);
    }
    *(unsigned*)_rx = *(unsigned*)rx;
}

// --- Список производителей по JEDEC ID ---
typedef struct {
    uint8_t id;
    const char *name;
} jedec_vendor_t;

jedec_vendor_t vendors[] = {
    {0xEF, "Winbond"},
    {0xC2, "Macronix"},
    {0x20, "Micron"},
    {0x1F, "Atmel"},
    {0x9D, "ISSI"},
    {0x01, "Spansion"},
    {0x00, "Unknown"}
};

// --- Функция для поиска имени производителя ---
inline static const char* get_vendor_name(uint8_t id) {
    for (size_t i = 0; i < sizeof(vendors) / sizeof(vendors[0]); i++) {
        if (vendors[i].id == id) return vendors[i].name;
    }
    return "Unknown";
}

// --- Функция расшифровки битов статусного регистра ---
inline static void print_status_bits(uint8_t status) {
    printf("  - Write Protect (WP): %s", (status & 0x80) ? "Enabled" : "Disabled");
    printf("  - Block Protect (BP): %s", (status & 0x0C) ? "Enabled" : "Disabled");
    printf("  - Write In Progress (WIP): %s", (status & 0x01) ? "Yes" : "No");
}

// --- Расшифровка Memory Type ---
inline static const char* get_memory_type(uint8_t type) {
    switch (type) {
        case 0x20: return "NOR Flash (Micron, Spansion, ISSI)";
        case 0x40: return "Serial NOR Flash (Winbond, Macronix, GigaDevice)";
        case 0x60: return "Parallel NOR Flash";
        case 0x70: return "NAND Flash (Micron, Samsung)";
        case 0x80: return "EEPROM / OTP Memory";
        case 0x90: return "Multi-Level Cell (MLC) NAND Flash";
        case 0xD0: return "Quad-SPI NOR Flash";
        default:   return "Unknown type of memory";
    }
}

// --- Универсальная функция для чтения информации о SPI Flash ---
void get_flash_info() {
    uint8_t rx[16] = {0};

    // --- Читаем JEDEC ID (0x9F) ---
    uint8_t cmd_jedec[4] = {0x9F, 0, 0, 0}; 
    _flash_do_cmd(cmd_jedec, rx, 4);
    printf("=== JEDEC ID (9Fh) ===");
    printf("Manufacturer: %02X-%02X %s", rx[0], rx[1], get_vendor_name(rx[1]));
    printf("Memory Type:  0x%02X %s", rx[2], get_memory_type(rx[2]));
    printf("Capacity:     0x%02X %d B", rx[3], 1 << rx[3]);

    // --- Читаем Manufacturer ID (0x90) ---
    uint8_t cmd_mfid[5] = {0x90, 0x00, 0x00, 0x00, 0x00}; 
    _flash_do_cmd(cmd_mfid, rx, 2);
    printf("=== Manufacturer ID (90h) ===");
    printf("Manufacturer: 0x%02X", rx[0]);
    printf("Device ID:    0x%02X", rx[1]);

    // --- Читаем SFDP (0x5A) ---
    uint8_t cmd_sfdp[5] = {0x5A, 0x00, 0x00, 0x00, 0x00}; 
    _flash_do_cmd(cmd_sfdp, rx, 8);
    printf("=== SFDP Header (5Ah) ===");
    printf("Signature:    %c%c%c%c", rx[0], rx[1], rx[2], rx[3]);
    printf("SFDP Version: 0x%02X%02X", rx[4], rx[5]);

    // --- Читаем Unique ID (0x4B) ---
    uint8_t cmd_uid[5] = {0x4B, 0, 0, 0, 0};  
    _flash_do_cmd(cmd_uid, rx, 8);
    printf("=== Unique ID (4Bh) ===");
    printf("ID: %02X %02X %02X %02X %02X %02X %02X %02X",
           rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6], rx[7]);

    // --- Читаем регистры статуса (0x05 и 0x35) ---
    uint8_t cmd_status1[2] = {0x05, 0};  
    _flash_do_cmd(cmd_status1, rx, 2);
    printf("=== Status Register 1 (05h) ===");
    printf("Raw Value: 0x%02X", rx[0]);
    print_status_bits(rx[0]);

    uint8_t cmd_status2[2] = {0x35, 0};  
    _flash_do_cmd(cmd_status2, rx, 2);
    printf("=== Status Register 2 (35h) ===");
    printf("Raw Value: 0x%02X", rx[0]);
    printf("  - Quad Enable (QE): %s", (rx[0] & 0x02) ? "Enabled" : "Disabled");
}

// Таблица производителей PSRAM по JEDEC ID
typedef struct {
    uint8_t id;
    const char *name;
} psram_vendor_t;

psram_vendor_t psram_vendors[] = {
    {0x0D, "AP Memory"},
    {0xC8, "GigaDevice"},
    {0x85, "ESMT"},
    {0xA1, "Alliance Memory"},
    {0x00, "Unknown"}
};

// --- Функция поиска производителя по MFID ---
inline static const char* get_psram_vendor(uint8_t id) {
    for (size_t i = 0; i < sizeof(psram_vendors) / sizeof(psram_vendors[0]); i++) {
        if (psram_vendors[i].id == id) return psram_vendors[i].name;
    }
    return "Unknown";
}

inline static const char* get_kgd_status(uint8_t kgd) {
    switch (kgd) {
        case 0x00: return "Reject";
        case 0x01: return "Good Die";
        case 0x02: return "Limited";
        default:   return "Unknown";
    }
}

void print_psram_info() {
    uint32_t psram32 = psram_size();
    if (!psram32) {
        printf("No PSRAM");
        return;
    }
    uint8_t rx8[8];
    psram_id(rx8);

    printf("PSRAM %d MB; MFID: %02X (%s)",
           psram32 >> 20, 
           rx8[0], get_psram_vendor(rx8[0])  // Производитель
    );
    printf("Known Good Die: %02X (%s)",
        rx8[1], get_kgd_status(rx8[1])            // KGD (тестирование)
    );
    printf("EID: %02X%02X-%02X%02X-%02X%02X",
           rx8[2], rx8[3], rx8[4], rx8[5], rx8[6], rx8[7] // Extended ID (серийник)
    );
}

void get_sdcard_info() {

}
