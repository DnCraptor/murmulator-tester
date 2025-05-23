#include "graphics.h"
#include <string.h>

extern volatile bool SELECT_VGA;

int get_text_cols() {
    return SELECT_VGA ? 80 : 53;
}

#ifdef DEBUG_LOG
#include "ff.h"
#endif

void draw_text(const char* string, uint32_t x, uint32_t y, uint8_t color, uint8_t bgcolor) {
    uint8_t* t_buf = text_buffer + TEXTMODE_COLS * 2 * y + 2 * x;
    for (int xi = TEXTMODE_COLS * 2; xi--;) {
        if (!*string) break;
        *t_buf++ = *string++;
        *t_buf++ = bgcolor << 4 | color & 0xF;
    }
#ifdef DEBUG_LOG
    FIL f;
    f_open(&f, DEBUG_LOG, FA_WRITE | FA_OPEN_ALWAYS | FA_OPEN_APPEND);
    UINT bw;
    f_write(&f, string, strlen(string), &bw);
    f_write(&f, "\n", 1, &bw);
    f_close(&f);
#endif
}

void draw_window(const char* title, uint32_t x, uint32_t y, uint32_t width, uint32_t height) {
    char line[width + 1];
    memset(line, 0, sizeof line);
    width--;
    height--;
    // Рисуем рамки

    memset(line, 0xCD, width); // ═══


    line[0] = 0xC9; // ╔
    line[width] = 0xBB; // ╗
    draw_text(line, x, y, 11, 1);

    line[0] = 0xC8; // ╚
    line[width] = 0xBC; //  ╝
    draw_text(line, x, height + y, 11, 1);

    memset(line, ' ', width);
    line[0] = line[width] = 0xBA;

    for (int i = 1; i < height; i++) {
        draw_text(line, x, y + i, 11, 1);
    }

    snprintf(line, width - 1, " %s ", title);
    draw_text(line, x + (width - strlen(line)) / 2, y, 14, 3);
}
