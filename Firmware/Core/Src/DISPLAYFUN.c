#include "DISPLAYFUN.h"
#include <string.h>
#include <stdio.h>
#include "CAN.h"
#include "ERROR.h"


// Speicher: Pages × (8 Zeilen × 21 Spalten)
static char s_pages[DISP_MAX_PAGES][DISP_ROWS * DISP_COLS];
static uint8_t s_active_page = 0;

// interne Helfer
static inline size_t idx(uint8_t row, uint8_t col) { return ((size_t)row * DISP_COLS) + col; }
static inline bool in_bounds(uint8_t row, uint8_t col) { return row < DISP_ROWS && col < DISP_COLS; }
static void clear_mem(char* mem) { memset(mem, ' ', DISP_ROWS * DISP_COLS); }

static uint8_t s_home_page = 0;  // Default-Home

void DISP_GoHome(void)
{
    DISP_SetPage(s_home_page);
}

void DISP_SendStatus(void)
{
    CAN_Message_t m = {0};
    m.id  = DISP_CAN_ID_STATUS;
    m.dlc = 1;
    m.data[0] = DISP_GetPage();
    CAN_QueueMessage(&m);
}


void DISP_Init(void)
{
    ssd1306_Init();
    for (uint8_t p = 0; p < DISP_MAX_PAGES; ++p) clear_mem(s_pages[p]);
    s_home_page = 0;
    s_active_page = s_home_page;

    // Beispiel: Header auf Page 0
    DISP_WriteText(0, 0, "Page 0");
    DISP_Render();
    DISP_SendStatus(); // einmalig den Start-Status schicken (optional)
}

void DISP_SetPage(uint8_t page)
{
    if (page >= DISP_MAX_PAGES) {
        Error_Register(ERROR_DISPLAY_INVALID_PAGE);
        return;
    }
    s_active_page = page;
    DISP_Render();
}

uint8_t DISP_GetPage(void) { return s_active_page; }

void DISP_ClearPage(uint8_t page)
{
    if (page >= DISP_MAX_PAGES) return;
    clear_mem(s_pages[page]);
}

void DISP_ClearActive(void) { clear_mem(s_pages[s_active_page]); }

void DISP_Render(void)
{
    // Blit der aktiven Page auf das Display
    for (uint8_t row = 0; row < DISP_ROWS; ++row) {
        ssd1306_SetCursor(0, row * 8);
        // Zeile als C-String puffern
        char line[DISP_COLS + 1];
        memcpy(line, &s_pages[s_active_page][idx(row, 0)], DISP_COLS);
        line[DISP_COLS] = '\0';
        ssd1306_WriteString(line, Font_6x8, White);
    }
    ssd1306_UpdateScreen();
}

void DISP_WriteText(uint8_t row, uint8_t col, const char* text)
{
    if (!text) return;
    if (row >= DISP_ROWS) { Error_Register(ERROR_DISPLAY_INVALID_ROW); return; }
    if (col >= DISP_COLS) { Error_Register(ERROR_DISPLAY_INVALID_COL); return; }

    size_t pos  = idx(row, col);
    size_t left = DISP_COLS - col;

    size_t i = 0;
    for (; i < left && text[i] != '\0'; ++i) {
        s_pages[s_active_page][pos + i] = text[i];
    }
    if (text[i] != '\0') {
        // wurde abgeschnitten
        Error_Register(ERROR_DISPLAY_WRITE_CLIPPED);
    }
}

void DISP_WriteChar(uint8_t row, uint8_t col, char c)
{
    if (row >= DISP_ROWS) { Error_Register(ERROR_DISPLAY_INVALID_ROW); return; }
    if (col >= DISP_COLS) { Error_Register(ERROR_DISPLAY_INVALID_COL); return; }
    s_pages[s_active_page][idx(row, col)] = c;
}

void DISP_WriteNumber(int32_t value, int8_t decimals, uint8_t min_width,
                      uint8_t row, uint8_t col, const char* unit)
{
	if (decimals < 0) {
	    Error_Register(ERROR_DISPLAY_BAD_DECIMALS);
	    decimals = 0; // defensiv
	}

	if (row >= DISP_ROWS) { Error_Register(ERROR_DISPLAY_INVALID_ROW); return; }
	if (col >= DISP_COLS) { Error_Register(ERROR_DISPLAY_INVALID_COL); return; }

	// Max. plausible Nachkommastellen begrenzen (z. B. 6)
	if (decimals > 6) {
	    Error_Register(ERROR_DISPLAY_BAD_DECIMALS);
	    decimals = 6;
	}

	// Restbreite der Zeile
	uint8_t rest_cols = DISP_COLS - col;
	// Wenn min_width größer als Rest: markieren, aber trotzdem so viel wie geht schreiben
	if (min_width > rest_cols) {
	    Error_Register(ERROR_DISPLAY_MINWIDTH_TOO_BIG);
	}

    // 1) Ganzzahlanteil als String ohne Punkt erzeugen
    char buf[32];        // Ziffern ohne Vorzeichen und ohne Punkt
    char out[40];        // Zahl mit optionalem '-' und Dezimalpunkt
    char final[48];      // out + optional unit
    char field[64];      // optionales Left-Padding vor final

    bool neg = (value < 0);
    uint32_t a = (neg) ? (uint32_t)(-value) : (uint32_t)value;

    if (a == 0) {
        // Spezialfall 0
        buf[0] = '0';
        buf[1] = '\0';
    } else {
        // a als String in buf (vorwärts)
        char rev[32];
        size_t n = 0;
        while (a > 0 && n < sizeof(rev)) {
            rev[n++] = (char)('0' + (a % 10));
            a /= 10;
        }
        // reverse kopieren
        size_t m = (n < sizeof(buf) - 1) ? n : (sizeof(buf) - 1);
        for (size_t i = 0; i < m; ++i) buf[i] = rev[n - 1 - i];
        buf[m] = '\0';
    }

    // 2) Dezimalpunkt einfügen → out
    if (decimals > 0) {
        size_t len2 = strlen(buf);
        size_t int_len = (len2 > (size_t)decimals) ? (len2 - (size_t)decimals) : 0;

        char intpart[32], fracpart[32];
        if (int_len == 0) {
            // 0.xxx (mit führenden Nullen im frac)
            strcpy(intpart, "0");
            size_t need_zeros = (size_t)decimals - len2;
            size_t p = 0;
            for (size_t i = 0; i < need_zeros && p < sizeof(fracpart) - 1; ++i) fracpart[p++] = '0';
            for (size_t i = 0; i < len2 && p < sizeof(fracpart) - 1; ++i) fracpart[p++] = buf[i];
            fracpart[p] = '\0';
        } else {
            if (int_len >= sizeof(intpart)) int_len = sizeof(intpart) - 1;
            memcpy(intpart, buf, int_len);
            intpart[int_len] = '\0';
            strncpy(fracpart, &buf[int_len], sizeof(fracpart) - 1);
            fracpart[sizeof(fracpart) - 1] = '\0';
        }

        // out manuell zusammenbauen (bounded)
        size_t w = 0, cap = sizeof(out);
        if (neg && w + 1 < cap) out[w++] = '-';
        for (size_t i = 0; intpart[i] && w + 1 < cap; ++i) out[w++] = intpart[i];
        if (w + 1 < cap) out[w++] = '.';
        for (size_t i = 0; fracpart[i] && w + 1 < cap; ++i) out[w++] = fracpart[i];
        out[w] = '\0';
    } else {
        // ohne Dezimalpunkt → out
        size_t w = 0, cap = sizeof(out);
        if (neg && w + 1 < cap) out[w++] = '-';
        for (size_t i = 0; buf[i] && w + 1 < cap; ++i) out[w++] = buf[i];
        out[w] = '\0';
    }

    // 3) Unit anhängen → final
    size_t f = 0, fcap = sizeof(final);
    for (size_t i = 0; out[i] && f + 1 < fcap; ++i) final[f++] = out[i];
    if (unit && unit[0]) {
        for (size_t i = 0; unit[i] && f + 1 < fcap; ++i) final[f++] = unit[i];
    }
    final[f] = '\0';

    // 4) Mindestbreite via left-padding
    size_t L = strlen(final);
    size_t pad = (min_width > L) ? (min_width - L) : 0;
    size_t w = 0, wcap = sizeof(field);
    for (size_t i = 0; i < pad && w + 1 < wcap; ++i) field[w++] = ' ';
    for (size_t i = 0; final[i] && w + 1 < wcap; ++i) field[w++] = final[i];
    field[w] = '\0';

    // 5) in Page-Buffer schreiben (clipped von DISP_WriteText)
    DISP_WriteText(row, col, field);
}

void DISP_WriteLapTime(uint32_t ms, uint8_t row, uint8_t col)
{
    // Format: M:SS.mmm
    uint32_t minutes = ms / 60000u;
    uint32_t rest    = ms % 60000u;
    uint32_t seconds = rest / 1000u;
    uint32_t millis  = rest % 1000u;

    char t[12];
    // Minuten ohne führende Null, Sekunden zweistellig, millis dreistellig
    snprintf(t, sizeof(t), "%lu:%02lu.%03lu",
             (unsigned long)minutes,
             (unsigned long)seconds,
             (unsigned long)millis);
    DISP_WriteText(row, col, t);
}


// CAN-Handler: 0x130 → Page setzen
void DISP_HandleCAN(uint16_t id, const uint8_t* data, uint8_t dlc, uint32_t now_ms)
{
    (void)now_ms;
    if (id != DISP_CAN_ID_SET_PAGE || dlc < 1 || !data) return;
    if (data[0] >= DISP_MAX_PAGES) {
        Error_Register(ERROR_DISPLAY_INVALID_PAGE);
        return;
    }
    DISP_SetPage(data[0]);
    DISP_SendStatus();
}

char* DISP_GetBufferForPage(uint8_t page)
{
    if (page >= DISP_MAX_PAGES) return NULL;
    return s_pages[page];
}
