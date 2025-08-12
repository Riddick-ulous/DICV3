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
    if (page >= DISP_MAX_PAGES) { Error_Register(ERROR_DISPLAY_INVALID_PAGE); return; }
    s_active_page = page;
    // KEIN Render hier!
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

void Display_ErrorStatus(uint8_t page, uint8_t row, uint8_t col)
{
    uint32_t errors = Error_GetAll();

    // Plausibilitäts-Checks
    if (page >= DISP_MAX_PAGES) {
        Error_Register(ERROR_DISPLAY_INVALID_PAGE);
        return;
    }
    if (row >= DISP_ROWS) {
        Error_Register(ERROR_DISPLAY_INVALID_ROW);
        return;
    }
    if (col >= DISP_COLS) {
        Error_Register(ERROR_DISPLAY_INVALID_COL);
        return;
    }

    // Temporär aktive Page sichern
    uint8_t old_page = DISP_GetPage();

    // Gewünschte Page aktivieren
    DISP_SetPage(page);

    // Fehlerstatus schreiben
    if (errors == 0) {
        DISP_WriteText(row, col, "Status: OK");
    } else {
        char buffer[DISP_COLS + 1];
        snprintf(buffer, sizeof(buffer), "Err: 0x%08lX", (unsigned long)errors);
        DISP_WriteText(row, col, buffer);
    }

    // Ursprüngliche Page wiederherstellen
    DISP_SetPage(old_page);
}

// 1bpp: jedes Byte = 8 Pixel, MSB = linkes Pixel (row-major)
void DISP_DrawBitmap(const uint8_t* bmp, uint8_t w, uint8_t h, uint8_t x, uint8_t y)
{
    if (!bmp) return;
    // Display löschen wir NICHT hier – nur blitten
    for (uint8_t row = 0; row < h; ++row) {
        for (uint8_t col = 0; col < w; ++col) {
            uint32_t bitIndex = (uint32_t)row * w + col;
            uint8_t byte      = bmp[bitIndex >> 3];
            uint8_t mask      = 0x80u >> (bitIndex & 7);
            if (byte & mask) {
                ssd1306_DrawPixel(x + col, y + row, White);
            } else {
                ssd1306_DrawPixel(x + col, y + row, Black); // optional: Hintergrund „löschen“
            }
        }
    }
}

// Erwartet XBM-Layout: 1bpp, row-major, LSB-first pro Byte.
// w,h = Logo-Größe in Pixeln.
// invert = true -> Farben invertieren.
// down_one_text_row = true -> zusätzlich 8 px nach unten schieben.
// Erweiterte Variante: zusätzlich msb_first-Flag steuerbar.
void DISP_ShowBootLogoEx(const uint8_t *logo,
                         uint16_t w, uint16_t h,
                         bool invert,
                         bool down_one_text_row,
                         bool msb_first)
{
    if (!logo || w == 0 || h == 0) return;

    const uint16_t DISP_W = 128;
    const uint16_t DISP_H = 64;

    // Zentrierung berechnen
    int16_t x0 = (int16_t)((DISP_W - w) / 2);
    int16_t y0 = (int16_t)((DISP_H - h) / 2);
    if (down_one_text_row) y0 += 8;

    // Optional: vorher alles schwarz machen (sauberer Splash)
    ssd1306_Fill(Black);

    // Blit mit Clipping & Bit-Extraktion (auseinander sortieren)
    BLIT_XBM_ToDisplay(logo, w, h, x0, y0, invert, msb_first);

    ssd1306_UpdateScreen();
}


// Konvertiert XBM (row-major; MSB- oder LSB-first in jedem Byte) -> SSD1306 Page-Format
void xbm_to_ssd1306_pages(uint8_t *dst, const uint8_t *src,
                                 uint16_t w, uint16_t h, bool msb_first)
{
    uint16_t pages = h / 8;
    uint16_t src_stride = (w + 7u) / 8u;   // Bytes pro Quellzeile

    for (uint16_t page = 0; page < pages; page++) {
        for (uint16_t x = 0; x < w; x++) {
            uint8_t out = 0;
            for (uint8_t bit = 0; bit < 8; bit++) {
                uint16_t y = (page * 8u) + bit;

                // Quellbyte + Bitposition in der XBM-Zeile finden
                uint32_t src_idx = (uint32_t)y * src_stride + (x / 8u);
                uint8_t  src_byte = src[src_idx];
                uint8_t  bpos = msb_first ? (7u - (x & 7u)) : (x & 7u);
                uint8_t  pix  = (src_byte >> bpos) & 1u;

                out |= (pix << bit); // in SSD1306-Page-Byte einsetzen
            }
            dst[page * w + x] = out;
        }
    }
}

// ===== Helpers zum "Auseinander sortieren" eines XBM =====

// optional: schnelles Bit-Reverse für MSB-first->LSB-first (falls du lieber Byte-vordrehen willst)
static inline uint8_t bitrev8(uint8_t v) {
    v = (uint8_t)((v & 0xF0u) >> 4) | (uint8_t)((v & 0x0Fu) << 4);
    v = (uint8_t)((v & 0xCCu) >> 2) | (uint8_t)((v & 0x33u) << 2);
    v = (uint8_t)((v & 0xAAu) >> 1) | (uint8_t)((v & 0x55u) << 1);
    return v;
}

// holt ein einzelnes Bit aus einem XBM-Buffer (row-major, 1bpp), w = Bildbreite
// msb_first=false: XBM-Standard (LSB-first je Byte). true: MSB-first Quellen.
static inline uint8_t XBM_GetBit(const uint8_t* xbm, uint16_t w,
                                 uint16_t x, uint16_t y, bool msb_first)
{
    const uint16_t bytes_per_row = (uint16_t)((w + 7u) / 8u);
    const uint8_t* row = xbm + (size_t)y * bytes_per_row;
    const uint16_t byte_idx = (uint16_t)(x >> 3);
    const uint8_t  bit_idx  = (uint8_t)(x & 7u);

    uint8_t b = row[byte_idx];
    if (!msb_first) {
        // LSB-first: Bit 0 gehört zu x%8==0
        return (uint8_t)((b >> bit_idx) & 0x1u);
    } else {
        // MSB-first: Bit 7 gehört zu x%8==0
        return (uint8_t)((b >> (7u - bit_idx)) & 0x1u);
    }
}

// blitted ein XBM auf das Display (setzt Pixel einzeln), mit Clipping, Invert und optionalem Offset.
static void BLIT_XBM_ToDisplay(const uint8_t* xbm, uint16_t w, uint16_t h,
                               int16_t x0, int16_t y0, bool invert, bool msb_first)
{
    const int16_t DISP_W = 128;
    const int16_t DISP_H = 64;

    // Sichtbares Fenster (Clipping)
    int16_t x_start = (x0 < 0) ? -x0 : 0;
    int16_t y_start = (y0 < 0) ? -y0 : 0;
    int16_t x_end   = (x0 + (int16_t)w > DISP_W) ? (DISP_W - x0) : (int16_t)w;
    int16_t y_end   = (y0 + (int16_t)h > DISP_H) ? (DISP_H - y0) : (int16_t)h;

    if (x_end <= 0 || y_end <= 0) return;

    for (int16_t yy = y_start; yy < y_end; ++yy) {
        for (int16_t xx = x_start; xx < x_end; ++xx) {
            uint8_t bit = XBM_GetBit(xbm, w, (uint16_t)xx, (uint16_t)yy, msb_first);
            if (invert) bit ^= 1u;
            ssd1306_DrawPixel((int16_t)(x0 + xx), (int16_t)(y0 + yy), bit ? White : Black);
        }
    }
}



