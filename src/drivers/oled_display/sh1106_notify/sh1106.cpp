// sh1106.cpp
#include "sh1106.hpp"
#include "font6x8.hpp"
#include <cstring>
#include <cstdint>
#include <lib/drivers/device/i2c.h>


// ----- SH1106 command constants (subset)
static constexpr uint8_t CMD_DISPLAY_OFF   = 0xAE;
static constexpr uint8_t CMD_DISPLAY_ON    = 0xAF;
static constexpr uint8_t CMD_SET_DISP_CLK  = 0xD5;
static constexpr uint8_t CMD_SET_MUX       = 0xA8;
static constexpr uint8_t CMD_SET_OFFSET    = 0xD3;
static constexpr uint8_t CMD_SET_START     = 0x40;
static constexpr uint8_t CMD_SEG_REMAP     = 0xA1;
static constexpr uint8_t CMD_COM_SCAN_DEC  = 0xC8;
static constexpr uint8_t CMD_SET_CONTRAST  = 0x81;
static constexpr uint8_t CMD_SET_PRECHRG   = 0xD9;
static constexpr uint8_t CMD_SET_VCOM      = 0xDB;
static constexpr uint8_t CMD_ENTIRE_ON     = 0xA4;
static constexpr uint8_t CMD_NORMAL_DISP   = 0xA6;

// NOTE: For SH1106 the charge pump control command is 0xAD (not 0x8D like SSD1306)
static constexpr uint8_t CMD_CHARGE_PUMP   = 0xAD;

// Control bytes for I2C
static constexpr uint8_t CTRL_CMD  = 0x00;
static constexpr uint8_t CTRL_DATA = 0x40;

// Visible area on most SH1106 128x64 modules starts at column 2
static constexpr uint8_t COL_OFFSET = 2;

// ---- SH1106 (plain I2C helper; NOT a ScheduledWorkItem)

SH1106::SH1106(int bus, int addr)
: device::I2C(0 /*devtype*/, "sh1106", bus, static_cast<uint16_t>(addr), 400000)
{}
int SH1106::init() {
    int ret = I2C::init();
    if (ret != PX4_OK) {
        PX4_ERR("I2C::init failed (%d)", ret);
        return ret;
    }

    const uint8_t init_seq[] = {
        0xAE,             // display off
        0xA1,             // segment remap (mirror X)
        0xC8,             // COM scan direction (mirror Y)
        0xA8, 0x3F,       // multiplex ratio: 1/64
        0xD5, 0x80,       // clock divide (default)
        0xD3, 0x00,       // display offset
        0xDB, 0x40,       // VCOMH deselect
        0x81, 0xCF,       // contrast
        0xAD, 0x8B,       // DC-DC control: internal on
        0x40,             // display start line = 0
        0xDA, 0x12,       // COM pins
        0xD9, 0xF1,       // pre-charge
        0xA4,             // resume display from RAM
        0xA6,             // normal (not inverted)
        0xAF              // display ON
    };

    ret = send_cmds(init_seq, sizeof(init_seq));
    if (ret != PX4_OK) return ret;

    clear();
    set_cursor(0, 0);
    return PX4_OK;
}

void SH1106::contrast(uint8_t c)
{
    const uint8_t cmds[] = { CMD_SET_CONTRAST, c };
    (void)send_cmds(cmds, sizeof(cmds));
}
void SH1106::clear() {
    for (uint8_t page = 0; page < 8; page++) {
        set_page_col(page, 0);
        uint8_t zeros[128] = {};
        send_data(zeros, sizeof(zeros));
    }
    set_cursor(0, 0);
}
void SH1106::set_cursor(uint8_t col, uint8_t row)
{
    _col = col;   // character column (0..20)
    _row = row;   // page      row   (0..7)
}

void SH1106::print(const char *s)
{
    while (*s) {
        char c = *s++;
        if (c == '\n') {
            _row++;
            _col = 0;
            if (_row >= 8) break;
            continue;
        }
        if (c < 0x20 || c > 0x7F) c = '?';

        // font is 6x8, one column pad included in glyph
        const uint8_t *ch = &g_font6x8[c - 0x20][0];

        set_page_col(_row, static_cast<uint8_t>(_col * 6));
        (void)send_data(ch, 6);

        _col++;
        if (_col >= 21) {
            _col = 0;
            _row++;
            if (_row >= 8) break;
        }
    }
}

void SH1106::print_line(uint8_t row, const char *s)
{
    set_page_col(row, 0);

    // One full page line (128 columns)
    uint8_t buf[128] = {};
    uint8_t x = 0;

    while (*s && x < 21) {
        char c = *s++;
        if (c < 0x20 || c > 0x7F) c = '?';

        const uint8_t *ch = &g_font6x8[c - 0x20][0];
        // copy 6 bytes per glyph
        std::memcpy(&buf[x * 6], ch, 6);
        x++;
    }

    (void)send_data(buf, sizeof(buf));
}

int SH1106::send_cmd(uint8_t c) {
    uint8_t buf[2] = {0x00, c};               // 0x00 = command
    return transfer(buf, sizeof(buf), nullptr, 0);
}

int SH1106::send_cmds(const uint8_t *cmds, size_t n) {
    for (size_t i = 0; i < n; i++) {
        int ret = send_cmd(cmds[i]);
        if (ret != PX4_OK) return ret;
    }
    return PX4_OK;
}

int SH1106::send_data(const uint8_t *data, size_t n) {
    while (n) {
        size_t chunk = n > 16 ? 16 : n;       // small chunks are safe
        uint8_t tmp[1 + 16];
        tmp[0] = 0x40;                        // 0x40 = data
        memcpy(&tmp[1], data, chunk);
        int ret = transfer(tmp, 1 + chunk, nullptr, 0);
        if (ret != PX4_OK) return ret;
        data += chunk;
        n -= chunk;
    }
    return PX4_OK;
}




void SH1106::set_page_col(uint8_t page, uint8_t col) {
    uint8_t hw_col = uint8_t(col + 2); // +2 for SH1106
    send_cmd(0xB0 | (page & 0x07));                 // page
    send_cmd(0x00 | (hw_col & 0x0F));               // low nibble
    send_cmd(0x10 | ((hw_col >> 4) & 0x0F));        // high nibble
}
