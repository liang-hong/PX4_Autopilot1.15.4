#pragma once
#include <cstdint>
#include <lib/drivers/device/i2c.h>  // <-- this one, not px4_platform_common/i2c.h

class SH1106 : public device::I2C {
public:
    SH1106(int bus, int addr);
    int  init();
    void clear();
    void contrast(uint8_t c);
    void set_cursor(uint8_t col, uint8_t row);
    void print(const char *s);
    void print_line(uint8_t row, const char *s);

private:
    int  send_cmd(uint8_t c);
    int  send_cmds(const uint8_t *cmds, size_t n);
    int  send_data(const uint8_t *data, size_t n);
    void set_page_col(uint8_t page, uint8_t col);

    uint8_t _col{0};
    uint8_t _row{0};
};