#include "board.h"

#include "esp_log.h"

static i2c_master_bus_handle_t s_bus;

i2c_master_bus_handle_t board_i2c_bus(void)
{
    if (!s_bus) {
        i2c_master_bus_config_t cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = PIN_I2C_SDA,
            .scl_io_num = PIN_I2C_SCL,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true, /* the board has external pull-ups; this only helps */
        };
        esp_err_t err = i2c_new_master_bus(&cfg, &s_bus);
        if (err != ESP_OK) {
            ESP_LOGE("board", "I2C bus init failed: %s", esp_err_to_name(err));
            s_bus = NULL;
        }
    }
    return s_bus;
}
