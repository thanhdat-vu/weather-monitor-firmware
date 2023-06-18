#include <string.h>
#include <stdint.h>

#include "ssd1306.h"
#include "font8x8_basic.h"

#include "esp_err.h"
#include "esp_log.h"

#include "driver/i2c.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"

#define GPIO_NUM_SCL 22
#define GPIO_NUM_SDA 21
#define I2C_MASTER_FREQ_HZ 400000

#define OLED_CMD_BITS 8
#define OLED_CMD_BITS 8

#define GPIO_NUM_RST -1

esp_lcd_panel_io_handle_t io_handle = NULL;
esp_lcd_panel_handle_t panel_handle = NULL;

int i2c_master_port = I2C_NUM_0;
static const char *TAG = "SSD1306";

static esp_err_t i2c_master_init(void)
{
  i2c_config_t i2c_conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = GPIO_NUM_SDA,
      .scl_io_num = GPIO_NUM_SCL,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_MASTER_FREQ_HZ,
      // .clk_flags = 0,
  };
  esp_err_t err = i2c_param_config(i2c_master_port, &i2c_conf);
  if (err != ESP_OK)
  {
    return err;
  }
  return i2c_driver_install(i2c_master_port, I2C_MODE_MASTER, 0, 0, 0);
}

static esp_err_t oled_panel_io_init(void)
{
  esp_lcd_panel_io_i2c_config_t io_conf = {
      .dev_addr = OLED_I2C_ADDRESS,
      .control_phase_bytes = 1,
      .dc_bit_offset = 6,
      .lcd_cmd_bits = OLED_CMD_BITS,
      .lcd_param_bits = OLED_CMD_BITS,
  };
  return esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)i2c_master_port, &io_conf, &io_handle);
}

static esp_err_t oled_panel_driver_init(void)
{
  esp_lcd_panel_dev_config_t panel_conf = {
      .reset_gpio_num = GPIO_NUM_RST,
      .bits_per_pixel = 1,
  };
  return esp_lcd_new_panel_ssd1306(io_handle, &panel_conf, &panel_handle);
}

void oled_i2c_init(void)
{
  ESP_LOGI(TAG, "Initialize I2C master bus...");
  ESP_ERROR_CHECK(i2c_master_init());
  ESP_LOGI(TAG, "Initialize panel IO...");
  ESP_ERROR_CHECK(oled_panel_io_init());
  ESP_LOGI(TAG, "Initialize SSD1306 panel driver...");
  ESP_ERROR_CHECK(oled_panel_driver_init());
}

static esp_err_t i2c_master_write_slave(uint8_t *data_wr, size_t size)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, OLED_I2C_ADDRESS << 1 | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true); //
  i2c_master_write(cmd, data_wr, size, true);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(i2c_master_port, cmd, 10 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

void ssd1306_init()
{
  oled_i2c_init();
  uint8_t data[] = {
      OLED_CMD_SET_CHARGE_PUMP, // 0x8D
      0x14,
      OLED_CMD_SET_SEGMENT_REMAP, // 0xA1
      OLED_CMD_SET_COM_SCAN_MODE, // 0xC8
      OLED_CMD_DISPLAY_NORMAL,    // 0xA6
      OLED_CMD_DISPLAY_OFF,       // 0xAE
      OLED_CMD_DISPLAY_ON,        // 0xAF
  };
  ESP_LOGI(TAG, "Initialize SSD1306...");
  ESP_ERROR_CHECK(i2c_master_write_slave(data, sizeof(data)));
}

void ssd1306_clear_screen()
{
  for (uint8_t i = 0; i < 8; i++)
  {
    uint8_t data[] = {
        0x00,
        0x10,
        0xB0 | i,
    };
    ESP_ERROR_CHECK(i2c_master_write_slave(data, sizeof(data)));
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
    for (uint8_t j = 0; j < 128; j++)
    {
      i2c_master_write_byte(cmd, 0x00, true);
    }
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(i2c_master_port, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
  }
}

void ssd1306_display_text(const void *arg_text)
{
  char *text = (char *)arg_text;
  uint8_t text_len = strlen(text);
  uint8_t cur_page = 0;
  uint8_t data[] = {
      0x00,            // control byte
      0x10,            // column address
      0xB0 | cur_page, // page address
  };
  ESP_ERROR_CHECK(i2c_master_write_slave(data, sizeof(data)));

  for (uint8_t i = 0; i < text_len; i++)
  {
    if (text[i] == '\n')
    {
      uint8_t data[] = {
          0x00,
          0x10,
          0xB0 | ++cur_page,
      };
      ESP_ERROR_CHECK(i2c_master_write_slave(data, sizeof(data)));
    }
    else
    {
      i2c_cmd_handle_t cmd = i2c_cmd_link_create();
      i2c_master_start(cmd);
      i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

      i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
      i2c_master_write(cmd, font8x8_basic_tr[(uint8_t)text[i]], 8, true);

      i2c_master_stop(cmd);
      i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
      i2c_cmd_link_delete(cmd);
    }
  }
}
