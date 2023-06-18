#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "dht11.h"
#include "ssd1306.h"
#include "mqtt.h"

#define TAG "APP_MAIN"

static struct dht11_reading dht11_results;

void app_main()
{
  DHT11_init(GPIO_NUM_4);
  ssd1306_init();
  ssd1306_clear_screen();
  mqtt_app_start();

  while (1)
  {
    dht11_results = DHT11_read();
    if (DHT11_OK == dht11_results.status)
    {
      char text[50];
      sprintf(text, "\nMini Project\nNhom 8\nNhiet do: %d do\nDo am: %d%%", dht11_results.temperature, dht11_results.humidity);
      char data[10];
      sprintf(data, "%d", dht11_results.temperature);
      mqtt_publish_data_to_topic("temperature", data);
      sprintf(data, "%d", dht11_results.humidity);
      mqtt_publish_data_to_topic("humidity", data);
      ssd1306_display_text(text);
    }

    vTaskDelay(10000 / portTICK_PERIOD_MS); // Delay for 2 seconds
  }
}
