#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_deep_sleep.h"
#include "driver/gpio.h"

#include "driver/i2c.h"
#include "BME280.h"

#define I2C_MASTER_SCL_IO    22    /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO    21    /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM   I2C_NUM_0   /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_MASTER_FREQ_HZ    100000     /*!< I2C master clock frequency */

static const char *TAG = "BME280";

RTC_DATA_ATTR static int boot_count = 0;

static esp_err_t event_handler(void *ctx, system_event_t *event);
static void i2c_init();
static void bme280_task();
static void blink_task();

void app_main(void)
{
    ++boot_count;
    ESP_LOGI(TAG, "Boot count: %d", boot_count);

    bme280_task();
    
    blink_task();
    
    const int deep_sleep_sec = 10;
    ESP_LOGI(TAG, "Entering deep sleep for %d seconds", deep_sleep_sec);
    esp_deep_sleep(1000000LL * deep_sleep_sec);
}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

static void i2c_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static void bme280_task()
{
    float rh;
    float temp;
    float pressure;
    int times = 0;
    
    i2c_init();
    
    i2c_bme280_begin();
    
    ESP_LOGI(TAG, "chip id %x\n", i2c_bme280_read_register(BME280_CHIP_ID_REG));
    
    temp = i2c_bme280_read_temp();
    pressure = i2c_bme280_read_pressure();
    rh = i2c_bme280_read_rh();
    
    ESP_LOGI(TAG, "RH %f, Temp %f, pressure %f\n", rh, temp, pressure/100.0);
    
    i2c_bme280_end();
}

static void blink_task()
{
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
    
    gpio_set_level(GPIO_NUM_4, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    gpio_set_level(GPIO_NUM_4, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    gpio_set_level(GPIO_NUM_4, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    gpio_set_level(GPIO_NUM_4, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}
