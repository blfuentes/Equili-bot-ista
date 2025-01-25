
#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_log.h>

#include <bmi160_wrapper.h>

static const char *TAG = "bmi160_wrapper";
static spi_device_handle_t spiHandle;

int8_t bmi_read_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t bmi_write_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len);
void bmi_delay(uint32_t period);

bmi160_dev bmi_init_spi()
{
    bmi160_dev dev;
    spi_bus_config_t spiBus;
    spi_device_interface_config_t spiIf;

    memset(&dev, 0, sizeof(bmi160_dev));
    memset(&spiBus, 0, sizeof(spi_bus_config_t));
    memset(&spiHandle, 0, sizeof(spi_device_handle_t));
    memset(&spiIf, 0, sizeof(spi_device_interface_config_t));

    // Dev config
    dev.intf = BMI160_SPI_INTF;
    dev.read = bmi_read_spi;
    dev.write = bmi_write_spi;
    dev.delay_ms = bmi_delay;

    // Bus config
    spiBus.mosi_io_num = GPIO_NUM_23;
    spiBus.miso_io_num = GPIO_NUM_19;
    spiBus.sclk_io_num = GPIO_NUM_18;
    spiBus.quadhd_io_num = -1;
    spiBus.quadwp_io_num = -1;
    spiBus.max_transfer_sz = 4092;
    
    //ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &busSpi, SPI_DMA_DISABLED));
    if (spi_bus_initialize(SPI3_HOST, &spiBus, SPI_DMA_DISABLED) != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI Bus cannot be initialized");
        return dev;
    }
    ESP_LOGI(TAG, "SPI Bus Initialized correctly");

    // Interface config
    spiIf.spics_io_num = GPIO_NUM_21; // Cambiame
    spiIf.clock_speed_hz = 5 * 1000 * 1000; // 5Mhz
    spiIf.mode = 0;
    spiIf.queue_size = 10;
    spiIf.address_bits = 8;
    
    //ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &spiIf, &spiHandle));
    if (spi_bus_add_device(SPI3_HOST, &spiIf, &spiHandle) != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI Device cannot be added");
        return dev;
    }
    ESP_LOGI(TAG, "SPI Device Added correctly");

    return dev;
}

int8_t bmi_read_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    // Transaction config
    spi_transaction_t spiTrans;
    memset(&spiTrans, 0, sizeof(spi_transaction_t));

    spiTrans.length = 8 * len;
    spiTrans.addr = reg_addr;
    spiTrans.rxlength = 8 * len;
    spiTrans.rx_buffer = data;

    spi_device_acquire_bus(spiHandle, portMAX_DELAY);
    if (spi_device_polling_transmit(spiHandle, &spiTrans) != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI Transaction ERROR");
        return -1;
    }
    ESP_LOGI(TAG, "SPI Transaction OK");
    spi_device_release_bus(spiHandle);
    
    return BMI160_OK;
};

int8_t bmi_write_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len)
{
    // Transaction config
    spi_transaction_t spiTrans;
    memset(&spiTrans, 0, sizeof(spi_transaction_t));

    spiTrans.length = 8 * len;
    spiTrans.addr = reg_addr;
    spiTrans.tx_buffer = read_data;

    spi_device_acquire_bus(spiHandle, portMAX_DELAY);
    if (spi_device_polling_transmit(spiHandle, &spiTrans) != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI Transaction ERROR");
        return -1;
    }
    ESP_LOGI(TAG, "SPI Transaction OK");
    spi_device_release_bus(spiHandle);

    return BMI160_OK;
};

void bmi_delay(uint32_t period)
{
    if (period < 10) { period = 10; }

    vTaskDelay(pdMS_TO_TICKS(period));
};