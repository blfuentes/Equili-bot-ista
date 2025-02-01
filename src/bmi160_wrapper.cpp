#include <esp_log.h>
#include <bmi160_wrapper.h>

int8_t bmi_read_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    // Transaction config
    spi_transaction_t spiTrans;
    memset(&spiTrans, 0, sizeof(spi_transaction_t));

    spiTrans.length = 8 * len;
    spiTrans.addr = reg_addr;
    spiTrans.rxlength = 8 * len;
    spiTrans.rx_buffer = data;

    spi_device_acquire_bus(Bmi160<Bmi160SpiConfig>::spiHandle, portMAX_DELAY);
    if (spi_device_polling_transmit(Bmi160<Bmi160SpiConfig>::spiHandle, &spiTrans) != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI Transaction ERROR");
        return BMI160_TRANSACTION_READ_FAILURE;
    }
    ESP_LOGI(TAG, "SPI Transaction OK");
    spi_device_release_bus(Bmi160<Bmi160SpiConfig>::spiHandle);
    
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

    spi_device_acquire_bus(Bmi160<Bmi160SpiConfig>::spiHandle, portMAX_DELAY);
    if (spi_device_polling_transmit(Bmi160<Bmi160SpiConfig>::spiHandle, &spiTrans) != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI Transaction ERROR");
        return BMI160_TRANSACTION_WRITE_FAILURE;
    }
    ESP_LOGI(TAG, "SPI Transaction OK");
    spi_device_release_bus(Bmi160<Bmi160SpiConfig>::spiHandle);

    return BMI160_OK;
};

void bmi_delay(uint32_t period)
{
    if (period < 10) { period = 10; }

    vTaskDelay(pdMS_TO_TICKS(period));
};

template<>
uint8_t Bmi160<Bmi160SpiConfig>::init(Bmi160SpiConfig config)
{
    spiBus.mosi_io_num = config.mosi;
    spiBus.miso_io_num = config.miso;
    spiBus.sclk_io_num = config.sclk;
    spiBus.quadhd_io_num = -1;
    spiBus.quadwp_io_num = -1;
    spiBus.max_transfer_sz = 4092;
    
    if (spi_bus_initialize(config.spidev, &spiBus, SPI_DMA_DISABLED) != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI Bus cannot be initialized");
        return BMI160_INIT_BUS_FAILURE;
    }
    ESP_LOGI(TAG, "SPI Bus Initialized correctly");

    spiIf.spics_io_num = config.cs; 
    spiIf.clock_speed_hz = config.speed;
    spiIf.mode = 0;
    spiIf.queue_size = 10;
    spiIf.address_bits = 8;
    
    if (spi_bus_add_device(config.spidev, &spiIf, &spiHandle) != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI Device cannot be added");
        return BMI160_INIT_DEVICE_FAILURE;
    }
    ESP_LOGI(TAG, "SPI Device Added correctly");

    
    dev.intf = BMI160_SPI_INTF;
    dev.read = bmi_read_spi;
    dev.write = bmi_write_spi;
    dev.delay_ms = bmi_delay;


    return configure();
}

template<>
uint8_t Bmi160<Bmi160I2cConfig>::init(Bmi160I2cConfig config)
{

    return configure();
};