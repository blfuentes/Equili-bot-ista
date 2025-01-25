
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

int8_t bmi_configure(bmi160_dev *dev)
{
    int8_t rslt;

    rslt = bmi160_init(dev);

    if (rslt == BMI160_OK)
    {
        ESP_LOGI(TAG, "BMI160 initialization success !");
        ESP_LOGI(TAG, "Chip ID 0x%X", dev->chip_id);

        /* Select the Output data rate, range of accelerometer sensor */
        dev->accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
        dev->accel_cfg.range = BMI160_ACCEL_RANGE_16G;
        dev->accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

        /* Select the power mode of accelerometer sensor */
        dev->accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

        /* Select the Output data rate, range of Gyroscope sensor */
        dev->gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
        dev->gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
        dev->gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

        /* Select the power mode of Gyroscope sensor */
        dev->gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

        /* Set the sensor configuration */
        return bmi160_set_sens_conf(dev);
    }
    else
    {
        ESP_LOGE(TAG,"BMI160 initialization failure !");
        return -1;
    }
};

int8_t bmi160_get_sensor_data_adjusted(
    uint8_t select_sensor, struct bmi160_sensor_data_adjusted *accel_adj, struct bmi160_sensor_data_adjusted *gyro_adj, const struct bmi160_dev *dev, bool adjusted)
{
    // Accelerator
    float accel_factor = 16.0f;
    switch(dev->accel_cfg.range) 
    {
        case BMI160_ACCEL_RANGE_2G:
            accel_factor = 2.0f;
            break;

        case BMI160_ACCEL_RANGE_4G:
            accel_factor = 4.0f;
            break;

        case BMI160_ACCEL_RANGE_8G:
            accel_factor = 8.0f;
            break;

        case BMI160_ACCEL_RANGE_16G:
            accel_factor = 16.0f;
            break;
    }
    float accToG = accel_factor / float((1 << 15) - 1);

    // Gyroscope
    float gyro_factor = 2000.0f;
    switch (dev->gyro_cfg.range)
    {
        case BMI160_GYRO_RANGE_125_DPS:
            gyro_factor = 125.0f;
            break;

        case BMI160_GYRO_RANGE_250_DPS:
            gyro_factor = 250.0f;
            break;

        case BMI160_GYRO_RANGE_500_DPS:
            gyro_factor = 500.0f;
            break;

        case BMI160_GYRO_RANGE_1000_DPS:
            gyro_factor = 1000.0f;
            break;
        
        case BMI160_GYRO_RANGE_2000_DPS:
            gyro_factor = 2000.0f;
            break;
    }
    float gyroToDps = gyro_factor / float((1 << 15) - 1);

    // Read data
    bmi160_sensor_data accel;
    bmi160_sensor_data gyro;

    if (bmi160_get_sensor_data(BMI160_ACCEL_SEL | BMI160_GYRO_SEL, &accel, &gyro, dev) == BMI160_OK)
    {
        if (adjusted) 
        {
            accel_adj->x = accel.x * accToG;
            accel_adj->y = accel.y * accToG;
            accel_adj->z = accel.z * accToG;
            accel_adj->sensortime = accel.sensortime;

            gyro_adj->x = gyro.x * gyroToDps;
            gyro_adj->y = gyro.y * gyroToDps;
            gyro_adj->z = gyro.z * gyroToDps;
            gyro_adj->sensortime = gyro.sensortime;            
        }
        else
        {
            accel_adj->x = accel.x;
            accel_adj->y = accel.y;
            accel_adj->z = accel.z;
            accel_adj->sensortime = accel.sensortime;

            gyro_adj->x = gyro.x;
            gyro_adj->y = gyro.y;
            gyro_adj->z = gyro.z;
            gyro_adj->sensortime = gyro.sensortime; 
        }

        return BMI160_OK;
    }
    else
    {
        return -1;
    }
};