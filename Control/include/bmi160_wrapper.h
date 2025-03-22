#ifndef __BMI160_WRAPPER_H__
#define __BMI160_WRAPPER_H__

#include <stdint.h>
#include <string_view>
#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <bmi160.h>

static const char *TAG = "bmi160_wrapper";

typedef struct {
    spi_host_device_t spidev;
    gpio_num_t mosi;
    gpio_num_t miso;
    gpio_num_t sclk;
    gpio_num_t cs;
    uint32_t speed;
} Bmi160SpiConfig;

typedef struct {
    gpio_num_t sda;
    gpio_num_t scl;
    uint8_t addr;
} Bmi160I2cConfig;

typedef struct
{
    float x;
    float y;
    float z;
    int sensortime;
} DataAdj;

typedef struct
{
    bmi160_sensor_data raw_data;
    DataAdj adj_data;
} Bmi160Data;

typedef enum
{
    BMI160_INIT_FAILURE = 1,
    BMI160_CONFIGURATION_FAILURE = 2,
    BMI160_DATA_FAILURE = 3,
    BMI160_TRANSACTION_READ_FAILURE = 4,
    BMI160_TRANSACTION_WRITE_FAILURE = 5,
    BMI160_INIT_BUS_FAILURE = 6,
    BMI160_INIT_DEVICE_FAILURE = 7

} ResultCode;

int8_t bmi_read_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t bmi_write_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len);
void bmi_delay(uint32_t period);

template<typename T>
class Bmi160{
public:
    Bmi160();

    uint8_t init(T config);
    uint8_t configure();
    uint8_t getData(Bmi160Data &acc, Bmi160Data &gyr);

    static spi_device_handle_t spiHandle;

private:
    bmi160_dev dev;
    float accScale;
    float gyrScale;

    float sensorTimeScale = 0.039f;
    
    // SPI
    spi_bus_config_t spiBus;
    spi_device_interface_config_t spiIf;

};

template<typename T>
spi_device_handle_t Bmi160<T>::spiHandle = {0};

template<typename T>
Bmi160<T>::Bmi160()
{
    memset(&dev, 0, sizeof(bmi160_dev));
    memset(&spiBus, 0, sizeof(spi_bus_config_t));
    memset(&spiIf, 0, sizeof(spi_device_interface_config_t));
}

template<typename T>
uint8_t Bmi160<T>::configure()
{
    int8_t rslt = bmi160_init(&dev);

    if (rslt == BMI160_OK)
    {
        ESP_LOGI(TAG,"BMI160 initialization success!");
        ESP_LOGI(TAG,"Chip ID 0x%X\n", dev.chip_id);
    }
    else
    {
        ESP_LOGE(TAG,"BMI160 initialization failure !\n");
        return BMI160_INIT_FAILURE;
    }

    /* Select the Output data rate, range of accelerometer sensor */
    dev.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
    dev.accel_cfg.range = BMI160_ACCEL_RANGE_16G;
    dev.accel_cfg.bw = BMI160_ACCEL_BW_OSR4_AVG1;

    /* Select the power mode of accelerometer sensor */
    dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    dev.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
    dev.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    dev.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

    // Accelerator
    float accel_factor = 16.0f;
    switch(dev.accel_cfg.range) 
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
    accScale = accel_factor / float((1 << 15) - 1);

    // Gyroscope
    float gyro_factor = 2000.0f;
    switch (dev.gyro_cfg.range)
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
    gyrScale = gyro_factor / float((1 << 15) - 1);

    /* Select the power mode of Gyroscope sensor */
    dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&dev);

    if( rslt != BMI160_OK)
    {
        ESP_LOGE(TAG, "Sensor configuration failed");
        return BMI160_CONFIGURATION_FAILURE;
    }

    return rslt;
}

template<typename T>
uint8_t Bmi160<T>::getData(Bmi160Data &acc, Bmi160Data &gyr)
{
    bmi160_sensor_data rawAcc, rawGyr;
    uint8_t rslt = bmi160_get_sensor_data(BMI160_ACCEL_SEL | BMI160_GYRO_SEL | BMI160_TIME_SEL, &rawAcc, &rawGyr, &dev);

    if(rslt == BMI160_OK)
    {
        acc.raw_data = rawAcc;
        acc.adj_data.x = rawAcc.x*accScale;
        acc.adj_data.y = rawAcc.y*accScale;
        acc.adj_data.z = rawAcc.z*accScale;
        acc.adj_data.sensortime = rawAcc.sensortime * sensorTimeScale;

        gyr.raw_data = rawGyr;
        gyr.adj_data.x = rawGyr.x*gyrScale;
        gyr.adj_data.y = rawGyr.y*gyrScale;
        gyr.adj_data.z = rawGyr.z*gyrScale;
        gyr.adj_data.sensortime = rawGyr.sensortime * sensorTimeScale;

        return rslt;
    }

    return BMI160_DATA_FAILURE;
}

#endif //__BMI160_WRAPPER_H__
