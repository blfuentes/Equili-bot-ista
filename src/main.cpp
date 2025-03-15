#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_log.h>

#include <MotorControl.h>
#include <bmi160_wrapper.h>
#include <PIDService.h>

// Logger tag for ESP-IDF logging
static const char * MAIN_TAG = "app_main";

// motor pins
constexpr gpio_num_t MOTOR_A_IN_1 = GPIO_NUM_18;
constexpr gpio_num_t MOTOR_A_IN_2 = GPIO_NUM_19;
constexpr gpio_num_t MOTOR_A_PWM = GPIO_NUM_23;

constexpr gpio_num_t MOTOR_B_IN_1 = GPIO_NUM_26;
constexpr gpio_num_t MOTOR_B_IN_2 = GPIO_NUM_27;
constexpr gpio_num_t MOTOR_B_PWM = GPIO_NUM_5;

constexpr gpio_num_t STBY = GPIO_NUM_33;

constexpr ledc_mode_t LEDC_SPEED_MODE = LEDC_LOW_SPEED_MODE;

MotorDefinition leftMotor;
MotorDefinition rightMotor;
PinGPIODefinition stby;

// bmi pins
Bmi160<Bmi160SpiConfig> imu;
gpio_num_t mosi_pin = GPIO_NUM_16;
gpio_num_t miso_pin = GPIO_NUM_21;
gpio_num_t sclk_pin = GPIO_NUM_4;
gpio_num_t cs_pin   = GPIO_NUM_17;

// PID
PidService pid(5.0, 0.0, 0.0);

extern "C" void app_main();

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Configuring motors
    leftMotor = MotorDefinition(MOTOR_A_IN_1, MOTOR_A_IN_2, 0, 1, MOTOR_A_PWM, LEDC_CHANNEL_0, LEDC_SPEED_MODE, LEDC_TIMER_0);
    rightMotor = MotorDefinition(MOTOR_B_IN_1, MOTOR_B_IN_2, 1, 0, MOTOR_B_PWM, LEDC_CHANNEL_1, LEDC_SPEED_MODE, LEDC_TIMER_1);

    stby = PinGPIODefinition(STBY, GPIO_MODE_OUTPUT, GPIO_PULLDOWN_DISABLE);

    leftMotor.Configure();
    rightMotor.Configure();
    stby.Configure();

    gpio_set_level(STBY, 1);

    // Configuring bmi
    Bmi160SpiConfig config = {
        .spidev = SPI3_HOST,
        .mosi = mosi_pin,
        .miso = miso_pin,
        .sclk = sclk_pin,
        .cs   = cs_pin,
        .speed= 4*1000*1000
    };

    if (imu.init(config) != BMI160_OK)
    {
        ESP_LOGE(MAIN_TAG, "Failed to initialize bmi160 device");
    }
    else
    {
        Bmi160Data accel_data, gyro_data;

        //
        float alpha = 0.0f; // [degree]
        float factor = 0.995f;
        float lastTime = 0.0f;
        float dt;
        int32_t motor;

        for(;;)
        {
            // get bmi data
            imu.getData(accel_data, gyro_data);

            dt = (gyro_data.adj_data.sensortime - lastTime)/1000.0f;
            lastTime = gyro_data.adj_data.sensortime;
    
            // acc.y look for the angle that it is zero when the robot is standing
            // and varies when the motor falls.
            alpha = (alpha + gyro_data.adj_data.x*dt ) * factor + accel_data.adj_data.y*9.8f * (1-factor);
            ESP_LOGI("MAIN", "Alpha: %.2f", alpha);
    
            // Step 2: Calculat.0, 0.0, 0.0);e motors using PID
            motor = pid.update(-alpha, dt);

            // printf("Raw-Accl: %6d %6d %6d\n", accel_data.raw_data.x, accel_data.raw_data.y, accel_data.raw_data.z);
            printf("Adj-Accl: %.2f %.2f %.2f\n", accel_data.adj_data.x, accel_data.adj_data.y, accel_data.adj_data.z);
            // printf("Raw-Gyro: %6d %6d %6d\n", gyro_data.raw_data.x, gyro_data.raw_data.y, gyro_data.raw_data.z);
            // printf("Adj-Gyro: %.2f %.2f %.2f\n", gyro_data.adj_data.x, gyro_data.adj_data.y, gyro_data.adj_data.z);

            // vTaskDelay(pdMS_TO_TICKS(1000));

            printf("Motor %6ld\n", motor);

            leftMotor.Drive(motor);
            // vTaskDelay(pdMS_TO_TICKS(200));
            // leftMotor.Stop();

            // vTaskDelay(pdMS_TO_TICKS(1000));

            rightMotor.Drive(motor);
            // vTaskDelay(pdMS_TO_TICKS(200));
            // rightMotor.Stop();
            
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}
