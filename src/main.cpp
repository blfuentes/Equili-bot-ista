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

MotorDefinition rightMotor;
MotorDefinition leftMotor;
PinGPIODefinition stby;

// bmi pins
Bmi160<Bmi160SpiConfig> imu;
gpio_num_t mosi_pin = GPIO_NUM_16;
gpio_num_t miso_pin = GPIO_NUM_21;
gpio_num_t sclk_pin = GPIO_NUM_4;
gpio_num_t cs_pin   = GPIO_NUM_17;

// PID
PidService pid(300, .00, 0);

extern "C" void app_main();

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Configuring motors
    rightMotor = MotorDefinition(MOTOR_A_IN_1, MOTOR_A_IN_2, 0, 1, MOTOR_A_PWM, LEDC_CHANNEL_0, LEDC_SPEED_MODE, LEDC_TIMER_0);
    leftMotor = MotorDefinition(MOTOR_B_IN_1, MOTOR_B_IN_2, 1, 0, MOTOR_B_PWM, LEDC_CHANNEL_1, LEDC_SPEED_MODE, LEDC_TIMER_1);

    stby = PinGPIODefinition(STBY, GPIO_MODE_OUTPUT, GPIO_PULLDOWN_DISABLE);

    rightMotor.Configure();
    leftMotor.Configure();
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

        // start calibration
        printf("Starting calibration...\n");
        float gyro_offset_x = 0.0f, gyro_offset_y = 0.0f, gyro_offset_z = 0.0f;

        // collect data
        const int num_samples = 100;
        for (int i = 0; i < num_samples; i++)
        {
            imu.getData(accel_data, gyro_data);
            gyro_offset_x += gyro_data.adj_data.x;
            gyro_offset_y += gyro_data.adj_data.y;
            gyro_offset_z += gyro_data.adj_data.z;
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // normalize
        gyro_offset_x /= num_samples;
        gyro_offset_y /= num_samples;
        gyro_offset_z /= num_samples;

        //
        float alpha = 0.0f; // [degree]
        float factor = 0.995f;
        float lastTime = 0.0f;
        float dt;
        int32_t motorSpeed;

        printf("Finished calibration!!\n");
        // flattern data
        gyro_data.adj_data.x = gyro_data.adj_data.x - gyro_offset_x;
        gyro_data.adj_data.y = gyro_data.adj_data.y - gyro_offset_y;
        gyro_data.adj_data.z = gyro_data.adj_data.z - gyro_offset_z;

        dt = (gyro_data.adj_data.sensortime - lastTime) / 1000.0f;
        lastTime = gyro_data.adj_data.sensortime;

        float offsetAlpha = (alpha + gyro_data.adj_data.x * dt) * factor + accel_data.adj_data.y * 9.8f * (1 - factor);
        for(;;)
        {
            // get bmi data
            imu.getData(accel_data, gyro_data);

            // flattern data
            gyro_data.adj_data.x = gyro_data.adj_data.x - gyro_offset_x;
            gyro_data.adj_data.y = gyro_data.adj_data.y - gyro_offset_y;
            gyro_data.adj_data.z = gyro_data.adj_data.z - gyro_offset_z;

            dt = (gyro_data.adj_data.sensortime - lastTime)/1000.0f;
            lastTime = gyro_data.adj_data.sensortime;

            alpha = (alpha + gyro_data.adj_data.x*dt ) * factor + accel_data.adj_data.y*9.8f * (1-factor);

            motorSpeed = pid.update(-alpha - offsetAlpha, dt);
            // if (alpha > -20 && alpha < 20) {
            //     motorSpeed = 0;
            // }
            if (motorSpeed > 1000) {
                motorSpeed = 400;
            }
            if (motorSpeed < -1000) {
                motorSpeed = -400;
            }
            // for(int s = 0; s < 1024; s++) {
            //     printf("Moving at %d speed\n", s);
            //     rightMotor.Drive(s);
            //     leftMotor.Drive(s);
            //     vTaskDelay(pdMS_TO_TICKS(10));
            // }
            printf("Angle: %6f - Motor speed: %6ld\n", alpha, motorSpeed);
            rightMotor.Drive(motorSpeed);
            leftMotor.Drive(motorSpeed);
            
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}
