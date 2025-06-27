#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_log.h>
#include <esp_now.h>

#include "libnow.h"

#include "MotorControl.h"
#include "bmi160_wrapper.h"
#include "PIDService.h"
#include "RobotControl.h"
#include <esp_timer.h>

// Libnow
#define MAC_ROBOT { 0xd8, 0xbc, 0x38, 0xf9, 0x3b, 0x4c }

// Logger tag for ESP-IDF logging
static const char *MAIN_TAG = "main_log";
static const char *MOTOR_TAG = "motor_log";
static const char *IMU_TAG = "imu_log";
static const char *ACTION_TAG = "action_log";
static const char *LIBNOW_TAG = "libnow_log";
static const char *MESSAGE_TAG = "message_log";

// motor pins
constexpr gpio_num_t MOTOR_A_IN_1 = GPIO_NUM_18;
constexpr gpio_num_t MOTOR_A_IN_2 = GPIO_NUM_19;
constexpr gpio_num_t MOTOR_A_PWM = GPIO_NUM_23;

constexpr gpio_num_t MOTOR_B_IN_1 = GPIO_NUM_26;
constexpr gpio_num_t MOTOR_B_IN_2 = GPIO_NUM_27;
constexpr gpio_num_t MOTOR_B_PWM = GPIO_NUM_5;

constexpr gpio_num_t STBY = GPIO_NUM_33;

constexpr ledc_mode_t LEDC_SPEED_MODE = LEDC_LOW_SPEED_MODE;

RobotDefinition robot;   
static const int BASE_SPEED = 392; // Default speed for motors
static const int CORRECTION_LIMIT = 20; // Limit for correction speed

// bmi pins
Bmi160<Bmi160SpiConfig> imu;
gpio_num_t mosi_pin = GPIO_NUM_16;
gpio_num_t miso_pin = GPIO_NUM_21;
gpio_num_t sclk_pin = GPIO_NUM_4;
gpio_num_t cs_pin   = GPIO_NUM_17;

// PID
PidService pid(200, 0, 0);

// Constants
constexpr float alpha_threshold = 0.5f; // Threshold for alpha to reset it to 0

extern "C" void app_main();

void doWhenMove(message_control_status msg)
{
    // deltaAlpha = (msg.x-2270)/4095.0f;
    // turn = (msg.y-2317)/4095.0f;
}

static void recvcb(const esp_now_recv_info_t * esp_now_info, const uint8_t *data, int data_len)
{
    // ESP_LOGI(MESSAGE_TAG, "Message received");
    message_control_status msg = *(message_control_status*)&data[0];
    doWhenMove(msg);
    ESP_LOGI(MESSAGE_TAG, "Message received. Mode: %d, Move X: %d, Move Y: %d, Param P: %f, Param I: %f, Param D: %.1f", 
        msg.mode, msg.move_x, msg.move_y, msg.param_p, msg.param_i, msg.param_d);

}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(500));

    // Robot parts
    MotorDefinition rightMotor;
    MotorDefinition leftMotor;
    PinGPIODefinition stby;

    Direction correctionDir = { X_Direction::X_CENTER, Y_Direction::Y_CENTER};

    // Initialize LibNow
    ESP_LOGI(LIBNOW_TAG, "Initializing LibNow...");
    libnow_init();
    libnow_addPeer(LibNowDst::DST_MANDO);
    esp_now_register_recv_cb(recvcb);
    ESP_LOGI(LIBNOW_TAG, "LibNow initialized");

    // Configuring motors
    ESP_LOGI(MAIN_TAG, "Configuring motors...");
    rightMotor = MotorDefinition(MOTOR_A_IN_1, MOTOR_A_IN_2, 0, 1, MOTOR_A_PWM, LEDC_CHANNEL_0, LEDC_SPEED_MODE, LEDC_TIMER_0);
    ESP_LOGI(MOTOR_TAG, "Right motor configured on GPIO %d and %d", MOTOR_A_IN_1, MOTOR_A_IN_2);
    leftMotor = MotorDefinition(MOTOR_B_IN_1, MOTOR_B_IN_2, 1, 0, MOTOR_B_PWM, LEDC_CHANNEL_1, LEDC_SPEED_MODE, LEDC_TIMER_1);
    ESP_LOGI(MOTOR_TAG, "Left motor configured on GPIO %d and %d", MOTOR_B_IN_1, MOTOR_B_IN_2);
    stby = PinGPIODefinition(STBY, GPIO_MODE_OUTPUT, GPIO_PULLDOWN_DISABLE);

    robot = RobotDefinition(leftMotor, rightMotor, stby, 0, 0);
    robot.Configure();

    ESP_LOGI(MOTOR_TAG, "Motors configured");
    ESP_LOGI(MAIN_TAG, "GPIOs configured");

    // Configuring bmi
    ESP_LOGI(IMU_TAG, "Configuring BMI160 IMU...");
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
        ESP_LOGE(IMU_TAG, "Failed to initialize bmi160 device");
    }
    else
    {
        Bmi160Data accel_data, gyro_data;

        // start calibration
        ESP_LOGI(IMU_TAG, "Starting calibration...");
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
        ESP_LOGI(IMU_TAG, "Calibration complete. Offsets: x: %f y: %f z: %f", gyro_offset_x, gyro_offset_y, gyro_offset_z);

        // calibrate offsetalhpa
        float alpha = 0.0f; // [degree]
        float factor = 0.995f;
        float lastTime = 0.0f;
        float dt;
        int32_t motorSpeed;

        // flattern data
        if (fabs(gyro_data.adj_data.x) < gyro_offset_x) gyro_data.adj_data.x = 0.0f;
        if (fabs(gyro_data.adj_data.y) < gyro_offset_y) gyro_data.adj_data.y = 0.0f;
        if (fabs(gyro_data.adj_data.z) < gyro_offset_z) gyro_data.adj_data.z = 0.0f;

        dt = (gyro_data.adj_data.sensortime - lastTime) / 1000.0f;
        lastTime = gyro_data.adj_data.sensortime;

        alpha = atan2f(accel_data.adj_data.y, accel_data.adj_data.z)*180.0f/M_PI_2;
        ESP_LOGI(IMU_TAG, "Initial alpha: %6f", alpha);
        // float offsetAlpha = (alpha + gyro_data.adj_data.x * dt) * factor + accel_data.adj_data.y * 9.8f * (1 - factor);
        // ESP_LOGI(IMU_TAG, "Initial alpha: %6f", offsetAlpha);
        for(;;)
        {
            // get bmi data
            imu.getData(accel_data, gyro_data);

            // flattern data
            // ESP_LOGI(IMU_TAG, "Adjusted gyro data: x: %f y: %f z: %f", gyro_data.adj_data.x, gyro_data.adj_data.y, gyro_data.adj_data.z);

            if (fabs(gyro_data.adj_data.x) < gyro_offset_x) gyro_data.adj_data.x = 0.0f;
            if (fabs(gyro_data.adj_data.y) < gyro_offset_y) gyro_data.adj_data.y = 0.0f;
            if (fabs(gyro_data.adj_data.z) < gyro_offset_z) gyro_data.adj_data.z = 0.0f;

            // ESP_LOGI(IMU_TAG, "Adjusted gyro data: x: %f y: %f z: %f", gyro_data.adj_data.x, gyro_data.adj_data.y, gyro_data.adj_data.z);
            // ESP_LOGI(IMU_TAG, "Adjusted accel data: x: %f y: %f z: %f", accel_data.adj_data.x, accel_data.adj_data.y, accel_data.adj_data.z);

            dt = (gyro_data.adj_data.sensortime - lastTime)/1000.0f;
            lastTime = gyro_data.adj_data.sensortime;

            alpha = (alpha + gyro_data.adj_data.x*dt ) * factor + atan2f(accel_data.adj_data.y, accel_data.adj_data.z)*180.0f/M_PI_2 * (1-factor);
            ESP_LOGI(IMU_TAG, "Alpha: %6f - Gyro X: %6f - Accel Y: %6f", alpha, gyro_data.adj_data.x, accel_data.adj_data.y);

            // alpha = (alpha + gyro_data.adj_data.x * dt ) * factor + accel_data.adj_data.y * 9.8f * (1-factor);

            if (fabs(alpha) < alpha_threshold) {
                alpha = 0.0f;
            }
            motorSpeed = pid.update(alpha, dt);

            Direction correctionDir = { X_Direction::X_CENTER, Y_Direction::Y_CENTER};
            if (motorSpeed > 1023) {
                motorSpeed = 1023;
            }
            if (motorSpeed < -1023) {
                motorSpeed = -1023;
            }

            if (motorSpeed != 0)
                correctionDir.vertical = motorSpeed > 0 ? Y_Direction::FORWARD : Y_Direction::BACKWARD;

            // ESP_LOGI(ACTION_TAG, "Angle: %6f - Motor speed: %6ld", alpha, motorSpeed);
            // robot.Drive(correctionDir, motorSpeed);
            
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}
