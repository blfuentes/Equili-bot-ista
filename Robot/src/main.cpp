#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_log.h>
#include <esp_now.h>
#include <esp_timer.h>

#include "libnow.h"

#include "MotorControl.h"
#include "bmi160_wrapper.h"
#include "PIDService.h"
#include "RobotControl.h"
#include "servo.h"

// Libnow
#define MAC_ROBOT { 0xd8, 0xbc, 0x38, 0xf9, 0x3b, 0x4c }

// Logger tag for ESP-IDF logging
static const char *MAIN_TAG = "main_log";
static const char *MOTOR_TAG = "motor_log";
static const char *IMU_TAG = "imu_log";
static const char *ACTION_TAG = "action_log";
static const char *LIBNOW_TAG = "libnow_log";
static const char *MESSAGE_TAG = "message_log";
static const char *CONTROL_TAG = "control_log";
static const char *SERVO_TAG = "servo_log";

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
static const int LEFT_MOTOR_CORRECTION = 25; // Correction for left motor
static const int RIGHT_MOTOR_CORRECTION = 0; // Correction for right motor
static const int BASE_SPEED = 350; // Default speed for motors
static const float deltaAlphaRange = 6.0f;
static const float turnRange = 700.0f;
float deltaAlpha = 0.0f;
float turn = 0.0f;

float initAccelYCorrection = 0.020020f; // Correction for initial acceleration on Y axis
float initAccelZCorrection = 0.982452f; // Correction for initial acceleration on Z axis
float expected_vertical_warmode = 88.7f; //alpha in war mode
float expected_vertical_standingmode = 90.5f; //alpha in standing mode
float expected_vertical = 88.5;


// bmi
Bmi160<Bmi160SpiConfig> imu;
gpio_num_t mosi_pin = GPIO_NUM_16;
gpio_num_t miso_pin = GPIO_NUM_21;
gpio_num_t sclk_pin = GPIO_NUM_4;
gpio_num_t cs_pin   = GPIO_NUM_17;

// Servo
Servo servo;
gpio_num_t servo_pin = GPIO_NUM_15;

// PID
constexpr float PID_KP = 250.0f; // Proportional gain
constexpr float PID_KI = 150.0f; // Integral gain
constexpr float PID_KD = 0.125f; // Derivative gain

// PidService pid(250, 150, 0.125);
PidService pid(PID_KP, PID_KI, PID_KD);

// Status
message_control_status last_msg = {0, 0, 0, 0, 0, 0};
ModeTypeTranslation current_mode = ModeTypeTranslation::MODE_WAR;

extern "C" void app_main();

bool messageChanged(const message_control_status &msg)
{
    return (last_msg.move_x != msg.move_x || last_msg.move_y != msg.move_y ||
            last_msg.param_p != msg.param_p || last_msg.param_i != msg.param_i ||
            last_msg.param_d != msg.param_d || last_msg.mode != msg.mode);
}

void doWhenMove(message_control_status msg)
{
    deltaAlpha = -(msg.move_y * 2)/4095.0f;
    turn = -(msg.move_x * 2)/4095.0f;

    current_mode = static_cast<ModeTypeTranslation>(msg.mode);
}

static void recvcb(const esp_now_recv_info_t * esp_now_info, const uint8_t *data, int data_len)
{
    message_control_status msg = *(message_control_status*)&data[0];
    if (messageChanged(msg))
    {
        ESP_LOGI(MESSAGE_TAG, "Message changed. Mode: %d, Move X: %d, Move Y: %d, Param P: %3.0f, Param I: %3.0f, Param D: %.3f", 
            msg.mode, msg.move_x, msg.move_y, msg.param_p, msg.param_i, msg.param_d);
        last_msg = msg; // Update last message
        doWhenMove(msg);
    }
}

void applyMode()
{
    switch (current_mode)
    {
        case ModeTypeTranslation::MODE_STANDING:
            // Apply standing mode settings
            expected_vertical = expected_vertical_standingmode;
            servo.setPos(90);
            break;
        case ModeTypeTranslation::MODE_WAR:
            // Apply war mode settings
            expected_vertical = expected_vertical_warmode;
            servo.setPos(0);
            break;
        default:
            break;
    }
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));

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

    robot = RobotDefinition(leftMotor, rightMotor, stby, LEFT_MOTOR_CORRECTION, RIGHT_MOTOR_CORRECTION);
    robot.Configure();

    ESP_LOGI(MOTOR_TAG, "Motors configured");
    ESP_LOGI(MAIN_TAG, "GPIOs configured");

    // Initialize servo
    ESP_LOGI(SERVO_TAG, "Initializing servo...");
    servo.initHw(servo_pin); // GPIO 25 for servo control
    servo.calibrate(409, 2048); // Calibrate servo with min and max duty cycle
    applyMode(); // Apply initial mode settings
    ESP_LOGI(SERVO_TAG, "Servo initialized");

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
        const int num_samples = 50;
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

        float alpha = 0.0f; // [degree]
        float factor = 0.985f;
        float lastTime = 0.0f;
        float dt;
        float motorSpeed, prev_motor = 0;

        float p,i,d;

        // flattern data
        gyro_data.adj_data.x -= gyro_offset_x;
        gyro_data.adj_data.y -= gyro_offset_y;
        gyro_data.adj_data.z -= gyro_offset_z;

        // float initAccelX = accel_data.adj_data.x;
        float initAccelY = 0 - initAccelYCorrection;// axis Y has to be zeroed
        float initAccelZ = 1 - initAccelZCorrection;// axis Z has to be 1.0

        ESP_LOGI(IMU_TAG, "Initial gyro data: x: %f y: %f z: %f", gyro_data.adj_data.x, gyro_data.adj_data.y, gyro_data.adj_data.z);
        ESP_LOGI(IMU_TAG, "Initial accel data: x: %f y: %f z: %f", accel_data.adj_data.x, accel_data.adj_data.y, accel_data.adj_data.z);
        
        // first lecture
        imu.getData(accel_data, gyro_data);
        lastTime = gyro_data.adj_data.sensortime;
        alpha = atan2f(accel_data.adj_data.z + initAccelZ, accel_data.adj_data.y + initAccelY) * 180.0f / M_PI;        

        ESP_LOGI(IMU_TAG, "Initial alpha: %6f", alpha);

        for (;;)
        {
            
            // Apply message control data
            applyMode();

            // get bmi data
            imu.getData(accel_data, gyro_data);

            // flattern data
            gyro_data.adj_data.x -= gyro_offset_x;
            gyro_data.adj_data.y -= gyro_offset_y;
            gyro_data.adj_data.z -= gyro_offset_z;

            dt = (gyro_data.adj_data.sensortime - lastTime) / 1000.0f;
            lastTime = gyro_data.adj_data.sensortime;

            float firstPart = alpha - gyro_data.adj_data.x * dt;
            float secondPart = atan2f(accel_data.adj_data.z + initAccelZ, accel_data.adj_data.y + initAccelY) * 180.0f / M_PI;
            alpha = firstPart * factor + secondPart * (1 - factor);

            float alphaError = expected_vertical - alpha - deltaAlpha * deltaAlphaRange;
            motorSpeed = pid.update(alphaError, dt);
            pid.getLastPid(p, i, d);

            motorSpeed = prev_motor * 0.1 + motorSpeed * 0.9;
            prev_motor = motorSpeed;

            correctionDir = { X_Direction::X_CENTER, Y_Direction::Y_CENTER };
            if (alphaError != 0){
                correctionDir.vertical = alphaError > 0 ? Y_Direction::FORWARD : Y_Direction::BACKWARD;
            }

            // ESP_LOGI(CONTROL_TAG, "Turn: %6f - Delta Alpha: %6f", turn, deltaAlpha);
            if (turn < 0.0f){
                correctionDir.horizontal = X_Direction::RIGHT;
            } else if (turn > 0.0f){
                correctionDir.horizontal = X_Direction::LEFT;
            } else {
                correctionDir.horizontal = X_Direction::X_CENTER;
            }

            motorSpeed = -(motorSpeed + (fabs(turn * turnRange)));

            int32_t speedForMotor = 0;
            if (motorSpeed > 1023)
                speedForMotor = 1023;
            else if (motorSpeed < -1023)
                speedForMotor = -1023;
            else
                speedForMotor = static_cast<int32_t>(motorSpeed);

            if (fabs(speedForMotor) < BASE_SPEED && correctionDir.horizontal == X_Direction::X_CENTER){
                ESP_LOGI(MOTOR_TAG, "Speed too low: %6ld at angle %6f", speedForMotor, alpha);
                robot.Stop(); // Stop the robot if speed is too low
                vTaskDelay(pdMS_TO_TICKS(20));
                continue; // Skip if speed is too low
            }

            ESP_LOGI(CONTROL_TAG, "Turn: %6f - Delta Alpha: %6f - Correction Dir: %s/%s", 
                turn, deltaAlpha, robot.X_DirectionToString(correctionDir.horizontal), robot.Y_DirectionToString(correctionDir.vertical));

            ESP_LOGI(ACTION_TAG, "Expected vertical: %6f - Alpha: %6f - Error: %6f - Direction: %s - Speed: %6ld", expected_vertical, alpha, alphaError, robot.Y_DirectionToString(correctionDir.vertical), speedForMotor);
            robot.Drive(correctionDir, speedForMotor);

            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}
