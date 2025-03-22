```
#include <servo.h>

Servo myServo;
gpio_num_t servo_pin = GPIO_NUM_15;

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000));

    myServo.initHw(servo_pin);
    uint32_t angle = 50;
    bool increasing = false;

    for(;;)
    {
        // Set the servo position
        myServo.setPos(angle);

        // Update the angle
        if (increasing) {
            angle++;
            if (angle >= 180) {
                increasing = false; // Reverse direction
            }
        } else {
            angle--;
            if (angle <= 0) {
                increasing = true; // Reverse direction
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```