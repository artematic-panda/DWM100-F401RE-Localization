#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "main.h"

#define SERVO_MIN_US    650     // calibrated range lower bound
#define SERVO_MAX_US    2300    // calibrated range upper bound

#define SERVO_MIN_ANGLE 0.0f
#define SERVO_MAX_ANGLE 180.0f

typedef struct {
    float current_angle;
    float target_angle;
    float speed_deg_per_ms;
    uint8_t is_moving;
} ServoState;

void servo_init(TIM_HandleTypeDef *htim);

void servo_set_us(uint32_t channel, uint16_t us);

void servo_write_angle(uint32_t channel, float angle);

void set_target_angle(ServoState *s, float new_angle);

void update_servo(ServoState *s, uint32_t channel);

void point_gimbal(ServoState *pan, ServoState *tilt, float x, float y, float z);

#endif /* INC_SERVO_H_ */
