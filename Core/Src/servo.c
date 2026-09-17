#include "servo.h"
#include <math.h>
// Standard Math constants
#ifndef PI
#define PI 3.14159265358979323846f
#endif
// -----------------------------
// Internal timer pointer
// -----------------------------
static TIM_HandleTypeDef *servo_htim = NULL;

// =============================
// Initialization
// =============================
void servo_init(TIM_HandleTypeDef *htim) {
   servo_htim = htim;
   // Start PWM on both channels
   HAL_TIM_PWM_Start(servo_htim, TIM_CHANNEL_3);
   HAL_TIM_PWM_Start(servo_htim, TIM_CHANNEL_4);
}

// =============================
// Write raw microseconds
// =============================
void servo_set_us(uint32_t channel, uint16_t us) {
   if (us < SERVO_MIN_US) {
   	us = SERVO_MIN_US;
   }
   if (us > SERVO_MAX_US) {
   	us = SERVO_MAX_US;
   }
   __HAL_TIM_SET_COMPARE(servo_htim, channel, us);
}

// =============================
// Convert angle -> pulse width
// =============================
void servo_write_angle(uint32_t channel, float angle) {
   if (angle < SERVO_MIN_ANGLE) {
   	angle = SERVO_MIN_ANGLE;
   }
   if (angle > SERVO_MAX_ANGLE) {
   	angle = SERVO_MAX_ANGLE;
   }
   // Linear map: (angle - min_ang) / (max_ang - min_ang)
   float ratio = (angle - SERVO_MIN_ANGLE) / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE);
   float pulse = SERVO_MIN_US + ratio * (SERVO_MAX_US - SERVO_MIN_US);
   servo_set_us(channel, (uint16_t)pulse);
}

// =============================
// Smooth-move target setter
// =============================
void set_target_angle(ServoState *s, float new_angle) {
   // 1. Clamp input to hardware limits
   if (new_angle < SERVO_MIN_ANGLE){
   	new_angle = SERVO_MIN_ANGLE;
   }
   if (new_angle > SERVO_MAX_ANGLE) {
   	new_angle = SERVO_MAX_ANGLE;
   }
   // 2. Set the goal
   s->target_angle = new_angle;
   // 3. Enable movement flag so update_servo knows to work
   s->is_moving = 1;
}

// =============================
// Smooth movement update
// =============================
void update_servo(ServoState *s, uint32_t channel) {
   if (!s->is_moving) {
       servo_write_angle(channel, s->current_angle);
       return;
   }
   float diff = s->target_angle - s->current_angle;
   
   // Check if we are close enough to stop
   if (fabsf(diff) < s->speed_deg_per_ms) {
       s->current_angle = s->target_angle;
       s->is_moving = 0;
   }
   else {
       // Move by speed step
       if (diff > 0) {
           s->current_angle += s->speed_deg_per_ms;
       } else {
           s->current_angle -= s->speed_deg_per_ms;
       }
   }
   // Write the new incremental position to hardware
   servo_write_angle(channel, s->current_angle);
}

// =============================
// Inverse Kinematics for Inverted Gimbal
// =============================
void point_gimbal(ServoState *pan_servo, ServoState *tilt_servo, float x, float y, float z) {

   // 1. Calculate Horizontal Distance
   float xyDistance = sqrtf((x * x) + (y * y));

   // 2. PAN CALCULATION
   // atan2 returns radians (-PI to +PI)
   float pan_rad = atan2f(y, x);
   float pan_deg = (pan_rad / PI) * 180;
//
   // Map Geometric Angle to Servo Angle
   // Geometric 0 (Forward) = Servo 88 degrees
   float pan_final = 90.0f + pan_deg;

   // 3. TILT CALCULATION
   // atan2(z, dist) returns pitch.
   // Returns a negative angle
   float tilt_rad = atan2f(z, xyDistance);
   float tilt_deg = (tilt_rad / PI) * 180;
   
   float tilt_final = 120 + tilt_deg;
   
   servo_write_angle(TIM_CHANNEL_3,  pan_final);
   servo_write_angle(TIM_CHANNEL_4, tilt_final);
}

