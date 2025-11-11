/**
 * @file motor_control.cpp
 * @brief Motor control implementation: steering servo and drive motor PWM control
 */

#include "mbed.h"
#include "motor_control.h"
#include <tuple>

/* =====================================
 * Motor Control Constants
 * ===================================== */

const uint32_t MOTOR_RESPONSE_PERIOD_MS = 50;   // Motor control task polling rate (~20 Hz)

/* =====================================
 * Hardware PWM & Motor Control Pins
 * ===================================== */

// Steering servo (PA_3)
PwmOut steering_servo_pwm(PA_3);

// Motor driver PWM (left and right motors)
PwmOut motor_right_pwm(PA_6);
PwmOut motor_left_pwm(PE_11);

// Motor driver direction pins (H-bridge enable)
DigitalOut motor_right_enable_forward(PF_12);
DigitalOut motor_right_enable_backward(PD_15);
DigitalOut motor_left_enable_forward(PF_13);
DigitalOut motor_left_enable_backward(PE_9);

/* =====================================
 * Motor Control Parameters
 * ===================================== */

uint8_t servo_center_angle = 100;       // Servo center position (degrees)
uint32_t pwm_period_us = 20000;         // PWM period (microseconds) - 20000us = 20ms for servo
uint8_t current_steering_angle = 0;     // Current steering angle

/* =====================================
 * Shared Rover Command Data
 * ===================================== */

RoverCommandData rover_cmd = {0, 0.0f, 0, 0, false};
Mutex rover_cmd_mutex;

/* =====================================
 * Motor Control Function Implementations
 * ===================================== */

float calculate_steering_pwm_duty(uint8_t steering_direction, float steering_angle_degrees)
{
    /**
     * Calculate steering servo PWM duty cycle from steering angle
     * steering_direction: 1=right, 3=left, other=straight
     * steering_angle_degrees: steering angle in degrees relative to center
     * Returns: PWM duty cycle (0.0 to 1.0)
     * 
     * Original formula from working code:
     * duty = 0.05 + (degree / 180.0) * (0.10 - 0.05)
     * This maps 0°-180° to 5%-10% duty cycle (1ms-2ms pulse on 20ms period)
     */
    
    float target_angle;
    
    if (steering_direction == 1) {
        // Turn right: subtract from center
        target_angle = servo_center_angle - steering_angle_degrees;
    } else if (steering_direction == 3) {
        // Turn left: add to center
        target_angle = servo_center_angle + steering_angle_degrees;
    } else {
        // Straight: use center
        target_angle = servo_center_angle;
    }
    
    // Clamp to valid servo range (0-180 degrees)
    if (target_angle < 0.0f) target_angle = 0.0f;
    if (target_angle > 180.0f) target_angle = 180.0f;
    
    // Original working formula: 0.05 + (degree / 180.0) * (0.10 - 0.05)
    float duty_cycle = 0.05f + (target_angle / 180.0f) * (0.10f - 0.05f);
    
    // Clamp duty cycle to safe range
    if (duty_cycle < 0.0f) duty_cycle = 0.0f;
    if (duty_cycle > 1.0f) duty_cycle = 1.0f;
    
    return duty_cycle;
}

std::tuple<float, uint8_t, uint8_t> calculate_motor_direction(uint8_t motor_direction, uint8_t speed_percent)
{
    /**
     * Determine motor direction and apply speed from direction flag
     * motor_direction: 1=forward, 2=backward, 0=stop
     * speed_percent: motor speed 0-100%
     * Returns: (motor_duty, enable_forward, enable_backward) tuple
     * 
     * Original working code mapping:
     * - backDirection == 1: EN_A=1, EN_B=0 (forward)
     * - backDirection == 2: EN_A=0, EN_B=1 (backward)
     * - else: EN_A=0, EN_B=0 (stop)
     */
    
    uint8_t enable_forward = 0, enable_backward = 0;
    float motor_duty = (float)speed_percent / 100.0f;
    
    if (motor_direction == 1) {
        // Forward: enable_forward=1, enable_backward=0
        enable_forward = 1;
        enable_backward = 0;
    } else if (motor_direction == 2) {
        // Backward: enable_forward=0, enable_backward=1
        enable_forward = 0;
        enable_backward = 1;
    } else {
        // Stop: enable_forward=0, enable_backward=0
        enable_forward = 0;
        enable_backward = 0;
        motor_duty = 0.0f;
    }
    
    return std::make_tuple(motor_duty, enable_forward, enable_backward);
}

void apply_motor_control(float steering_duty, uint8_t enable_forward, uint8_t enable_backward,
                         uint8_t pwm_period_us_val, float motor_speed_percent)
{
    /**
     * Apply PWM and direction signals to motor hardware
     * steering_duty: normalized duty cycle (0.0 to 1.0)
     * enable_forward, enable_backward: direction pins for H-bridge
     * pwm_period_us_val: PWM period in microseconds (ignored - using global pwm_period_us)
     * motor_speed_percent: duty cycle as percentage (0-100)
     */
    
    // Configure PWM periods
    // Servo: 20ms period (standard servo control signal)
    steering_servo_pwm.period_us(pwm_period_us);  // 20000us = 20ms
    
    // Motors: 50us period (20 kHz for smooth motor control)
    motor_right_pwm.period_us(50);
    motor_left_pwm.period_us(50);
    
    // Set steering servo PWM (steering_servo_pwm)
    steering_servo_pwm.write(steering_duty);
    
    // Set motor direction pins
    // Left motor: normal direction
    motor_left_enable_forward = enable_forward;
    motor_left_enable_backward = enable_backward;
    
    // Right motor: OPPOSITE direction (differential drive requires opposite rotation)
    motor_right_enable_forward = enable_backward;   // Swap: forward command = backward pin
    motor_right_enable_backward = enable_forward;   // Swap: backward command = forward pin
    
    // Set motor speed (duty cycle)
    float normalized_duty = motor_speed_percent / 100.0f;
    motor_right_pwm.write(normalized_duty);
    motor_left_pwm.write(normalized_duty);
}
