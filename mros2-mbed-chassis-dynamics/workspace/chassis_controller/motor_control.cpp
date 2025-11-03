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
uint8_t pwm_period_us = 20;             // PWM period (microseconds)
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
     * steering_direction: 0=straight, 1=left, 2=right (unused in current implementation)
     * steering_angle_degrees: steering angle in degrees relative to center
     * Returns: PWM duty cycle percentage (5%-10% range for standard servo)
     */
    
    // Servo range: typically 90° to 110° maps to 5% to 10% duty cycle (1.0ms to 2.0ms on 20ms period)
    // Center position = 100° = 7.5% duty cycle (1.5ms)
    
    float servo_min_angle = 90.0f;
    float servo_max_angle = 110.0f;
    float servo_min_duty = 5.0f;
    float servo_max_duty = 10.0f;
    
    float target_angle = servo_center_angle + steering_angle_degrees;
    
    // Clamp to servo range
    if (target_angle < servo_min_angle) target_angle = servo_min_angle;
    if (target_angle > servo_max_angle) target_angle = servo_max_angle;
    
    // Map angle to duty cycle: (angle - min_angle) / (max_angle - min_angle) * (max_duty - min_duty) + min_duty
    float duty_cycle = ((target_angle - servo_min_angle) / (servo_max_angle - servo_min_angle)) * 
                       (servo_max_duty - servo_min_duty) + servo_min_duty;
    
    return duty_cycle;
}

std::tuple<float, uint8_t, uint8_t> calculate_motor_direction(uint8_t motor_direction, uint8_t speed_percent)
{
    /**
     * Determine motor direction and apply speed from direction flag
     * motor_direction: 0=forward, 1=backward, 2=stop
     * speed_percent: motor speed 0-100%
     * Returns: (motor_duty, enable_forward, enable_backward) tuple
     */
    
    uint8_t enable_forward = 0, enable_backward = 0;
    float motor_duty = (float)speed_percent / 100.0f;
    
    if (motor_direction == 0) {
        // Forward: enable_forward=1, enable_backward=0
        enable_forward = 1;
        enable_backward = 0;
    } else if (motor_direction == 1) {
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
     * pwm_period_us_val: PWM period in microseconds
     * motor_speed_percent: duty cycle as percentage (0-100)
     */
    
    // Configure PWM period (20 us = 50 kHz)
    steering_servo_pwm.period_us(pwm_period_us_val);
    motor_right_pwm.period_us(pwm_period_us_val);
    motor_left_pwm.period_us(pwm_period_us_val);
    
    // Set steering servo PWM (steering_servo_pwm)
    steering_servo_pwm.write(steering_duty);
    
    // Set motor direction pins
    motor_right_enable_forward = enable_forward;
    motor_right_enable_backward = enable_backward;
    motor_left_enable_forward = enable_forward;
    motor_left_enable_backward = enable_backward;
    
    // Set motor speed (duty cycle)
    float normalized_duty = motor_speed_percent / 100.0f;
    motor_right_pwm.write(normalized_duty);
    motor_left_pwm.write(normalized_duty);
}
