/**
 * @file motor_control.h
 * @brief Motor control and steering servo PWM calculations
 *
 * Provides interfaces for:
 * - Steering servo PWM duty cycle calculation
 * - Motor direction and speed determination
 * - Hardware PWM and H-bridge control signal application
 *
 * Hardware pins:
 *   - Steering servo: PA_3 (PwmOut)
 *   - Motor right PWM: PA_6, Motor left PWM: PE_11
 *   - Motor direction: PF_12, PD_15, PF_13, PE_9 (DigitalOut)
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <tuple>
#include <cstdint>

/* =====================================
 * Motor Control Constants
 * ===================================== */

/// Motor control task polling rate (~20 Hz control loop)
extern const uint32_t MOTOR_RESPONSE_PERIOD_MS;

/// Servo center position (degrees)
extern uint8_t servo_center_angle;

/// PWM period in microseconds (20 us = 50 kHz)
extern uint8_t pwm_period_us;

/// Current steering angle (degrees)
extern uint8_t current_steering_angle;

/* =====================================
 * Rover Command Data Structure
 * ===================================== */

/// Shared rover command data with flag for update notification
struct RoverCommandData {
    uint8_t front_direction;        ///< 0=straight, 1=left, 2=right
    float steering_angle;           ///< Steering angle in degrees
    uint8_t back_direction;         ///< 0=forward, 1=backward, 2=stop
    uint8_t motor_speed;            ///< Motor speed 0-100%
    bool command_updated;           ///< Flag: new command available
};

/// Global rover command with Mutex protection (defined in motor_control.cpp)
extern RoverCommandData rover_cmd;
extern class Mutex rover_cmd_mutex;

/* =====================================
 * Motor Control Functions
 * ===================================== */

/**
 * Calculate steering servo PWM duty cycle from steering angle
 *
 * @param steering_direction 0=straight, 1=left, 2=right (currently unused)
 * @param steering_angle_degrees Steering angle relative to center (degrees)
 * @return PWM duty cycle percentage (5%-10% range for standard servo)
 *
 * Servo mapping: 90°-110° → 5%-10% duty (1.0ms-2.0ms on 20ms period)
 * Center position: 100° → 7.5% duty (1.5ms)
 */
float calculate_steering_pwm_duty(uint8_t steering_direction, float steering_angle_degrees);

/**
 * Determine motor direction and duty cycle from direction flag and speed
 *
 * @param motor_direction 0=forward, 1=backward, 2=stop
 * @param speed_percent Motor speed 0-100%
 * @return Tuple of (motor_duty, enable_forward, enable_backward)
 *
 * Motor mapping:
 *   - Forward: enable_forward=1, enable_backward=0
 *   - Backward: enable_forward=0, enable_backward=1
 *   - Stop: both=0, duty=0.0
 */
std::tuple<float, uint8_t, uint8_t> calculate_motor_direction(uint8_t motor_direction, uint8_t speed_percent);

/**
 * Apply PWM and direction signals to motor hardware
 *
 * @param steering_duty Steering servo duty cycle (0.0-1.0 normalized)
 * @param enable_forward H-bridge forward enable pin value
 * @param enable_backward H-bridge backward enable pin value
 * @param pwm_period_us_val PWM period in microseconds
 * @param motor_speed_percent Motor PWM duty cycle as percentage (0-100)
 *
 * Configures:
 * - Servo PWM output (PA_3)
 * - Motor PWM outputs (PA_6 right, PE_11 left)
 * - Motor direction pins (PF_12, PD_15, PF_13, PE_9)
 */
void apply_motor_control(float steering_duty, uint8_t enable_forward, uint8_t enable_backward,
                         uint8_t pwm_period_us_val, float motor_speed_percent);

#endif // MOTOR_CONTROL_H
