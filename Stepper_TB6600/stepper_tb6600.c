#include "stepper_tb6600.h"

#ifdef STM32F0
#include "stm32f0xx_hal.h" /* Import HAL library */
#include "stm32f0xx_hal_tim.h"
#elif defined(STM32F1)
#include "stm32f1xx_hal.h" /* Import HAL library */
#include "stm32f1xx_hal_tim.h"
#elif defined(STM32F2)
#include "stm32f2xx_hal.h" /* Import HAL library */
#include "stm32f2xx_hal_tim.h"
#elif defined(STM32F3)
#include "stm32f3xx_hal.h" /* Import HAL library */
#include "stm32f3xx_hal_tim.h"
#elif defined(STM32F4)
#include "stm32f4xx_hal.h" /* Import HAL library */
#include "stm32f4xx_hal_tim.h"
#elif defined(STM32F7)
#include "stm32f7xx_hal.h" /* Import HAL library */
#include "stm32f7xx_hal_tim.h"
#elif defined(STM32G0)
#include "stm32g0xx_hal.h" /* Import HAL library */
#include "stm32g0xx_hal_tim.h"
#elif defined(STM32G4)
#include "stm32g4xx_hal.h" /* Import HAL library */
#include "stm32g4xx_hal_tim.h"

/* ------------ Check High Performance series ------------ */

#elif defined(STM32H7)
#include "stm32h7xx_hal.h" /* Import HAL library */
#include "stm32h7xx_hal_tim.h"

/* ------------ Check Ultra low power series ------------- */

#elif defined(STM32L0)
#include "stm32l0xx_hal.h" /* Import HAL library */
#include "stm32l0xx_hal_tim.h"
#elif defined(STM32L1)
#include "stm32l1xx_hal.h" /* Import HAL library */ 0
#include "stm32l1xx_hal_tim.h"
#elif defined(STM32L5)
#include "stm32l5xx_hal.h" /* Import HAL library */
#include "stm32l5xx_hal_tim.h"
#elif defined(STM32L4)
#include "stm32l4xx_hal.h" /* Import HAL library */
#include "stm32l4xx_hal_tim.h"
#elif defined(STM32H7)
#include "stm32h7xx_hal.h" /* Import HAL library */
#include "stm32h7xx_hal_tim.h"
#else
#endif /* STM32F1 */

// #############################################################################################

// Initialize Stepper Motor
void Stepper_Init(StepperMotor_t *motor)
{
    HAL_GPIO_WritePin(motor->DIR_PORT, motor->DIR_PIN, CW);         // Default direction
    HAL_GPIO_WritePin(motor->EN_PORT, motor->EN_PIN, GPIO_PIN_SET); // Disable stepper initially
    HAL_TIM_PWM_Start(motor->htim, motor->channel);                 // Start PWM for step pulses

    // Calculate distance_per_rev based on pulley diameter
    motor->distance_per_rev = motor->pulley_diameter * M_PI; // Circumference = π * diameter
    motor->current_position = 0;
    motor->target_position = 0;
    motor->current_speed = 0;
    motor->max_speed = 100.0;   // Default max speed (mm/sec)
    motor->acceleration = 50.0; // Default acceleration (mm/sec²)
    motor->is_moving = false;
}

// Enable Stepper Driver (TB6600)
void Stepper_Enable(StepperMotor_t *motor)
{
    HAL_GPIO_WritePin(motor->EN_PORT, motor->EN_PIN, GPIO_PIN_RESET); // Enable driver
}

// Disable Stepper Driver (TB6600)
void Stepper_Disable(StepperMotor_t *motor)
{
    HAL_GPIO_WritePin(motor->EN_PORT, motor->EN_PIN, GPIO_PIN_SET); // Disable driver
}

// Set Stepper Direction
void Stepper_SetDirection(StepperMotor_t *motor, uint8_t direction)
{
    HAL_GPIO_WritePin(motor->DIR_PORT, motor->DIR_PIN, (direction) ? CCW : CW);
}

// Set Stepper Speed (in mm/sec)
void Stepper_SetSpeed(StepperMotor_t *motor, float speed_mm_per_sec)
{
    if (speed_mm_per_sec > motor->max_speed)
    {
        speed_mm_per_sec = motor->max_speed;
    }
    float steps_per_mm = (motor->steps_per_rev * motor->microstep_setting) / motor->distance_per_rev;
    float pulse_freq = speed_mm_per_sec * steps_per_mm; // Hz
    uint32_t timer_period = (1000000.0 / pulse_freq);   // Microseconds

    if (timer_period < 100)
        timer_period = 100; // Prevent too fast pulses

    __HAL_TIM_SET_AUTORELOAD(motor->htim, timer_period);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, timer_period / 2); // 50% duty cycle
    motor->current_speed = speed_mm_per_sec;
}

// Set Acceleration (in mm/sec²)
void Stepper_SetAcceleration(StepperMotor_t *motor, float acceleration_mm_per_sec2)
{
    motor->acceleration = acceleration_mm_per_sec2;
}

// Move to a specific position (in mm)
void Stepper_MoveToPosition(StepperMotor_t *motor, float target_position_mm)
{
    float steps_per_mm = (motor->steps_per_rev * motor->microstep_setting) / motor->distance_per_rev;
    motor->target_position = (int32_t)(target_position_mm * steps_per_mm);
    motor->is_moving = true;
    Stepper_Enable(motor);
}

// Stop the Stepper Motor
void Stepper_Stop(StepperMotor_t *motor)
{
    motor->target_position = motor->current_position;
    motor->is_moving = false;
    HAL_TIM_PWM_Stop(motor->htim, motor->channel); // Stop PWM
    Stepper_Disable(motor);                        // Disable driver to prevent heating
}

// Non-blocking update function (call this in a timer interrupt or main loop)
void Stepper_Update(StepperMotor_t *motor)
{
    if (!motor->is_moving)
        return;

    float steps_per_mm = (motor->steps_per_rev * motor->microstep_setting) / motor->distance_per_rev;
    int32_t steps_remaining = motor->target_position - motor->current_position;

    if (steps_remaining == 0)
    {
        motor->is_moving = false;
        Stepper_Stop(motor);
        return;
    }

    // Calculate speed based on acceleration and remaining steps
    float distance_remaining = fabsf(steps_remaining / steps_per_mm);
    float required_speed = sqrtf(2 * motor->acceleration * distance_remaining);

    if (required_speed > motor->max_speed)
    {
        required_speed = motor->max_speed;
    }

    Stepper_SetSpeed(motor, required_speed);

    // Update current position
    if (steps_remaining > 0)
    {
        motor->current_position++;
        Stepper_SetDirection(motor, 1); // Forward
    }
    else
    {
        motor->current_position--;
        Stepper_SetDirection(motor, 0); // Reverse
    }
}

// Get current position (in steps)
int32_t Stepper_GetCurrentPosition(StepperMotor_t *motor)
{
    return motor->current_position;
}

// Check if the motor is moving
bool Stepper_IsMoving(StepperMotor_t *motor)
{
    return motor->is_moving;
}