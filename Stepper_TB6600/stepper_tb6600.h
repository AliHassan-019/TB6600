#ifndef STEPPER_TB6600_H
#define STEPPER_TB6600_H

/*
  Author:     Ali Hassan
  WebSite:    https://github.com/AliHassan-019 
  */

#include <stdbool.h>
#include <math.h>
/* --------------- Check Mainstream series --------------- */

	#ifdef STM32F0
		#include "stm32f0xx_hal.h"       /* Import HAL library */
		#include "stm32f0xx_hal_tim.h" 
	#elif defined(STM32F1)
		#include "stm32f1xx_hal.h"       /* Import HAL library */
		#include "stm32f1xx_hal_tim.h"
	#elif defined(STM32F2)
		#include "stm32f2xx_hal.h"       /* Import HAL library */
		#include "stm32f2xx_hal_tim.h"
	#elif defined(STM32F3)
		#include "stm32f3xx_hal.h"       /* Import HAL library */
		#include "stm32f3xx_hal_tim.h"
	#elif defined(STM32F4)
		#include "stm32f4xx_hal.h"       /* Import HAL library */
		#include "stm32f4xx_hal_tim.h"
	#elif defined(STM32F7)
		#include "stm32f7xx_hal.h"       /* Import HAL library */
		#include "stm32f7xx_hal_tim.h"
	#elif defined(STM32G0)
		#include "stm32g0xx_hal.h"       /* Import HAL library */
		#include "stm32g0xx_hal_tim.h"
	#elif defined(STM32G4)
		#include "stm32g4xx_hal.h"       /* Import HAL library */
		#include "stm32g4xx_hal_tim.h"

	/* ------------ Check High Performance series ------------ */

	#elif defined(STM32H7)
		#include "stm32h7xx_hal.h"       /* Import HAL library */
		#include "stm32h7xx_hal_tim.h"

	/* ------------ Check Ultra low power series ------------- */

	#elif defined(STM32L0)
		#include "stm32l0xx_hal.h"       /* Import HAL library */
		#include "stm32l0xx_hal_tim.h"
	#elif defined(STM32L1)
		#include "stm32l1xx_hal.h"       /* Import HAL library */0
		#include "stm32l1xx_hal_tim.h"
	#elif defined(STM32L5)
		#include "stm32l5xx_hal.h"       /* Import HAL library */
		#include "stm32l5xx_hal_tim.h"
	#elif defined(STM32L4)
		#include "stm32l4xx_hal.h"       /* Import HAL library */
		#include "stm32l4xx_hal_tim.h"
	#elif defined(STM32H7)
		#include "stm32h7xx_hal.h"       /* Import HAL library */
		#include "stm32h7xx_hal_tim.h"
	#else
	#endif /* STM32F1 */

//####################################################################################################################


#define M_PI 3.14159265358979323846
#define CW  0   // Clockwise
#define CCW 1   // Counterclockwise


// Stepper Motor Configuration Struct
typedef struct {
    GPIO_TypeDef *DIR_PORT;   // Direction pin port
    uint16_t DIR_PIN;         // Direction pin number
    GPIO_TypeDef *EN_PORT;    // Enable pin port
    uint16_t EN_PIN;          // Enable pin number
    TIM_HandleTypeDef *htim;  // Timer handle (used for PWM step pulse)
    uint32_t channel;         // PWM Channel (TIM_CHANNEL_1, TIM_CHANNEL_2, etc.)
    uint16_t steps_per_rev;   // Steps per revolution (e.g., 200 for 1.8° stepper)
    uint8_t microstep_setting;// Microstepping (1, 2, 4, 8, 16, etc.)
    float pulley_diameter;    // Pulley diameter in mm
    float distance_per_rev;   // Distance moved per full revolution (calculated internally)
    int32_t current_position; // Current position in steps
    int32_t target_position;  // Target position in steps
    float current_speed;      // Current speed in mm/sec
    float max_speed;          // Maximum speed in mm/sec
    float acceleration;       // Acceleration in mm/sec²
    bool is_moving;           // Flag to indicate if the motor is moving
} StepperMotor_t;

// Function prototypes
void Stepper_Init(StepperMotor_t *motor);
void Stepper_Enable(StepperMotor_t *motor);
void Stepper_Disable(StepperMotor_t *motor);
void Stepper_SetDirection(StepperMotor_t *motor, uint8_t direction);
void Stepper_SetSpeed(StepperMotor_t *motor, float speed_mm_per_sec);
void Stepper_SetAcceleration(StepperMotor_t *motor, float acceleration_mm_per_sec2);
void Stepper_MoveToPosition(StepperMotor_t *motor, float target_position_mm);
void Stepper_Stop(StepperMotor_t *motor);
void Stepper_Update(StepperMotor_t *motor);  // Non-blocking update function
int32_t Stepper_GetCurrentPosition(StepperMotor_t *motor);
bool Stepper_IsMoving(StepperMotor_t *motor);

#endif // STEPPER_TB6600_H