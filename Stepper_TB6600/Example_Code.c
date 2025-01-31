#include "stm32f1xx_hal.h"  // Include the HAL library for your STM32 series
#include "stepper_tb6600.h" // Include the stepper motor library

// Declare a stepper motor instance
StepperMotor_t stepper = {
    .DIR_PORT = GPIOA,
    .DIR_PIN = GPIO_PIN_1,
    .EN_PORT = GPIOA,
    .EN_PIN = GPIO_PIN_2,
    .htim = &htim2,
    .channel = TIM_CHANNEL_1,
    .steps_per_rev = 200,
    .microstep_setting = 16,
    .motion_system = MOTION_SYSTEM_BELTED,
    .pulley_diameter = 10,
    .max_speed = 100.0,
    .acceleration = 50.0
};

// Function prototypes
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);

int main(void) {
    // Initialize the HAL Library
    HAL_Init();

    // Configure the system clock
    SystemClock_Config();

    // Initialize all configured peripherals
    MX_GPIO_Init();
    MX_TIM2_Init();

    // Initialize the stepper motor
    
    Stepper_Init(&stepper);            // Initialize the motor

    // Set motor parameters
    Stepper_SetSpeed(&stepper, 50.0);         // Set speed to 50 mm/sec
    Stepper_SetAcceleration(&stepper, 25.0);  // Set acceleration to 25 mm/sec²

    // Move the motor to a specific position
    Stepper_MoveToPosition(&stepper, 100.0);  // Move to 100 mm position

    // Main loop
    while (1) {
        // Update the motor (non-blocking)
        Stepper_Update(&stepper);

        // Check if the motor has reached the target position
        if (!Stepper_IsMoving(&stepper)) {
            // Perform other tasks or stop the program
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);  // Toggle an LED (if available)
            HAL_Delay(500);  // Delay for 500 ms
        }
    }
}

// System Clock Configuration
void SystemClock_Config(void) {
    // Configure the system clock (e.g., 72 MHz for STM32F103)
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        // Initialization Error
        while (1);
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        // Initialization Error
        while (1);
    }
}

// GPIO Initialization Function
static void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();  // Enable GPIOA clock
    __HAL_RCC_GPIOC_CLK_ENABLE();  // Enable GPIOC clock (for LED)

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure direction and enable pins
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure onboard LED (if available)
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

// TIM2 Initialization Function (for PWM)
static void MX_TIM2_Init(void) {
    __HAL_RCC_TIM2_CLK_ENABLE();  // Enable TIM2 clock

    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_HandleTypeDef htim2 = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 72 - 1;  // Prescaler for 1 MHz timer clock (72 MHz / 72)
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 1000 - 1;   // Auto-reload value (1 kHz PWM frequency)
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
        // Initialization Error
        while (1);
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 500;  // 50% duty cycle
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        // Initialization Error
        while (1);
    }

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  // Start PWM
}