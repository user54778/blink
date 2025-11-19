#include "stm32f070xb.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"
#include "stm32f0xx_hal_rcc.h"

void SystemClockConfig(void);
void ErrorHandler(void);
// static void initGPIO(void);

int main() {
  // Initialize HAL; reset peripherals, init flash interface and systick
  HAL_Init();

  // Configure system clock
  SystemClockConfig();

  // init peripherals here
  // we need to a) init gpio ports, b) configure LD2 LED
  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  // initGPIO();
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  volatile int j = 0;
  while (j < 100) {
    /*
    GPIOA->ODR |= (1 << 2);
    HAL_Delay(500);
    GPIOA->ODR &= ~(1 << 2);
    */
    // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    // HAL_Delay(500);
    // __IO uint32_t ODR;/*!< GPIO port output data register,
    // Address offset: 0x14 */
    // turn pin LD3 (PA5, or GPIO_PIN_5) ON
    // GPIOA->BSRR = (1 << 5);
    GPIOA->ODR |= (1 << 5);
    // HAL_Delay(2000);
    for (volatile int i = 0; i < 500000; i++)
      __asm__("nop");

    // GPIOA->BSRR = ~(1 << 5);
    // GPIOA->BSRR = (1 << (5 + 16));
    GPIOA->ODR &= ~(1 << 5);
    for (volatile int i = 0; i < 500000; i++)
      __asm__("nop");
    // HAL_Delay(2000);
    j++;
  }
}

/*
static void initGPIO() {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // enable each GPIO clock
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  // why does it writepin first?
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // ?
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}
*/

//
void SystemClockConfig() {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    ErrorHandler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    ErrorHandler();
  }
}

// Custom Error_Handler
void ErrorHandler(void) {
  __disable_irq();
  while (1) {
  }
}
