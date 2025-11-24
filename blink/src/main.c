#include "stm32f070xb.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"
#include "stm32f0xx_hal_rcc.h"
#include "system_stm32f0xx.h"
#include <stdint.h>

// external global button_pressed variable
extern volatile uint8_t button_pressed;

void SystemClockConfig(void);
void ErrorHandler(void);
void delayMs(uint32_t ms);
uint8_t tickDelay(uint32_t interval);

int main() {
  // Initialize HAL; reset peripherals, init flash interface and systick
  HAL_Init();

  __enable_irq();

  // Configure system clock
  SystemClockConfig();

  // configure the SysTick interrupt to interrupt every 1 ms
  SysTick_Config(SystemCoreClock / 1000);

  // init peripherals here
  // we need to a) init gpio ports, b) configure LD2 LED
  // configure gpio
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  // initGPIO();
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // confighure EXTI line
  // NOTE: Step 1 in configuring IRQ
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // interrupt on "falling edge"
  GPIO_InitStruct.Pull = GPIO_PULLUP;          // active-low button
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // configure NVIC to enable IRQ in NVIC
  NVIC->ISER[0] |=
      (1 << EXTI4_15_IRQn); // NOTE: Use ISER AND NOT!!!! ICER (SET vs CLEAR!!!)
  EXTI->IMR |= (1 << 13);

  uint32_t start = HAL_GetTick();
  HAL_Delay(1000);
  uint32_t end = HAL_GetTick();
  if (end - start > 1100) {
    for (volatile uint32_t i = 0; i < 1000; i++) {
      __asm__("nop");
    }
  }

  // volatile int j = 0;
  // Used for decreasing length of blink delay
  volatile uint32_t blink_delay = 5000;
  // used for determining how much time has passed
  static uint32_t last_toggle_time = 0;
  while (1) {
    /*
    // an example of using interrupts to toggle a pin on and off
    if (!button_pressed) {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
      HAL_Delay(200);
    }
    GPIOA->ODR &= ~(1 << 5);
    HAL_Delay(200);
    */

    // Instead of using the button to halt the LED, for each button press, the
    // system should *decrease* the length of the *delay* by 50%.
    // if (button_pressed) {
    if (button_pressed) {
      // set delay period
      blink_delay /= 2;
      // set button pressed
      if (blink_delay < 10) {
        blink_delay = 10000;
      }
      button_pressed = 0;
    }

    // if enough time has passed
    // i.e., an we haven't clicked a button press in at least blink delay time
    if (HAL_GetTick() - last_toggle_time >= blink_delay) {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
      last_toggle_time = HAL_GetTick(); // clear how much time has passed
    }
    //}
    // if enough time has passed
    // toggle LED
    // clear how much time has passed

    // B1 is configured as a pull-down resistor.
    // Button will pull the pin LOW, so our condition is inverted

    /* Solution 1
    if (!(GPIOC->IDR & (1 << 13))) {
      GPIOA->ODR &= ~(1 << 5);
      delayMs(500);
    } else { // button not pressed
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
      delayMs(100);
    }
    */

    // Solution 2
    /*
    if (!(GPIOC->IDR & (1 << 13))) {
      GPIOA->ODR &= ~(1 << 5);
    } else {
      if (tickDelay(100)) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
      }
    }
    */

    // 3: Instead of using the button to halt the LED, marketing wants to test
    // different blink rates
    // by tapping the button. For each button press, the system should decrease
    // the length of the delay by 50% (until it gets to near zero, at which
    // point it should go back to the initial delay).
    //
    // Step a: We need to IRQ on a button press -> HOW? And what should it do?
    // Probably just activate?
  }
}

// Configure the CPU clock tree, NOT THE CPU TIMERS!!!!
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

// Custom E
void ErrorHandler(void) {
  __disable_irq();
  while (1) {
  }
}

// temp solution to not having HAL_Delay
void delayMs(uint32_t ms) {
  // 8 MHz * PLL (x6) = 48 MHz clock
  for (volatile uint32_t i = 0; i < ms * 8000; i++) {
    __asm__("nop");
  }
}

// implement the interrupt handler for system ticks
void SysTick_Handler() { HAL_IncTick(); }

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
//
//
// GPIOA->BSRR = ~(1 << 5);
// GPIOA->BSRR = (1 << (5 + 16));

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
