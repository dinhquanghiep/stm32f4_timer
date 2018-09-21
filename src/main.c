/**
  ******************************************************************************
  * @file     main.c
  * @author   hiepdq
  * @version  xxx
  * @date     06.09.2018
  * @brief    This file provides main firmware functions for MCU 
  *           Try to using all peripheral and standard coding style    
  *           Sử dụng DAC tạo sóng sin và sóng vuông với tần số bất kỳ
  *           Sử dụng ADC để đọc lại giá trị của DAC                           
 ===============================================================================      
                       ##### How to use this driver #####
 ===============================================================================
  
  ******************************************************************************
  * @attention
  *
  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stm32f4xx.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_tim.h>
#include <misc.h>
#if defined (__GNUC__)
#include <malloc.h>
#elif defined (__ICCARM__)

#endif

/* Private macro -------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define LED_GREEN   GPIO_Pin_12
#define LED_ORANGE  GPIO_Pin_13
#define LED_RED     GPIO_Pin_14
#define LED_BLUE    GPIO_Pin_15
#define USER_BUTTON GPIO_Pin_0
/* Private typedef -----------------------------------------------------------*/
/* Public variables ----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint32_t time_ms = 0;
static volatile uint8_t tim8_update_num = 0;
static volatile uint32_t freq_clock_TIM1_CH1 = 0;
static volatile uint32_t duty_clock_TIM1_CH1 = 0;
static volatile uint32_t freq_clock_TIM1_CH2 = 0;
static volatile uint32_t freq_clock_TIM8_CH3 = 0;
// static volatile uint32_t duty_clock1 = 0;
// static volatile uint32_t duty_clock2 = 0;
/* Private function prototypes -----------------------------------------------*/
static void rcc_config(void);
static void gpio_config(void);
static void timer1_config(void);
static void timer8_config(void);
static void nvic_config(void);
/* Public functions ----------------------------------------------------------*/
/** @brief  Delay in ms
  * @param  the time to delay(unit: ms)
  * 
  * @retval None
  */
void delay_ms(uint32_t ms) {
  uint32_t curr_time_ms = time_ms;
  while (ms) {
    if (curr_time_ms != time_ms) {
      ms--;
      curr_time_ms = time_ms;
    }
  }
}
/* Private functions ---------------------------------------------------------*/
/** @brief  Config the clocks for system
  * @param  None
  * 
  * @retval None
  */
static void rcc_config(void) {
  RCC_DeInit();
  RCC_HSEConfig(RCC_HSE_ON);
  while (RCC_WaitForHSEStartUp() == ERROR) {
    /* Waitng for HSE config  */
  }
  RCC_HSICmd(DISABLE);
  RCC_PLLConfig(RCC_PLLSource_HSE, 4, 168, 2, 4);
  RCC_PLLCmd(ENABLE);
  RCC_ClockSecuritySystemCmd(DISABLE);
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
  RCC_HCLKConfig(RCC_SYSCLK_Div1);
  RCC_PCLK1Config(RCC_HCLK_Div4);
  RCC_PCLK2Config(RCC_HCLK_Div2);

  /* Config for system timer, interrupt every 1ms */
  /* 168000 if using core clock 168Mhz, 21000 if using external clock */
  SysTick_Config(167999); 
  // SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
}

/** @brief  Config for GPIO
  * @param  None
  * 
  * @retval None
  */
static void gpio_config(void) {
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* Config for PIN A0 as input: User Button */
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct.GPIO_Pin = USER_BUTTON;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Config for 4 pins PD12 PD13 PD14 PD15 as output: LED */
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Pin = LED_BLUE | LED_GREEN | LED_RED | LED_ORANGE;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOD, &GPIO_InitStruct);
  GPIO_WriteBit(GPIOD, LED_BLUE | LED_GREEN | LED_ORANGE | LED_RED, Bit_RESET);

  /* Config for pin PA8, PE11 as AF: TIM1 */
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);

  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
  GPIO_Init(GPIOE, &GPIO_InitStruct);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);

  /* Config for pin PC8 as AF: TIM8*/
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOC, &GPIO_InitStruct);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);
}

/** @brief  Config for TIM1 CH1 and CH2 measure the Frequency
  * @param  None
  * 
  * @retval None
  */
static void timer1_config(void){
  // TIM_DeInit(TIM1);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  /** APB2 clock        = 168Mhz
    * Prescaler         = 0
    * Timer clock       = 168Mhz
    * Timer period      = 0xFFFF
    * Timer Auto-reload = 0xFFFF
  */

  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
  TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStruct.TIM_Period = 0xFFFF;
  TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct);

  TIM_ICInitTypeDef TIM_ICInitStruct;
  TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStruct.TIM_ICFilter = 0;
  TIM_ICInit(TIM1, &TIM_ICInitStruct);

  /* The input signal is same as CH1 */
  TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Falling;
  TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_IndirectTI;
  TIM_ICInit(TIM1, &TIM_ICInitStruct);

  TIM_SelectInputTrigger(TIM1, TIM_TS_TI1FP1);
  TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Reset);
  TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);

  TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);
  TIM_Cmd(TIM1, ENABLE);
}


/** @brief  Config for TIM8 CH3 measure the low Frequency
  * @param  None
  * 
  * @retval None
  */
static void timer8_config(void){
  // TIM_DeInit(TIM8);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
  /** APB2 clock        = 168Mhz
    * Prescaler         = 0
    * Timer clock       = 168Mhz
    * Timer period      = 0xFFFF
    * Timer Auto-reload = 0xFFFF
  */
  /* Config for timer base */
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
  TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStruct.TIM_Period = 0xFFFF;
  TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStruct);

  /* Config for Input capture */
  TIM_ICInitTypeDef TIM_ICInitStruct;
  TIM_ICInitStruct.TIM_Channel = TIM_Channel_3;
  TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStruct.TIM_ICFilter = 0;
  TIM_ICInit(TIM8, &TIM_ICInitStruct);
  
  TIM_Cmd(TIM8, ENABLE);
  TIM_ITConfig(TIM8, TIM_IT_CC3 | TIM_IT_Update, ENABLE);
}

/** @brief  Config for NVIC
  *         TIM1 input capture for CH1 PWM input mode
  *         TIM8 input capture for CH3 and update
  * @param  None
  * 
  * @retval None
  */
static void nvic_config(void) {
  NVIC_InitTypeDef NVIC_InitStruct;
  NVIC_InitStruct.NVIC_IRQChannel = TIM1_CC_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);

  NVIC_InitStruct.NVIC_IRQChannel = TIM8_CC_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);

  NVIC_InitStruct.NVIC_IRQChannel = TIM8_UP_TIM13_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
}

/* Main source ---------------------------------------------------------------*/
int main(void) {
  rcc_config();
  gpio_config();
  timer1_config();
  timer8_config();
  nvic_config();
  // NVIC_SetPriority()
  while (1) {
    GPIO_ToggleBits(GPIOD, LED_BLUE);
    delay_ms(500);
  }
  return 0;
}

/**
  * @brief  This function handles TIM8_UP_TIM13_IRQHandler interrupt request.
  * @param  None
  * @retval None
  */

void TIM1_CC_IRQHandler(void) {
  static uint32_t capture1_CH1 = 0;
  
  #if DEBUG
  if (TIM_GetFlagStatus(TIM1, TIM_FLAG_CC1OF) || TIM_GetFlagStatus(TIM1, TIM_FLAG_CC2OF)) {
    while (1) {
      /* waiting here for Correct the frequency to avoid over*/
    }
  }
  #endif

  if (TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET) {
    capture1_CH1 = TIM_GetCapture1(TIM1);
    if (capture1_CH1 != 0) {
      freq_clock_TIM1_CH1 = 168000000 / capture1_CH1;
      /* The CCxIF is cleared after reading the value from CCRx */
      uint32_t duty_high = TIM_GetCapture2(TIM1);
      duty_clock_TIM1_CH1 = duty_high * 100 / (capture1_CH1 + duty_high);
    }
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
  }
}

/**
  * @brief  This function handles TIM8_CC_IQRHandler interrupt request.
  * @param  None
  * @retval None
  */
void TIM8_CC_IRQHandler(void) {
  static uint32_t capture1 = 0;
  static uint32_t capture2 = 0;
  static uint8_t capture_num = 0;
  if (capture_num == 0) {
    capture1 = TIM_GetCapture3(TIM8);
    tim8_update_num = 0;
    capture_num++;
  } else {
    capture2 = TIM_GetCapture3(TIM8);
    if (tim8_update_num > 1) {
      tim8_update_num -= 1;
      capture2 += 0xFFFF - capture1; 
    } else if (tim8_update_num == 1) {
      capture2 += 0xFFFF - capture1; 
      tim8_update_num = 0;
    } else {
      tim8_update_num = 0;
      capture2 -= capture1;
    }

    freq_clock_TIM8_CH3 = 168000000 / (capture2 + (tim8_update_num << 16));
    capture_num = 0;
  }
  TIM_ClearITPendingBit(TIM8, TIM_IT_CC3);
}

/**
  * @brief  This function handles TIM8_UP_TIM13_IRQHandler interrupt request.
  * @param  None
  * @retval None
  */
void TIM8_UP_TIM13_IRQHandler(void) {
  if (TIM_GetITStatus(TIM8, TIM_IT_Update) != RESET) {
    tim8_update_num++;
    TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
  }
}