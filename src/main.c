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
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct.GPIO_Pin = USER_BUTTON;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Pin = LED_BLUE | LED_GREEN | LED_RED | LED_ORANGE;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOD, &GPIO_InitStruct);
  GPIO_WriteBit(GPIOD, LED_BLUE, Bit_SET);
}

/** @brief  Config for timer
  * @param  None
  * 
  * @retval None
  */
static void timer1_config(void){
  // TIM_DeInit(TIM1);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  /* using time as counter for delay 1s */
  /** APB2 clock        = 168Mhz
    * Prescaler         = 9999
    * Timer clock       = 168Mhz
    * Timer period      = 1s
    * Timer Auto-reload = 16799
  */
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  TIM_TimeBaseInitStruct.TIM_Prescaler = 9999;
  TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStruct.TIM_Period = 16799;
  TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct);

  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM1, ENABLE);
}


/** @brief  Config for timer
  * @param  None
  * 
  * @retval None
  */
static void timer8_config(void){
  // TIM_DeInit(TIM8);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
/* using time as counter for delay 2s */
  /** APB2 clock        = 168Mhz
    * Prescaler         = 19999
    * Timer clock       = 168Mhz
    * Timer period      = 2s
    * Timer Auto-reload = 16799
  */
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  TIM_TimeBaseInitStruct.TIM_Prescaler = 19999;
  TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Down;
  TIM_TimeBaseInitStruct.TIM_Period = 16799;
  TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStruct);

  TIM_Cmd(TIM8, ENABLE);
  TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);
}

/** @brief  Config for NVIC
  * @param  None
  * 
  * @retval None
  */
static void nvic_config(void) {
  NVIC_InitTypeDef NVIC_InitStruct;
  NVIC_InitStruct.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);

  NVIC_InitStruct.NVIC_IRQChannel = TIM8_UP_TIM13_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
}

/* Main source ---------------------------------------------------------------*/
int main(void) {
  rcc_config();
  gpio_config();
  timer1_config();
  timer8_config();
  // TIM_PrescalerConfig(TIM1, 0x0F, TIM_PSCReloadMode_Immediate);
  // TIM_PrescalerConfig(TIM1, 0xFFFF, TIM_PSCReloadMode_Update);
  // TIM_CounterModeConfig(TIM1, TIM_CounterMode_Down);
  // TIM_ARRPreloadConfig(TIM1, ENABLE);
  // TIM_SetAutoreload(TIM1, 0xFFFF);
  nvic_config();
  while (1) {
    GPIO_ToggleBits(GPIOD, LED_BLUE);
    delay_ms(500);
  }
  return 0;
}

/**
  * @brief  This function handles TIM1_UP_TIM10_IRQHandler interrupt request.
  * @param  None
  * @retval None
  */
void TIM1_UP_TIM10_IRQHandler(void) {
  if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
    GPIO_ToggleBits(GPIOD, LED_GREEN);
  }
  TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
}

/**
  * @brief  This function handles TIM8_UP_TIM13_IRQHandler interrupt request.
  * @param  None
  * @retval None
  */
void TIM8_UP_TIM13_IRQHandler(void) {
  if (TIM_GetITStatus(TIM8, TIM_IT_Update) != RESET) {
    GPIO_ToggleBits(GPIOD, LED_ORANGE);
  }
  TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
}
