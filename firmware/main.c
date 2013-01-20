/*******************************************************************************
* File Name          : main.c
* Author             : Martin Thomas, main-skeleton based on code from the
*                      STMicroelectronics MCD Application Team
* Version            : see VERSION_STRING below
* Date               : see VERSION_STRING below
* Description        : Main program body for the SD-Card tests
********************************************************************************
* License: 3BSD
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32f10x.h"
#include "main.h"
#include "stm32f10x_rcc.h"

/* Private function prototypes -----------------------------------------------*/
void Periph_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);

/* Public functions -- -------------------------------------------------------*/

/*******************************************************************************
* Function Name  : main_systick_action
* Description    : operations to be done every 1ms
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SysTick_Handler(void)
{
  static uint16_t cnt = 0;
  static uint8_t flip = 0;

  cnt++;
  if (cnt >= 500)
  {
    cnt = 0;
    /* alive sign */
    if (flip)
    {
      // PB5
      GPIO_SetBits(GPIOB, GPIO_Pin_5);
    }
    else
    {
      // PB5
      GPIO_ResetBits(GPIOB, GPIO_Pin_5);
    }
    flip = !flip;
  }
}

/*******************************************************************************
* Function Name  : main
* Description    : Main program
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main(void)
{
  /* System Clocks Configuration */
  Periph_Configuration();

  /* NVIC configuration */
  NVIC_Configuration();

  /* Configure the GPIO ports */
  GPIO_Configuration();

  /* Turn on/off LED(s) -- PB5 */
  GPIO_SetBits(GPIOB, GPIO_Pin_5);

  /* Setup SysTick Timer for 1 millisecond interrupts, also enables Systick and Systick-Interrupt */
  if (SysTick_Config(SystemCoreClock / 1000))
  {
    /* Capture error */
    while (1);
  }

  while (1)
  {

  }
}

/*******************************************************************************
* Function Name  : PeriphConfiguration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Periph_Configuration(void)
{
  /* Enable GPIOB clock. PB5 used for the LED. */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
}

/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure PB5 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
extern uint32_t _isr_vectorsflash_offs;
void NVIC_Configuration(void)
{
  /* Set the Vector Table base location at 0x08000000+_isr_vectorsflash_offs */
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, (uint32_t)&_isr_vectorsflash_offs);
}
