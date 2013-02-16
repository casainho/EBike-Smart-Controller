/*
 * EBike Smart Controller
 *
 * Copyright (C) Jorge Pinto aka Casainho, 2012, 2013.
 *
 *   casainho [at] gmail [dot] com
 *     www.casainho.net
 *
 * Released under the GPL License, Version 3
 */

/*
 * PHASE_A -- green
 * PHASE_B -- yellow
 * PHASE_B -- blue
 *
 * PA6  (TIM3_CH1)      -- Hall sensor 1
 * PA7  (TIM3_CH2)      -- Hall sensor 2
 * PB0  (TIM3_CH3)      -- Hall sensor 3
 */


#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_dac.h"
#include "dac.h"
#include "config.h"

#define HALL_SENSORS_MASK_PA ((1 << 6) | (1 << 7))
#define HALL_SENSORS_MASK_PB (1 << 0)

GPIO_InitTypeDef GPIO_InitStructure;

void phase_a_h_on (void)
{
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void phase_a_h_off (void)
{
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void phase_a_l_on (void)
{
  GPIO_SetBits(GPIOB, GPIO_Pin_13);
}

void phase_a_l_off (void)
{
  GPIO_ResetBits(GPIOB, GPIO_Pin_13);
}


void phase_b_h_on (void)
{
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void phase_b_h_off (void)
{
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void phase_b_l_on (void)
{
  GPIO_SetBits(GPIOB, GPIO_Pin_14);
}

void phase_b_l_off (void)
{
  GPIO_ResetBits(GPIOB, GPIO_Pin_14);
}


void phase_c_h_on (void)
{
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void phase_c_h_off (void)
{
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void phase_c_l_on (void)
{
  GPIO_SetBits(GPIOB, GPIO_Pin_15);
}

void phase_c_l_off (void)
{
  GPIO_ResetBits(GPIOB, GPIO_Pin_15);
}


void commutation_sector_1 (void)
{
  phase_a_h_on ();
  phase_a_l_off ();

  phase_b_h_off ();
  phase_b_l_on ();

  phase_c_h_off ();
  phase_c_l_off ();
}

void commutation_sector_2 (void)
{
  phase_a_h_on ();
  phase_a_l_off ();

  phase_b_h_off ();
  phase_b_l_off ();

  phase_c_h_off ();
  phase_c_l_on ();
}

void commutation_sector_3 (void)
{
  phase_a_h_off ();
  phase_a_l_off ();

  phase_b_h_on ();
  phase_b_l_off ();

  phase_c_h_off ();
  phase_c_l_on ();
}

void commutation_sector_4 (void)
{
  phase_a_h_off ();
  phase_a_l_on ();

  phase_b_h_on ();
  phase_b_l_off ();

  phase_c_h_off ();
  phase_c_l_off ();
}

void commutation_sector_5 (void)
{
  phase_a_h_off ();
  phase_a_l_on ();

  phase_b_h_off ();
  phase_b_l_off ();

  phase_c_h_on ();
  phase_c_l_off ();
}

void commutation_sector_6 (void)
{
  phase_a_h_off ();
  phase_a_l_off ();

  phase_b_h_off ();
  phase_b_l_on ();

  phase_c_h_on ();
  phase_c_l_off ();
}

void commutation_disable (void)
{
  phase_a_h_off ();
  phase_a_l_off ();

  phase_b_h_off ();
  phase_b_l_off ();

  phase_c_h_off ();
  phase_c_l_off ();
}

unsigned int get_current_sector (void)
{
  static unsigned int hall_sensors = 0;
  unsigned int i;

  static unsigned int table[6] =
  {
         //  ba     c
     64, //  01000000 == 64
    192, //  11000000 == 192
    128, //  10000000 == 128
    129, //  10000001 == 129
      1, //  00000001 == 1
     65  //  01000001 == 65
  };

  hall_sensors = (GPIO_ReadInputData (GPIOA) & (HALL_SENSORS_MASK_PA)); // mask other pins
  hall_sensors |= (GPIO_ReadInputData (GPIOB) & (HALL_SENSORS_MASK_PB)); // mask other pins

  // go trough the table to identify the indice and calc the sector number
  for (i = 0; i < 6; i++)
  {
    if (table[i] == hall_sensors)
    {
      return (i + 1); // return the sector number
    }
  }

  return 0;
}

void commutate (void)
{
  volatile unsigned int sector;

  sector = get_current_sector ();

  switch (sector)
  {
    case 1:
    commutation_sector_1 ();
    break;

    case 2:
    commutation_sector_2 ();
    break;

    case 3:
    commutation_sector_3 ();
    break;

    case 4:
    commutation_sector_4 ();
    break;

    case 5:
    commutation_sector_5 ();
    break;

    case 6:
    commutation_sector_6 ();
    break;

    default:
    commutation_disable ();
    break;
  }
}

void commutation_sector (unsigned int sector)
{
  switch (sector)
  {
    case 1:
    commutation_sector_1 ();
    break;

    case 2:
    commutation_sector_2 ();
    break;

    case 3:
    commutation_sector_3 ();
    break;

    case 4:
    commutation_sector_4 ();
    break;

    case 5:
    commutation_sector_5 ();
    break;

    case 6:
    commutation_sector_6 ();
    break;

    default:
    commutation_disable ();
    break;
  }
}

unsigned int increment_sector (unsigned int sector)
{
  if (sector < 6)
  {
    sector++;
  }
  else // sector = 6
  {
    sector = 1;
  }

  return sector;
}

unsigned int decrement_sector (unsigned int sector)
{
  if (sector > 1)
  {
    sector--;
  }
  else // sector = 1
  {
    sector = 6;
  }

  return sector;
}
