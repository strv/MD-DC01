#include "main.h"

void led_on(const int num)
{
  switch (num)
  {
  case 0:
    LL_GPIO_SetOutputPin(LED0_GPIO_Port, LED0_Pin);
    break;
  case 1:
    LL_GPIO_SetOutputPin(LED1_GPIO_Port, LED1_Pin);
    break;
  case 2:
    LL_GPIO_SetOutputPin(LED2_GPIO_Port, LED2_Pin);
    break;
  case 3:
    LL_GPIO_SetOutputPin(LED3_GPIO_Port, LED3_Pin);
    break;
  }
}

void led_off(const int num)
{
  switch (num)
  {
  case 0:
    LL_GPIO_ResetOutputPin(LED0_GPIO_Port, LED0_Pin);
    break;
  case 1:
    LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);
    break;
  case 2:
    LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);
    break;
  case 3:
    LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);
    break;
  }
}

void led_set(const int bits, const int mask)
{
  if ((mask & 0x01) && (bits & 0x01))
    LL_GPIO_SetOutputPin(LED0_GPIO_Port, LED0_Pin);
  else if (mask & 0x01)
    LL_GPIO_ResetOutputPin(LED0_GPIO_Port, LED0_Pin);

  if ((mask & 0x02) && (bits & 0x02))
    LL_GPIO_SetOutputPin(LED1_GPIO_Port, LED1_Pin);
  else if (mask & 0x02)
    LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);

  if ((mask & 0x04) && (bits & 0x04))
    LL_GPIO_SetOutputPin(LED2_GPIO_Port, LED2_Pin);
  else if (mask & 0x04)
    LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);

  if ((mask & 0x08) && (bits & 0x08))
    LL_GPIO_SetOutputPin(LED3_GPIO_Port, LED3_Pin);
  else if (mask & 0x08)
    LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);
}
