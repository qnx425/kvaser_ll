#include "stm32f1xx_hal.h"
#include "xcan_timestamp.h"
#include "xcan_led.h"
#include "xcan_protocol.h"
#include "xcan_usb.h"

#include "main.h"

extern void SystemClock_Config(void);

int __io_putchar( int ch ) {
#ifdef DEBUG
	return ITM_SendChar( ch );
#else
	return ch;
#endif
}

/*
 * https://github.com/rogerclarkmelbourne/STM32duino-bootloader
 * On "generic" boards, the USB reset (to force re-enumeration by the host),
 * is triggered by reconfiguring the USB D+ line (PA12) into GPIO mode,
 * and driving PA12 low for a short period,
 * before setting the pin back to its USB operational mode.
 */
static void USBD_ForceDisconnect(void)
{
  RCC->APB2ENR |= 4;          // I/O port A clock enable
  GPIOA->CRH   &= 0xFFF0FFFF; // General purpose output push-pull
  GPIOA->CRH   |= 0x00020000; // Output mode, max speed 2 MHz
  GPIOA->BRR   |= 1uL << 12;  // Reset the corresponding ODRx bit

  HAL_Delay( 3000 );

  GPIOA->CRH   &= 0xFFF0FFFF; // Floating input (reset state)
  GPIOA->CRH   |= 0x00040000; // Input mode (reset state)
}

int main( void )
{
  HAL_Init();

  SystemClock_Config();

  USBD_ForceDisconnect();

  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_GPIO_Init();

  ITM->TCR |= 1;
  ITM->TER |= 1;

  xcan_usb_init();
  xcan_protocol_init();

  for(;;)
  {
    xcan_usb_poll();
    xcan_led_poll();
    xcan_protocol_poll();
  }
}
