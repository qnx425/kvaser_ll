#include <assert.h>
#include <stm32f1xx_hal.h>
#include "xcan_timestamp.h"

uint32_t xcan_timestamp_millis( void )
{
  return HAL_GetTick();
}

uint64_t xcan_timestamp_us( void )
{
  uint32_t tim_lo, tim_hi;
  do
  {
    tim_hi = TIM4->CNT;
    tim_lo = xcan_timestamp32_us();
  }
  while( tim_hi != TIM4->CNT );

  return ((uint64_t)tim_hi<<32u)|tim_lo;
}

uint32_t xcan_timestamp32_us( void )
{
  return ( TIM3->CNT << 16 ) | TIM2->CNT;
}

void xcan_timestamp_ticks( uint16_t *ptime )
{
  uint64_t ticks = xcan_timestamp_us();
  /* !!!!! tick clock SWOPTION_24_MHZ_CLK */
  ticks *= 24u;

  ptime[0] = ( ticks & 0xFFFF );
  ptime[1] = ( (ticks>>16u) & 0xFFFF );
  ptime[2] = ( (ticks>>32u) & 0xFFFF );
}

void xcan_timestamp_ticks_from_ts( uint16_t *ptime, uint64_t ts )
{
  uint64_t ticks = ts;
  ticks *= 24u;

  ptime[0] = ( ticks & 0xFFFF );
  ptime[1] = ( (ticks>>16u) & 0xFFFF );
  ptime[2] = ( (ticks>>32u) & 0xFFFF );
}
