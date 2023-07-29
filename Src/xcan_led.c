#include <assert.h>
#include "stm32f1xx_hal.h"
#include "xcan_timestamp.h"
#include "xcan_led.h"
#include "boards.h"

static struct
{
  int      mode;
  uint32_t arg;
  uint32_t delay;
  uint32_t timestamp;
  uint8_t  state;
}
led_mode_array[LED_TOTAL] = { 0 };

void xcan_led_set_mode( int led, int mode, uint16_t arg )
{
  assert( led < LED_TOTAL );
  uint32_t ts = xcan_timestamp_millis();

  led_mode_array[led].mode = mode;
  if( !led_mode_array[led].timestamp )
  {
    led_mode_array[led].timestamp = ts;
  }
  led_mode_array[led].delay = 0;

  /* set guard time */
  if( mode == LED_MODE_BLINK_FAST || mode == LED_MODE_BLINK_SLOW )
  {
    led_mode_array[led].delay = ( mode == LED_MODE_BLINK_FAST ) ? 50: 200;
  }
  led_mode_array[led].arg = arg?(ts + arg):0;
}

static void xcan_led_update_state( int led, uint8_t state )
{
  HAL_GPIO_WritePin( IOPIN_PORT, IOPIN_LED0, state ? GPIO_PIN_RESET : GPIO_PIN_SET );
}

void xcan_led_poll( void )
{
  uint32_t ts_ms = xcan_timestamp_millis();

  for( int i = 0; i < LED_TOTAL; i++ )
  {
    if( !led_mode_array[i].timestamp  )
      continue;
    if( (uint32_t)( ts_ms - led_mode_array[i].timestamp ) < led_mode_array[i].delay )
      continue;

    switch( led_mode_array[i].mode )
    {
      default:
      case LED_MODE_NONE:
        led_mode_array[i].timestamp = 0;
      break;
      case LED_MODE_OFF:
      case LED_MODE_ON:
        led_mode_array[i].state = ( led_mode_array[i].mode == LED_MODE_ON );
        led_mode_array[i].timestamp = led_mode_array[i].arg;
      break;
      case LED_MODE_BLINK_FAST:
      case LED_MODE_BLINK_SLOW:
        led_mode_array[i].state ^= 1;
        led_mode_array[i].timestamp += led_mode_array[i].delay;
      break;
    }
    xcan_led_update_state( i, led_mode_array[i].state );

    /* check guard time */
    if( led_mode_array[i].arg && led_mode_array[i].arg <= ts_ms )
    {
      xcan_led_set_mode( i, LED_MODE_OFF, 0 );
    }
  }
}
