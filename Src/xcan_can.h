#pragma once
#include <stdint.h>
// kvn
#ifndef CAN_CLOCK_MHZ
#define CAN_CLOCK_MHZ ( 40 )
#endif

#define CAN_CLOCK_HZ ( CAN_CLOCK_MHZ * 1000 * 1000 )

// sample point at 80%
#define CAN_TSEG1 ( 7 )
#define CAN_TSEG2 ( 2 )
#define CAN_SJW   ( CAN_TSEG2 )

#define CAN_BIT_LEN ( 1 + CAN_TSEG1 + CAN_TSEG2 )
//
#define CAN_FLAG_ECHO  (0x20)
#define CAN_FLAG_RTR   (0x40)
#define CAN_FLAG_EXTID (0x80)

#define CAN_ERROR_FLAG_BUSOFF    (1<<0)
#define CAN_ERROR_FLAG_RX_OVF    (1<<1)
#define CAN_ERROR_FLAG_TX_ERR    (1<<2)

typedef struct
{
  uint32_t  id;
  uint8_t   data[8];
  uint8_t   dlc;
  uint8_t   flags;
  uint8_t   dummy;
  uint64_t  timestamp;
}
can_message_t;

void xcan_can_init( void );
void xcan_can_poll( void );
void xcan_can_set_bitrate( uint32_t bitRate );
int xcan_can_send_message( const can_message_t *msg );
void xcan_can_install_rx_callback( void (*cb)( can_message_t * ) );
void xcan_can_install_tx_callback( void (*cb)( can_message_t * ) );
void xcan_can_install_error_callback( void (*cb)( uint8_t, uint8_t, uint8_t ) );
void xcan_can_set_bus_active( uint16_t mode );
void xcan_can_set_silent( uint8_t silent_mode );
void xcan_can_set_loopback( uint8_t loopback );
