#include <stm32f1xx_hal.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "xcan_can.h"
#include "xcan_timestamp.h"

#include "main.h"
#include "mcp251xfd_driver_config.h"
#include "mcp251xfd_ff_config.h"

/*
 * 1. GCC does not inline any functions when not optimizing - https://gcc.gnu.org/onlinedocs/gcc/Inline.html
 * 2. you must have the symbols defined somewhere - https://stackoverflow.com/questions/16245521/c99-inline-function-in-c-file
 * 3. https://stackoverflow.com/questions/19068705/undefined-reference-when-calling-inline-function
 * 4. https://stackoverflow.com/questions/18635665/c99-referring-to-inline-function-undefined-reference-to-xxx-and-why-should-i-pu
 *
 * function defined inline will never emit an externally visible function
 * with -O0 optimization flag we get a linker error 'undefined reference to'
 * So to fix this error, just add extern inline for this function
 */
extern inline eERRORRESULT MCP251XFD_WriteRAM32(MCP251XFD *pComp, uint16_t address, uint32_t data);
extern inline eERRORRESULT MCP251XFD_ReadRAM32(MCP251XFD *pComp, uint16_t address, uint32_t* data);
extern inline eERRORRESULT MCP251XFD_WriteSFR8(MCP251XFD *pComp, uint16_t address, const uint8_t data);
extern inline eERRORRESULT MCP251XFD_WriteSFR16(MCP251XFD *pComp, uint16_t address, const uint16_t data);
extern inline eERRORRESULT MCP251XFD_WriteSFR32(MCP251XFD *pComp, uint16_t address, const uint32_t data);
extern inline eERRORRESULT MCP251XFD_ReadSFR8(MCP251XFD *pComp, uint16_t address, uint8_t* data);
extern inline eERRORRESULT MCP251XFD_ReadSFR16(MCP251XFD *pComp, uint16_t address, uint16_t* data);
extern inline eERRORRESULT MCP251XFD_ReadSFR32(MCP251XFD *pComp, uint16_t address, uint32_t* data);
extern inline eERRORRESULT MCP251XFD_StartCAN20(MCP251XFD *pComp);
extern inline eERRORRESULT MCP251XFD_StartCANFD(MCP251XFD *pComp);
extern inline eERRORRESULT MCP251XFD_StartCANListenOnly(MCP251XFD *pComp);

extern eERRORRESULT ConfigureMCP251XFDDeviceOnEXT1(void);

#define CAN_TX_FIFO_SIZE (64)

static struct
{
  uint32_t tx_msgs;
  uint32_t tx_errs;
  uint32_t tx_ovfs;

  uint32_t rx_msgs;
  uint32_t rx_errs;
  uint32_t rx_ovfs;

  can_message_t tx_fifo[CAN_TX_FIFO_SIZE];
  uint32_t tx_head;
  uint32_t tx_tail;
  void (*rx_cb)(can_message_t *);
  void (*tx_cb)(can_message_t *);
  void (*err_cb)( uint8_t err, uint8_t rx_err, uint8_t tx_err );
}
can_dev = { 0 };

void xcan_can_init(void)
{
	eMCP251XFD_Devices device = eMPC251XFD_DEVICE_COUNT;

	if( ERR_OK == ConfigureMCP251XFDDeviceOnEXT1() ) {
	  MCP251XFD_GetDeviceID( CANEXT1, &device, NULL, NULL );
#ifdef DEBUG
	  printf( "  device : %s \r\n", MCP251XFD_DevicesNames[device] );
#endif
	}

	if( MCP2518FD != device ) {
	  Error_Handler();
	}
}

void xcan_can_set_bitrate( uint32_t bitRate )
{
	MCP251XFD *p_can = CANEXT1;
	MCP251XFD_BitTimeConfig *p_bt = p_can->UserDriverData;

	p_bt->NBRP   = CAN_CLOCK_HZ / CAN_BIT_LEN / bitRate - 1;

	p_bt->NTSEG1 = CAN_TSEG1 - 1;
	p_bt->NTSEG2 = CAN_TSEG2 - 1;
	p_bt->NSJW   = CAN_SJW   - 1;

    MCP251XFD_SetBitTimeConfiguration( p_can, p_bt, true );
}

void xcan_can_install_rx_callback( void (*cb)( can_message_t * ) )
{
  can_dev.rx_cb = cb;
}

void xcan_can_install_tx_callback( void (*cb)( can_message_t * ) )
{
  can_dev.tx_cb = cb;
}

void xcan_can_install_error_callback( void (*cb)( uint8_t, uint8_t, uint8_t ) )
{
  can_dev.err_cb = cb;
}

static int xcan_try_send_message( const can_message_t *p_msg )
{
  eERRORRESULT 			ErrorExt1 	= ERR_OK;
  eMCP251XFD_FIFOstatus 	FIFOstatus 	= 0;

  ErrorExt1 = MCP251XFD_GetFIFOStatus(CANEXT1, TXFIFO, &FIFOstatus);
  if (ErrorExt1 != ERR_OK) return ErrorExt1;

  if( FIFOstatus & MCP251XFD_TX_FIFO_NOT_FULL )
  {
      MCP251XFD_CANMessage 			TansmitMessage;
      eMCP251XFD_MessageCtrlFlags	controlFlags 	= MCP251XFD_NO_MESSAGE_CTRL_FLAGS;
      const bool 					rtr      		= p_msg->flags & CAN_FLAG_RTR ? true : false;

      if( p_msg->flags & CAN_FLAG_EXTID ) {
      	TansmitMessage.MessageID = p_msg->id & 0x1FFFFFFF;
      	controlFlags |= MCP251XFD_EXTENDED_MESSAGE_ID;
      }
      else {
      	TansmitMessage.MessageID = p_msg->id & 0x7FF;
      	controlFlags |= MCP251XFD_STANDARD_MESSAGE_ID;
      }

	  if( rtr ) {
		controlFlags |= MCP251XFD_REMOTE_TRANSMISSION_REQUEST;
	  }

      TansmitMessage.ControlFlags	= controlFlags;
      TansmitMessage.DLC 			= ( eMCP251XFD_DataLength )p_msg->dlc;
      TansmitMessage.PayloadData 	= p_msg->data;

      ErrorExt1 = MCP251XFD_TransmitMessageToFIFO(CANEXT1, &TansmitMessage, TXFIFO, true);

      if( ERR_OK == ErrorExt1 ) {
    	  ++can_dev.tx_msgs;
      }
      return (int)ErrorExt1;
  }

  return -1;
}

static void xcan_can_flush_tx( void )
{
  can_message_t *p_msg;

  /* empty fifo */
  if( can_dev.tx_head == can_dev.tx_tail )
    return;
  
  p_msg = &can_dev.tx_fifo[can_dev.tx_tail];
  if( xcan_try_send_message( p_msg ) < 0 )
    return;
  /* update fifo index */
  can_dev.tx_tail = (can_dev.tx_tail+1)&(CAN_TX_FIFO_SIZE-1);

  if( can_dev.tx_cb )
  {
    can_dev.tx_cb( p_msg );
  }
}

int xcan_can_send_message( const can_message_t *p_msg )
{
  if( !p_msg )
    return 0;

  uint32_t  tx_head_next = (can_dev.tx_head+1)&(CAN_TX_FIFO_SIZE-1);
  /* overflow ? just skip it */
  if( tx_head_next == can_dev.tx_tail )
  {
    ++can_dev.tx_ovfs;
    return -1;
  }

  can_dev.tx_fifo[can_dev.tx_head] = *p_msg;
  can_dev.tx_head = tx_head_next;

  return 0;
}

static bool busActive    = false;
static bool loopbackMode = false;
static bool silentMode 	 = false;

void xcan_can_set_silent( uint8_t silent_mode )
{
	silentMode = silent_mode ? true : false;
}

void xcan_can_set_loopback( uint8_t loopback )
{
	loopbackMode = loopback ? true : false;
}

void xcan_can_set_bus_active( uint16_t mode )
{
	MCP251XFD *p_can = CANEXT1;

	if( mode ) {
	  if( silentMode ) {
		  MCP251XFD_StartCANListenOnly( p_can );
	  }
	  else {
		  if( loopbackMode ) {
			  MCP251XFD_RequestOperationMode( p_can, MCP251XFD_EXTERNAL_LOOPBACK_MODE, true );
		  }
		  else {
			  MCP251XFD_StartCAN20( p_can );
		  }
	  }
	}
	else {
	  MCP251XFD_RequestOperationMode( p_can, MCP251XFD_CONFIGURATION_MODE, true );
	}

	busActive = mode ? true : false;
}

static void xcan_can_rx_frame( MCP251XFD *hfdcan, eMCP251XFD_FIFO fifo )
{
	can_message_t msg = { 0 };

	eERRORRESULT 			ErrorExt1 			= ERR_OK;
	uint32_t 				MessageTimeStamp 	= 0;
	uint8_t 				RxPayloadData[64];
	MCP251XFD_CANMessage	ReceivedMessage;

	ReceivedMessage.PayloadData = &RxPayloadData[0];
	// that will be received
	ErrorExt1 = MCP251XFD_ReceiveMessageFromFIFO( hfdcan, &ReceivedMessage, MCP251XFD_PAYLOAD_64BYTE,
				&MessageTimeStamp, fifo);
	if (ErrorExt1 == ERR_OK)
	{
	//***** Do what you want with the message *****
	  msg.id = ReceivedMessage.MessageID;

	  if( ReceivedMessage.ControlFlags & MCP251XFD_EXTENDED_MESSAGE_ID )
	  {
		msg.flags |= CAN_FLAG_EXTID;
	  }

	  if( ReceivedMessage.ControlFlags & MCP251XFD_REMOTE_TRANSMISSION_REQUEST )
	  {
		msg.flags |= CAN_FLAG_RTR;
	  }

	  msg.dlc = (uint8_t)ReceivedMessage.DLC;

	  msg.timestamp = xcan_timestamp_us();

	  if( msg.dlc ) {
		  memcpy( msg.data, RxPayloadData, msg.dlc );
	  }

	  if( can_dev.rx_cb )
	  {
		can_dev.rx_cb( &msg );
	  }

	  ++can_dev.rx_msgs;
	}
}

void checkForReceiveMessage( MCP251XFD * );

void xcan_can_poll(void)
{
	if( busActive ) {
		checkForReceiveMessage( CANEXT1 );
		xcan_can_flush_tx();
	}
}

void checkForReceiveMessage( MCP251XFD *hfdcan )
{
	eERRORRESULT 			ErrorExt1 	= ERR_OK;
	eMCP251XFD_FIFOstatus 	FIFOstatus 	= 0;

	ErrorExt1 = MCP251XFD_GetFIFOStatus( hfdcan, RXFIFO, &FIFOstatus );

	if( ErrorExt1 == ERR_OK ) {
		if( FIFOstatus & MCP251XFD_RX_FIFO_NOT_EMPTY ) {
			xcan_can_rx_frame( hfdcan, RXFIFO );
		}
	}

	uint8_t txErrorCount = 0;
	uint8_t rxErrorCount = 0;
	eMCP251XFD_TXRXErrorStatus status = 0;

	ErrorExt1 = MCP251XFD_GetTransmitReceiveErrorCountAndStatus(	hfdcan,
																	&txErrorCount,
																	&rxErrorCount,
																	&status	);

	if( ErrorExt1 == ERR_OK ) {
		uint8_t can_err = status & MCP251XFD_TX_BUS_PASSIVE_STATE ? 0x20 : 0;

		if( status & MCP251XFD_TX_BUS_OFF_STATE ) {
			can_err |= CAN_ERROR_FLAG_BUSOFF;
		}

		if( can_dev.err_cb ) {
			can_dev.err_cb( can_err, txErrorCount, rxErrorCount );
		}
	}
}
