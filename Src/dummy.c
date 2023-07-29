#include "xcan_can.h"

#ifdef DEBUG
const char* cmdStr[] = {
    "0                                   ",
    "1                                   ",
    "2                                   ",
    "3                                   ",
    "4                                   ",
    "5                                   ",
    "6                                   ",
    "7                                   ",
    "8                                   ",
    "9                                   ",
    "10                                  ",
    "11                                  ",
    "CMD_RX_STD_MESSAGE                  ", //  12
    "CMD_TX_STD_MESSAGE                  ", //  13
    "CMD_RX_EXT_MESSAGE                  ", //  14
    "CMD_TX_EXT_MESSAGE                  ", //  15
    "CMD_SET_BUSPARAMS_REQ               ", //  16
    "CMD_GET_BUSPARAMS_REQ               ", //  17
    "CMD_GET_BUSPARAMS_RESP              ", //  18
    "CMD_GET_CHIP_STATE_REQ              ", //  19
    "CMD_CHIP_STATE_EVENT                ", //  20
    "CMD_SET_DRIVERMODE_REQ              ", //  21
    "CMD_GET_DRIVERMODE_REQ              ", //  22
    "CMD_GET_DRIVERMODE_RESP             ", //  23
    "CMD_RESET_CHIP_REQ                  ", //  24
    "CMD_RESET_CARD_REQ                  ", //  25
    "CMD_START_CHIP_REQ                  ", //  26
    "CMD_START_CHIP_RESP                 ", //  27
    "CMD_STOP_CHIP_REQ                   ", //  28
    "CMD_STOP_CHIP_RESP                  ", //  29
    "CMD_READ_CLOCK_REQ                  ", //  30
    "CMD_READ_CLOCK_RESP                 ", //  31
    "CMD_GET_CARD_INFO_2                 ", //  32
    "33                                  ",
    "CMD_GET_CARD_INFO_REQ               ", //  34
    "CMD_GET_CARD_INFO_RESP              ", //  35
    "CMD_GET_INTERFACE_INFO_REQ          ", //  36
    "CMD_GET_INTERFACE_INFO_RESP         ", //  37
    "CMD_GET_SOFTWARE_INFO_REQ           ", //  38
    "CMD_GET_SOFTWARE_INFO_RESP          ", //  39
    "CMD_GET_BUSLOAD_REQ                 ", //  40
    "CMD_GET_BUSLOAD_RESP                ", //  41
    "CMD_RESET_STATISTICS                ", //  42
    "CMD_CHECK_LICENSE_REQ               ", //  43
    "CMD_CHECK_LICENSE_RESP              ", //  44
    "CMD_ERROR_EVENT                     ", //  45
    "46 reserved                         ",
    "47 reserved                         ",
    "CMD_FLUSH_QUEUE                     ", //  48
    "CMD_RESET_ERROR_COUNTER             ", //  49
    "CMD_TX_ACKNOWLEDGE                  ", //  50
    "CMD_CAN_ERROR_EVENT                 ", //  51
    "CMD_MEMO_GET_DATA                   ", //  52
    "CMD_MEMO_PUT_DATA                   ", //  53
    "CMD_MEMO_PUT_DATA_START             ", //  54
    "CMD_MEMO_ASYNCOP_START              ", //  55
    "CMD_MEMO_ASYNCOP_GET_DATA           ", //  56
    "CMD_MEMO_ASYNCOP_CANCEL             ", //  57
    "CMD_MEMO_ASYNCOP_FINISHED           ", //  58
    "CMD_DISK_FULL_INFO                  ", //  59
    "CMD_TX_REQUEST                      ", //  60
    "CMD_SET_HEARTBEAT_RATE_REQ          ", //  61
    "CMD_HEARTBEAT_RESP                  ", //  62
    "CMD_SET_AUTO_TX_BUFFER              ", //  63
    "CMD_GET_EXTENDED_INFO               ", //  64
    "CMD_TCP_KEEPALIVE                   ", //  65
    "CMD_TX_INTERVAL_REQ                 ", //  66
    "CMD_TX_INTERVAL_RESP                ", //  67
    "CMD_FILO_FLUSH_QUEUE_RESP           ", //  68
    "69 reused                           ",
    "70 reused                           ",
    "71 reused                           ",
    "CMD_AUTO_TX_BUFFER_REQ              ", //  72
    "CMD_AUTO_TX_BUFFER_RESP             ", //  73
    "CMD_SET_TRANSCEIVER_MODE_REQ        ", //  74
    "CMD_TREF_SOFNR                      ", //  75
    "CMD_SOFTSYNC_ONOFF                  ", //  76
    "CMD_USB_THROTTLE                    ", //  77
    "CMD_SOUND                           ", //  78
    "CMD_LOG_TRIG_STARTUP                ", //  79
    "CMD_SELF_TEST_REQ                   ", //  80
    "CMD_SELF_TEST_RESP                  ", //  81
    "CMD_USB_THROTTLE_SCALED             ", //  82
    "83 reused                           ",
    "84 reused                           ",
    "85 reused                           ",
    "CMD_SET_IO_PORTS_REQ                ", //  86
    "CMD_GET_IO_PORTS_REQ                ", //  87
    "CMD_GET_IO_PORTS_RESP               ", //  88
    "89 reused                           ",
    "90 reused                           ",
    "91 reused                           ",
    "92 reused                           ",
    "93 reused                           ",
    "94 reused                           ",
    "CMD_GET_CAPABILITIES_REQ            ", //  95
    "CMD_GET_CAPABILITIES_RESP           ", //  96
    "CMD_GET_TRANSCEIVER_INFO_REQ        ", //  97
    "CMD_GET_TRANSCEIVER_INFO_RESP       ", //  98
    "CMD_MEMO_CONFIG_MODE                ", //  99
    "100 reused                          ",
    "CMD_LED_ACTION_REQ                  ", // 101
    "CMD_LED_ACTION_RESP                 ", // 102
    "CMD_INTERNAL_DUMMY                  ", // 103
    "CMD_READ_USER_PARAMETER             ", // 104
    "CMD_MEMO_CPLD_PRG                   ", // 105
    "CMD_LOG_MESSAGE                     ", // 106
    "CMD_LOG_TRIG                        ", // 107
    "CMD_LOG_RTC_TIME                    ", // 108
    "CMD_SCRIPT_ENVVAR_CTRL_REQ          ", // 109
    "CMD_SCRIPT_ENVVAR_CTRL_RESP         ", // 110
    "CMD_SCRIPT_ENVVAR_TRANSFER_CTRL_REQ ", // 111 // PC wants to set value in VM
    "CMD_SCRIPT_ENVVAR_TRANSFER_CTRL_RESP", // 112 // PC wants to set value in VM
    "CMD_SCRIPT_ENVVAR_TRANSFER_BULK     ", // 113 // PC wants to set value in VM
    "114                                 ",
    "115                                 ",
    "CMD_SCRIPT_CTRL_REQ                 ", // 116
    "CMD_SCRIPT_CTRL_RESP                ", // 117
    "CMD_SCRIPT_ENVVAR_NOTIFY_EVENT      ", // 118
    "CMD_GET_DEVICE_NAME_REQ             ", // 119
    "CMD_GET_DEVICE_NAME_RESP            ", // 120
    "CMD_PING_REQ                        ", // 121
    "CMD_PING_RESP                       ", // 122
    "CMD_SET_UNLOCK_CODE                 ", // 123
    "CMD_WRITE_USER_PARAMETER            ", // 124
    "CMD_CONFUSED_RESP                   ", // 125
};

const char* capSubCmdStr[] = {
    "CAP_SUB_CMD_DUMMY_NOT_IMPLEMENTED",
    "CAP_SUB_CMD_DUMMY_UNAVAILABLE    ",
    "CAP_SUB_CMD_SILENT_MODE          ",
    "CAP_SUB_CMD_ERRFRAME             ",
    "CAP_SUB_CMD_BUS_STATS            ",
    "CAP_SUB_CMD_ERRCOUNT_READ        ",
    "CAP_SUB_CMD_SINGLE_SHOT          ",
    "CAP_SUB_CMD_SYNC_TX_FLUSH        ",
    "CAP_SUB_CMD_HAS_LOGGER           ",
    "CAP_SUB_CMD_HAS_REMOTE           ",
    "CAP_SUB_CMD_HAS_SCRIPT           ",
};

const char* driverModeStr[] = {
    "0                       ",
    "DRIVERMODE_NORMAL       ",
    "DRIVERMODE_SILENT       ",
    "DRIVERMODE_SELFRECEPTION",
    "DRIVERMODE_OFF          ",
};
#endif
#if 0
void xcan_can_init( void ) {}
void xcan_can_poll( void ) {}
void xcan_can_set_bitrate( uint32_t bitRate ) {}
int xcan_can_send_message( const can_message_t *msg ) { return 0; }
void xcan_can_install_rx_callback( void (*cb)( can_message_t * ) ) {}
void xcan_can_install_tx_callback( void (*cb)( can_message_t * ) ) {}
void xcan_can_install_error_callback( void (*cb)( uint8_t, uint8_t, uint8_t ) ) {}
void xcan_can_set_bus_active( uint16_t mode ) {}
void xcan_can_set_silent( uint8_t silent_mode ) {}
void xcan_can_set_loopback( uint8_t loopback ) {}
#endif
