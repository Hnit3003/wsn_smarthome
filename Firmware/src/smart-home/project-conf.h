#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

/* RPL Mode */
#define RPL_CONF_MOP RPL_MOP_NON_STORING

/* USB serial takes space, free more space elsewhere */
#define SICSLOWPAN_CONF_FRAG                        0
#define UIP_CONF_BUFFER_SIZE                        160

/* IEEE802.15.4 PANID */
#define IEEE802154_CONF_PANID                       0x81a5
/* Do not start TSCH at init, wait for NETSTACK_MAC.on() */
#define TSCH_CONF_AUTOSTART                         0

/* 6TiSCH minimal schedule length.
 * Larger values result in less frequent active slots: reduces capacity and saves energy. */
#define TSCH_SCHEDULE_CONF_DEFAULT_LENGTH           3

/* Logging */
#define LOG_CONF_LEVEL                              LOG_LEVEL_WARN

#define LOG_CONF_LEVEL_RPL                          LOG_CONF_LEVEL
#define LOG_CONF_LEVEL_TCPIP                        LOG_CONF_LEVEL
#define LOG_CONF_LEVEL_IPV6                         LOG_CONF_LEVEL
#define LOG_CONF_LEVEL_6LOWPAN                      LOG_CONF_LEVEL
#define LOG_CONF_LEVEL_MAC                          LOG_CONF_LEVEL
/* Do not enable LOG_CONF_LEVEL_FRAMER on SimpleLink,
   that will cause it to print from an interrupt context. */
#ifndef CONTIKI_TARGET_SIMPLELINK
#define LOG_CONF_LEVEL_FRAMER                       LOG_CONF_LEVEL
#endif
#define TSCH_LOG_CONF_PER_SLOT                      0

/*----------------------------------NODE INFOR------------------------------------------*/
#define MESSG_MAX_LENGHT                            10
#define UART_CMD1                                   (char *)"led on"
#define UART_CMD2                                   (char *)"led off"

#define CC2538DK_PIN_LEFT                           20                  //PC4
#define CC2538DK_PIN_UP                             21                  //PC5
#define CC2538DK_PIN_DOWN                           22                  //PC6
#define CC2538DK_PIN_RIGHT                          23                  //PC7

#define CC2538DK_PIN_LED1                           16                  //PC0
#define CC2538DK_PIN_LED2                           17                  //PC1
#define CC2538DK_PIN_LED3                           8                   //PB0
#define CC2538DK_PIN_LED4                           9                   //PB1                          

typedef struct
{
    uint8_t sec;
    uint8_t min;
    uint8_t hour;
    uint8_t date;
    uint8_t month;
    uint8_t year;
} rtc_t;

typedef enum
{
    ROOT,
    OUTDOOR_NODE,
    DOORCTRL_NODE
} node_t;

typedef enum
{
    CLOSE = 0,
    OPEN
} doorstate_t;

typedef enum
{
    RECEIVED,
    NOTYET
} is_root_received_t;
/*--------------------------------------------------------------------------------------*/

#endif /* PROJECT_CONF_H_ */