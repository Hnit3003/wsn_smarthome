#include "contiki.h"
#include "net/routing/routing.h"
#include "net/netstack.h"
#include "net/ipv6/simple-udp.h"
#include "dev/serial-line.h"

#include "arch/cpu/cc2538/dev/uart.h"
#include "dev/gpio-hal.h"
#include "dev/uart.h"

#include "project-conf.h"

#include "sys/log.h"
#define LOG_MODULE "Root-Node"

#define WITH_SERVER_REPLY           1
#define UDP_CLIENT_PORT	            8765
#define UDP_SERVER_PORT	            5678
#define ROOTNODE_SEND_INTERVAL      (5*CLOCK_SECOND)

void outdoor_node_recv_handler(const uint8_t *messg_recv);
void doorctrl_node_recv_handler(const uint8_t *messg_recv);
void uart_send();
static void gpio_init();

static struct simple_udp_connection udp_conn;
uip_ipaddr_t dest_ipaddr;

node_t node_rcv;

static uint8_t soilhumid_percent = 0;

typedef struct 
{
    uint8_t temp_int;
    uint8_t temp_dec;
    uint8_t humid_int;
    uint8_t humid_dec;
} sht3x_packet_t;

sht3x_packet_t sht3x_packet;

typedef struct 
{
    uint8_t light_high;
    uint8_t light_low;
} max44009_packet_t;

/*
[0]: home_air_humid_int
[1]: home_air_humid_dec
[2]: home_temperature_int
[3]: home_temperature_dec
[4]: door_update  -   0/1
[5]: door_state   -   0/1   close/open
[6]: door_hour
[7]: door_min
[8]: door_sec
[9]: door_day
[10]:door_month
[11]: door_year
[12]: otdr_light_int
[13]: otdr_light_dec
[14]: otdr_temperature_int
[15]: otdr_temperature_dec
[16]: otdr_air_humid_int
[17]: otdr_air_humid_dec
[18]: otdr_soild_humid_int
[19]: otdr_soild_humid_dec
*/

uint8_t uart_buf[20] = {0};
uint8_t door_update = 0;
uint8_t door_state = 0;
uint8_t door_hour = 0;
uint8_t door_min = 0;
uint8_t door_sec = 0;
uint8_t door_day = 0;
uint8_t door_month = 0;
uint8_t door_year = 0;

static max44009_packet_t max44009_packet;

static uint16_t max44009_light = 0;

PROCESS(udp_server_process, "UDP server");
PROCESS(uart_process, "UART process");
AUTOSTART_PROCESSES(&udp_server_process, &uart_process);
/*---------------------------------------------------------------------------*/
static void
udp_rx_callback(struct simple_udp_connection *c,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen)
{
    dest_ipaddr = *sender_addr;
    node_rcv = *(data+0);

    switch (node_rcv)
    {
    case OUTDOOR_NODE:
        outdoor_node_recv_handler(data);
        break;
    
    case DOORCTRL_NODE:
        doorctrl_node_recv_handler(data);
        break;

    default:
        break;
    }

}

static struct etimer periodic_timer;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_server_process, ev, data)
{
    PROCESS_BEGIN();
    gpio_init();
    /* Initialize DAG root */
    NETSTACK_ROUTING.root_start();
    NETSTACK_MAC.on();

    /* Initialize UDP connection */
    simple_udp_register(&udp_conn, UDP_SERVER_PORT, NULL,
                        UDP_CLIENT_PORT, udp_rx_callback);
    while(1)
    {
        etimer_set(&periodic_timer, ROOTNODE_SEND_INTERVAL);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&periodic_timer));
        uart_send();
    }

    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(uart_process, ev, data)
{
    char *cmd_rx;
    
    PROCESS_BEGIN();

    while(1)
    {
        PROCESS_YIELD();
        if(ev == serial_line_event_message)
        {
            cmd_rx = data;
            printf("Data received over UART %s\n", cmd_rx);
        if (strcmp(cmd_rx, UART_CMD1) == 0)
        {
            // printf("CMD1");
        } else if (strcmp(cmd_rx, UART_CMD2) == 0)
        {
            // printf("CMD2");
        }
        }
    }
    PROCESS_END();
}
/*---------------------------OUTDOOR NODE HANDLER----------------------------*/
void outdoor_node_recv_handler(const uint8_t *messg_recv)
{
  
    sht3x_packet.temp_int = *(messg_recv+1);
    sht3x_packet.temp_dec = *(messg_recv+2);
    sht3x_packet.humid_int = *(messg_recv+3);
    sht3x_packet.humid_dec = *(messg_recv+4);

    max44009_packet.light_high = *(messg_recv+5);
    max44009_packet.light_low = *(messg_recv+6);
    soilhumid_percent = *(messg_recv+7);

    max44009_light = ((uint16_t)max44009_packet.light_high << 8) + ((uint16_t)max44009_packet.light_low);
}

/*---------------------------OUTDOOR NODE HANDLER----------------------------*/
void doorctrl_node_recv_handler(const uint8_t *messg_recv)
{
    etimer_stop(&periodic_timer);
    uint8_t messg[MESSG_MAX_LENGHT] = {};
    door_update = 1;
    for (uint8_t i = 0; i < 8; i++)
    {
        messg[i] = *(messg_recv+i);
    }
    // printf("Door state update %d at %d:%d:%d - %d/%d/20%d to root", messg[1], messg[2], messg[3], messg[4], messg[5], messg[6], messg[7]);

    door_state = messg[1];
    door_hour =  messg[2];
    door_min = messg[3];
    door_sec = messg[4];
    door_day = messg[5];
    door_month = messg[6];
    door_year = messg[7];
    if (messg[1] == OPEN)
    {
        gpio_hal_arch_set_pin(0, CC2538DK_PIN_LED1);
    } else if (messg[1] == CLOSE)
    {
        gpio_hal_arch_clear_pin(0, CC2538DK_PIN_LED1);
    }

    is_root_received_t reply = RECEIVED;
    messg[0] = reply;
    simple_udp_sendto(&udp_conn, messg, 2, &dest_ipaddr);

    // uart_send();
    etimer_set(&periodic_timer, 3*CLOCK_SECOND);
    etimer_restart(&periodic_timer);

}

static void gpio_init()
{
  //DOOR LED
  gpio_hal_arch_pin_set_output(0, CC2538DK_PIN_LED1);
  gpio_hal_arch_clear_pin(0, CC2538DK_PIN_LED1);
}

void uart_send()
{
    printf("%02d.%02d%02d.%02d%d%d%02d%02d%02d%02d%02d%02d%04d%02d.%02d%02d.%02d%02d", 
    sht3x_packet.humid_int, sht3x_packet.humid_dec, sht3x_packet.temp_int, sht3x_packet.temp_dec,
    door_update, door_state, door_hour, door_min, door_sec, door_day, door_month, door_year, max44009_light, 
    sht3x_packet.temp_int, sht3x_packet.temp_dec, sht3x_packet.humid_int, sht3x_packet.humid_dec, soilhumid_percent);
    if (door_update == 1) door_update = 0;   
}
