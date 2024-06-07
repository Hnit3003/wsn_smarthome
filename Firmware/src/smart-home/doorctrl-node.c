#include <stdint.h>
#include <inttypes.h>

#include "contiki.h"
#include "net/routing/routing.h"
#include "random.h"
#include "net/netstack.h"
#include "net/ipv6/simple-udp.h"
#include "sys/log.h"
#include "dev/serial-line.h"

#include "dev/gpio-hal.h"
#include "os/dev/button-hal.h"
#include "dev/leds.h"
#include "i2c.h"

#include "project-conf.h"
/* ---------------------------NODE PARAMETERS-------------------------------*/
static node_t child_mote = DOORCTRL_NODE;
static is_root_received_t root_rep = NOTYET;

#define LOG_MODULE "DoorCtrl-Node"

#define UDP_CLIENT_PORT	                8765
#define UDP_SERVER_PORT	                5678

#define DOORCTRL_SEND_INTERVAL		      (3 * CLOCK_SECOND)

// #define I2C_SDA_PORT	                GPIO_B_NUM
// #define I2C_SDA_PIN		              3
// #define I2C_SCL_PORT	                GPIO_B_NUM
// #define I2C_SCL_PIN		              2

static uint8_t messg[MESSG_MAX_LENGHT];
static uip_ipaddr_t dest_ipaddr;
static struct simple_udp_connection udp_conn;

static void node_send();
static void gpio_init();
/* ---------------------------------------------------------------------------*/
/* --------------------------DS3231-RTC PARAMETERS----------------------------*/
#define DS3231_I2C_ERR                  0x01
#define DS3231_I2C_ERR_NONE             0x00

#define DS3231_SLAVE_ADDRESS            0x68

#define DS3231_SETTIME                  0

#if DS3231_SETTIME
#define SECOND                          0x00
#define MINUTE                          0x35
#define HOUR                            0x11
#define DAY                             0x07
#define DATE                            0x25
#define MONTH                           0x05
#define YEAR                            0x24
#endif    /* DS3231_SETTIME*/

typedef struct
{
    uint8_t sec;
    uint8_t min;
    uint8_t hour;
    uint8_t day;
    uint8_t date;
    uint8_t month;
    uint8_t year;
} dstime_t;

dstime_t dstime;
dstime_t dstime_queue[64] = {};
static struct etimer ds3231_timer;

void ds3231_write_time();
uint8_t ds3231_read_time(dstime_t *time);
void time_uint8t_format(dstime_t *time);
static uint8_t ds3231_i2c_check_busy(uint16_t us_timeout);
/* ---------------------------------------------------------------------------*/
/* ---------------------------DOOR-BUTTONS------------------------------------*/
static doorstate_t door_state, door_state_last;
static doorstate_t door_queue[64] = {};
static uint8_t queue_count = 0;
/* ---------------------------------------------------------------------------*/
/*------------------------PROCESSES DECLARATION-------------------------------*/
PROCESS(udp_client_process, "UDP client Door-Control Node");
PROCESS(button_process, "Button process");
PROCESS(ds3231_process, "I2C DS3231 process");
AUTOSTART_PROCESSES(&udp_client_process, &button_process, &ds3231_process);
/*---------------------------------------------------------------------------*/
/*------------------------UDP-CALLBACK FUNCTION------------------------------*/
static void udp_rx_callback(struct simple_udp_connection *c,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen)
{
  /* DEBUG */
  root_rep = *data;
  if (root_rep == RECEIVED)
  {
    printf("Root received message\n");
    if (door_queue[0] == *(data+1))
    {
      door_queue[63] = 0;
      for (uint8_t i = 1; i <= queue_count; i++)
      {
        door_queue[i-1] = door_queue[i];
        dstime_queue[i-1] = dstime_queue[i];
      }
      queue_count--;
    }
    // door_state_last = *(data+1);
  } else printf("Root reply error message");
  /* END DEBUG*/
}
/*---------------------------------------------------------------------------*/
/*-----------------------------UDP-PROCESS-----------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
  static struct etimer periodic_timer;

  PROCESS_BEGIN();

  gpio_init();
  NETSTACK_MAC.on();
  /* Initialize UDP connection */
  simple_udp_register(&udp_conn, UDP_CLIENT_PORT, NULL,
                      UDP_SERVER_PORT, udp_rx_callback);
  while(1)
  {
    etimer_set(&periodic_timer, DOORCTRL_SEND_INTERVAL);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&periodic_timer));
      if(NETSTACK_ROUTING.node_is_reachable() && NETSTACK_ROUTING.get_root_ipaddr(&dest_ipaddr)) 
      {
        gpio_hal_arch_clear_pin(0, CC2538DK_PIN_LED1);
        if (queue_count > 0)
        {
          node_send();
        }
      } else 
      {
        printf("Node is not in DAG\n");
        gpio_hal_arch_set_pin(0, CC2538DK_PIN_LED1);
      }
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/*------------------------DOOR-BUTTONS PROCESS-------------------------------*/
PROCESS_THREAD(button_process, ev, data)
{
  button_hal_button_t *btn;

  PROCESS_BEGIN();
  door_state = door_state_last = CLOSE;
  btn = button_hal_get_by_index(0);

  /* DEBUG */
  printf("Device button count: %u.\n", button_hal_button_count);
  if(btn) {
  printf("%s on pin %u with ID=0, Logic=%s, Pull=%s\n",
          BUTTON_HAL_GET_DESCRIPTION(btn), btn->pin,
          btn->negative_logic ? "Negative" : "Positive",
          btn->pull == GPIO_HAL_PIN_CFG_PULL_UP ? "Pull Up" : "Pull Down");
  }
  /* END DEBUG */
  while(1)
  {
    PROCESS_YIELD();
    if ((ev == button_hal_press_event) && (queue_count <= 64))
    {
      i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_NORMAL_BUS_SPEED);
      if (ds3231_read_time(&dstime) == DS3231_I2C_ERR_NONE)
      {
        time_uint8t_format(&dstime);
      } else printf("DS3231-RTC: Error!!!");

      btn = (button_hal_button_t *)data;
      printf("Button ID %d Pressed\n", btn->unique_id);
      if (btn == button_hal_get_by_id(BUTTON_HAL_ID_BUTTON_THREE))
      { 
        gpio_hal_arch_set_pin(0, CC2538DK_PIN_LED4);
        // door_state = OPEN;
        door_queue[queue_count] = OPEN;
        dstime_queue[queue_count] = dstime;
        queue_count++;
      }
    } else if ((ev == button_hal_release_event) && (queue_count <= 64))
    {
      i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_NORMAL_BUS_SPEED);
      if (ds3231_read_time(&dstime) == DS3231_I2C_ERR_NONE)
      {
        time_uint8t_format(&dstime);
      } else printf("DS3231-RTC: Error!!!");
      
      btn = (button_hal_button_t *)data;
      printf("Button ID %d Release\n", btn->unique_id);
      if (btn == button_hal_get_by_id(BUTTON_HAL_ID_BUTTON_THREE))
      {
        gpio_hal_arch_clear_pin(0, CC2538DK_PIN_LED4);
        // door_state = CLOSE;
        door_queue[queue_count] = CLOSE;
        dstime_queue[queue_count] = dstime;
        queue_count++;
      }
    }

  }
  PROCESS_END();
}
/*-----------------------------------------------------------------------------*/
/*-----------------------------DS3231-RTC PROCESS------------------------------*/
PROCESS_THREAD(ds3231_process, ev, data)
{
	PROCESS_BEGIN();
#if DS3231_SETTIME
  i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_NORMAL_BUS_SPEED);
	ds3231_write_time();
#endif
	while(1) 
  {
		etimer_set(&ds3231_timer, CLOCK_CONF_SECOND * 1);
		PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
		/* We must init I2C each time, because the module lose his state when enter PM2 */
		// i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_NORMAL_BUS_SPEED);
    // if (ds3231_read_time(&dstime) == DS3231_I2C_ERR_NONE)
    // {
    //   time_uint8t_format(&dstime);
    // } else printf("DS3231-RTC: Error!!!");
	}
	PROCESS_END();
}
/*----------------------------------------------------------------------------*/
/*-----------------------------FUNCTION DEFINITION----------------------------*/
static uint8_t ds3231_i2c_check_busy(uint16_t us_timeout)
{
    uint16_t count = 0;
    while (i2c_master_busy())
    {
        count = count+1;
        if (count >= us_timeout) return DS3231_I2C_ERR;
    }
    return DS3231_I2C_ERR_NONE;
}

#if DS3231_SETTIME
void ds3231_write_time()
{
  /* Set pointer address */
  i2c_master_set_slave_address(DS3231_SLAVE_ADDRESS, I2C_SEND);
  i2c_master_data_put(0x00);
  i2c_master_command(I2C_MASTER_CMD_BURST_SEND_START);
  while (i2c_master_busy()) {}
  if (i2c_master_error() == I2C_MASTER_ERR_NONE)
  {
      /* Set second */
      i2c_master_data_put(SECOND);
      i2c_master_command(I2C_MASTER_CMD_BURST_SEND_CONT);
      while (i2c_master_busy()) {}
      if (i2c_master_error() == I2C_MASTER_ERR_NONE)
      {
          /* Set minute */
          i2c_master_data_put(MINUTE);
          i2c_master_command(I2C_MASTER_CMD_BURST_SEND_CONT);
          while (i2c_master_busy()) {}
          if (i2c_master_error() == I2C_MASTER_ERR_NONE)
          {
              /* Set hour */
              i2c_master_data_put(HOUR);
              i2c_master_command(I2C_MASTER_CMD_BURST_SEND_CONT);
              while (i2c_master_busy()) {}
              if (i2c_master_error() == I2C_MASTER_ERR_NONE)
              {
                  /* Set day */
                  i2c_master_data_put(DAY);
                  i2c_master_command(I2C_MASTER_CMD_BURST_SEND_CONT);
                  while (i2c_master_busy()) {}
                  if (i2c_master_error() == I2C_MASTER_ERR_NONE)
                  {
                      /* Set date */
                      i2c_master_data_put(DATE);
                      i2c_master_command(I2C_MASTER_CMD_BURST_SEND_CONT);
                      while (i2c_master_busy()) {}
                      if (i2c_master_error() == I2C_MASTER_ERR_NONE)
                      {
                          /* Set month */
                          i2c_master_data_put(MONTH);
                          i2c_master_command(I2C_MASTER_CMD_BURST_SEND_CONT);
                          while (i2c_master_busy()) {}
                          if (i2c_master_error() == I2C_MASTER_ERR_NONE)
                          {
                              /* Set year */
                              i2c_master_data_put(YEAR);
                              i2c_master_command(I2C_MASTER_CMD_BURST_SEND_FINISH);
                              while (i2c_master_busy()) {}
                          }
                      }
                  }
              }
          }
      }
  }
}
#endif

uint8_t ds3231_read_time(dstime_t *time)
{
    /* Set pointer address */
    i2c_master_set_slave_address(DS3231_SLAVE_ADDRESS, I2C_SEND);
    i2c_master_data_put(0x00);
    i2c_master_command(I2C_MASTER_CMD_BURST_SEND_START);
    // while (i2c_master_busy()) {}
    if (ds3231_i2c_check_busy(1000) != DS3231_I2C_ERR_NONE) return DS3231_I2C_ERR;
    if (i2c_master_error() == I2C_MASTER_ERR_NONE)
    {
        /* Read reg 0x00 -> second */
        i2c_master_set_slave_address(DS3231_SLAVE_ADDRESS, I2C_RECEIVE);
        i2c_master_command(I2C_MASTER_CMD_BURST_RECEIVE_START);
        // while (i2c_master_busy()) {}
        if (ds3231_i2c_check_busy(1000) != DS3231_I2C_ERR_NONE) return DS3231_I2C_ERR;
        if (i2c_master_error() == I2C_MASTER_ERR_NONE) 
        {
            time->sec = i2c_master_data_get();
            /* Read reg 0x01 -> minute */
            i2c_master_command(I2C_MASTER_CMD_BURST_RECEIVE_CONT);
            // while (i2c_master_busy()) {}
            if (ds3231_i2c_check_busy(1000) != DS3231_I2C_ERR_NONE) return DS3231_I2C_ERR;
            if (i2c_master_error() == I2C_MASTER_ERR_NONE) 
            {
                time->min = i2c_master_data_get();
                /* Read reg 0x02 -> hour */
                i2c_master_command(I2C_MASTER_CMD_BURST_RECEIVE_CONT);
                // while (i2c_master_busy()) {}
                if (ds3231_i2c_check_busy(1000) != DS3231_I2C_ERR_NONE) return DS3231_I2C_ERR;
                if (i2c_master_error() == I2C_MASTER_ERR_NONE) 
                {
                    time->hour = i2c_master_data_get();
                    /* Read reg 0x03 -> day */
                    i2c_master_command(I2C_MASTER_CMD_BURST_RECEIVE_CONT);
                    // while (i2c_master_busy()) {}
                    if (ds3231_i2c_check_busy(1000) != DS3231_I2C_ERR_NONE) return DS3231_I2C_ERR;
                    if (i2c_master_error() == I2C_MASTER_ERR_NONE) 
                    {
                        time->day = i2c_master_data_get();
                        /* Read reg 0x04 -> date */
                        i2c_master_command(I2C_MASTER_CMD_BURST_RECEIVE_CONT);
                        // while(i2c_master_busy()) {}
                        if (ds3231_i2c_check_busy(1000) != DS3231_I2C_ERR_NONE) return DS3231_I2C_ERR;
                        if (i2c_master_error() == I2C_MASTER_ERR_NONE)
                        {  
                            time->date = i2c_master_data_get();
                            /* Read reg 0x05 -> month */
                            i2c_master_command(I2C_MASTER_CMD_BURST_RECEIVE_CONT);
                            // while(i2c_master_busy()) {}
                            if (ds3231_i2c_check_busy(1000) != DS3231_I2C_ERR_NONE) return DS3231_I2C_ERR;
                            if (i2c_master_error() == I2C_MASTER_ERR_NONE)
                            {  
                                time->month = i2c_master_data_get();
                                /* Read reg 0x06 -> year */
                                i2c_master_command(I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
                                // while(i2c_master_busy()) {}
                                if (ds3231_i2c_check_busy(1000) != DS3231_I2C_ERR_NONE) return DS3231_I2C_ERR;
                                if (i2c_master_error() == I2C_MASTER_ERR_NONE)
                                {  
                                    time->year = i2c_master_data_get();
                                    // while(i2c_master_busy()) {}
                                    if (ds3231_i2c_check_busy(1000) != DS3231_I2C_ERR_NONE) return DS3231_I2C_ERR;
                                }
                            }
                        }
                    }
                }
            }           
        }
    }
    return DS3231_I2C_ERR_NONE;
}

static void gpio_init()
{
  //DOOR LED
  gpio_hal_arch_pin_set_output(0, CC2538DK_PIN_LED1);
  gpio_hal_arch_pin_set_output(0, CC2538DK_PIN_LED4);
  gpio_hal_arch_clear_pin(0, CC2538DK_PIN_LED4);
  gpio_hal_arch_set_pin(0, CC2538DK_PIN_LED1);
  
}

void time_uint8t_format(dstime_t *time)
{
    // printf("%02x:%02x:%02x %02x/%02x/20%02x\n", time->hour, time->min, time->sec, time->date, time->month, time->year);
    time->sec = ((time->sec & 0x70)>>4)*10 + (time->sec & 0x0F);
    time->min = ((time->min & 0x70)>>4)*10 + (time->min & 0x0F);
    time->hour = ((time->hour & 0x30)>>4)*10 + (time->hour & 0x0F);
    time->date = ((time->date & 0x30)>>4)*10 + (time->date & 0x0F);
    time->month = ((time->month & 0x10)>>4)*10 + (time->month & 0x0F);
    time->year = ((time->year & 0xF0)>>4)*10 + (time->year & 0x0F); 
    printf("%d:%d:%d %d/%d/20%d\n", time->hour, time->min, time->sec, time->date, time->month, time->year);
}

static void node_send()
{
    messg[0] = child_mote;
    messg[1] = door_queue[0];
    messg[2] = dstime_queue[0].hour;
    messg[3] = dstime_queue[0].min;
    messg[4] = dstime_queue[0].sec;
    messg[5] = dstime_queue[0].date;
    messg[6] = dstime_queue[0].month;
    messg[7] = dstime_queue[0].year;
    simple_udp_sendto(&udp_conn, messg, 8, &dest_ipaddr);

    /* Send to DAG root */
    /* DEBUG */
    printf("Sending frame sequence: %d %d:%d:%d - %d/%d/20%d to root:\n", messg[1], messg[2], messg[3], messg[4], messg[5], messg[6], messg[7]);
    /* END DEBUG*/
}