#include <stdint.h>
#include <inttypes.h>

#include "contiki.h"
#include "net/routing/routing.h"
#include "random.h"
#include "net/netstack.h"
#include "net/ipv6/simple-udp.h"
#include "sys/log.h"
#include "lib/sensors.h"
// #include "sys/ctimer.h"
// #include "dev/serial-line.h"
// #include "arch/cpu/cc2538/dev/uart.h"
#include "i2c.h"

#include "arch/cpu/cc2538/dev/adc.h"

#include "project-conf.h"

/* ---------------------------NODE PARAMETERS---------------------------------*/
static node_t child_node = OUTDOOR_NODE;

#define LOG_MODULE "OutDoor-Node"

#define UDP_CLIENT_PORT	        8765  
#define UDP_SERVER_PORT	        5678

#define OUTDOORNODE_INTERVAL    (7*CLOCK_SECOND)
#define ADCGET_INTERVAL         (5*CLOCK_SECOND)

static void node_send();

static uint8_t messg[MESSG_MAX_LENGHT];
/*-----------------OUTDOOR PACKET--------------------           |<-byt->|
|---0---|---1---|---2---|---3---|---4---|---5---|---6---|---7---|---8---|
|_______|_______|_______|_______|_______|_______|_______|_______|_______|
|node_id|tempint|tempdec|humiint|humidec|light-h|brigt-l|soilh-h|soil-lo|
|_______|_______|_______|_______|_______|_______|_______|_______|_______|
*/
static uip_ipaddr_t dest_ipaddr;

static struct simple_udp_connection udp_conn;
/* ---------------------------------------------------------------------------*/
/* ------------------SHT3X - TEMPERATURE-HUMIDITY PARAMETERS------------------*/
#define SHT3X_SLAVE_ADDRESS     0x44
#define SHT3X_HCOMMAND          0x24        // Disable Clock stretching
#define SHT3X_LCOMMAND          0x00        // Repeatability high

#define SHT3X_BRK_HCOMMAND      0x30
#define SHT3X_BRK_LCOMMAND      0x93

#define SHT3X_I2C_ERR           0x01
#define SHT3X_I2C_ERR_NONE      0x00

typedef struct 
{
    uint16_t temperature;
    uint8_t temp_crc;
    uint16_t humidity;
    uint8_t humi_crc;
} sht3x_t;

typedef struct 
{
    uint8_t temp_int;
    uint8_t temp_dec;
    uint8_t humid_int;
    uint8_t humid_dec;
} sht3x_packet_t;

sht3x_t sht3x_rawdata;
sht3x_packet_t sht3x_packet;
static struct etimer sht3x_timer;

uint8_t sht3x_i2c_check_busy(uint16_t us_timeout);
void sht3x_packet_convert(sht3x_t *sht3x_rawdata, sht3x_packet_t *sht3x_packet, uint8_t preci);
uint8_t sht3x_read(sht3x_t *sht3x_rawdata);
/* ---------------------------------------------------------------------------*/
/* -----------------------MAX44009 - LIGHT PARAMETERS-------------------------*/
#define MAX44009_SENSOR "MAX44009 Sensor"

/*---------------------------------------------------------------------------*/
#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* MAX44009 status */
#define MAX44009_ERROR                      (0x00)
#define MAX44009_SUCCESS                    (0x01)
#define MAX44009_ACTIVATE                   (SENSORS_ACTIVE)
#define MAX44009_READ_RAW_LIGHT             (2)
#define MAX44009_READ_LIGHT                 (3)
#define MAX44009_RESET                      (4)
#define MAX44009_NONE                       (5)
#define MAX44009_ADDRESS                    (0x4A)
#define MAX44009_NOT_FOUND                  (0x00)

/* MAX44009 register addresses */
#define MAX44009_INT_STATUS_ADDR            (0x00)      /* R */
#define MAX44009_INT_ENABLE_ADDR            (0x01)      /* R/W */
#define MAX44009_CONFIG_ADDR                (0x02)      /* R/W */
#define MAX44009_LUX_HIGH_ADDR              (0x03)      /* R */
#define MAX44009_LUX_LOW_ADDR               (0x04)      /* R */
#define MAX44009_THR_HIGH_ADDR              (0x05)      /* R/W */
#define MAX44009_THR_LOW_ADDR               (0x06)      /* R/W */
#define MAX44009_THR_TIMER_ADDR             (0x07)      /* R/W */

/* Name MAX44009 register values */
#define MAX44009_INT_STATUS_OFF             (0x00)
#define MAX44009_INT_STATUS_ON              (0x01)
#define MAX44009_INT_DISABLED               (0x00)
#define MAX44009_INT_ENABLED                (0x01)

#define MAX44009_CONFIG_DEFAULT             (0 << 7)
#define MAX44009_CONFIG_CONTINUOUS          (1 << 7)
#define MAX44009_CONFIG_AUTO                (0 << 6)
#define MAX44009_CONFIG_MANUAL              (1 << 6)
#define MAX44009_CONFIG_CDR_NORMAL          (0 << 5)
#define MAX44009_CONFIG_CDR_DIVIDED         (1 << 5)
#define MAX44009_CONFIG_INTEGRATION_800ms   (0 << 0)
#define MAX44009_CONFIG_INTEGRATION_400ms   (1 << 0)
#define MAX44009_CONFIG_INTEGRATION_200ms   (2 << 0)
#define MAX44009_CONFIG_INTEGRATION_100ms   (3 << 0)
#define MAX44009_CONFIG_INTEGRATION_50ms    (4 << 0)
#define MAX44009_CONFIG_INTEGRATION_25ms    (5 << 0)
#define MAX44009_CONFIG_INTEGRATION_12ms    (6 << 0)
#define MAX44009_CONFIG_INTEGRATION_6ms     (7 << 0)

#define MAX44009_DEFAULT_CONFIGURATION      (MAX44009_CONFIG_DEFAULT | \
                                             MAX44009_CONFIG_AUTO | \
                                             MAX44009_CONFIG_CDR_NORMAL | \
                                             MAX44009_CONFIG_INTEGRATION_100ms)

#define MAX44009_USER_CONFIGURATION         (MAX44009_CONFIG_DEFAULT | \
                                             MAX44009_CONFIG_AUTO | \
                                             MAX44009_CONFIG_CDR_NORMAL | \
                                             MAX44009_CONFIG_INTEGRATION_800ms)

#define MAX44009_RESET_EN                    0

uint8_t max44009_i2c_check_busy(uint16_t us_timeout);         
static uint8_t max44009_burst_send(uint8_t slave_addr, uint8_t *data, uint8_t len, uint16_t us_timeout);
uint8_t max44009_single_send(uint8_t slave_addr, uint8_t data);
uint8_t max44009_single_receive(uint8_t slave_addr, uint8_t *data);
static uint8_t max44009_init(void);

#if MAX44009_RESET_EN
static uint8_t max44009_reset(void);
#endif
static uint16_t max44009_read_light(void);
static uint16_t max44009_convert_light(uint16_t lux);
static int value(int type);

typedef struct 
{
    uint8_t light_high;
    uint8_t light_low;
} max44009_packet_t;

max44009_packet_t max44009_packet;

static uint16_t max44009_light;
static struct etimer max44009_timer;
/* ---------------------------------------------------------------------------*/
/* ------------------------SOIL-HUMIDITY PARAMETERS---------------------------*/
static int16_t soilhumid = 0;
static uint8_t soilhumid_percent = 0;
/* ---------------------------------------------------------------------------*/
/*------------------------PROCESSES DECLARATION-------------------------------*/
PROCESS(udp_client_process, "UDP client Out-Door Control mote");
PROCESS(adc_process, "ADC process");
PROCESS(i2c_sht3x_process, "I2C SHT3X process");
PROCESS(i2c_max44009_process, "I2C MAX44009 process");
AUTOSTART_PROCESSES(&udp_client_process, &adc_process, &i2c_sht3x_process, &i2c_max44009_process);
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

  /* END DEBUG*/
}
/*---------------------------------------------------------------------------*/
/*-----------------------------UDP-PROCESS-----------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
  static struct etimer periodic_timer;

  PROCESS_BEGIN();

  gpio_hal_arch_pin_set_output(0, CC2538DK_PIN_LED1);
  gpio_hal_arch_set_pin(0, CC2538DK_PIN_LED1);

  NETSTACK_MAC.on();

  /* Initialize UDP connection */
  simple_udp_register(&udp_conn, UDP_CLIENT_PORT, NULL,
                      UDP_SERVER_PORT, udp_rx_callback);

  while(1)
  {
    etimer_set(&periodic_timer, OUTDOORNODE_INTERVAL);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&periodic_timer));
    if (NETSTACK_ROUTING.node_is_reachable() && NETSTACK_ROUTING.get_root_ipaddr(&dest_ipaddr)) 
    {
      gpio_hal_arch_clear_pin(0, CC2538DK_PIN_LED1);
      node_send();
    } else 
    {
      gpio_hal_arch_set_pin(0, CC2538DK_PIN_LED1);
      printf("Not in DAG\n");
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(adc_process, ev, data)
{
  static struct etimer adc_timer;

  PROCESS_BEGIN();
  etimer_set(&adc_timer, ADCGET_INTERVAL);
  adc_init();
  while(1)
  {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&adc_timer));
    adc_init();
    soilhumid = adc_get(SOC_ADC_ADCCON_CH_AIN7, SOC_ADC_ADCCON_REF_EXT_SINGLE, SOC_ADC_ADCCON_DIV_256);

    if (soilhumid > 0) 
    {
        soilhumid = soilhumid >> 4;
    }
    else 
    {
        soilhumid = 0;
    }

    if (soilhumid <= 0)
    {
        soilhumid_percent = 0;
    } else if (soilhumid > 2040)
    {
        soilhumid_percent = 99;
    } else
    {
        soilhumid_percent = soilhumid/20;
    }

    /* DEBUG UART */
    // printf("ADC value: %d -> ", adc_value>>4);
    // for (uint8_t count = 15; count > 0; count--)
    // {
    //   if (((adc_value>>count) & 0x1) == 1)
    //   {
    //     printf("1");
    //   } else
    //   {
    //     printf("0");
    //   }
    // }
    // printf("\n");
    printf("soilhumid_adc: %d\n ", soilhumid);
    etimer_set(&adc_timer, ADCGET_INTERVAL);
  }

  PROCESS_END();  
}
/*---------------------------------------------------------------------------*/
/*---------------------------SHT3X-I2C-PROCESS-------------------------------*/
PROCESS_THREAD(i2c_sht3x_process, ev, data)
{
	PROCESS_BEGIN();
	while(1) 
    {
		etimer_set(&sht3x_timer, CLOCK_CONF_SECOND * 5);
		PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
		/* We must init I2C each time, because the module lose his state when enter PM2 */
		i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, 
            I2C_SCL_PORT, I2C_SCL_PIN, 
            I2C_SCL_NORMAL_BUS_SPEED);

        if (sht3x_read(&sht3x_rawdata) !=  SHT3X_I2C_ERR)
        {
            sht3x_packet_convert(&sht3x_rawdata, &sht3x_packet, 2);
            printf("Temperature: %d.%d, Humidity: %d.%d\n", 
            sht3x_packet.temp_int, sht3x_packet.temp_dec, 
            sht3x_packet.humid_int, sht3x_packet.humid_dec);
        } else printf("SHT3X-TEMP_HUMID: Error!!!");
	}
	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/*---------------------------SHT3X-I2C-PROCESS-------------------------------*/
PROCESS_THREAD(i2c_max44009_process, ev, data)
{
    PROCESS_BEGIN();

    i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_NORMAL_BUS_SPEED);
    if (max44009_init() != MAX44009_SUCCESS) printf("MAX44009 I2C Error!\n");

    while(1)
    {
        etimer_set(&max44009_timer, CLOCK_CONF_SECOND*3);
        PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
        /* We must init I2C each time, because the module lose his state when enter PM2 */
        i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, 
            I2C_SCL_PORT, I2C_SCL_PIN, 
            I2C_SCL_NORMAL_BUS_SPEED);

        max44009_light = value(MAX44009_READ_LIGHT);
        if (max44009_light != MAX44009_ERROR)
        {
            max44009_packet.light_high = max44009_light >> 8;
            max44009_packet.light_low = max44009_light & 0x00FF;
            
            printf("light: %d lux\n", max44009_light);
            printf("ligh_packet: %02x %02x\n", max44009_packet.light_high, max44009_packet.light_low);
        } else printf("MAX44009: Read light error!!");
    }
    PROCESS_END();
}
/*----------------------------------------------------------------------------*/
/*-----------------------------FUNCTION DEFINITION----------------------------*/
static void node_send()
{

    messg[0] = child_node;
    messg[1] = sht3x_packet.temp_int;
    messg[2] = sht3x_packet.temp_dec;
    messg[3] = sht3x_packet.humid_int;
    messg[4] = sht3x_packet.humid_dec;
    messg[5] = max44009_packet.light_high;
    messg[6] = max44009_packet.light_low;
    messg[7] = soilhumid_percent;
    simple_udp_sendto(&udp_conn, messg, 9, &dest_ipaddr);

    /* Send to DAG root */
    /* DEBUG */
    printf("Sending message temperature: %d.%d humidity: %d.%d light: %d soild_humid: %d  to root", messg[1], messg[2], messg[3], messg[4], ((uint16_t)messg[5]<<8) + (uint16_t)messg[6], messg[7]);
    /* END DEBUG*/
}

uint8_t sht3x_i2c_check_busy(uint16_t us_timeout)
{
    uint16_t count = 0;
    while (i2c_master_busy())
    {
        count = count+1;
        if (count >= us_timeout) return SHT3X_I2C_ERR;
    }
    return SHT3X_I2C_ERR_NONE;
}

void sht3x_packet_convert(sht3x_t *sht3x_rawdata, sht3x_packet_t *sht3x_packet, uint8_t preci)
{
    float temp = ((float)sht3x_rawdata->temperature/65535)*175 - 45;
    float humid = ((float)sht3x_rawdata->humidity/65535)*100;

    int temp_int = (int)temp, temp_decimal = 0, humid_int = (int)humid, humid_decimal = 0; 
    preci = preci > 2 ? 2 : preci;
    uint8_t count = preci;

    temp -= temp_int;
    humid -= humid_int;
    while ((sht3x_rawdata->temperature != 0) && (count-- > 0))
    {
        temp_decimal *= 10;
        temp *= 10;
        temp_decimal += (int)temp;
        temp -= (int)temp;
    }
    sht3x_packet->temp_int = temp_int;
    sht3x_packet->temp_dec = temp_decimal;

    count = preci;
    while ((sht3x_rawdata->humidity != 0) && (count-- > 0))
    {
        humid_decimal *= 10;
        humid *= 10;
        humid_decimal += (int)humid;
        humid -= (int)humid;
    }
    sht3x_packet->humid_int = humid_int;
    sht3x_packet->humid_dec = humid_decimal;
}

uint8_t sht3x_read(sht3x_t *sht3x_rawdata)
{
    i2c_master_set_slave_address(SHT3X_SLAVE_ADDRESS, I2C_SEND);
    /* Send command: Disable Clock stretching */
    i2c_master_data_put(SHT3X_HCOMMAND);
    i2c_master_command(I2C_MASTER_CMD_BURST_SEND_START);
    // while (i2c_master_busy()) {}
    if (sht3x_i2c_check_busy(1000) != SHT3X_I2C_ERR_NONE) return SHT3X_I2C_ERR;
    if (i2c_master_error() == I2C_MASTER_ERR_NONE)
    {
        i2c_master_data_put(SHT3X_LCOMMAND);
        i2c_master_command(I2C_MASTER_CMD_BURST_SEND_CONT);
        // while (i2c_master_busy()) {}
        if (sht3x_i2c_check_busy(1000) != SHT3X_I2C_ERR_NONE) return SHT3X_I2C_ERR;
        if (i2c_master_error() == I2C_MASTER_ERR_NONE)
        {
            /*
            * Temp max measurement time for SHT3X is 15ms
            * Wait for 20ms
            */
            clock_delay_usec(20000);	// Maybe cause of problem with watchdog

            i2c_master_set_slave_address(SHT3X_SLAVE_ADDRESS, I2C_RECEIVE);
            i2c_master_command(I2C_MASTER_CMD_BURST_RECEIVE_START);
            // while (i2c_master_busy()) {}
            if (sht3x_i2c_check_busy(1000) != SHT3X_I2C_ERR_NONE) return SHT3X_I2C_ERR;
            if (i2c_master_error() != I2C_MASTER_ERR_NONE)
            {
                clock_delay_usec(20000);	// Maybe cause of problem with watchdog
                i2c_master_set_slave_address(SHT3X_SLAVE_ADDRESS, I2C_RECEIVE);
                i2c_master_command(I2C_MASTER_CMD_BURST_RECEIVE_START);
                // while (i2c_master_busy()) {}
                if (sht3x_i2c_check_busy(1000) != SHT3X_I2C_ERR_NONE) return SHT3X_I2C_ERR;
            }
            if (i2c_master_error() == I2C_MASTER_ERR_NONE)
            {
                sht3x_rawdata->temperature = i2c_master_data_get();
                sht3x_rawdata->temperature = sht3x_rawdata->temperature << 8;
                i2c_master_command(I2C_MASTER_CMD_BURST_RECEIVE_CONT);
                // while (i2c_master_busy()) {}
                if (sht3x_i2c_check_busy(1000) != SHT3X_I2C_ERR_NONE) return SHT3X_I2C_ERR;
                if (i2c_master_error() == I2C_MASTER_ERR_NONE)
                {
                    sht3x_rawdata->temperature |= i2c_master_data_get();
                    i2c_master_command(I2C_MASTER_CMD_BURST_RECEIVE_CONT);
                    // while (i2c_master_busy()) {}
                    if (sht3x_i2c_check_busy(1000) != SHT3X_I2C_ERR_NONE) return SHT3X_I2C_ERR;
                    if (i2c_master_error() == I2C_MASTER_ERR_NONE)
                    {
                        sht3x_rawdata->temp_crc = i2c_master_data_get();
                        i2c_master_command(I2C_MASTER_CMD_BURST_RECEIVE_CONT);
                        // while (i2c_master_busy()) {}
                        if (sht3x_i2c_check_busy(1000) != SHT3X_I2C_ERR_NONE) return SHT3X_I2C_ERR;
                        if (i2c_master_error() == I2C_MASTER_ERR_NONE)
                        {
                            sht3x_rawdata->humidity = i2c_master_data_get();
                            sht3x_rawdata->humidity = sht3x_rawdata->humidity << 8;
                            i2c_master_command(I2C_MASTER_CMD_BURST_RECEIVE_CONT);
                            // while (i2c_master_busy()) {}
                            if (sht3x_i2c_check_busy(1000) != SHT3X_I2C_ERR_NONE) return SHT3X_I2C_ERR;
                            if (i2c_master_error() == I2C_MASTER_ERR_NONE)
                            {
                                sht3x_rawdata->humidity |= i2c_master_data_get();
                                i2c_master_command(I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
                                // while (i2c_master_busy()) {}
                                if (sht3x_i2c_check_busy(1000) != SHT3X_I2C_ERR_NONE) return SHT3X_I2C_ERR;
                                if (i2c_master_error() == I2C_MASTER_ERR_NONE)
                                {
                                    sht3x_rawdata->humi_crc = i2c_master_data_get() << 8;
                                    // while (i2c_master_busy()) {}
                                    if (sht3x_i2c_check_busy(1000) != SHT3X_I2C_ERR_NONE) return SHT3X_I2C_ERR;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return SHT3X_I2C_ERR_NONE;
}

uint8_t max44009_i2c_check_busy(uint16_t us_timeout)
{
    uint16_t count = 0;
    while (i2c_master_busy())
    {
      count = count+1;
      if (count >= us_timeout) return MAX44009_ERROR;
    }
    return MAX44009_SUCCESS;
}

static uint8_t max44009_burst_send(uint8_t slave_addr, uint8_t *data, uint8_t len, uint16_t us_timeout)
{
  uint8_t sent;
  if((len == 0) || (data == NULL)) {
    return I2CM_STAT_INVALID;
  }
  if(len == 1) {
    return i2c_single_send(slave_addr, data[0]);
  }
  i2c_master_set_slave_address(slave_addr, I2C_SEND);
  i2c_master_data_put(data[0]);
  i2c_master_command(I2C_MASTER_CMD_BURST_SEND_START);
  if (max44009_i2c_check_busy(us_timeout) != MAX44009_SUCCESS) return MAX44009_ERROR;
  if (i2c_master_error() == I2C_MASTER_ERR_NONE) 
  {
    for (sent = 1; sent <= (len - 2); sent++) 
    {
      i2c_master_data_put(data[sent]);
      i2c_master_command(I2C_MASTER_CMD_BURST_SEND_CONT);
      if (max44009_i2c_check_busy(us_timeout) != MAX44009_SUCCESS) return MAX44009_ERROR;
    }
    /* This should be the last byte, stop sending */
    i2c_master_data_put(data[len - 1]);
    i2c_master_command(I2C_MASTER_CMD_BURST_SEND_FINISH);
    if (max44009_i2c_check_busy(us_timeout) != MAX44009_SUCCESS) return MAX44009_ERROR;
  }

  /* Return the STAT register of I2C module if error occurred, I2C_MASTER_ERR_NONE otherwise */
  return MAX44009_SUCCESS;
}

uint8_t max44009_single_send(uint8_t slave_addr, uint8_t data)
{
  i2c_master_set_slave_address(slave_addr, I2C_SEND);
  i2c_master_data_put(data);
  i2c_master_command(I2C_MASTER_CMD_SINGLE_SEND);

  // while(i2c_master_busy());
  if (max44009_i2c_check_busy(1000) != MAX44009_SUCCESS) return MAX44009_ERROR;

  /* Return the STAT register of I2C module if error occured, I2C_MASTER_ERR_NONE otherwise */
  return MAX44009_SUCCESS;
}

uint8_t max44009_single_receive(uint8_t slave_addr, uint8_t *data)
{
  i2c_master_set_slave_address(slave_addr, I2C_RECEIVE);
  i2c_master_command(I2C_MASTER_CMD_SINGLE_RECEIVE);

  // while(i2c_master_busy());
  if (max44009_i2c_check_busy(1000) != MAX44009_SUCCESS) return MAX44009_ERROR;
  if (i2c_master_error() == I2C_MASTER_ERR_NONE) 
  {
    *data = i2c_master_data_get();
  }
  return MAX44009_SUCCESS;
}

static uint8_t max44009_init(void)
{
   uint8_t max44009_address[5] = { MAX44009_INT_ENABLE_ADDR, MAX44009_CONFIG_ADDR, \
                                  MAX44009_THR_HIGH_ADDR, MAX44009_THR_LOW_ADDR, \
                                  MAX44009_THR_TIMER_ADDR };
  uint8_t max44009_value[5];
  uint8_t max44009_data[2];
  uint8_t i;

  max44009_value[0] = (MAX44009_INT_STATUS_OFF);
  max44009_value[1] = (MAX44009_USER_CONFIGURATION);
  max44009_value[2] = (0xFF);
  max44009_value[3] = (0x00);
  max44009_value[4] = (0xFF);

  for(i = 0; i < sizeof(max44009_address) / sizeof(max44009_address[0]); i++) 
  {
    max44009_data[0] = max44009_address[i];
    max44009_data[1] = max44009_value[i];
    if (max44009_burst_send(MAX44009_ADDRESS, max44009_data, 2, 1000) != MAX44009_SUCCESS) return MAX44009_ERROR;
    // i2c_burst_send(MAX44009_ADDRESS, max44009_data, 2);
  }

  return MAX44009_SUCCESS;
}

#if MAX44009_RESET_EN
static uint8_t max44009_reset(void)
{
  uint8_t max44009_address[5] = { MAX44009_INT_ENABLE_ADDR, MAX44009_CONFIG_ADDR, 
                                  MAX44009_THR_HIGH_ADDR, MAX44009_THR_LOW_ADDR, 
                                  MAX44009_THR_TIMER_ADDR };
  uint8_t max44009_value[5] = { 0x00, 0x03, 0xFF, 0x00, 0xFF };
  uint8_t max44009_data[2];
  uint8_t i;

  for(i = 0; i < sizeof(max44009_address) / sizeof(max44009_address[0]); i++) 
  {
    max44009_data[0] = max44009_address[i];
    max44009_data[1] = max44009_value[i];
    if (max44009_burst_send(MAX44009_ADDRESS, max44009_data, 2, 1000) != MAX44009_SUCCESS) return MAX44009_ERROR; 
    // i2c_burst_send(MAX44009_ADDRESS, max44009_data, 2);
  }
}
#endif

static uint16_t max44009_read_light(void)
{
  uint8_t exponent, mantissa;
  uint8_t max44009_data[2];
  uint32_t result;

  if (max44009_single_send(MAX44009_ADDRESS, MAX44009_LUX_HIGH_ADDR) != MAX44009_SUCCESS) return MAX44009_ERROR;
  if (max44009_single_receive(MAX44009_ADDRESS, &max44009_data[0]) != MAX44009_SUCCESS) return MAX44009_ERROR;
  if (max44009_single_send(MAX44009_ADDRESS, MAX44009_LUX_LOW_ADDR) != MAX44009_SUCCESS) return MAX44009_ERROR;
  if (max44009_single_receive(MAX44009_ADDRESS, &max44009_data[1]) != MAX44009_SUCCESS) return MAX44009_ERROR;

  exponent = ((max44009_data[0] >> 4) & 0x0E);
  mantissa = ((max44009_data[0] & 0x0F) << 4) | (max44009_data[1] & 0x0F);

  result = ((uint16_t)exponent << 8) | ((uint16_t)mantissa << 0);

  return result;
}

static uint16_t max44009_convert_light(uint16_t lux)
{
  uint8_t exponent, mantissa;
  uint32_t result;

  exponent = (lux >> 8) & 0xFF;
  exponent = (exponent == 0x0F ? exponent & 0x0E : exponent);
  mantissa = (lux >> 0) & 0xFF;

  result = 45 * (2 ^ exponent * mantissa) / 10;

  return (uint16_t)result;
}

static int value(int type)
{
  uint16_t value;

  if(type == MAX44009_READ_RAW_LIGHT) {
    return max44009_read_light();
  } else if(type == MAX44009_READ_LIGHT) {
    value = max44009_read_light();
    return max44009_convert_light(value);
  } else {
    PRINTF("MAX44009: invalid value requested\n");
    return MAX44009_ERROR;
  }
}

