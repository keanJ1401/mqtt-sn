#include <stdlib.h>
#include <stdio.h> /* For printf() */
#include <math.h>
#include <string.h>
#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ip/uip-debug.h"
#include "dev/watchdog.h"
#include "dev/leds.h"
#include "net/rpl/rpl.h"
#include "lib/random.h"
#include "clock.h"
#include "net/ipv6/uip-ds6.h"
#include "mqtt_sn.h"
#include "net/rime/rime.h"
#include "net/ip/uip.h"
#include "dev/adc.h"
#include "dev/gpio.h"
#include "sys/etimer.h"
#include "sys/ctimer.h" // inoder to add call back timer
#include "dev/i2c.h"
#include "bmpx8x.h"
#include "si7021.h"
#define MAX4409_ADDRESS 0x4A
#define INTERVAL_READING 30*(CLOCK_SECOND)
/*----------------------------------Timer-----------------------------------------*/
static struct etimer timer_adc;
static struct ctimer ct;
#define BUSYWAIT_UNTIL(max_time) \
  do { \
    rtimer_clock_t t0; \
    t0 = RTIMER_NOW(); \
    while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time))) { \
      watchdog_periodic(); \
    } \
  } while(0)


unsigned char MQcalib = 0, CalibFlag=0;
float MQvalCalib=0;
float RL_VALUE=5.0; 
float RO_CLEAN_AIR_FACTOR=9.83;
int CALIBARAION_SAMPLE_TIMES=25;
int CALIBRATION_SAMPLE_INTERVAL=500;
int READ_SAMPLE_TIMES=25;
int READ_SAMPLE_INTERVAL= (5) ;
/**********************Application Related Macros**********************************/
#define         GAS_LPG             0   
#define         GAS_CO              1   
#define         GAS_SMOKE           2    

/*****************************Globals***********************************************/
float           LPGCurve[3]  =  {2.3,0.21,-0.47};
float           COCurve[3]  =  {2.3,0.72,-0.34};
float           SmokeCurve[3] ={2.3,0.53,-0.44};                                                
float           Ro           =  10;  

/*---------------------------------------------------------------------------*/
int adc_raw_value();
float MQResistanceCalculation(int raw_adc);
void MQCalibration();
float MQRead();
float MQGetGasPercentage(float rs_ro_ratio, int gas_id);
float MQGetPercentage(float rs_ro_ratio, float *pcurve);
void max44009_init();
uint32_t read_light_max44009();

/*---------------------------------------------------------------------------*/
void max44009_init() {
  i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN,
           I2C_SCL_NORMAL_BUS_SPEED);
  uint8_t max_data[2] = {0x02, 0x40};
  i2c_burst_send(MAX4409_ADDRESS, max_data, 2);
}

uint32_t read_light_max44009() {
	uint8_t exponent, mantissa;
	uint8_t max44009_data[2];
	uint32_t result;	

	i2c_single_send(MAX4409_ADDRESS, 0x03);
	i2c_single_receive(MAX4409_ADDRESS, &max44009_data[0]);
	i2c_single_send(MAX4409_ADDRESS, 0x04);
	i2c_single_receive(MAX4409_ADDRESS, &max44009_data[1]);

	exponent = (max44009_data[0] >> 4);
	mantissa = ((max44009_data[0] & 0x0F) << 4) | (max44009_data[1] & 0x0F);
	result = pow(2, exponent) * mantissa * 4.5;
	return result;	
}
/*---------------------------------------------------------------------------*/
int adc_raw_value(){
  return (adc_get(SOC_ADC_ADCCON_CH_AIN2,SOC_ADC_ADCCON_REF_AVDD5,SOC_ADC_ADCCON_DIV_512) >> 5); 
}

float MQResistanceCalculation(int raw_adc)
{
  return (((float)5.0*(2047-raw_adc)/raw_adc));
}

void MQCalibration()
{
  
  if(MQcalib < CALIBARAION_SAMPLE_TIMES) {
      MQvalCalib += MQResistanceCalculation(adc_raw_value());
      
      ctimer_restart(&ct);
  } 
  if(MQcalib ==CALIBARAION_SAMPLE_TIMES ){
    MQvalCalib = MQvalCalib/CALIBARAION_SAMPLE_TIMES;            
    MQvalCalib = MQvalCalib/RO_CLEAN_AIR_FACTOR;
    CalibFlag = 1;        
  }                          
  MQcalib++;
}

float MQRead()
{
  int i;
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(adc_raw_value());
    BUSYWAIT_UNTIL(READ_SAMPLE_INTERVAL);
  }
  rs = rs/READ_SAMPLE_TIMES;
  return rs;  
}
 

 float MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } else if ( gas_id == GAS_CO ) {
     return MQGetPercentage(rs_ro_ratio,COCurve);
  } else if ( gas_id == GAS_SMOKE ) {
     return MQGetPercentage(rs_ro_ratio,SmokeCurve);
  }    
  return 0;
}


float  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,( ((log10(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}



/*----------------------------------MQTT-SN-----------------------------------------*/
static uint16_t udp_port = 1884;
static uint16_t keep_alive = 60;
static uint16_t broker_address[] = {0xfd00, 0, 0, 0, 0, 0, 0, 0x1};
static char     device_id[17];
static char     *topics_mqtt[] = {"sensor/mq2",
                                  "sensor/bmp180",
                                  "sensor/si7021",
				  "sensor/max44009",
                                        };
/*---------------------------------------------------------------------------*/
mqtt_sn_con_t mqtt_sn_connection;

void mqtt_sn_callback(char *topic, char *message){
//  printf("\nMessage received:");
//  printf("\nTopic:%s Message:%s",topic,message);
	return;
}

void init_broker(void){
  char *all_topics[ss(topics_mqtt)+1];
  sprintf(device_id,"%02X%02X%02X%02X%02X%02X%02X%02X",
          linkaddr_node_addr.u8[0],linkaddr_node_addr.u8[1],
          linkaddr_node_addr.u8[2],linkaddr_node_addr.u8[3],
          linkaddr_node_addr.u8[4],linkaddr_node_addr.u8[5],
          linkaddr_node_addr.u8[6],linkaddr_node_addr.u8[7]);

  mqtt_sn_connection.client_id     = device_id;
  mqtt_sn_connection.udp_port      = udp_port;
  mqtt_sn_connection.ipv6_broker   = broker_address;
  mqtt_sn_connection.keep_alive    = keep_alive;
//  mqtt_sn_connection.will_topic    = 0x00;
//  mqtt_sn_connection.will_message  = 0x00;
  mqtt_sn_init();

  size_t i;
  for(i=0;i<ss(topics_mqtt);i++)
    all_topics[i] = topics_mqtt[i];

  mqtt_sn_create_sck(mqtt_sn_connection,
                     all_topics,
                     ss(all_topics),
                     mqtt_sn_callback);
}

/*---------------------------------------------------------------------------*/

PROCESS(MQtest, "MQtest test");
PROCESS(initMQ, "initMQ");
AUTOSTART_PROCESSES(&MQtest, &initMQ);

PROCESS_THREAD(MQtest, ev, data)
{
  PROCESS_BEGIN();
  char data_pub[64];
  static uint16_t pressure;
  static uint16_t temperature1;
  static uint16_t temperature2;
  static uint16_t humdity;
  static uint32_t light;
  adc_init();
  //lpm_set_max_pm(0);
  etimer_set(&timer_adc, INTERVAL_READING);
  SENSORS_ACTIVATE(bmpx8x);

  while(1){    
    PROCESS_WAIT_EVENT_UNTIL(ev ==PROCESS_EVENT_TIMER);  
	// Pubblish MQ2
	if(CalibFlag == 1){
	Ro = MQvalCalib;  
	float iPPM_LPG = 0;
	float iPPM_CO = 0;
	float iPPM_Smoke = 0;
	iPPM_LPG = MQGetGasPercentage((float)(MQRead()/Ro),GAS_LPG);
	iPPM_CO = MQGetGasPercentage((float)(MQRead()/Ro),GAS_CO);
	iPPM_Smoke = MQGetGasPercentage((float)(MQRead()/Ro),GAS_SMOKE);
	memset(data_pub,0,sizeof(data_pub)); // clear string
	sprintf(data_pub,"{\"LPG\":\"%d\", \"CO\":\"%d\", \"Smoke\":\"%d\"}",(int)(iPPM_LPG),(int)(iPPM_CO),(int)(iPPM_Smoke) );
	// printf("%s\t",data_pub);
	mqtt_sn_pub("sensor/mq2", data_pub, false, 0);
	}
	// Publish MAX44009
	light = read_light_max44009();
	memset(data_pub, 0, sizeof(data_pub));
	sprintf(data_pub, "{\"Light\": \"%lu.%lu\"}", light / 100, light % 100);
	// printf("%s\t", data_pub);
	mqtt_sn_pub("sensor/max44009" , data_pub, false, 0);
	// Publish BMP180
	pressure = bmpx8x.value(BMPx8x_READ_PRESSURE);
	temperature1 = bmpx8x.value(BMPx8x_READ_TEMP);
	memset(data_pub, 0, sizeof(data_pub));
	sprintf(data_pub, "{\"Press\": \"%u.%u\", \"Temp\": \"%d.%u\"}", pressure / 10, pressure % 10, temperature1 / 10, temperature1 % 10);
	// printf("%s\t", data_pub);
	mqtt_sn_pub("sensor/bmp180" , data_pub, false, 0);

	// Publish SI7021
	temperature2 = si7021_readTemp(TEMP_NOHOLD);
	humdity = si7021_readHumd(RH_NOHOLD);
	memset(data_pub, 0, sizeof(data_pub));
	float tf = (float) temperature2 * 1.00;
	float hf = (float) humdity * 1.00;
	float t = (float) (tf * 175.75) / 65536.0 - 46.85;
	float h = (float) (hf * 125) / 65536.0 - 6.0;
	sprintf(data_pub, "{\"Temp\": \"%ld.%2u\", \"Hum\": \"%ld.%2u\"}", (long)t, (unsigned)((t-floor(t))*100), (long)h, (unsigned)((h-floor(h))*100));
	mqtt_sn_pub("sensor/si7021" , data_pub, false, 0);
	// printf("%s\t\n", data_pub);

    etimer_reset(&timer_adc);
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(initMQ, ev, data)
{
    PROCESS_BEGIN();

    init_broker();
    max44009_init();
    si7021_config();

    SENSORS_ACTIVATE(bmpx8x);
    ctimer_set(&ct, (CLOCK_SECOND) * 0.5, MQCalibration,0);

    while(1){
    PROCESS_YIELD();
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
