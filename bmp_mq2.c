/*
 * Copyright (c) 2012, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** \addtogroup cc2538-examples
 * @{
 *
 * \defgroup cc2538-echo-server cc2538dk UDP Echo Server Project
 *
 *  Tests that a node can correctly join an RPL network and also tests UDP
 *  functionality
 * @{
 *
 * \file
 *  An example of a simple UDP echo server for the cc2538dk platform
 */
#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include <string.h>
#define DEBUG DEBUG_PRINT
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
#include <stdlib.h>
#include "dev/adc.h"
#include "dev/gpio.h"
#include <stdio.h> /* For printf() */
#include "sys/etimer.h"
#include "sys/ctimer.h" // inoder to add call back timer
#include "dev/i2c.h"
#include "dev/bmpx8x.h"
#include <math.h>
/*----------------------------------Timer-----------------------------------------*/
static struct etimer timer_adc;
static struct ctimer ct;
/*----------------------------------MQTT-SN-----------------------------------------*/
static uint16_t udp_port = 1884;
static uint16_t keep_alive = 5;
static uint16_t broker_address[] = {0xfd00, 0, 0, 0, 0, 0, 0, 0x1};
static char     device_id[17];
static char     topic_hw[25];
static char     *topics_mqtt[] = {"sensor/mq2",
                                  "sensor/bmp180"
                                        };
/*---------------------------------------------------------------------------*/
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
/************************Hardware Related Macros************************************/
float RL_VALUE=5.0;                                     //define the load resistance on the board, in kilo ohms
float RO_CLEAN_AIR_FACTOR=9.83;                     //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                    //which is derived from the chart in datasheet
/***********************Software Related Macros************************************/
int CALIBARAION_SAMPLE_TIMES=50;                    //define how many samples you are going to take in the calibration phase
int CALIBRATION_SAMPLE_INTERVAL=500;                //define the time interal(in milisecond) between each samples in the
                                                    //cablibration phase
int READ_SAMPLE_TIMES=50;                            //define how many samples you are going to take in normal operation
int READ_SAMPLE_INTERVAL= (5) ;              //define the time interal(in 5 milisecond) between each samples in 
                                                    //normal operation
/**********************Application Related Macros**********************************/
#define         GAS_LPG             0   
#define         GAS_CO              1   
#define         GAS_SMOKE           2    

/*****************************Globals***********************************************/
float           LPGCurve[3]  =  {2.3,0.21,-0.47};   //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent"
                                                    //to the original curve. 
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59) 
float           COCurve[3]  =  {2.3,0.72,-0.34};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15) 
float           SmokeCurve[3] ={2.3,0.53,-0.44};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)                                                     
float           Ro           =  10;                 //Ro is initialized to 10 kilo ohms

/*---------------------------------------------------------------------------*/
int adc_raw_value();
float MQResistanceCalculation(int raw_adc);
void MQCalibration();
float MQRead();
float MQGetGasPercentage(float rs_ro_ratio, int gas_id);
float MQGetPercentage(float rs_ro_ratio, float *pcurve);
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
int adc_raw_value(){
  return (adc_get(SOC_ADC_ADCCON_CH_AIN2,SOC_ADC_ADCCON_REF_AVDD5,SOC_ADC_ADCCON_DIV_512) >>5); 
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
    MQvalCalib = MQvalCalib/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value
    MQvalCalib = MQvalCalib/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro                                        
    CalibFlag = 1;        
  }                          
  printf("MQvalCalib = %d \n",(int)MQvalCalib );                       
  printf("CalibFlag = %d \n",CalibFlag );
  printf("CALIBARAION_SAMPLE_TIMES = %d \n",MQcalib );
  MQcalib++;
}

float MQRead()
{
  int i;
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(adc_raw_value());
//    delay(READ_SAMPLE_INTERVAL);

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
  int a = rs_ro_ratio*100;
  return (pow(10,( ((log10(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}
PROCESS(MQtest, "MQtest test");
PROCESS(initMQ, "initMQ");
AUTOSTART_PROCESSES(&MQtest,&initMQ);

mqtt_sn_con_t mqtt_sn_connection;

void mqtt_sn_callback(char *topic, char *message){
  printf("\nMessage received:");
  printf("\nTopic:%s Message:%s",topic,message);
}

void init_broker(void){
  char *all_topics[ss(topics_mqtt)+1];
  sprintf(device_id,"%02X%02X%02X%02X%02X%02X%02X%02X",
          linkaddr_node_addr.u8[0],linkaddr_node_addr.u8[1],
          linkaddr_node_addr.u8[2],linkaddr_node_addr.u8[3],
          linkaddr_node_addr.u8[4],linkaddr_node_addr.u8[5],
          linkaddr_node_addr.u8[6],linkaddr_node_addr.u8[7]);
  sprintf(topic_hw,"Hello addr:%02X%02X",linkaddr_node_addr.u8[6],linkaddr_node_addr.u8[7]);

  mqtt_sn_connection.client_id     = device_id;
  mqtt_sn_connection.udp_port      = udp_port;
  mqtt_sn_connection.ipv6_broker   = broker_address;
  mqtt_sn_connection.keep_alive    = keep_alive;
  mqtt_sn_connection.will_topic    = 0x00;
  mqtt_sn_connection.will_message  = 0x00;

  mqtt_sn_init();

  size_t i;
  for(i=0;i<ss(topics_mqtt);i++)
    all_topics[i] = topics_mqtt[i];
  all_topics[i] = topic_hw;

  mqtt_sn_create_sck(mqtt_sn_connection,
                     all_topics,
                     ss(all_topics),
                     mqtt_sn_callback);
  mqtt_sn_sub(topic_hw,0);
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(MQtest, ev, data)
{
  PROCESS_BEGIN();
  char data_pub[64];
  static uint16_t pressure;
  static uint16_t temperature;
  adc_init();
  //lpm_set_max_pm(0);
  etimer_set(&timer_adc,10*(CLOCK_SECOND));
  SENSORS_ACTIVATE(bmpx8x);
  char MQ2_data[100];
  while(1){ 
    
   PROCESS_WAIT_EVENT_UNTIL(ev ==PROCESS_EVENT_TIMER);  
    
    if(CalibFlag == 1){
     // CalibFlag=0;
      Ro = MQvalCalib;  
      leds_off(LEDS_RED);
      leds_on(LEDS_BLUE);

       float iPPM_LPG = 0;
       float iPPM_CO = 0;
       float iPPM_Smoke = 0;
      printf("Ro= %d kohm\n",(int)Ro);
      iPPM_LPG = MQGetGasPercentage( (float)(MQRead()/Ro),GAS_LPG);
      iPPM_CO = MQGetGasPercentage(  (float)(MQRead()/Ro),GAS_CO);
      iPPM_Smoke = MQGetGasPercentage( (float)(MQRead()/Ro),GAS_SMOKE);
      printf("ADC: %d", adc_raw_value());
      memset(MQ2_data,0,sizeof(MQ2_data)); // clear string
      sprintf(MQ2_data,"{\"LPG\":\"%d\", \"CO\":\"%d\", \"Smoke\":\"%d\"}",(int)(iPPM_LPG),(int)(iPPM_CO),(int)(iPPM_Smoke) );
     // memcpy(MQ2_data, "Temp:", sizeof("off"));
      printf("%s",MQ2_data);
      mqtt_sn_pub("sensor/mq2", MQ2_data, false, 0);
    }

    if((pressure != BMPx8x_ERROR) && (temperature != BMPx8x_ERROR)) {
          memset(data_pub, 0, sizeof(data_pub));
          sprintf(data_pub, "{\"Press\": \"%u.%u\", \"Temp\": \"%d.%u\"}", pressure / 10, pressure % 10, temperature / 10, temperature % 10);
          printf("%s", data_pub);
          mqtt_sn_pub("sensor/bmp180" , data_pub, false, 0);
    } else {
          mqtt_sn_pub("sensor/bmp180", "Error to reading sensor", false, 0);
    }

    etimer_reset(&timer_adc);
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(initMQ, ev, data)
{
  PROCESS_BEGIN();
  init_broker();
// leds_on(LEDS_RED);
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
