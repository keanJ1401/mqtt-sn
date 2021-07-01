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



#include "dev/adc.h"
#include "dev/gpio.h"
#include <stdio.h> /* For printf() */

#include "sys/etimer.h"
#include "sys/ctimer.h" // inoder to add call back timer

#include <math.h>
/*----------------------------------DHTtimer-----------------------------------------*/
static struct etimer timer_adc;
static struct ctimer ct;
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
  return ( ((float)5.0*(2047-raw_adc)/raw_adc));
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

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define UIP_UDP_BUF  ((struct uip_udp_hdr *)&uip_buf[uip_l2_l3_hdr_len])

#define MAX_PAYLOAD_LEN 120
/*---------------------------------------------------------------------------*/
static struct uip_udp_conn *server_conn;
static char buf[MAX_PAYLOAD_LEN];
static uint16_t len;
/*---------------------------------------------------------------------------*/
PROCESS(MQtest, "MQtest test");
PROCESS(initMQ, "initMQ");
/*---------------------------------------------------------------------------*//*---------------------------------------------------------------------------*/
AUTOSTART_PROCESSES(&MQtest,&initMQ);
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/

PROCESS_THREAD(MQtest, ev, data)
{
  PROCESS_BEGIN();
  adc_init();
  lpm_set_max_pm(0);
  etimer_set(&timer_adc,10*(CLOCK_SECOND));
  
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
      printf("Ro= %d kohm, ADC: %d\n",(int)Ro, adc_raw_value());
      iPPM_LPG = MQGetGasPercentage( (float)(MQRead()/Ro),GAS_LPG);
      iPPM_CO = MQGetGasPercentage(  (float)(MQRead()/Ro),GAS_CO);
      iPPM_Smoke = MQGetGasPercentage( (float)(MQRead()/Ro),GAS_SMOKE);

      memset(MQ2_data,0,sizeof(MQ2_data)); // clear string
      sprintf(MQ2_data,"{\"LPG\":\"%d\", \"CO\":\"%d\", \"Smoke\":\"%d\"}",(int)(iPPM_LPG),(int)(iPPM_CO),(int)(iPPM_Smoke) );
     // memcpy(MQ2_data, "Temp:", sizeof("off"));
      printf("%s",MQ2_data);
      printf("Length of string b = %d \n",strlen(MQ2_data));
      s
    }
    
    etimer_reset(&timer_adc);
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(initMQ, ev, data)
{
  PROCESS_BEGIN();
 // adc_init();

  // leds_on(LEDS_RED);
  ctimer_set(&ct, (CLOCK_SECOND) * 0.5, MQCalibration,0);
  //ctimer_set(&ct, (CLOCK_SECOND>>3) * 2, ctimer_callback_example);
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
