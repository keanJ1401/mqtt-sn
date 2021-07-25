/**
  Licensed to the Apache Software Foundation (ASF) under one
  or more contributor license agreements.  See the NOTICE file
  distributed with this work for additional information
  regarding copyright ownership.  The ASF licenses this file
  to you under the Apache License, Version 2.0 (the
  "License"); you may not use this file except in compliance
  with the License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing,
  software distributed under the License is distributed on an
  "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
  KIND, either express or implied.  See the License for the
  specific language governing permissions and limitations
  under the License.

 *******************************************************************************
 * @file main_core.c
 * @author Ânderson Ignacio da Silva
 * @date 19 Ago 2016
 * @brief Main code to test MQTT-SN on Contiki-OS
 * @see http://www.aignacio.com
 * @license This project is licensed by APACHE 2.0.
 */

#include "contiki.h"
#include "lib/random.h"
#include "clock.h"
#include "sys/ctimer.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "mqtt_sn.h"
#include "dev/leds.h"
#include "net/rime/rime.h"
#include "net/ip/uip.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "lpm.h"
#include "net/netstack.h"
#include "cc2538-rf.h"
#include <math.h>
#include "dev/i2c.h"
#include "sys/rtimer.h"

#define PERIOD_T 30*RTIMER_SECOND
#define PERIOD_T2 45*RTIMER_SECOND

static uint16_t udp_port = 1884;
static uint16_t keep_alive = 5;
static uint16_t broker_address[] = {0xfd00, 0, 0, 0, 0, 0, 0, 0x1};
static struct   etimer time_poll;
// static uint16_t tick_process = 0;
static char     device_id[17];
static char     topic_hw[25];
static char     *topics_mqtt[] = {"sensor/max44009",
  };
// static char     *will_topic = "/6lowpan_node/offline";
// static char     *will_message = "O dispositivo esta offline";
// This topics will run so much faster than others
#define MAX4409_ADDRESS 0x4A
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
  //mqtt_sn_connection.will_topic    = will_topic;   // Configure as 0x00 if you don't want to use
  //mqtt_sn_connection.will_message  = will_message; // Configure as 0x00 if you don't want to use
  mqtt_sn_connection.will_topic    = 0x00;
  mqtt_sn_connection.will_message  = 0x00;

  mqtt_sn_init();   // Inicializa alocação de eventos e a principal PROCESS_THREAD do MQTT-SN

  size_t i;
  for(i=0;i<ss(topics_mqtt);i++)
    all_topics[i] = topics_mqtt[i];
  all_topics[i] = topic_hw;

  mqtt_sn_create_sck(mqtt_sn_connection,
                     all_topics,
                     ss(all_topics),
                     mqtt_sn_callback);
}
void setup_before_resume(void) {
  i2c_master_enable();
}


void cleanup_before_sleep(void) {
        i2c_master_disable();
	return;
}

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
PROCESS(init_system_process, "[Contiki-OS] Initializing OS");
AUTOSTART_PROCESSES(&init_system_process);
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(init_system_process, ev, data) {
  PROCESS_BEGIN();
  char data_pub[64];
  etimer_set(&time_poll, 5*CLOCK_SECOND);
  init_broker();
  max44009_init();
 /* NETSTACK_RDC.off(0);
  NETSTACK_MAC.off(0);
  cc2538_rf_driver.off();
  cleanup_before_sleep();
*/
  while(1) {
      PROCESS_YIELD();
      setup_before_resume();
  //mqtt_sn_ping_send();
      uint32_t light;
      light = read_light_max44009();
      memset(data_pub, 0, sizeof(data_pub));
      sprintf(data_pub, "{\"Light\": \"%lu.%lu\"}", light / 100, light % 100);
	// printf("%s\t", data_pub);
      mqtt_sn_pub("sensor/max44009" , data_pub, false, 0);
      printf("%s", data_pub);
     // mqtt_sn_disconnect(10);
     // NETSTACK_RDC.off(0);
     // NETSTACK_MAC.off(0);
     // cc2538_rf_driver.off();
      cleanup_before_sleep();
    //  lpm_set_max_pm(1);

      if (etimer_expired(&time_poll))
        etimer_reset(&time_poll);
  }
  PROCESS_END();
}
