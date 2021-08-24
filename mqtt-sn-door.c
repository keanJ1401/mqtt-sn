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
/*---------------------------------------------------------------------------*/
#include "dev/servo.h"
#include "dev/motion-sensor.h"
#define LEDS_OFF_HYSTERISIS      RTIMER_SECOND
#define DOOR_TIMEOUT             (10 * CLOCK_SECOND)
#define RSSI_INTERVAL            (60 * CLOCK_SECOND)
#define SENSOR_READ_INTERVAL     (10 * CLOCK_SECOND)
#include "sys/rtimer.h"
#include "sys/ctimer.h"
#include "sys/etimer.h"
static struct rtimer rt;
static struct ctimer ct;
static struct etimer et;
static uint8_t deg = 1;
/*---------------------------------------------------------------------------*/
int sicslowpan_get_last_rssi();
static uint16_t udp_port = 1884;
static uint16_t keep_alive = 60;
static uint16_t broker_address[] = {0xfd00, 0, 0, 0, 0, 0, 0, 0x1};
static char     device_id[17];
static char     topic_hw[25];
static char     *topics_mqtt[] = {
                                  "actuator/door/control",
                                  "actuator/door/mode",
                                  "sensor/node2/rssi",
                                  };

mqtt_sn_con_t mqtt_sn_connection;
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
void
ct_callback(void *ptr) 
{
  
  ctimer_reset(&ct);
  if (deg != 0) 
  {
    deg = 0;
    servo_position(SERVO_CHANNEL_5, GPIO_A_NUM, 5, deg);
    // printf("Door closing\n");
    // SENSORS_DEACTIVATE(motion_sensor);
  }
}
void
rt_callback(struct rtimer *t, void *ptr)
{
  leds_off(LEDS_RED);

  ctimer_set(&ct, DOOR_TIMEOUT, ct_callback, NULL);
}

/*---------------------------------------------------------------------------*/
static void
presence_callback(uint8_t value)
{
  // printf("*** Presence detected!\n");
  leds_on(LEDS_RED);

  if (deg != 90) 
  {
    deg = 90;
    servo_position(SERVO_CHANNEL_5, GPIO_A_NUM, 5, deg);
    printf("%d", value);
    // printf("Door opening\n");
  }
  rtimer_set(&rt, RTIMER_NOW() + LEDS_OFF_HYSTERISIS, 5, rt_callback, NULL); 
}
/*---------------------------------------------------------------------------*/

void mqtt_sn_callback(char *topic, char *message)
{
  printf("\nTopic:%s Message:%s", topic, message);
    if(strcmp(topic, topics_mqtt[0]) == 0)
    {
      if(strcmp(message, "1") == 0) {
        // printf("DOOR OPEN\n");
        if (deg != 90) 
          {
            deg = 90;
            servo_position(SERVO_CHANNEL_5, GPIO_A_NUM, 5, deg);
          }
      } 
      else if(strcmp(message, "0") == 0) {
        // printf("DOOR CLOSE\n");
        if (deg != 1) 
        {
          deg = 1;
          servo_position(SERVO_CHANNEL_5, GPIO_A_NUM, 5, deg);
        }
      }
    }
    else if(strcmp(topic, topics_mqtt[1]) == 0)
    {
      if(strcmp(message, "1") == 0) {
        printf("DOOR: AUTO_MODE\n");
        if (deg != 1) 
        {
          deg = 1;
          servo_position(SERVO_CHANNEL_5, GPIO_A_NUM, 5, deg);
        }
        // MOTION_REGISTER_INT(presence_callback);
        SENSORS_ACTIVATE(motion_sensor);
      } 
      else if(strcmp(message, "0") == 0) {
        printf("DOOR: MANUAL MODE\n");
      }
    }
}

void init_broker(void){
  char *all_topics[ss(topics_mqtt)+1];
  sprintf(device_id,"%02X%02X%02X%02X%02X%02X%02X%02X", linkaddr_node_addr.u8[0],linkaddr_node_addr.u8[1], linkaddr_node_addr.u8[2],linkaddr_node_addr.u8[3], linkaddr_node_addr.u8[4],linkaddr_node_addr.u8[5], linkaddr_node_addr.u8[6],linkaddr_node_addr.u8[7]);
  sprintf(topic_hw,"Hello addr:%02X%02X",linkaddr_node_addr.u8[6],linkaddr_node_addr.u8[7]);
  mqtt_sn_connection.client_id     = device_id;
  mqtt_sn_connection.udp_port      = udp_port;
  mqtt_sn_connection.ipv6_broker   = broker_address;
  mqtt_sn_connection.keep_alive    = keep_alive;
  mqtt_sn_connection.will_topic    = 0x00;
  mqtt_sn_connection.will_message  = 0x00;

  mqtt_sn_init();

  size_t i;
  for(i=0;i<ss(topics_mqtt);i++) {
    all_topics[i] = topics_mqtt[i];
  }
  mqtt_sn_create_sck(mqtt_sn_connection, all_topics, ss(all_topics), mqtt_sn_callback); 
  mqtt_sn_sub(all_topics[1], 0);
  mqtt_sn_sub(all_topics[0], 0); 
  // mqtt_sn_sub(*all_topics, 0); 


}
/*---------------------------------------------------------------------------*/
PROCESS(init_system_process, "[Contiki-OS] Initializing OS");
AUTOSTART_PROCESSES(&init_system_process);

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(init_system_process, ev, data) {
  
    PROCESS_BEGIN();
  char data_pub[64];
  init_broker();
  // lpm_set_max_pm(0);
  servo_position(SERVO_CHANNEL_5, GPIO_A_NUM, 5, deg);
  // SENSORS_ACTIVATE(motion_sensor);
  MOTION_REGISTER_INT(presence_callback);
  etimer_set(&et, RSSI_INTERVAL);
  while(1) {
      PROCESS_WAIT_EVENT();
      memset(data_pub, 0, sizeof(data_pub));
      sprintf(data_pub, "{\"RSSI\": \"%d\"}", sicslowpan_get_last_rssi());
      mqtt_sn_pub(topics_mqtt[3], data_pub, false, 0);
      // SENSORS_DEACTIVATE(motion_sensor);

  }
  PROCESS_END();
}
