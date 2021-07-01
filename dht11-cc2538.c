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
#include "net/ip/uip-debug.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
// #include "powertrace.h"


/*-------------------------DHT22 library-------------------------------------*/
#include "dev/dht22.h"

static uint16_t udp_port = 1884;
static uint16_t keep_alive = 2;
static uint16_t broker_address[] = {0xfd00, 0, 0, 0, 0, 0, 0, 0x1};
static char     device_id[17];
static char     clientid[32];
static char     topic_hw[25];
static char     *topics_mqtt[] = {
                                "home/livingroom/dht11",
                            		};
static char     device[16] = "cc2538";   
static struct etimer et;


mqtt_sn_con_t mqtt_sn_connection;

void mqtt_sn_callback(char *topic, char *message){
  printf("\nMessage received:");
  printf("\nTopic:%s Message:%s",topic,message);
}

void init_broker(void){
  char *all_topics[ss(topics_mqtt)+1];

  sprintf(device_id,"%02x%02x%02x%02x%02x%02x%02x%02x",
          linkaddr_node_addr.u8[0],linkaddr_node_addr.u8[1],
          linkaddr_node_addr.u8[2],linkaddr_node_addr.u8[3],
          linkaddr_node_addr.u8[4],linkaddr_node_addr.u8[5],
          linkaddr_node_addr.u8[6],linkaddr_node_addr.u8[7]);
  sprintf(topic_hw,device_id);

  sprintf(clientid,"%s_%02x%02x%02x%02x", device, 
          linkaddr_node_addr.u8[4],linkaddr_node_addr.u8[5],
          linkaddr_node_addr.u8[6],linkaddr_node_addr.u8[7]);

  mqtt_sn_connection.client_id     = clientid;
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
  mqtt_sn_sub(topic_hw,0);
}
/*---------------------------------------------------------------------------*/


PROCESS(init_system_process, "[Contiki-OS] Initializing OS");

AUTOSTART_PROCESSES(&init_system_process);
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(init_system_process, ev, data) {
  int16_t temperature, humidity;
  char     dht11_temp[5];
  char     dht11_hum[5];
  PROCESS_BEGIN();
  //lpm_set_max_pm(0)
  SENSORS_ACTIVATE(dht22);
  debug_os("Initializing the MQTT_SN_DEMO");

  init_broker();

  etimer_set(&et, CLOCK_SECOND);

  while(1) {
      PROCESS_WAIT_EVENT();

      // Printf to serial
      if(dht22_read_all(&temperature, &humidity) != DHT22_ERROR) {
      printf("Temperature:%02d ºC, ", temperature);
      printf("Humidity:%02d RH\n", humidity);

      // Wrap and Publish
      memset(dht11_temp, 0, sizeof(dht11_temp));
      sprintf(dht11_temp, ": %02d}", temperature);
      memset(dht11_hum, 0, sizeof(dht11_hum));
      sprintf(dht11_hum, ": %02d}", humidity);

      mqtt_sn_pub("{\"dht11_temperature\"}", dht11_temp, true, 0);
      mqtt_sn_pub("{\"dht11_humidity\"}", dht11_hum, true, 0);
      leds_off(LEDS_GREEN);                           
/*
      sprintf(pub_test,"%s",topic_hw);
      mqtt_sn_pub("/topic_1",pub_test,true,0);
      */
      // debug_os("State MQTT:%s",mqtt_sn_check_status_string());

}
else {
      
       leds_toggle(LEDS_GREEN);
    }
          if (etimer_expired(&et))
        etimer_reset(&et);
  }
  PROCESS_END();
}
