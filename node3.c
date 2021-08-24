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
#include "ti-lib.h"
#include "board-peripherals.h"
/*---------------------------------------------------------------------------*/
#define RSSI_INTERVAL            (60 * CLOCK_SECOND)
#include "sys/etimer.h"
static struct etimer et;
/*---------------------------------------------------------------------------*/
int sicslowpan_get_last_rssi();
static uint16_t udp_port = 1884;
static uint16_t keep_alive = 60;
static uint16_t broker_address[] = {0xfd00, 0, 0, 0, 0, 0, 0, 0x1};
static char     device_id[17];
static char     topic_hw[25];
static char     *topics_mqtt[] = {
                                  "actuator/light1",
                                  "actuator/light2",
				  "actuator/fan",
                                  "sensor/node3/rssi",
                                  };

mqtt_sn_con_t mqtt_sn_connection;

/*---------------------------------------------------------------------------*/
void mqtt_sn_callback(char *topic, char *message)
{
  printf("\nTopic:%s Message:%s", topic, message);
    if(strcmp(topic, topics_mqtt[0]) == 0)
    {
      if(strcmp(message, "1") == 0) 
	{
	GPIO_writeDio(BOARD_IOID_DIO0, 1);
	printf("Led ON");
	} 
      else if(strcmp(message, "0") == 0)
	{
	GPIO_writeDio(BOARD_IOID_DIO0, 0);
        }
 
    }
    else if(strcmp(topic, topics_mqtt[1]) == 0)
    {
      if(strcmp(message, "1") == 0) 
	{
	GPIO_writeDio(BOARD_IOID_DIO1 , 1);
      	} 
      else if(strcmp(message, "0") == 0) 
	{
	GPIO_writeDio(BOARD_IOID_DIO1 , 0);
	}
    }
    else if(strcmp(topic, topics_mqtt[2]) == 0)
    {
      if(strcmp(message, "1") == 0) 
	{
	GPIO_writeDio(BOARD_IOID_DIO30, 1);
      	} 
      else if(strcmp(message, "0") == 0) 
	{
	GPIO_writeDio(BOARD_IOID_DIO30, 0);
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
  mqtt_sn_sub(all_topics[2], 0); 
  // mqtt_sn_sub(*all_topics, 0); 


}
/*---------------------------------------------------------------------------*/
PROCESS(init_system_process, "[Contiki-OS] Initializing OS");
AUTOSTART_PROCESSES(&init_system_process);

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(init_system_process, ev, data) {
  GPIO_setOutputEnableDio(BOARD_IOID_DIO0, GPIO_OUTPUT_ENABLE);
  GPIO_setOutputEnableDio(BOARD_IOID_DIO1, GPIO_OUTPUT_ENABLE);
  GPIO_setOutputEnableDio(BOARD_IOID_DIO30, GPIO_OUTPUT_ENABLE);
  PROCESS_BEGIN();
  char data_pub[64];
  init_broker();
  etimer_set(&et, RSSI_INTERVAL);
  while(1) {
      PROCESS_WAIT_EVENT();
      memset(data_pub, 0, sizeof(data_pub));
      sprintf(data_pub, "{\"RSSI\": \"%d\"}", sicslowpan_get_last_rssi());
      mqtt_sn_pub(topics_mqtt[3], data_pub, false, 0);
      printf("%d", sicslowpan_get_last_rssi());

  }
  PROCESS_END();
}
