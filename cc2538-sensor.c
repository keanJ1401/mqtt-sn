/*---------------------------------------------------------------------------*/
/**
 * \author
 *         Nguyen Huu Hoang <nhhoang.dev@gmail.com>
 */
/*---------------------------------------------------------------------------*/
#include <stdio.h>
#include "contiki.h"
#include "dev/i2c.h"
#include "dev/leds.h"
#include "dev/tsl256x.h"
#include "dev/bmpx8x.h"
#include "dev/si7021.h"
#include "dev/gpio.h"
/*---------------------------------------------------------------------------*/
/* Default sensor's integration cycle is 402ms */
#define SENSOR_READ_INTERVAL (3*CLOCK_SECOND)
#define SI7021_DBG 1
/*---------------------------------------------------------------------------*/
PROCESS(demo, "TSL256X BMP085/BMP180 ");

AUTOSTART_PROCESSES(&demo);
/*---------------------------------------------------------------------------*/
static struct etimer et;
/*---------------------------------------------------------------------------*/
void
light_interrupt_callback(uint8_t value)
{
  printf("* Light sensor interrupt!\n");
  leds_toggle(LEDS_RED);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(demo, ev, data)
{
  PROCESS_BEGIN();
    static uint16_t light;
    static uint16_t pressure;
    static int16_t temperature;
    static uint16_t blinkLed;
	GPIO_SET_OUTPUT(GPIO_B_BASE, (0x01 | 0x01<<1 | 0x01<<2 | 0x01<<3 | 0x01<<4 | 0x01<<5));
	GPIO_CLR_PIN(GPIO_B_BASE, (0x01 | 0x01<<1 | 0x01<<2 | 0x01<<3 | 0x01<<4 | 0x01<<5));
	SENSORS_ACTIVATE(bmpx8x);
  /* Print the sensor used, teh default is the TSL2561 (from Grove) */
  if(TSL256X_REF == TSL2561_SENSOR_REF) {
    printf("Light sensor test --> TSL2561\n");
  } else if(TSL256X_REF == TSL2563_SENSOR_REF) {
    printf("Light sensor test --> TSL2563\n");
  } else {
    printf("Unknown light sensor reference, aborting\n");
    // PROCESS_EXIT();
  }

  /* Use Contiki's sensor macro to enable the sensor */
  SENSORS_ACTIVATE(tsl256x);

  /* Default integration time is 402ms with 1x gain, use the below call to
   * change the gain and timming, see tsl2563.h for more options
   */
  /* tsl256x.configure(TSL256X_TIMMING_CFG, TSL256X_G16X_402MS); */

  /* Register the interrupt handler */
  TSL256X_REGISTER_INT(light_interrupt_callback);

  /* Enable the interrupt source for values over the threshold.  The sensor
   * compares against the value of CH0, one way to find out the required
   * threshold for a given lux quantity is to enable the DEBUG flag and see
   * the CH0 value for a given measurement.  The other is to reverse the
   * calculations done in the calculate_lux() function.  The below value roughly
   * represents a 2500 lux threshold, same as pointing a flashlight directly
   */
  tsl256x.configure(TSL256X_INT_OVER, 0x15B8);

  /* And periodically poll the sensor */

  while(1) {
    etimer_set(&et, SENSOR_READ_INTERVAL);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
		blinkLed++;
		pressure = bmpx8x.value(BMPx8x_READ_PRESSURE);
    temperature = bmpx8x.value(BMPx8x_READ_TEMP);
    light = tsl256x.value(TSL256X_VAL_READ);

		if(blinkLed % 2 == 0){
			//GPIO_CLR_PIN(GPIO_B_BASE, (0x01 | 0x01<<1 | 0x01<<2 | 0x01<<3 | 0x01<<4 | 0x01<<5));
		}else{
			//GPIO_SET_PIN(GPIO_B_BASE, (0x01 | 0x01<<1 | 0x01<<2 | 0x01<<3 | 0x01<<4 | 0x01<<5));
		}
    if(light != TSL256X_ERROR) {
      printf("TSL2561 : Light = %u\n", (uint16_t)light);
			if(light < 5){
					GPIO_SET_PIN(GPIO_B_BASE, (0x01 | 0x01<<1 | 0x01<<2 ));
			}
			else{
					GPIO_CLR_PIN(GPIO_B_BASE, (0x01 | 0x01<<1 | 0x01<<2 ));

			}

    } else {
      printf("Error, enable the DEBUG flag in the tsl256x driver for info, ");
      printf("or check if the sensor is properly connected\n");
    // PROCESS_EXIT();
    }
		
		if((pressure != BMPx8x_ERROR) && (temperature != BMPx8x_ERROR)) {
      printf("BMPx8x : Pressure = %u.%u(hPa), ", pressure / 10, pressure % 10);
      printf("Temperature = %d.%u(ºC)\n", temperature / 10, temperature % 10);
    } else {
      printf("Error, enable the DEBUG flag in the BMPx8x driver for info, ");
      printf("or check if the sensor is properly connected\n");
     // PROCESS_EXIT();
    }
		si7021_readTemp(TEMP_NOHOLD);
		si7021_readHumd(RH_NOHOLD);
			printf("-----------------------------------------------\n");
  }
  PROCESS_END();
}




/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
