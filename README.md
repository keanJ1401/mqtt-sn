# mqtt-sn on WSN (6LoWPAN)



SDA ---- PA6 |   BMP180 > | I2C
SCL ---- PA5 |   SI7021 > | I2C
ADC ---- PA2 |   MQ2    > | ADC
```
# My contiki, put folder on example folder, or you can change path on Makefile
git clone https://github.com/keanJ1401/contiki.git
cd contiki/example
git clone https://github.com/keanJ1401/mqtt-sn.git
make TARGET = cc2538dk
```
