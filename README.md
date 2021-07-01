# mqtt-sn on WSN (6LoWPAN)


CC2538DK hardware
- SDA ---- PA6 |   BMP180 > | I2C <br>
- SCL ---- PA5 |   SI7021 > | I2C <br>
- ADC ---- PA2 |   MQ2    > | ADC <br>
```
# This is my contiki (I change something on it), put this respo on example folder of contiki,
# or you can change the contiki path on Makefile
git clone https://github.com/keanJ1401/contiki.git
cd contiki/example
git clone https://github.com/keanJ1401/mqtt-sn.git
make TARGET = cc2538dk
```
