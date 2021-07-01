# WSN (6LoWPAN) using MQTT-SN


Topology
```
|-----------|                 |----------------|            |-----------------------|               |-------------|
|  6LOWPAN  | ----PUBLISH---- | MOSQUITTO-RSMB | ++BRIDGE++ | BROKER (EMQ X BROKER) |---SUBSCRIBE---|  END-USERS  |
|-----------|                 |----------------|            |-----------------------|               |-------------|
```

CC2538DK hardware (can resetup port on contiki/platform/cc2538dk/dev/board.h)
```
# SDA ---- PA6 |   BMP180 > | I2C communication
# SCL ---- PA5 |   SI7021 > | I2C communication
# ---------------------------------------------
# ADC ---- PA2 |   MQ2    > | ADC communication
```
```
# This is my contiki (I had changed libaries on it), put this repo on example folder of contiki,
# or you can change the contiki path on Makefile
git clone https://github.com/keanJ1401/contiki.git
cd contiki/example
git clone https://github.com/keanJ1401/mqtt-sn.git
make TARGET = cc2538dk
```
mqtt-sn-tool
```
cd mqtt-sn/tools/mqtt-sn-tools
make clean && make
sudo make install
mqtt-sn-sub -p 1884 -h fd00::1 -t "#" -v
```

MQTT-SN Gateway
```
cd mqtt-sn/tools/mosquitto.rsmb/rsmb/src
make clean && make
./broker_mqtts config.mqtt
```

Border Router
```
# CC2538 label port: /dev/ttyUSB*
# CC2650 label port: /dev/ttyACM* 
cd contiki/tools
make tunslip6
sudo ./tunslip6 -B 115200 -s /dev/ttyUSB0 fd00::1/64
```

