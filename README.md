## Linux device driver for MHZ19B CO2 sensor

This driver uses UART serdev bus interface, IIO subsystem.

Tested on Raspberry Pi 3A+ with kernel 6.6.78.

Datasheet: https://www.winsen-sensor.com/d/files/infrared-gas-sensor/mh-z19b-co2-ver1_0.pdf

## Usage:
```bash
$ cat /sys/bus/iio/devices/iio\:device0/in_concentration_co2_raw 
1139
$ cat /sys/bus/iio/devices/iio\:device0/in_concentration_co2_raw 
1137
$ cat /sys/bus/iio/devices/iio\:device0/in_concentration_co2_raw
1137
...

```
