# README #

### What is this about? ###

Cellabox - Open Source Air Quality Sensor Modules

This repository contains firmware for nRF52840 with a Thread network implementation (OpenThread) and a connection to the cloud platform of thethings.iO.
The firmware supports different sensors:

* Temperature and humidity (T&H) = Sensirion SHTC1 (1.8V)
* Temperature and humidity (T&H) = Sensirion SHT30 (2.7V)
* Barometric pressure = ST LPS22HB (1.8, 2.7V)
* Ozone (O3) = SPEC Sensors 110-406 + TI LMP91000 (analog-front-end)
* Nitrogen dioxide (NO2) = SPEC Sensors 110-507 + TI LMP91000 (analog-front-end)
* Sulfur dioxide (SO2) = SPEC Sensors 110-601 + TI LMP91000 (analog-front-end)
* Carbon monoxide (CO) = SPEC Sensors 110-102 + TI LMP91000 (analog-front-end)
* Particulate matters PM2.5/PM10 = Honeywell HPMA115S0-XXX (5V)

(c) 2017-2018 Cellabox, all rights reserved.

### License ###

Cellabox is released under the [BSD 3-Clause license]. See the [`LICENSE`](https://github.com/cellabox/cellabox/blob/master/LICENSE) file for more information.
Please use and reference to Cellabox in case you use this software or part(s) of it.

### Sensor Modules ###

The firmware is able to run in different sensor modules. The firmware initializes itself the correct way, depending on the state of the configuration pins:

* Config = 0: temperature and humidity, battery (3x1.5V), Vsupply=1.8V, T&H = Sensirion SHTC1
* Config = 1: indoor air quality, battery (3x1.5V), Vsupply=1.8V, VOC = Sensirion SGPC3, T&H = Sensirion SHTC1
* Config = 2: outdoor air quality, battery (3x1.5V), Vsupply=2.7V, O3 = SPEC Sensors 110-406, NO2 = SPEC Sensors 110-507, SO2 = SPEC Sensors 110-601, CO = SPEC Sensors 110-102, Pressure = ST LPS22HB, T&H = Sensirion SHT30
* Config = 3: indoor particulate matter, 5Vdc power, Vsupply=1.8V, PM2.5/PM10 = Honeywell HPMA115S0-XXX, VOC/eCO2 = Sensirion SGP30, T&H = Sensirion SHTC1
* Config = 4: outdoor particulate matter, solar power, Vsupply=2.7V, PM2.5/PM10 = Honeywell HPMA115S0-XXX, UV = VEML6075, T&H = Sensirion SHT30

### How do I get set up? ###

* Install Eclipse for nRF52840 (see points below)
** Nordic tutorial: https://devzone.nordicsemi.com/tutorials/7/
** ARM for eclipse tutorial: http://gnuarmeclipse.github.io/plugins/install/
* Setup your Eclipse environment: Eclipse_Mars_HowToSetup_2017.docx
* Hardware: Preview Development Kit nRF52840

### Contribution guidelines ###

* Every air quality enthusiast can join this project.
* Do only push code which is tested on your hardware.

### Who do I talk to? ###

* Reto Keller, info@cellabox.com