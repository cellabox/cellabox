![Image of Cellabox](https://raw.githubusercontent.com/cellabox/cellabox/master/doc/cellabox_opensource_logo.png)

Cellabox is an **Open Source Air Quality Project**. According to the [WHO](http://www.who.int/phe/news/march-2017/en/), air pollution from both outdoor and indoor sources represents the single largest environmental risk to health globally.
Help to solve the air quality problem and join Cellabox! Better Air Quality Through Better Data.

This repository contains firmware (C code) for nRF52840 with a **Thread network** implementation (OpenThread) and a connection to the cloud platform of **thethings.iO**.
* [Nordic nRF52840](https://www.nordicsemi.com/eng/Products/nRF52840)
* [OpenThread](https://openthread.io/) network
* Drivers for sensors from [Sensirion](https://www.sensirion.com), [SPEC Sensors](https://www.spec-sensors.com/) and [ST Microelectronics](http://www.st.com/en/mems-and-sensors/lps22hb.html)
* Cloud connection with CoAP to [thethings.iO](https://thethings.io/)

We don't have any sponsors or supporters.
We very much appreciate if you were able to use some of our code and want to say thank you:

[HELP CELLABOX](https://www.cellabox.com/support)

(c) 2017-2018 [Cellabox](https://www.cellabox.com), all rights reserved.

# Cellabox FW concept

IMPORTANT: there is only one single FW build for all modules (the so called Cellaboxes). The FW initializes itself depending on the module identity (ID = [0...63].
Which IO pins are used for the module ID? - [here is the pinout](https://github.com/cellabox/cellabox/wiki/FW:-pinout-nRF52840).

# List of sensor modules

### Climate Module
* ID = 0
* Temperature and humidity
* [Climate Module Wiki](https://github.com/cellabox/cellabox/wiki/ID=0:-Climate-Module)
	
### Indoor Air Quality Module
* ID = 1
* VOC, T&H, pressure
* [Indoor Air Quality Module Wiki](https://github.com/cellabox/cellabox/wiki/ID=1:-Indoor-Air-Quality-Module)
	
### Outdoor Air Quality Module
* ID = 2
* O3, NO2, SO2, CO, T&H, pressure
* [Outdoor Air Quality Module Wiki](https://github.com/cellabox/cellabox/wiki/ID=2:-Outdoor-Air-Quality-Module)
	
### Indoor PM2.5 Module
* ID = 3
* PM2.5/PM10, VOC, eCO2, T&H, pressure
* [Indoor PM2.5 Module Wiki](https://github.com/cellabox/cellabox/wiki/ID=3:-Indoor-PM2.5-Module)

### Outdoor PM2.5 Module
* ID = 4
* PM2.5/PM10, T&H, pressure
* [Outdoor PM2.5 Module Wiki](https://github.com/cellabox/cellabox/wiki/ID=4:-Outdoor-PM2.5-Module)

### [Define your own Module](#how-to-define-your-own-module)
* ID = ???
* Sensors = ???
* ??? Module Wiki

# List of sensors

The firmware supports different sensors:

* **Temperature and humidity (T&H)** = [Sensirion SHTC1 (1.8V)](https://github.com/cellabox/cellabox/blob/master/hw/datasheets/Sensirion_Humidity_Sensors_SHTC1_Datasheet_V4.pdf)
* **Temperature and humidity (T&H)** = [Sensirion SHT30 (2.7V)](https://github.com/cellabox/cellabox/blob/master/hw/datasheets/Sensirion_Humidity_Sensors_SHT30_Datasheet_V4.pdf)
* **Barometric pressure** = [ST LPS22HB](https://github.com/cellabox/cellabox/blob/master/hw/datasheets/ST_BarometricPressure_Sensor_LPS22HB_Datasheet.pdf)
* **Volatile Organic Compounds (VOC)** = [Sensirion SGPC3](https://github.com/cellabox/cellabox/blob/master/hw/datasheets/Sensirion_Gas_Sensors_SGPC3_Preliminary_Datasheet.pdf)
* **Volatile Organic Compounds (VOC), equivalent CO2 (eCO2) = [Sensirion SGP30](https://github.com/cellabox/cellabox/blob/master/hw/datasheets/Sensirion_Gas_Sensors_SGP30_Preliminary_Datasheet.pdf)
* **Ozone (O3)** = [SPEC Sensors 110-401](https://github.com/cellabox/cellabox/blob/master/hw/datasheets/SPEC-SENSORS_3SP_O3_5-P-Package-110-401.pdf + [TI LMP91000](https://github.com/cellabox/cellabox/blob/master/hw/datasheets/TI_LMP91000_AnalogFrontEnd_GasSensors_2016.pdf)
* **Nitrogen dioxide (NO2)** = [SPEC Sensors 110-501](https://github.com/cellabox/cellabox/blob/master/hw/datasheets/SPEC-SENSORS_3SP_NO2_5-P-Package-110-501.pdf) + [TI LMP91000](https://github.com/cellabox/cellabox/blob/master/hw/datasheets/TI_LMP91000_AnalogFrontEnd_GasSensors_2016.pdf)
* **Sulfur dioxide (SO2)** = [SPEC Sensors 110-601](https://github.com/cellabox/cellabox/blob/master/hw/datasheets/SPEC-SENSORS_3SP_SO2_20-P-Package-110-601.pdf) + [TI LMP91000](https://github.com/cellabox/cellabox/blob/master/hw/datasheets/TI_LMP91000_AnalogFrontEnd_GasSensors_2016.pdf)
* **Carbon monoxide (CO)** = [SPEC Sensors 110-102](https://github.com/cellabox/cellabox/blob/master/hw/datasheets/SPEC-SENSORS_3SP_CO_1000-P-Package-110-102.pdf) + [TI LMP91000](https://github.com/cellabox/cellabox/blob/master/hw/datasheets/TI_LMP91000_AnalogFrontEnd_GasSensors_2016.pdf)
* **Particulate matters PM2.5/PM10** = [Honeywell HPMA115S0-XXX (5V)](https://sensing.honeywell.com/sensors/particle-sensors/hpm-series)

# How to get started

* **Install Eclipse** for nRF52840:
	* Nordic tutorial: https://devzone.nordicsemi.com/tutorials/7/
	* ARM for eclipse tutorial: http://gnuarmeclipse.github.io/plugins/install/
* **Setup Eclipse** for Cellabox: [Eclipse_Mars_HowToSetup_2017.docx](/doc/Eclipse_Mars_HowToSetup_2017.docx)
* **Order development kit**: [Preview Development Kit nRF52840](https://www.nordicsemi.com/eng/Products/nRF52840-Preview-DK)
* **Order sensors**: order the sensors you need for the module(s).
* **Wire your module**: [see our schematics here](https://github.com/cellabox/cellabox/tree/master/hw/kicad).
* (Optional) Define your own module: [here](#how-to-define-your-own-module).

# How to define your own module

1. Contact Reto Keller, info@cellabox.com
2. Reserve an ID number with adding your sensor to the list above.
3. If you are planning to implement a new sensors: add the sensor to the [list of sensors](#list-of-sensors).
4. Add a page with the module description to the [Wiki](https://github.com/cellabox/cellabox/wiki)
5. Add a folder with your schematic like this: *cellabox/hw/kicad/id=?_sensor-name/*. Draw your schematic with the free and open source ECAD [KiCAD](http://kicad-pcb.org/).
6. Implement your module and sensors on the nRF52840.
7. Once tested, commit your changes to this repository.

# Contribution guidelines

* **TODO**: If you hard code something or you have to continue there later — write TODO in the comment and start a task in the [Cellabox Project](https://github.com/cellabox/cellabox/projects/4). Also start a [Cellabox Project](https://github.com/cellabox/cellabox/projects/4) task to describe future features of your module or the FW in general.
* **Bugs and issues**: Bugs and feature requests — [submit to the Issue Tracker](https://github.com/cellabox/cellabox/issues)
* **Questions**: Stack Overflow — [post questions using the `cellabox` tag](http://stackoverflow.com/questions/tagged/cellabox)
* **ECAD**: use [KiCAD](http://kicad-pcb.org/).

# License

Cellabox is released under the [BSD 3-Clause license]. See the [`LICENSE`](https://github.com/cellabox/cellabox/blob/master/LICENSE) file for more information.
Please use and reference to **Cellabox** in case you use this software or part(s) of it.
