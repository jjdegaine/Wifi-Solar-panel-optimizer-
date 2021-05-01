---
marp: false
theme: default
title: Solar Panel Optimizer with WIFI Slave SSR
html: true
---

# Solar Pannel Optimizer with WIFI slave SSR

    The Goal of this WEB site is to explain why and how I create a Solar Panel Optimzer with a WIFI slave SSR
    First of all I have decided to install solar panel to compensate the energy needed for the simming tool pump.
 
    The pump is a 1.1kW so I install 4 * 250W solar panel on a firewood storage shed roof.

[lien] (https://www.oscaro-power.com/kit-solaire-autoconsommation/706-3835-kit-solaire-autoconsommation-le-petit-kit-meilleur-prix.html#/175-nombre_de_panneau_kit-4/768-type_de_fixation-fibrociment)



    Due to mains regulation I realized that the excess energy must not be sent to grid. 
    
    It will be the case in winter (no swimming pool), in spring or autum when the swimming pump is connected only 3 or 4 hours per day.

    May be also in summer when the pump is connected 6 or 7 hours per day.

    In winter the excess energy could be used to heat my workshop in the basement.

    In the other seasons a second small pump with a waterfall and a swimming pool heater could used this excess energy.

    Thereafter my home_eletric_wiring

![nomimage](home_eletric_wiring.jpg)

    Many thanks to my colleague Xavier, Nabil, Regis, Benjamin

    Please note that it was my first Hardware and Software project since my Ph.D. forty years ago...
    
    

# Optimizer description

The issue was the existing electric wiring so I decided to create a Solar Panel Optimizer with a WIFI slave SSR. see my home electric wiring description.

A lot of opimizer exist, commercial one and DIY project. The commercial optimizer are quite expensive and not so efficent. After several weeks reading website and DIY optimizer description I decided to create my own project, and it was quite fun !

 The solar Panel Optimizer is based on a processor ESP32-DEVKITC-32U which is compatible with arduino. The existing DIY project are based on arduino.

 https://mk2pvrouter.co.uk/index.html

 http://ptiwatt.kyna.eu/?post/2018/07/23/Fabriquer-un-power-router

 http://forum-photovoltaique.fr/viewtopic.php?f=18&t=38146
 

 ESP32 module embed a dual core with a dual-core32-bit and a wi-fi link. 
 One core will be used for power calculation, the second core for wi-fi link
 The power calculation is mainly based on ptiwatt router.



# Hardware description


 A power supply to provide +8V and regulated +5V 

![nomimage](power_supply.jpg)

 An optocoupler H11A1 to detect zero cross interruption, 

![nomimage](Zero_cross_detection.jpg)

a small shift is compensated by software (dimthreshold). the falling edge is hidden by software (first_it_zero_cross)

![nomimage](Zero_cross_detection_IT.jpg)

 Voltage and current measurement using ADC with a shift of 3.3V/2

![nomimage](UI_measurement.jpg)

A command for the SSR

![nomimage](SSR_control.jpg)

The full Schematic is available on GITHUB

![nomimage](schema.jpg)


 the first version of PCB was tested and needs some modifications, the updated version was not tested.
 
 the two version of GERBER for manufactoring are available on GITHUB
 
 PCB supplier: https://jlcpcb.com/

 BOM supplier: mainly aliexpress, and friends...

 


# Software description


    see comments on source code :-)    https://github.com/jjdegaine/Wifi-Solar-panel-optimizer-

    One software for the Server and one software for the client

    ESP32 processor needs a specific environmment on Arduino software
    see link to install ESP32 environmment  https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/
    I use a board ESP32 Dev Module
    Some other libraries are needed. I don't remember exactly which are mandatory and which have been installed for testing

![nomimage](libraries_1.jpg) 

![nomimage](libraries_2.jpg) 

![nomimage](libraries_3.jpg)

The softawre needs some calibration depending on components used.

    measure U and I ADC 0Volt using software "testminmax_esp32" and modify values
    Connect the ESP board WITHOUT the mains.

```c++
//

float ADC_V_0V = 467 ;
float ADC_I_0A = 467 ;
```


    measure shift IT zero cross using software "dim final"  and modify value 
    Connect an incandescent lamp , a startup with DIM=0 lamp shines. DIM will slowly increase, suddenly the lamp turn off. Note the DIM value.

```c++
byte dimthreshold=30 ;	// dimthreshold; value to added at dim to compensate phase shift
```
    measure mains voltage and modify value Vcalibration. Voltage and Current can be displayed on the OLED using the switch SW2

    ==> Vcalibration
```c++
float Vcalibration     = 0.90;   // to obtain the mains exact value 
```

    measure mains current using and known power charge and modify value

    ==> Icalibration
```c++
float Icalibration     = 93;     // current in milliampères
```
   The boars is ready to be used.
 
Wi-fi

    an UDP link is used to reduce data transfert, only power value is transmitted with an Acknowledge by the client.

    The power value is transmitted each 50msec (byte send_UDP_max); 

    A Time To live is used to check the WiFi activity and restart Wifi link if needed.

    a small M5STACK module can be used as a remote display.

    Wifi parameter to be modified
```c++
const char *ssid = "BB9ESERVER";   // for example to be changed 
const char *password = "BB9ESERVER";  // for examplet  to be changed
```
    


