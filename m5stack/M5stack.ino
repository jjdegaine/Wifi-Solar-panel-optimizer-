/*


_____________________________________________________________________
|																                                  	|
|              Power display   on M5STACK module                    |
|                                    				                       	|
|																                                  	|
_____________________________________________________________________


version 1.0 April 2021

*/



#include <Esp.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <M5Stack.h>


// initialization wifi

const int channel = 4;  // define channel 4 seems to be the best for wifi....

WiFiUDP Udp; // Creation of wifi Udp instance, UDP is used to maximized the timing transfert

unsigned int localPort = 9999;

const char *ssid = "BB9ESERVER";   // for example to be changed 
const char *password = "BB9ESERVER";  // for examplet  to be changed


IPAddress ipServidor(192, 168, 4, 1);   // default IP for server
IPAddress ipCliente(192, 168, 4, 10);   // Different IP than server
IPAddress Subnet(255, 255, 255, 0);

  
float Power_wifi;  // power to be sent by wifi
                   
char mystring_power_wifi [50] ;       // string transmitted by wifi
unsigned long time_udp_now;
unsigned long time_udp_limit = 10000 ; // time to leave UDP 10 sec



//_____________________________________________________________________________________________
//
// SETUP
//_____________________________________________________________________________________________

void setup() {                  // Begin setup

  // Initialize the M5Stack object
  M5.begin();
  M5.Lcd.clear(BLACK);
  M5.Lcd.setCursor(0,0);
  M5.Lcd.setTextSize(4);
  M5.Lcd.print("ready ...");
  delay(1000);

// USB init
 Serial.begin(115200);

 

delay(5); //  
  WiFi.begin(ssid, password, channel);
  WiFi.mode(WIFI_STA); // ESP-32 as client
  WiFi.config(ipCliente, ipServidor, Subnet);
  Udp.begin(localPort);
  delay(5); //
  M5.Lcd.print("end init UDP client");

//____________________________________________________________________________________________
// End setup
//____________________________________________________________________________________________

}
                              



void loop()
{
/*--------------------------------------------------*/
/*---------------------- Tasks Wifi ----------------*/
/*--------------------------------------------------*/
//

      if (long (millis() - time_udp_now > time_udp_limit))             // comparing durations
              {                    
              
              // refresh screen
              M5.Lcd.clear(BLACK);
              M5.Lcd.setCursor(0,0);
              M5.Lcd.print("time to leave UDP");
              

              WiFi.begin(ssid, password, channel);
              WiFi.mode(WIFI_STA); // ESP-32 as client
              WiFi.config(ipCliente, ipServidor, Subnet);
              Udp.begin(localPort);
              delay(5); // 
              M5.Lcd.print ("end init UDP client");
              time_udp_now= millis();

              
              }
         
  int packetSize = Udp.parsePacket();
   if (packetSize) {
    
    Udp.read(mystring_power_wifi,packetSize);
    
    Power_wifi = strtof(mystring_power_wifi, NULL);
// refresh screen
      M5.Lcd.clear(BLACK);
      M5.Lcd.setCursor(0,0);
      M5.Lcd.print ("power");
      M5.Lcd.print ( Power_wifi);

      time_udp_now= millis();

      }		
               

    } // end for loop wifi

