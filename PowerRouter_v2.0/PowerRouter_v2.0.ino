/*

________________________________________________________________
|                                                               |
|       auteur : Philippe de Craene <dcphilippe@yahoo.fr        |
|           pour l' Association P'TITWATT                       |
_________________________________________________________________

Toute contribution en vue de l’amélioration de l’appareil est la bienvenue ! Il vous est juste
demandé de conserver mon nom et mon email dans l’entête du programme, et bien sûr de partager 
avec moi cette amélioration. Merci.

merci à Ryan McLaughlin <ryanjmclaughlin@gmail.com> pour avoir étudié et mis au point la partie 
commande du SCR il y a quelques années et que j'ai repris dans ce programme :)
source : https://web.archive.org/web/20091212193047/http://www.arduino.cc:80/cgi-bin/yabb2/YaBB.pl?num=1230333861/15

chronologie des versions :

version 3.5 - 9 july 2019    - test if no energy detected which started the WatchDog

____________________________________________________________________________________________


_____________________________________________________________________
|																                                  	|
|              modification by J.J 2021					          	|
|																                                  	|
_____________________________________________________________________

an ESP32 is used instead of an classic arduino Atmel AVR. The goal is to use the wifi link to transmit to an another ESP32 module the energy to be used to optimize the solar panel energy

PIN description

 IN description

 - PIN 5 static relay command
 
 - PIN 14 input WINTER 

 - PIN 15 relay command 2

 - PIN 16 LED power static relay

 - PIN 17 relay command 1

 - PIN 18 overflow LED

 - PIN 19 zero cross voltage

 - PIN 21 SDA for OLED SSD1306

 - PIN 22 SCL for OLED SSD1306

 - PIN26 input VERBOSE

 - PIN27 input CALIBRATION

 - PIN34 analog for voltage measuement

 - PIN35 analog for intensity measurement


version 2.0 first release version
*/


// init to use the two core of the ESP32; one core for power calculation and one core for wifi

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#include <Esp.h>
#include <WiFi.h>
#include <WiFiUdp.h>

//oled

#include "SSD1306.h"
SSD1306Wire display(0x3c, SDA, SCL);   // ADDRESS, SDA, SCL

// initialization wifi

const int channel = 4;  // define channel 4 seems to be the best for wifi....

WiFiUDP Udp; // Creation of wifi Udp instance, UDP is used to maximized the timing transfert

unsigned int localPort = 9999;

const char *ssid = "BB9ESERVER";   // for example to be changed 
const char *password = "BB9ESERVER";  // for example  to be changed


IPAddress ipServidor(192, 168, 4, 1);   // default IP for server
IPAddress ipCliente(192, 168, 4, 10);   // Different IP than server


// Information to be displayed

bool CALIBRATION = false;   // to calibrate Vcalibration and Icalibration
bool VERBOSE = false ;       // to verify dim and dimstep 
bool WINTER = false	;		 	  // winter -> no wifi summer --> wifi

bool do_nothing = false ; // 


float Vcalibration     = 0.90;   // to obtain the mains exact value 
float Icalibration     = 93;     // current in milliampères
float phasecalibration = 1.7;    // value to compensate  the phase shift linked to the sensors. 
byte totalCount        = 20;     // number of half perid used for measurement
float ADC_V_0V = 467 ; // ADC value for 0V input 3.3V/2
float ADC_I_0A = 467 ; // ADC value for 0V input 3.3V/2

// Threshold value for power adjustment: 

int tresholdP     = 50000;           // Threshold to start power adjustment 1 = 1mW ; 

unsigned long unballasting_timeout = 10000; // timeout to avoid relay command to often: 10 secondes
unsigned long unballasting_time;            // timer for unballasting 
byte unballasting_counter = 0;             // counter mains half period
byte unballasting_dim_min = 5;             // value of dim to start relay
byte unballasting_dim_max = 64;             // The resistive charge connected on the relay must be lower than half the resistice charge connected on the SSR

// reaction rate coefficient
// reaction_coeff define the DIM value to be added or substract
// If too small the control loop is too slow
// if too large the control loop is unstable
// reaction_coeff ~ (control loop resistance power )/4  Watt

unsigned int reaction_coeff  = 90; 

// Input and ouput of the ESP32

const byte SCR_pin           = 5;
const byte pin_winter        = 14;
const byte unballast_relay2  = 15;    
const byte unballast_relay1  = 17;    
const byte SCRLED            = 16;     
const byte limiteLED         = 18; 
const byte SDA_PIN           = 21;
const byte CLK_PIN           = 22;
const byte pin_verbose       = 26;
const byte pin_calibration   = 27;
const byte voltageSensorPin  = 34;     
const byte currentSensorPin  = 35;      
const byte zeroCrossPin      = 19;      

// zero-crossing interruption  :
 
byte dimthreshold=30 ;					// dimthreshold; value to added at dim to compensate phase shift
byte dimmax = 128;              // max value to start SSR command
byte dim = dimmax;              // Dimming level (0-128)  0 = on, 128 = 0ff 
byte dim_sinus [129] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 5, 5, 6, 7, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 18, 19, 20, 21, 23, 24, 25, 27, 28, 31, 32, 34, 35, 37, 39, 41, 43, 44, 47, 49, 50, 53, 54, 57, 58, 60, 63, 64, 65, 68, 70, 71, 74, 77, 78, 79, 82, 84, 86, 87, 89, 91, 93, 94, 96, 99, 100, 101, 103, 104, 106, 107, 108, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 122, 123, 124, 124, 124, 125, 125, 126, 126, 127, 127, 127, 127, 127, 127, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128} ;


byte dimphase = dim + dimthreshold; 
byte dimphasemax = dimmax + dimthreshold;

byte reset_wifi = 0;			// counter for wifi reset due to time to leave
byte wifi_wait = 0;       // 
        

// wifi UDP

byte ack = 0; // byte received ack from client
byte send_UDP = 0 ; //
byte send_UDP_max = 5; // send UDP data each 5*10 msec
volatile bool send_UDP_wifi = false;

unsigned long time_udp_now;
unsigned long time_udp_limit = 5000 ; // time to leave UDP 5 sec
unsigned long timeout_now;

signed long wait_it_limit = 3 ;  // delay 3msec
signed long it_elapsed; // counter for delay 3 msec

char periodStep = 68;                            // 68 * 127 = 10msec, calibration using oscilloscope
volatile int i = 0;                              // Variable to use as a counter
volatile bool zero_cross = false;                // zero cross flag for SSR
volatile bool zero_cross_flag = false;           // zero cross flag for power calculation
volatile bool first_it_zero_cross = false ;      // flag first IT on rising edge zero cross
volatile bool wait_2msec ; // flag no IT on falling edge 
volatile bool TTL = false ; // time to leave UDP
volatile bool UDP_OK = false; 



// Voltage and current measurement  :

int readV, memo_readV, readI;   // voltage and current withn ADC (0 à 1023 bits)
float rPower, V, I, sqV, sumV = 0, sqI, sumI = 0, instP, sumP = 0;  
float Power_wifi;
char mystring_power_wifi [50] ;       // string to be transmitted by wifi
byte zero_crossCount = 0;          // half period counter
    
// other value :

int dimstep;                    // DIM step value 

unsigned int memo_temps = 0;   


bool relay_1 ; // Flag relay 1
bool relay_2 ; // Flag relay 2

// init timer IT
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//init external PIN IT
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// define two tasks for UI & wifi_udp
void TaskUI( void *pvParameters );
void Taskwifi_udp( void *pvParameters );




//____________________________________________________________________________________________
//
// ZERO CROSS DETECT : interruption at each mains zero cross
// the interruption is not fully in line with the real sinus ==> dimthreshold will compensate the phase
// the interruption is define on rising edge BUT due to the slope on falling edge there is a "false"
// interruption on the falling edge ==> first_it_zero_cross with 3msec time delay
//____________________________________________________________________________________________

void IRAM_ATTR zero_cross_detect() {   // 
     portENTER_CRITICAL_ISR(&mux);
     portEXIT_CRITICAL_ISR(&mux);
     zero_cross_flag = true;   // Flag for power calculation
     zero_cross = true;        // Flag for SSR
     first_it_zero_cross = true ;  // flag to start a delay 2msec
     digitalWrite(SCRLED, LOW); //reset SSR LED
     
      send_UDP ++ ;
     if (send_UDP > send_UDP_max)
     {
       send_UDP=0; // reset counter send_UDP
       send_UDP_wifi = true ; // ready to send UDP 
     }
   
}  



/* _________________________________________________________________
 *
 * IT timer task
 * _________________________________________________________________
*/ 
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  
  portEXIT_CRITICAL_ISR(&timerMux);
  
   if(zero_cross == true && dimphase < dimphasemax )  // First check to make sure the zero-cross has 
 {                                                    // happened else do nothing

      
     
     if(i>dimphase) {            // i is a counter which is used to SSR command delay 
                                // i minimum ==> start SSR just after zero crossing half period ==> max power
                                // i maximum ==> start SSR at the end of the zero crossing half period ==> minimum power
       digitalWrite(SCR_pin, HIGH);     // start SSR
       delayMicroseconds(50);             // Pause briefly to ensure the SSR turned on
       digitalWrite(SCR_pin, LOW);      // Turn off the SSR gate, 
       i = 0;                             // Reset the accumulator
       digitalWrite(SCRLED, HIGH);      // start led SSR 
       zero_cross = false;
     } 
    else {  
      i++; 
      }           // If the dimming value has not been reached, incriment the counter
     
 }      // End zero_cross check

}


//_____________________________________________________________________________________________
//
// SETUP
//_____________________________________________________________________________________________

void setup() {                  // Begin setup

 pinMode(SCR_pin, OUTPUT);            // Set the SSR pin as output
 pinMode(unballast_relay1, OUTPUT);    // Set the Delest pin as output
 pinMode(unballast_relay2, OUTPUT);    // Set the Delest pin as output
 pinMode(SCRLED,  OUTPUT);            // Set the LED pin as output
 pinMode(limiteLED, OUTPUT);            // Set the limite pin LED as output
 pinMode(zeroCrossPin, INPUT_PULLUP);   // set the zerocross pin with pullup for interrupt
 pinMode(pin_winter, INPUT); 
 pinMode(pin_verbose, INPUT);    
 pinMode(pin_calibration, INPUT); 

unballasting_time= millis(); // set up timer unballasting


// USB init
Serial.begin(115200);

// work around I²C bug at start up   https://github.com/esp8266/Arduino/issues/1025

delay(2000);
  //try i2c bus recovery at 100kHz = 5uS high, 5uS low
  pinMode(SDA_PIN, OUTPUT);//keeping SDA high during recovery
  digitalWrite(SDA_PIN, HIGH);
  pinMode(CLK_PIN, OUTPUT);
  for (int i = 0; i < 10; i++) { //9nth cycle acts as NACK
    digitalWrite(CLK_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(CLK_PIN, LOW);
    delayMicroseconds(5);
  }

  //a STOP signal (SDA from low to high while CLK is high)
  digitalWrite(SDA_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(CLK_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(SDA_PIN, HIGH);
  delayMicroseconds(2);
  //bus status is now : FREE

  Serial.println("bus recovery done, starting scan in 2 secs");
  //return to power up mode
  pinMode(SDA_PIN, INPUT);
  pinMode(CLK_PIN, INPUT);
  delay(2000);

  Wire.begin(SDA_PIN, CLK_PIN);
  


 //init OLED
display.init();
display.flipScreenVertically();
display.setFont(ArialMT_Plain_24);
display.drawString(0, 0, "Ready");
display.display();


 Serial.println ();

 Serial.println(); 
 Serial.println("Ready ...");
 //display.drawString(0, 0, "Ready");
 //display.display();

 Serial.println ();
 delay(500); 
 if( VERBOSE == true ) Serial.print("  Pu (W) || dimstep |  dim || ");
 else Serial.println("GO"); 
 Serial.println();

 // display.setFont(ArialMT_Plain_24);
  display.clear();


 digitalWrite(unballast_relay1, LOW);    // unballast relay 1 init
 digitalWrite(unballast_relay2, LOW);    // unballast relay 2 init

 
  //init wifi_udp


  WiFi.softAP(ssid, password,channel);  // ESP-32 as access point
  delay(500); // Delay to wait Wifi init 
  Udp.begin(localPort);
  Serial.println("init access point UDP OK");

  display.drawString(0, 22, "UDP OK");
  display.display();
  
 // init timer 
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, periodStep , true);
  timerAlarmEnable(timer); 

 // init interrupt on PIN  zero_crossing
 attachInterrupt(digitalPinToInterrupt(zeroCrossPin), zero_cross_detect, RISING);  
  
 
 // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(
    TaskUI
    ,  "TaskUI"   // A name just for humans
    ,  20000  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    Taskwifi_udp
    ,  "wifi_udp"
    ,  20000  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
   
  
 
}                

//____________________________________________________________________________________________
// End setup
//____________________________________________________________________________________________


                              



void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks UI ------------------*/
/*--------------------------------------------------*/
//


// 
// Power calculation using ADC value ==> rPower
//____________________________________________________________________________________________
//
 
void TaskUI(void *pvParameters)  // This is the task UI.
{
  (void) pvParameters;

  for (;;) // A Task shall never return or exit.
  {
    
  
  unsigned int numberOfSamples = 0;
  sumV = 0;
  sumI = 0;
  sumP = 0;
  unsigned int time_now_second = millis()/1000;      // timer in second





// Count 20 zero cross to calculate U / I / P


 if( zero_crossCount >= totalCount ) { zero_crossCount = 0; }


// Value are measure during totalcount half period  
  while( zero_crossCount < totalCount ) {
    if( zero_cross_flag == true ) {        // Half period counter
      zero_cross_flag = false;
      zero_crossCount++; 
    } 
    
    numberOfSamples++;                         // counter of samples U and I
    
    memo_readV = readV;                  // 
    readV = analogRead(voltageSensorPin) / 4;   // Voltage Value  0V = bit ADC_V_0V. 12bits ADC ==> /4 ==> max 1024
    
	if( memo_readV == 0 && readV == 0 ) { break; } // exit the while if no powersupply
    readI = analogRead(currentSensorPin) /4 ;   // Current value - 0A = bit ADC 12bits ADC ==> /4 ==> max 1024

  
// RMS Current and Voltage 

    if( CALIBRATION == true ) {                         // 
      sqV= (readV -ADC_V_0V) * (readV -ADC_V_0V);             // ADC_V_0V ==> 0volt 
      sumV += sqV;               
      sqI = (readI -ADC_I_0A) * (readI -ADC_I_0A);
	    sumI += sqI;
    } 
 
// instantaneous power calculation 
    instP = ((memo_readV -ADC_V_0V) + phasecalibration * ((readV -ADC_V_0V) - (memo_readV -ADC_V_0V))) * (readI -ADC_I_0A); 
    sumP +=instP;  


// function delay 2msec

    if (first_it_zero_cross == true  )            // first IT on rising edge ==> start a delay during 3msec to avoid false zero cross detection
      {            
       
       it_elapsed = millis () + wait_it_limit;
      
       detachInterrupt(digitalPinToInterrupt(zeroCrossPin)); // invalid interrupt during 3msec to avoid false interrupt during falling edge
       first_it_zero_cross = false;      // flag for IT zero_cross
       wait_2msec = true ;
      }
      
      if (wait_2msec == true && long (millis() - it_elapsed) >= 0 )        // check if delay > 3msec to validate interrupt zero cross, wait_it is incremeted by it timer ( 75usec)
      {
      
        attachInterrupt(digitalPinToInterrupt(zeroCrossPin), zero_cross_detect, RISING);
        wait_2msec=false ; 
      }

  
 }      // end while sur zero_crossCount


// Power calculation

  if( numberOfSamples > 0 ) {
    if( CALIBRATION == true ) { 
      V = Vcalibration * sqrt(sumV / numberOfSamples);
      I = Icalibration * sqrt(sumI / numberOfSamples);
    }
    rPower = Vcalibration * Icalibration * sumP / numberOfSamples;
 
    Power_wifi =  rPower/1000 ; // Power wifi using float 
  }
	
//____________________________________________________________________________________________
//
// Power to be unbalanced to avoid injection of electricity to the grid
//
//____________________________________________________________________________________________
//
// dimstep calculation.  
//
  if( rPower > 0 ) { dimstep = (rPower/1000)/reaction_coeff + 1; } 
  else { dimstep = 1 - (rPower/1000)/reaction_coeff; }
  
  // when rPower is less than tresholdP ==> unlalanced power must increased ==> DIM must be reduced

  if( rPower < tresholdP ) {      
    if( dim > dimstep )  dim -= dimstep; else  dim = 0;
  } 

// when rPower is higher than tresholdP ==> unlalanced power must decreased ==> DIM must be increasad

  else if( rPower > tresholdP ) {                   
    if( dim + dimstep < dimmax ) dim += dimstep;  else  dim = dimmax; 
  }

  if(dim < 1) { digitalWrite(limiteLED, HIGH); }  // if dim is at the minimum, control regulation is at the maximum 
  else { digitalWrite(limiteLED, LOW); }
  

// Value to used by the timer interrupt due to real phase between interruption and mains
dimphase = dim_sinus [ dim ] + dimthreshold;

// Relay command. to avoid control regulation with a large power (which imply large harmonic) two relay are used to command fixed power charge. 
// to avoid instability the DIM value is confirm 10 times and the relay remains stable during unballasting_timeout time
//  a thershold is added using dim_min and dim_max
//
  if (long (millis() - unballasting_time > unballasting_timeout))
   {
    
     if (dim < unballasting_dim_min)  // DIM is minimum => power in SSR is maximum
      {
      
        if (unballasting_counter > 10) // dim is < unballasting_dim_min during 10 half period
        {
          
          if (relay_1 == true) 
          {
            if(relay_2 == true)
              {
                unballasting_counter = 0;      // overflow
                digitalWrite(limiteLED, HIGH) ;
                
              }
            else 
              {
                digitalWrite( unballast_relay2, HIGH) ; // set relay 2 
                relay_2 =true;
                unballasting_counter= 0 ;
                unballasting_time = millis() ;
              }
          }     
          else
              {
              digitalWrite (unballast_relay1, HIGH)  ; //set relay 1
              relay_1 = true;
              unballasting_counter= 0 ;
              unballasting_time = millis() ;
              }     
          
      
       }  
        unballasting_counter ++ ;
      }
      //
      // 
      //
      if (dim > unballasting_dim_max) {
        //
        if (unballasting_counter > 10 ) // 
        {
          if (relay_2 == true)
          {
            digitalWrite (unballast_relay2, LOW) ; 
            relay_2 = false;
            unballasting_counter = 0 ;
            unballasting_time= millis();
          }
          else
          {
            digitalWrite (unballast_relay1, LOW) ;
            relay_1 = false;
            unballasting_time= millis();
            unballasting_counter = 0 ;
          }
         }
         unballasting_counter ++ ;
        } 
      
      }
  

 

  // Display each 2 seconds


  if( time_now_second >= memo_temps +2 ) {

    if ( (CALIBRATION ==false) || (VERBOSE == false)) {
          memo_temps = time_now_second;


          Serial.print("P= ");
          Serial.print(rPower/1000);   
          Serial.print("w");
          Serial.print("dim: ");
          Serial.print(dim);
          Serial.print ("dimphase ");
          Serial.println (dimphase) ;



          display.setColor(BLACK);        // clear first line
          display.fillRect(0, 0, 128, 22);
          display.setColor(WHITE); 

          display.drawString(0, 0, String(int(Power_wifi)) + "||" + String (dim));
          display.display();
        }
      }

          // 
      
        if( CALIBRATION == true ) {
      	  Serial.print(V);
      	  Serial.print("  |  ");
          Serial.print(I/1000);
          Serial.print("  |  ");
          Serial.print(rPower/1000);
          Serial.println();

          display.clear();
          display.drawString(0, 0, String(int(V)) + "||" + String(int(I/1000))) ;
          display.drawString(0, 22, String(int(Power_wifi)));
          display.display();

        }
        if( VERBOSE == true ) {
          Serial.print(rPower/1000);
          Serial.print("  ||     ");
          Serial.print(dimstep);
          Serial.print("  ||  ");
          Serial.print(dim);
          Serial.print(" ||  ");
          Serial.print(dimphase);
          Serial.print(" ||  ");
          Serial.print (relay_1);
          Serial.print(" ||  ");
          Serial.print (relay_2);
          Serial.print(" ||  ");
          Serial.print (unballasting_counter);
          Serial.print(" ||  ");
          Serial.print (millis() - unballasting_time);
       
          Serial.println();

        }
  
        else { delay(1); }           // needed for stability

// update switches winter, verbose, calibration

       WINTER = digitalRead (pin_winter);
      
       VERBOSE = digitalRead (pin_verbose);
     
        CALIBRATION = digitalRead (pin_calibration);
      

        // display WIFI information
        if (TTL == true)
              {
              display.setColor(BLACK);        // clear second  line
              display.fillRect(0, 22, 128, 22);
              display.setColor(WHITE); 
              display.drawString(0, 22, "TIME UDP");
              display.display();
              TTL= false ;
              }
        if ( UDP_OK == true) 
            {
              display.setColor(BLACK);        // clear second  line
              display.fillRect(0, 22, 128, 22);
              display.setColor(WHITE); 
              display.drawString(0, 22, "UDP OK");
              display.display();
            UDP_OK = false ;
            }

  } 
  
}                              // end task UI


/*--------------------------------------------------*/
/*---------------------- Tasks Wifi ----------------*/
/*--------------------------------------------------*/
//

void Taskwifi_udp(void *pvParameters)  // This is a task.
{
    (void) pvParameters;
    
    time_udp_now= millis(); 
    

    for (;;) // A Task shall never return or exit.
    {
  	   while(send_UDP_wifi == false )
  	   {
  		wifi_wait=0; // loop to wait update DIM
  	    		
  	   }

       // logic: we want wifi if not (calibration or verbose or winter)
      if (((CALIBRATION == false) && (VERBOSE == false) && (WINTER == true)))
      
      {
 
        // verification time to leave UDP
                   
              if (long (millis() - time_udp_now > time_udp_limit))             // comparing durations
              {                    
              TTL = true ;

              WiFi.disconnect();
              WiFi.softAP(ssid, password,channel);  // ESP-32 as access point
              
              timeout_now= millis() ;
              while ( long(millis() - timeout_now < 500 )) { do_nothing = true;} // delay 500 msec

              Udp.begin(localPort);

              timeout_now= millis() ;
              while ( long(millis() - timeout_now < 5000 )) { do_nothing = true;} // delay 5000 msec

              UDP_OK = true ;
            
              time_udp_now= millis(); // reset time to leave
             
              
              }
         
      		send_UDP_wifi = false ; 
      		Udp.beginPacket(ipCliente,9999);   //Initiate transmission of data

          sprintf(mystring_power_wifi, "%g", Power_wifi); 

          Udp.print (mystring_power_wifi) ; 
       
      		Udp.endPacket();  // Close communication
        
      		// read acknowledge from client
              int packetSize = Udp.parsePacket();
              if (packetSize) 
            		{
                    int len = Udp.read(&ack, 1);
                    time_udp_now= millis(); // reset time to leave
            		} 
       }

     } // end for loop wifi
    
  }
