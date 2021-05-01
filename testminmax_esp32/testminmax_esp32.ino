// minMaxAndRangeChecker

// A simple tool to investigate the ADC values that are seen at the

// first four analogue inputs of an Atmega chip, as used on an emonTx

//
// Robin Emley (calypso_rae on the Open Energy Monitor forum)
//
// 20th April 2013
//
#include <Esp.h>

const byte voltageSensorPin  = 34;     
const byte currentSensorPin  = 35; 

int val_a0, val_a1;
int minVal_a0, minVal_a1;
int maxVal_a0, maxVal_a1;

int loopCount = 0;

unsigned long timeAtLastDisplay = 0;

byte displayLineCounter = 0;

 

void setup(void)

{

  Serial.begin(115200);

  Serial.print("ready ...");

  delay(7000);

  Serial.println ();

  Serial.println(" The Min, Max and Range ADC values for analog inputs 0 to 3:");

}

 

void loop(void)

{

  val_a0 = analogRead(voltageSensorPin) / 4; //  Voltage Value  0V = bit ADC_V_0V. 12bits ADC ==> /4 ==> max 102

  val_a1 = analogRead(currentSensorPin) /4 ;   // Current value - 0A = bit ADC 12bits ADC ==> /4 ==> max 1024

   

  if (val_a0 < minVal_a0) { minVal_a0 = val_a0;}

  if (val_a0 > maxVal_a0) { maxVal_a0 = val_a0;}

  if (val_a1 < minVal_a1) { minVal_a1 = val_a1;}

  if (val_a1 > maxVal_a1) { maxVal_a1 = val_a1;}

     

  unsigned long timeNow = millis();

  if ((timeNow - timeAtLastDisplay) >= 3000)

  {

    timeAtLastDisplay = timeNow;

   

    displayVal(minVal_a0);

    displayVal(maxVal_a0);

    displayVal(maxVal_a0 - minVal_a0);

    Serial.print(";  ");

 

    displayVal(minVal_a1);

    displayVal(maxVal_a1);

    displayVal(maxVal_a1 - minVal_a1);

    Serial.print(";  ");

 

    

    resetMinAndMaxValues(); 

   

    displayLineCounter++;

    if (displayLineCounter >= 5)

    {

      Serial.println();

      displayLineCounter = 0;

      delay(2000); // to allow time for data to be accessed

    }

  } 

}

 

void resetMinAndMaxValues()

{

  minVal_a0 = 1023, minVal_a1 = 1023 ;

  maxVal_a0 = 0, maxVal_a1 = 0 ;

}

 

void displayVal(int intVal)

{

  char strVal[4];

  byte lenOfStrVal;

 

  // display the value as a right-justified integer

//  intVal = surplusPowerSetting; // apply integer rounding

  itoa(intVal, strVal, 10); // decimal conversion to string

  lenOfStrVal = strlen(strVal); // determine length of string

            

  for (int i = 0; i < (4 - lenOfStrVal); i++)

  {

    Serial.print(' ');

  }

 

  Serial.print(strVal);

}
