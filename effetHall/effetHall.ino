/*  Firmware for the **49* (eg SS495) hall-sensor based filament diameter sensor.
    Reads analog value from the sensor and provides a mapped and filtered diameter reading over I2C (optional analog output)
    Built for ATTiny85
    Upload firmware via programmer, using internal 16 MHz clock on ATTiny
    Compact filament sensor hardware and PCB: [URL]
    Licensed CC-0 / Public Domain by Thomas Sanladerer
*/


#define A_IN A0 //SS495 output and push-button input (for calibration procedure)




#define smooth 10 //intensity of digital lowpass filtering

float dia = 1.7;

void setup() {
  //setup pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(A_IN, INPUT);
  Serial.begin(9600);

  

  //blink to indicate sensor powered and ready
  digitalWrite(LED_BUILTIN, HIGH);
  delay(50);
  digitalWrite(LED_BUILTIN, LOW);
  delay(50);
}

void loop() {
  //get fresh reading
  short in = analogRead(A_IN);

  //lowpass filter
  dia += (convert2dia(in) - dia) / smooth;
  Serial.println(dia);

  //light LED and pull up FAULT if sensor saturated, button pressed or diameter low
  
}


float convert2dia(float in) {
  //converts an ADC reading to diameter
  //Inspired by Sprinter / Marlin thermistor reading
  byte numtemps = 6;
  const float table[numtemps][2] = {
    //{ADC reading in, diameter out}

    //REPLACE THESE WITH YOUR OWN READINGS AND DRILL BIT DIAMETERS
    
    { 0  , 3 },  // safety
    { 619  , 2.09 }, //2mm drill bit
    { 702  , 1.70 }, //1.7mm
    { 817  , 1.40 }, //1.4mm
    { 1000  , 1 }, // 1mm
    { 1023  , 0 } //safety
  };
  byte i;
  float out;
  for (i = 1; i < numtemps; i++)
  {
    //check if we've found the appropriate row
    if (table[i][0] > in)
    {
      float slope = ((float)table[i][1] - table[i - 1][1]) / ((float)table[i][0] - table[i - 1][0]);
      float indiff = ((float)in - table[i - 1][0]);
      float outdiff = slope * indiff;
      float out = outdiff + table[i - 1][1];
      return (out);
      break;
    }
  }
}

void update_dia(){
  //get fresh reading
  short in = analogRead(A_IN);

  //lowpass filter
  dia += (convert2dia(in) - dia) / smooth;
}
