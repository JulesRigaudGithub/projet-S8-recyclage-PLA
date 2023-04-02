///////////////////////////////////////////////////////////////////////////////
//////////////////////// BRANCHEMENTS /////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

/*Capteur effet hall InFideL
 * 
 * hallOutput  ------------- A0
 * 
 */

/*Moteur pas-à-pas
 * 
 * Bleu  ------------- A+
 * Vert ------------- A-
 * Jaune ------------- B+
 * Rouge ------------- B-
 * 
 */

 /* Ventilateur
  * Bleu ------- 6 (PWM)
  * Jaune ------- NONE
  */


/*Ecran LCD
 * 
 * Gnd -------- Gnd
 * VCC -------- 5V
 * SDA -------- A4
 * SCL -------- A5
 * 
 * 
 */

/* Encodeur
 * 
 * clk ------- 2
 * DT  ------- 1
 * SW  ------- 3
 *  +  ------- +5V
 * GND ------- GND
 * 
 */
 

///////////////////////////////////////////////////////////////////////////////
//////////////////////// CODE  ////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////



////////////////////////////////// Display ////////////////////////////////////
uint32_t pre_displaytime = 0;
#define Delta_display 2000 // time between display in ms

uint32_t pre_dynamic_displaytime = 0;
#define Delta_dynamic_display 500 // time between display in ms

# define sensi 10 //sensibilité de l'encodeur rotatif
int active_line = 0; // 0: aucune ligne active | 1:premiere ligne active | 2:deuxieme ligne active
bool active_number = false; // if true we can change the target temperature

////////////////////////////////// Ecran LCD //////////////////////////////////
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x20,20,4);



////////////////////////////////// Encodeur  //////////////////////////////////
#define clkPin 2
#define DTPin 1
#define SwitchPin 3
volatile int encoder_pos = 0;
//int old_encoder_pos = 0;

bool sw_button = false;

///////////////////////////////  PWM Controle Ventilateur  ////////////////////////////////

#define PWM_pin 6

int PWM_ratio = 50;

byte PWM_8bit = map(PWM_ratio, 0, 100, 0, 255);
 
bool new_ratio = true;




////////////////////////////// Asservissement /////////////////////////////////
#define asservissement_cycle 500 // time between two calculation of the speed consign
uint32_t pre_asservissement = 0;

#define K_p  10.0
#define K_d  0
#define K_i  0.1
//puller
float consigne_dia = 1.7;
float somme_erreur = 0;
float erreur_pre = 0;

float pi = 3.14159;
float DIAMETRE_PULLER = 41;
float alpha = 0.1;

////////////////////////////// Moteur pas-à_pas /////////////////////////////////

const int stepPin = 4;
const int dirPin = 5;
const int stepsPerRev=200;
int pulseWidthMicros = 1000;  // microseconds
float RPM = 70;

int microsBtwnSteps = 2000;


int prec_time = 0;
bool pulsation_en_cours = false;

////////////////////////////// Capteur InFidel /////////////////////////////////

#define smooth 70
#define Analog_InFidel A0

float diametre = 1.7;



///////////////////////////////////////////////////////////////////////////////
////////////////////////   MAIN CODE  /////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////




void setup() {
  Serial.begin(115200);

  //Ventilateur
  pinMode(PWM_pin,OUTPUT);

//  //Stepper
//  pinMode(stepPin, OUTPUT);
//  pinMode(dirPin, OUTPUT);

  //InFidel
  pinMode(Analog_InFidel, INPUT);
  
  //User interface
  init_LCD();
  init_encodeur();
}


void loop() {
  ////// Run stepper //////
  //spin();

  ////// PWM handling //////
  if(new_ratio){
    new_ratio = false;
    set_fan_speed();
  }

  ////// InFidel //////////
  update_dia();
  Serial.println(analogRead(Analog_InFidel));
  
  ////// Display handling //////
  if(millis()-pre_displaytime>Delta_display)
  {
    pre_displaytime=millis();
//    Serial.print("t, Temperature1, Temperature2,  PWM_ratio 1 , WM_ratio 2");
//    Serial.print(millis());Serial.print(" , ");
//    Serial.print(thermo1.temperature(RNOMINAL, RREF));Serial.print(" , ");
//    Serial.print(thermo2.temperature(RNOMINAL, RREF));Serial.print(" , ");
//    Serial.print(PWM_ratio_1);Serial.print(" , ");
//    Serial.println(PWM_ratio_2);
    display_LCD();
  }

  ////// dynamic Display handling //////
  if(millis()-pre_dynamic_displaytime>Delta_dynamic_display)
  {
    pre_dynamic_displaytime = millis();
    dynamic_display_handler();
  }

 
  ////// Asservissement handling //////
//  if(millis()-pre_asservissement>asservissement_cycle){
//    pre_asservissement=millis();
//    asservissement();
//    //asservissement_PID(); }
//    //stepper_speed(RPM);
//  }
  
}


///////////////////////////////////////////////////////////////////////////////
////////////////////////  USEFUL FUNCTION  ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

////////////////////////////////// Asservissement //////////////////////////////////


float vitesse(){
  //en mm par seconde
  return (DIAMETRE_PULLER*pi*RPM/60.);
}


float debit(){
  //en mm cube par seconde
  return (vitesse()*pi*diametre*diametre/4.0);
}

float consigne_RPM(){
  return ((60.*4.*debit())/(DIAMETRE_PULLER*pi*pi*consigne_dia*consigne_dia));
}

void asservissement(){
  RPM = (1-alpha)*RPM + alpha*consigne_RPM();
  if(RPM<0){RPM = 0;}
  if(RPM>300){RPM = 300;}
}


void asservissement_PID(){
  float erreur = consigne_dia - diametre;
  float delta_erreur = erreur - erreur_pre;
  erreur_pre = erreur;

  //determination de la commande
  RPM = K_p * erreur + K_i * somme_erreur*float(asservissement_cycle)/1000 +K_d*delta_erreur;

  // Prise en compte de la saturation de la commande
  if(RPM<0){RPM = 0;somme_erreur += erreur;}
  else if(RPM>300){RPM = 300;}
  else {somme_erreur+=erreur;} // on bloque l'effet intégrale lorsque la commande sature
}



////////////////////////////////// Ventilateur //////////////////////////////////

void set_fan_speed(){
  // Conversion du « pourcentage » de rapport cyclique en une valeur comprise entre 0 et 255
  int PWM_8bit = map(PWM_ratio, 0, 100, 0, 255);

  // Génération du signal PWM
  pinMode(PWM_pin, OUTPUT);                            // Définition de la broche D5 en tant que « SORTIE »
  analogWrite(PWM_pin, PWM_8bit);    // Génération du signal PWM, avec le rapport cyclique voulu
  
}


////////////////////////////////// InFidel //////////////////////////////////

void update_dia(){
  //get fresh reading
  short in = analogRead(Analog_InFidel);

  //lowpass filter
  diametre += (convert2dia(in) - diametre) / smooth;
}



float convert2dia(float in) {
  //converts an ADC reading to diameter
  //Inspired by Sprinter / Marlin thermistor reading
  byte numtemps = 6;
  const float table[numtemps][2] = {
    //{ADC reading in, diameter out}

    //REPLACE THESE WITH YOUR OWN READINGS AND DRILL BIT DIAMETERS
    
    { 0  , 3 },  // safety
    { 583  , 2.18 }, //2mm drill bit
    { 670  , 1.70 }, //1.7mm
    { 783  , 1.40 }, //1.4mm
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


////////////////////////////////// Stepper //////////////////////////////////

void spin(){
 //Makes 200 pulses for making one full cycle rotation
 if(pulsation_en_cours){
    if(micros()- prec_time > pulseWidthMicros){
      digitalWrite(stepPin, LOW);
      prec_time = micros();
      pulsation_en_cours = false;
    }
  }
 else{
    if(micros()- prec_time > microsBtwnSteps){
      digitalWrite(stepPin, HIGH);
      prec_time = micros();
      pulsation_en_cours = true;
   }
 }
}


void stepper_speed(float RPM){
  microsBtwnSteps = int((1000000*60)/(200*RPM));
}
////////////////////////////////// Ecran LCD //////////////////////////////////

void init_LCD()
{  
   lcd.init(); //initialisation de l'afficheur
   lcd.backlight();
   
   lcd.setCursor(1,0);
   lcd.print("Dia:    /");
   lcd.setCursor(5,0);
   lcd.print(diametre);
   lcd.setCursor(11,0);
   lcd.print(consigne_dia);

   lcd.setCursor(1,1);
   lcd.print("Fan:     %");
   lcd.setCursor(6,1);
   lcd.print(PWM_ratio);
  
}

void display_LCD()
{
   lcd.setCursor(5,0);
   lcd.print(diametre);
}

void dynamic_display_handler()
{
  if(not active_number)
  {
    //Selection des lignes
    if(encoder_pos%10<5) {  active_line=1;  lcd.setCursor(0,0); lcd.print(">");lcd.setCursor(0,1);lcd.print(" "); }
    else {active_line=2;  lcd.setCursor(0,0); lcd.print(" ");lcd.setCursor(0,1);lcd.print(">");}
    //Si l'utilisateur clique on passe à la selection de la cible de la ligne désigné
    if(sw_button)
    {
      sw_button=false;
      active_number=true;
      lcd.setCursor(0,0); lcd.print(" ");lcd.setCursor(0,1);lcd.print(" ");
      if(active_line==1){encoder_pos=int(consigne_dia*100*sensi);lcd.setCursor(10,0);lcd.print("<");lcd.setCursor(15,0);lcd.print(">");} 
      else{encoder_pos=PWM_ratio*sensi;lcd.setCursor(5,1);lcd.print("<");lcd.setCursor(9,1);lcd.print(">");}     
    }
  }
  else
  {
    if(active_line==1){
      if(encoder_pos/sensi<0){encoder_pos = 0;}
      if(encoder_pos/sensi>999){encoder_pos = 999*sensi;}
      lcd.setCursor(11,0);lcd.print(float(int(encoder_pos/sensi))/100.);
    }
    else{
      if(encoder_pos/sensi<0){encoder_pos = 0;}
      if(encoder_pos/sensi>100){encoder_pos = 100*sensi;}
      lcd.setCursor(6,1);lcd.print("   ");lcd.setCursor(6,1); lcd.print(int(encoder_pos/sensi));
    }
    if(sw_button){
      
      sw_button=false;
      active_number=false;
      if(active_line==1){
        consigne_dia=float(int(encoder_pos/sensi))/100.;
        lcd.setCursor(10,0);lcd.print(" ");lcd.setCursor(15,0);lcd.print(" ");
      }
      else{
        PWM_ratio=int(encoder_pos/sensi);
        new_ratio = true;
        lcd.setCursor(5,1);lcd.print(" ");lcd.setCursor(9,1);lcd.print(" ");
      }     
      encoder_pos=0;
    }
  }
}


////////////////////////////////// Encodeur //////////////////////////////////

void init_encodeur()
{
  pinMode(clkPin,INPUT_PULLUP);
  pinMode(DTPin,INPUT_PULLUP);
  pinMode(SwitchPin,INPUT_PULLUP);
  attachInterrupt(0,rotary_encoder,CHANGE);
  attachInterrupt(digitalPinToInterrupt(3),switch_handler,RISING);
}

void rotary_encoder(){
  int VB = digitalRead(clkPin);
  int VA = digitalRead(DTPin);
  if (VA==VB) {
    --encoder_pos;
  }
  else{
    ++encoder_pos;
  }
}

void switch_handler()
{
  sw_button=true;
}
  
