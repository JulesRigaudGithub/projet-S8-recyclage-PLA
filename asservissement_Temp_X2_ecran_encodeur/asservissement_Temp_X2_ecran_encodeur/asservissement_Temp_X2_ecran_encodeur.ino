/*************************************************** 
  This is a library for the Adafruit PT100/P1000 RTD Sensor w/MAX31865

  Designed specifically to work with the Adafruit RTD Sensor
  ----> https://www.adafruit.com/products/3328

  This sensor uses SPI to communicate, 4 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/


///////////////////////////////////////////////////////////////////////////////
//////////////////////// BRANCHEMENTS /////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

/*Capteur temp 1 (rouge)
 * 
 * CS  ------------- 10
 * SDI ------------- 11
 * SD0 ------------- 12
 * CLK ------------- 13
 * 
 */

/*Capteur temp 2 (bleu)
 * 
 * CS  ------------- 9
 * SDI ------------- 11
 * SD0 ------------- 12
 * CLK ------------- 13
 * 
 */

 /*
  * Commande 1 ------- 5
  * Commande 2 ------- 6
  * 
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


////////////////////////////////// Max31865  //////////////////////////////////
#include <Adafruit_MAX31865.h>
// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo1 = Adafruit_MAX31865(10, 11, 12, 13);
Adafruit_MAX31865 thermo2 = Adafruit_MAX31865(9, 11, 12, 13);
// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0


////////////////////////////////// Display ////////////////////////////////////
uint32_t pre_displaytime = 0;
#define Delta_display 2000 // time between display in ms

uint32_t pre_dynamic_displaytime = 0;
#define Delta_dynamic_display 500 // time between display in ms

# define sensi 10 //sensibilité de l'encoseur rotatif
int active_line = 0; // 0: aucune ligne active | 1:premiere ligne active | 2:deuxieme ligne active
bool active_number = false; // if true we can change the target temperature

////////////////////////////////// Ecran LCD //////////////////////////////////
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4);



////////////////////////////////// Encodeur  //////////////////////////////////
#define clkPin 2
#define DTPin 1
#define SwitchPin 3
volatile int encoder_pos = 0;
int old_encoder_pos = 0;

bool sw_button = false;

///////////////////////////////  PWM Controle  ////////////////////////////////

#define PWM_pin_1 5
#define PWM_pin_2 6
int PWM_ratio_1 = 50; 
int PWM_ratio_2 = 50; 
#define PWM_cycle 500 //duration of a PWM cycle in ms
uint32_t pre_PWM = 0;


////////////////////////////// Asservissement /////////////////////////////////
#define asservissement_cycle 5000 // time between two calculation of the pwm consign
uint32_t pre_asservissement = 0;

#define K_p  10.0
#define K_d  0
#define K_i  0.1
//premier bloc de chauffe
int temp_1 = 0;
float T_consigne_1 = 175;
float somme_erreur_1 = 0;
float erreur_pre_1 = 0;

//deuxieme bloc de chauffe
int temp_2 = 0;
float T_consigne_2 = 150;
float somme_erreur_2 = 0;
float erreur_pre_2 = 0;



///////////////////////////////////////////////////////////////////////////////
////////////////////////   MAIN CODE  /////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////




void setup() {
  Serial.begin(115200);
  init_Max31865_1();
  init_Max31865_2();
  pinMode(PWM_pin_1,OUTPUT);
  pinMode(PWM_pin_2,OUTPUT);

  init_LCD();
  init_encodeur();
}


void loop() {



  ////// Display handling //////
  if(millis()-pre_displaytime>Delta_display)
  {
    pre_displaytime=millis();
    Serial.print("t, Temperature1, Temperature2,  PWM_ratio 1 , WM_ratio 2");
    Serial.print(millis());Serial.print(" , ");
    Serial.print(thermo1.temperature(RNOMINAL, RREF));Serial.print(" , ");
    Serial.print(thermo2.temperature(RNOMINAL, RREF));Serial.print(" , ");
    Serial.print(PWM_ratio_1);Serial.print(" , ");
    Serial.println(PWM_ratio_2);
    display_LCD();
  }

  ////// dynamic Display handling //////
  if(millis()-pre_dynamic_displaytime>Delta_dynamic_display)
  {
    dynamic_display_handler();
  }

  ////// PWM handling //////
  //detection of a new PWM cycle
  if(millis()-pre_PWM>PWM_cycle)
  {
    //asservissement_1();
    
    pre_PWM = millis();
    digitalWrite(PWM_pin_1,HIGH);
  }
  // detection of the falling edge time
  if(millis()-pre_PWM>= (float) PWM_cycle*PWM_ratio_1*0.01)  {   digitalWrite(PWM_pin_1,LOW); }
  if(millis()-pre_PWM>= (float) PWM_cycle*PWM_ratio_2*0.01)  {   digitalWrite(PWM_pin_2,LOW); }

 

  ////// Asservissement handling //////
  if(millis()-pre_asservissement>asservissement_cycle) {  pre_asservissement=millis();asservissement_1();asservissement_2(); }
  
  

}


///////////////////////////////////////////////////////////////////////////////
////////////////////////  USEFUL FUNCTION  ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////




void init_Max31865_1()
{
  Serial.println("Adafruit MAX31865 PT100 Sensor Test! (1)");
  thermo1.begin(MAX31865_2WIRE);  // set to 2WIRE or 4WIRE as necessary
  uint16_t rtd = thermo1.readRTD();

  Serial.print("RTD value: "); Serial.println(rtd);
  float ratio = rtd;
  ratio /= 32768;
  Serial.print("Ratio = "); Serial.println(ratio,8);
  Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
  Serial.print("Temperature = "); Serial.println(thermo1.temperature(RNOMINAL, RREF));

  // Check and print any faults
  uint8_t fault = thermo1.readFault();
  if (fault) {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage"); 
    }
    thermo1.clearFault();
  }
  Serial.println();
  delay(1000);
  
}



void init_Max31865_2()
{
  Serial.println("Adafruit MAX31865 PT100 Sensor Test! (2)");
  thermo2.begin(MAX31865_2WIRE);  // set to 2WIRE or 4WIRE as necessary
  uint16_t rtd = thermo2.readRTD();

  Serial.print("RTD value: "); Serial.println(rtd);
  float ratio = rtd;
  ratio /= 32768;
  Serial.print("Ratio = "); Serial.println(ratio,8);
  Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
  Serial.print("Temperature = "); Serial.println(thermo2.temperature(RNOMINAL, RREF));

  // Check and print any faults
  uint8_t fault = thermo2.readFault();
  if (fault) {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage"); 
    }
    thermo1.clearFault();
  }
  Serial.println();
  delay(1000);
  
}


void asservissement_Hysteresis()
{
  float temp = thermo1.temperature(RNOMINAL, RREF);
  if(temp<T_consigne_1-20 and PWM_ratio_1<50){PWM_ratio_1=100;}
  if(temp>T_consigne_1+20 and PWM_ratio_1>50){PWM_ratio_1=10;}
}


void asservissement_1()
{

  float temp = thermo1.temperature(RNOMINAL, RREF);
  temp_1 = temp;
  float erreur = T_consigne_1 - temp;
  float delta_erreur = erreur - erreur_pre_1;
  erreur_pre_1 = erreur;

  //determination de la commande
  PWM_ratio_1 = K_p * erreur + K_i * somme_erreur_1*float(PWM_cycle)/1000 +K_d*delta_erreur;

  // Prise en compte de la saturation de la commande
  if(PWM_ratio_1<0){PWM_ratio_1=0;somme_erreur_1+=erreur;}
  else if (PWM_ratio_1>105){PWM_ratio_1=105;} // Mettre un ratio > 100 permet de s'assurer que on ne coupe pas le courant pour le rallumer juste apres
  else {somme_erreur_1+=erreur;} // on bloque l'effet intégrale lorsque la commande sature
}


void asservissement_2()
{

  float temp = thermo2.temperature(RNOMINAL, RREF);
  temp_2 = temp;
  float erreur = T_consigne_2 - temp;
  float delta_erreur = erreur - erreur_pre_2;
  erreur_pre_2 = erreur;

  //determination de la commande
  PWM_ratio_2 = K_p * erreur + K_i * somme_erreur_2*float(PWM_cycle)/1000 +K_d*delta_erreur;

  // Prise en compte de la saturation de la commande
  if(PWM_ratio_2<0){PWM_ratio_2=0;somme_erreur_2+=erreur;}
  else if (PWM_ratio_2>105){PWM_ratio_2=105;} // Mettre un ratio > 100 permet de s'assurer que on ne coupe pas le courant pour le rallumer juste apres
  else {somme_erreur_2+=erreur;} // on bloque l'effet intégrale lorsque la commande sature
}

void init_LCD()
{  
   lcd.init(); //initialisation de l'afficheur
   lcd.backlight();
   
   lcd.setCursor(1,0);
   lcd.print("T1:     /");
   lcd.setCursor(5,0);
   lcd.print(temp_1);
   lcd.setCursor(11,0);
   lcd.print(int(T_consigne_1));

   lcd.setCursor(1,1);
   lcd.print("T2:     /");
   lcd.setCursor(5,1);
   lcd.print(temp_2);
   lcd.setCursor(11,1);
   lcd.print(int(T_consigne_2));
  
}

void display_LCD()
{
   lcd.setCursor(5,0);
   lcd.print(temp_1);
 
   lcd.setCursor(5,1);
   lcd.print(temp_2);
}

void dynamic_display_handler()
{
  if(not active_number)
  {
    //Selection des lignes
    if(encoder_pos%20<10) {  active_line=1;  lcd.setCursor(0,0); lcd.print(">");lcd.setCursor(0,1);lcd.print(" "); }
    else {  active_line=2;  lcd.setCursor(0,0); lcd.print(" ");lcd.setCursor(0,1);lcd.print(">"); }
    //Si l'utilisateur clique on passe à la selection de la température cible de la ligne désigné
    if(sw_button)
    {
      sw_button=false;
      active_number=true;
      if(active_line==1){encoder_pos=T_consigne_1*sensi;} else{encoder_pos=T_consigne_2*sensi;}
      lcd.setCursor(0,0); lcd.print(" ");lcd.setCursor(0,1);lcd.print(" ");
      lcd.setCursor(10,active_line-1);lcd.print("<");lcd.setCursor(14,active_line-1);lcd.print(">");
    }
  }
  else
  {
    if(encoder_pos/sensi<0){encoder_pos = 0;}
    if(encoder_pos/sensi>999){encoder_pos = 999*sensi;}
    lcd.setCursor(11,active_line-1);lcd.print(int(encoder_pos/sensi));
    if(sw_button)
    {
      sw_button=false;
      active_number=false;
      if(active_line==1){T_consigne_1=int(encoder_pos/sensi);} else{T_consigne_2=int(encoder_pos/sensi);}      
      lcd.setCursor(10,active_line-1);lcd.print(" ");lcd.setCursor(14,active_line-1);lcd.print(" ");
      encoder_pos=0;
    }
  }
}

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
    ++encoder_pos;
  }
  else{
    --encoder_pos;
  }
}

void switch_handler()
{
  sw_button=true;
}
  
