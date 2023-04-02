////////////////////////////////////////////////////////////////////////////////
//////////////////////// BRANCHEMENTS /////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

/*
 * 
 * clk ------- 1
 * DT  ------- 2
 * SW  ------- 3
 *  +  ------- +5V
 * GND ------- GND
 * 
 */


///////////////////////////////////////////////////////////////////////////////
//////////////////////// MAIN CODE // /////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x20,20,4) ;


#define clkPin 1
#define DTPin 2
#define SwitchPin 3
volatile int encoder_pos = 50;
int old_encoder_pos = 0;

bool sw_button = false;

int mesured_temp = 69;
 

uint32_t prec_time;

void setup() {
  // put your setup code here, to run once:
  lcd.init();
  Serial.begin(9600);
  pinMode(clkPin,INPUT_PULLUP);
  pinMode(DTPin,INPUT_PULLUP);
  pinMode(SwitchPin,INPUT_PULLUP);
  attachInterrupt(0,rotary_encoder,CHANGE);
  attachInterrupt(digitalPinToInterrupt(3),switch_handler,RISING);

  prec_time = millis();

  lcd.backlight();
  lcd.clear();
  lcd.print("Target:50 C");
  lcd.setCursor(0,1);
  lcd.print("Sensor:** C");

}

void loop() {
  // put your main code here, to run repeatedly:
  if (encoder_pos!=old_encoder_pos)
  {
    old_encoder_pos = encoder_pos;
    Serial.println(encoder_pos);
    write_target_temp();
  }

  if(sw_button)
  {
    sw_button=false;
    Serial.println("BOUTTON !!");
  }

  if (millis() - prec_time > 2000)
  {
    write_sensor_temp();
    
    prec_time = millis();
  }
}




///////////////////////////////////////////////////////////////////////////////
//////////////////////// USEFUL FUNCTION //////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


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

void write_target_temp()
{
  lcd.setCursor(7,0);
  String message = String(encoder_pos)+" C        ";
  lcd.print(message);
}

void write_sensor_temp()
{
  
  lcd.setCursor(7,1);
  String message = String(mesured_temp)+" C        ";
  lcd.print(message);
}
