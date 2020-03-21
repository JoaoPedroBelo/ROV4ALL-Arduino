#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "MPU9250.h"


// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);
//Contantes
//VALORES DE REFERENCIA RETIRADOS DE INUMEROS TESTES, valores do sensor
//EIXO DO X
#define RefSul 40
#define RefNorte 4
//EIXO do Y
#define RefOeste 30

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);
int status;

void setup()
{
  // initialize the LCD
  lcd.begin();

  // Turn on the blacklight and print a message.
  lcd.backlight();
  lcd.print("Hello, world!");
}

void loop()
{
  // Do nothing here...
    //lcd.print("teste!");
    lcd.setCursor(2, 5);
      lcd.print("1");


}
