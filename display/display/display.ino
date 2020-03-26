#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "MPU9250.h"


// Define um display de 2 linhas por 16 colunas
LiquidCrystal_I2C lcd(0x27, 16, 2);

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
    lcd.setCursor(11, 1);
    lcd.print("1");
}
