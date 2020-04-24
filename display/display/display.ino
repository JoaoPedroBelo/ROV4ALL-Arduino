#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "MPU9250.h"


// Define um display de 2 linhas por 16 colunas
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup()
{
  // Inicia o LCD
  lcd.begin();

  // Liga a luz do LCD
  lcd.backlight();

}

void loop()
{
    //Posiciona o cursor
    lcd.setCursor(0, 0);
    //Escreve
    lcd.print("Hello, world!");

}
