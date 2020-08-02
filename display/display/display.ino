#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "MPU9250.h"
#include <DallasTemperature.h>


// Define um display de 2 linhas por 16 colunas
LiquidCrystal_I2C lcd(0x27, 16, 2);
//para o sensor de temperatura
#define temperatura_pin 13
OneWire oneWire(temperatura_pin);
DallasTemperature sensors(&oneWire);
float Celcius=0;
void setup()
{
    // serial para a consola (testes)
  Serial.begin(38400);
    //Temperatura 
  sensors.begin();
  sensors.requestTemperatures(); 
  delay(1000);
    // Inicia o I2C
  Wire.begin(); 

  delay(4000);

}

void loop()
{
  
  Temperatura();

}

void Temperatura()
{
  sensors.requestTemperatures(); 
  Celcius=sensors.getTempCByIndex(0);
  if(Celcius != -127) //pode aparecer -127 quando os motores estão a trabalhar a alta velocidade
  {
  lcd.setCursor(11,0); //Posiciona para escrever
  Serial.println(Celcius);
  lcd.print(Celcius);
  lcd.setCursor(15,0);
  lcd.print("C");
  }

}
