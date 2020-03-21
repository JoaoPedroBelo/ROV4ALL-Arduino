#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "MPU9250.h"


// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);
//giroscopio
MPU9250 IMU(Wire, 0x68);


//Contantes
//VALORES DE REFERENCIA RETIRADOS DE INUMEROS TESTES, valores do sensor
//EIXO DO X
#define RefSul 35
#define RefNorte 4
//EIXO do Y
#define RefOeste 25

int status;

void setup()
{
  // initialize the LCD
  lcd.begin();
  // Turn on the blacklight and print a message.
  lcd.backlight();
  //Default data
  lcd.print("a iniciar");
  // serial to display data
  Serial.begin(9600);
  // start communication with IMU
  status = IMU.begin();
  if (status < 0)
  {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1)
    {
    }
  }
  // setting the accelerometer full scale range to +/-8G
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);
  
}

void loop()
{

  //lcd.print("teste!");
  IMU.readSensor();
  //Para saber a orientação do ROV
  int posicaox = IMU.getMagX_uT();
  int posicaoy = IMU.getMagY_uT();

  //Chama a função da bossula
  Bossula(posicaox,posicaoy);

  //Delay para obter dados
  delay(300);

}

int Bossula(float posicaox, float posicaoy)
{

  //NORTE
  if (posicaox < RefNorte)
  {
    lcd.setCursor(2, 5);
    lcd.clear();
    lcd.print("Norte");
  }
  //OESTE e ESTE
  else if (posicaox <= RefSul && posicaox >= RefNorte)
  {
    //Oeste
    if (posicaoy > RefOeste)
    {
       lcd.setCursor(2, 5);
       lcd.clear();
       lcd.print("Este");
    }
    //este
    else
    {
       lcd.setCursor(2, 5);
       lcd.clear();
       lcd.print("Oeste");
    }
  }
  //SUL
  else if (posicaox > RefSul)
  {
     lcd.setCursor(2, 5);
     lcd.clear();
     lcd.print("Sul");
  }
}
