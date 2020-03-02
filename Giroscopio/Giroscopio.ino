
//Imports
#include "MPU9250.h"

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
  // serial to display data
  Serial.begin(9600);
  while (!Serial)
  {

  }
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
  // read the sensor
  IMU.readSensor();

  // display the data
  //Serial.println(IMU.getAccelX_mss(),6);
  //Serial.println(IMU.getAccelY_mss(),6);
  //Serial.print("\t");
  //Serial.print(IMU.getAccelY_mss(),6);
  //Serial.print("\t");
  //Serial.print(IMU.getAccelZ_mss(),6);
  /*Serial.print("\t");
  Serial.print(IMU.getGyroX_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroY_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroZ_rads(),6);*/

//Para saber a orientação do ROV
  int posicaox = IMU.getMagX_uT();
  int posicaoy = IMU.getMagY_uT();
  //NORTE
  if (posicaox < RefNorte)
  {
    Serial.println("Norte");
  }
  //OESTE e ESTE
  else if (posicaox <= RefSul && posicaox >=RefNorte)
  {
    //Oeste
    if (posicaoy > RefOeste)
    {
      Serial.println("Oeste");
    }
    //este
    else
    {
      Serial.println("Este");
    }
  }
  //SUL
  else if (posicaox > RefSul)
  {
    Serial.println("Sul");
  }

  //Delay para obter dados
  delay(500);
}