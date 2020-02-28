
#include "MPU9250.h"


// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;
int calibrate;
void setup() {
  // serial to display data
  Serial.begin(38400);
  while(!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  //calibrate
  IMU.calibrateAccel();
  IMU.calibrateMag();


}

void loop() {
  // read the sensor
  IMU.readSensor();
  // display the data
  //se o x é positivo é para a frente se o X é negativo é para tras
  //Serial.println(IMU.getAccelX_mss(),6);
  //Serial.print("\t");
  //Serial.println(IMU.getAccelY_mss(),6);
  //Serial.print("\t");
  //Serial.println(IMU.getAccelZ_mss(),6);
  //Serial.print("\t");
  //Serial.println(IMU.getGyroX_rads(),6);
  //Serial.print("\t");
  //Serial.println(IMU.getGyroY_rads(),6);
  //Serial.print("\t");
  //Serial.println(IMU.getGyroZ_rads(),6);
  //Serial.print("\t");

  //NORTE -20
  //SUL 57
  //ESTE 10
  //OESTE 25
  Serial.println(IMU.getMagX_uT(),6);
  //Serial.print("\t");
  //Serial.print(IMU.getMagY_uT(),6);
  //Serial.print("\t");
  //Serial.print(IMU.getMagZ_uT(),6);
  //Serial.print("\t");
  //Serial.println(IMU.getTemperature_C(),6);


   
  delay(100);
}
