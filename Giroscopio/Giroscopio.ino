#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9250.h"

MPU9250 accelgyro;
I2Cdev   I2C_M;
uint8_t buffer_m[6];
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t   mx, my, mz;
float heading;
float tiltheading;
float Axyz[3];
float Gxyz[3];
float Mxyz[3];


#define sample_num_mdate  5000      

volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];

static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;

volatile int mx_max =0;
volatile int my_max =0;
volatile int mz_max =0;

volatile int mx_min =0;
volatile int my_min =0;
volatile int mz_min =0;


void setup() {
  Wire.begin();

  //Verificar sempre se a consola esta no mesmo valor que abaixo, neste caso é o 38400
  Serial.begin(38400);

  // Inicia os dispositivos I2C
  Serial.println("a iniciar dispositivos2C");
  accelgyro.initialize();

  // verifica a ligação
  Serial.println("a testar as ligações...");
  Serial.println(accelgyro.testConnection() ? "MPU9250 Ligado com sucesso" : "MPU9250 verifique as ligações");
  
  delay(1000);
  Serial.println("     ");
 
 //Usar esta função apenas se os dados estiverem bastante errados pois esta função irá calibrar o sensor
  //Mxyz_init_calibrated() 
}

void loop() 
{   
  
  getAccel_Data();
  getGyro_Data();
  getCompassDate_calibrated(); // A bússola é calibrada aqui
  getHeading();       //Antes de utilizar esta função é necessário efetuar a chamada da função getCompassDate_calibrated    
  getTiltHeading();           
  
  Serial.println("Parametros de calibração ");
  Serial.print(mx_centre);
  Serial.print("         ");
  Serial.print(my_centre);
  Serial.print("         ");
  Serial.println(mz_centre);
  Serial.println("     ");
  
  
  Serial.println("aceleração(g) de X,Y,Z:");
  Serial.print(Axyz[0]); 
  Serial.print(",");
  Serial.print(Axyz[1]); 
  Serial.print(",");
  Serial.println(Axyz[2]); 
  Serial.println("giro (graus) de X,Y,Z:");
  Serial.print(Gxyz[0]); 
  Serial.print(",");
  Serial.print(Gxyz[1]); 
  Serial.print(",");
  Serial.println(Gxyz[2]); 
  Serial.println("bússola de X,Y,Z:");
  Serial.print(Mxyz[0]); 
  Serial.print(",");
  Serial.print(Mxyz[1]); 
  Serial.print(",");
  Serial.println(Mxyz[2]);
  Serial.println("O ângulo no sentido horário entre o norte magnético e o eixo X:");
  Serial.print(heading);
  Serial.println(" ");
  Serial.println("O ângulo no sentido horário entre o norte magnético e a projeção do eixo X positivo no plano horizontal:");
  Serial.println(tiltheading);
  Serial.println("   ");
  Serial.println("   ");
  Serial.println("   ");

  delay(2000);
  
}


void getHeading(void)
{
  heading=180*atan2(Mxyz[1],Mxyz[0])/PI;
  if(heading <0) heading +=360;
}

void getTiltHeading(void)
{
  float pitch = asin(-Axyz[0]);
  float roll = asin(Axyz[1]/cos(pitch));

  float xh = Mxyz[0] * cos(pitch) + Mxyz[2] * sin(pitch);
  float yh = Mxyz[0] * sin(roll) * sin(pitch) + Mxyz[1] * cos(roll) - Mxyz[2] * sin(roll) * cos(pitch);
  float zh = -Mxyz[0] * cos(roll) * sin(pitch) + Mxyz[1] * sin(roll) + Mxyz[2] * cos(roll) * cos(pitch);
  tiltheading = 180 * atan2(yh, xh)/PI;
  if(yh<0)    tiltheading +=360;
}



void Mxyz_init_calibrated ()
{
  
  Serial.println(F("Antes de usar o 9DOF, precisamos calibrar a bússola primeiro, isso leva cerca de 2 minutos."));
  Serial.print("  ");
  Serial.println(F("Durante a calibração, você deve girar e girar o 9DOF o tempo todo em 2 minutos."));
  Serial.print("  ");
  Serial.println(F("Se você estiver pronto, envie um dado de comando 'ready' para iniciar a amostra e calibrar."));
  while(!Serial.find("ready")); 
  Serial.println("  ");
  Serial.println("ready");
  Serial.println("Amostra começando ......");
  Serial.println("esperando ......");
  
  get_calibration_Data ();
  
  Serial.println("     ");
  Serial.println("parâmetro de calibração da bússola ");
  Serial.print(mx_centre);
  Serial.print("     ");
  Serial.print(my_centre);
  Serial.print("     ");
  Serial.println(mz_centre);
  Serial.println("    ");
}

void get_calibration_Data ()
{
    for (int i=0; i<sample_num_mdate;i++)
      {
      get_one_sample_date_mxyz();

      
      if (mx_sample[2]>=mx_sample[1])mx_sample[1] = mx_sample[2];     
      if (my_sample[2]>=my_sample[1])my_sample[1] = my_sample[2]; //encontra o valor máximo      
      if (mz_sample[2]>=mz_sample[1])mz_sample[1] = mz_sample[2];   
      
      if (mx_sample[2]<=mx_sample[0])mx_sample[0] = mx_sample[2];
      if (my_sample[2]<=my_sample[0])my_sample[0] = my_sample[2];//encontra o valor mínimo
      if (mz_sample[2]<=mz_sample[0])mz_sample[0] = mz_sample[2];
            
      }
      
      mx_max = mx_sample[1];
      my_max = my_sample[1];
      mz_max = mz_sample[1];      
          
      mx_min = mx_sample[0];
      my_min = my_sample[0];
      mz_min = mz_sample[0];
  

  
      mx_centre = (mx_max + mx_min)/2;
      my_centre = (my_max + my_min)/2;
      mz_centre = (mz_max + mz_min)/2;  
  
}

void get_one_sample_date_mxyz()
{   
    getCompass_Data();
    mx_sample[2] = Mxyz[0];
    my_sample[2] = Mxyz[1];
    mz_sample[2] = Mxyz[2];
} 

void getAccel_Data(void)
{
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Axyz[0] = (double) ax / 16384;
  Axyz[1] = (double) ay / 16384;
  Axyz[2] = (double) az / 16384; 
}

void getGyro_Data(void)
{
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Gxyz[0] = (double) gx * 250 / 32768;
  Gxyz[1] = (double) gy * 250 / 32768;
  Gxyz[2] = (double) gz * 250 / 32768;
}

void getCompass_Data(void)
{
  I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //ativar o magnetômetro
  delay(10);
  I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);
  
    mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0] ;
  my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2] ;
  mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4] ;  
  
  Mxyz[0] = (double) mx * 4800 / 8192;
  Mxyz[1] = (double) my * 4800 / 8192;
  Mxyz[2] = (double) mz * 4800 / 8192;
}

void getCompassDate_calibrated ()
{
  getCompass_Data();
  Mxyz[0] = Mxyz[0] - mx_centre;
  Mxyz[1] = Mxyz[1] - my_centre;
  Mxyz[2] = Mxyz[2] - mz_centre;  
}
