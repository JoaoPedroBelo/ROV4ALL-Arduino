#include "Wire.h"
#include <LiquidCrystal_I2C.h>
#include "I2Cdev.h"
#include "MPU9250.h"
#include <OneWire.h>
#include <DallasTemperature.h>


// Define um display 2 linhas por 12 coluna
LiquidCrystal_I2C lcd(0x27, 16, 2);
//para o giroscpio
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

//para o sensor detemperatura
#define temperatura_pin 13
OneWire oneWire(temperatura_pin);
DallasTemperature sensors(&oneWire);
float Celcius=0;


//Contantes
//Valor de referência para a bussola
//EIXO DO X
#define RefSul 35
#define RefNorte 4

//EIXO do Y
#define RefOeste 25

//Motores
// Motor A -> Lado direito
int MotorA_IN1 = 4;
int MotorA_IN2 = 5;
// Motor B -> Lado esquerdo
int MotorB_IN3 = 2;
int MotorB_IN4 = 3;

//Subida e Descida
int MotorC_IN1 = 7;
int MotorC_IN2 = 8;

//Velocidade
int MotorA_ENA = 6;// PIN PWM (~)
int MotorB_ENB = 9;
int MotorC_ENA = 10;
int velocidade =0;

//botões
int buttonUP = 12;
int buttonDown = 11;

//led botões
int buttonLed= A3;

void setup()
{
  // inicia o LCD
  lcd.begin();
  // Liga a luz
  lcd.backlight();
  
  //Temperatura 
  sensors.begin();

  //Mensagem de inicio
  lcd.print("a iniciar");
  
  // Inicia o I2C
  Wire.begin(); 

  // serial para a consola (testes)
  Serial.begin(38400);

  //Motores
  pinMode(MotorA_IN1, OUTPUT); //Motor A
  pinMode(MotorA_IN2, OUTPUT); //Motor A
  pinMode(MotorB_IN3, OUTPUT); //Motor B
  pinMode(MotorB_IN4, OUTPUT); //Motor B
  pinMode(MotorC_IN1, OUTPUT); //Motor Subida / Descida
  pinMode(MotorC_IN2, OUTPUT); //Motor Subida / Descida
  pinMode(MotorA_ENA, OUTPUT); //Velocidade do motor
  pinMode(MotorB_ENB, OUTPUT); //Velocidade do motor
  pinMode(MotorC_ENA, OUTPUT); //Velocidade do motor 

  //Botão
  pinMode(buttonUP, INPUT);
  pinMode(buttonDown, INPUT);
  pinMode(buttonLed, OUTPUT);

  // A iniciar I2C
  Serial.println("a iniciar I2C....");
  accelgyro.initialize();

  // verifica as ligacoes
  Serial.println("Testando ligacoes...");
  Serial.println(accelgyro.testConnection() ? "MPU9250 ligado com sucesso" : "MPU9250 ligacao falhou");
  
  delay(1000);
  
}

void loop()
{
  //Liga o led dos botões
  digitalWrite(buttonLed,HIGH);

  //lê giroscopio
  getAccel_Data();
  getGyro_Data();
  getCompassDate_calibrated(); // calibra o compass
  
  //Dados para analisar aceleração Posteriormente
  Serial.println("Acceleration(g) of X,Y,Z:");
  Serial.print(Axyz[0]); 
  Serial.print(",");
  Serial.print(Axyz[1]); 
  Serial.print(",");
  Serial.println(Axyz[2]);
  
  //Chama a função dos motores traseiros
  ControloMotoresTraseirosPotenciometro(analogRead(A1),  analogRead(A0));

  //Chama a função do motor central
  ControloMotorCentralPotenciometro(digitalRead(buttonUP),digitalRead(buttonDown));

  //Chama a função da bússola
  bussola(Mxyz[0], Mxyz[1]);

  //Chama a função da temperatura
  Temperatura();

  //Delay para obter dados
  delay(100);
}


void bussola(float posicaoX, float posicaoY)
{
  lcd.setCursor(0,0); //Posiciona para escrever
  lcd.print("         "); //Apaga

  //NORTE
  if (posicaoX < RefNorte)
  {
    lcd.setCursor(0,0); //Posiciona para escrever
    lcd.print("Norte"); //escreve
  }

  //OESTE e ESTE
  else if (posicaoX <= RefSul && posicaoX >= RefNorte)
  {

    //Oeste
    if (posicaoY > RefOeste)
    {
      lcd.setCursor(0,0); //Posiciona para escrever
      lcd.print("Este"); //escreve
    }

    //este
    else
    {
      lcd.setCursor(0,0); //Posiciona para escrever
      lcd.print("Oeste"); //escreve
    }
  }

  //SUL
  else if (posicaoX > RefSul)
  {
    lcd.setCursor(0,0); //Posiciona para escrever
    lcd.print("Sul"); //escreve
  }
}

void ControloMotorCentralPotenciometro (int estadoSubida, int estadoDescida)
{
  //Le o potenciometro
  velocidade = analogRead(A2);
  //Converte para as unidades do motor (0-255)
  velocidade = velocidade*0.23; 
  analogWrite(MotorC_ENA,velocidade);// injecta a velocidade no motor
  
  //Le o potenciometro
  velocidade = analogRead(A2);
  //Converte para as unidades do motor (0-255)
  velocidade = velocidade*0.23; 
    if (estadoSubida == LOW) {  //botao clicado
      digitalWrite(MotorC_IN1, LOW);
      digitalWrite(MotorC_IN2, HIGH);
      
      lcd.setCursor(0,1); //Posiciona para escrever
      lcd.print("          "); //escreve
      lcd.setCursor(0,1); //Posiciona para escrever
      lcd.print("Subir"); //escreve

      }
     if (estadoDescida == LOW) {  //botao clicado
      digitalWrite(MotorC_IN1, HIGH);
      digitalWrite(MotorC_IN2, LOW);
      
      lcd.setCursor(0,1); //Posiciona para escrever
      lcd.print("          "); //escreve
      lcd.setCursor(0,1); //Posiciona para escrever
      lcd.print("Descer"); //escreve
      }

   if  (estadoSubida != LOW && estadoDescida != LOW){ //nenhum clicado, motor desligado
      digitalWrite(MotorC_IN1, LOW);
      digitalWrite(MotorC_IN2, LOW);
  }
  
}


void ControloMotoresTraseirosPotenciometro(int eixoX, int eixoY)
{
  lcd.setCursor(12,1); //Posiciona para escrever
  lcd.print("    "); //limpa
  //Le o potenciometro
  velocidade = analogRead(A2);
  //Converte para as unidades do motor (0-255)
  velocidade = velocidade*0.23; 
  analogWrite(MotorA_ENA,velocidade);// injecta a velocidade no motor
  analogWrite(MotorB_ENB,velocidade);
  
  int VelocidadePerc = map(velocidade, 0, 220, 0, 100); //Velocidade em percentagem
  if(VelocidadePerc>100){
    VelocidadePerc=100;
  }
  //Apenas cosmetico
  if(VelocidadePerc == 100){
      lcd.setCursor(12,1); //Posiciona para escrever
  }
  else if (VelocidadePerc > 10){
    lcd.setCursor(13,1); //Posiciona para escrever
  }
  else if (VelocidadePerc < 10){
    lcd.setCursor(14,1); //Posiciona para escrever
  }
  lcd.print(VelocidadePerc); //Imprima velocidade percentagem
  lcd.setCursor(15,1); //Posiciona para escrever
  lcd.print("%"); //Imprima velocidade percentagem

  Serial.println(VelocidadePerc); //Velocidade para ecra
  if ((eixoX > 460 && eixoX < 564) && (eixoY > 460 && eixoY < 564)) // neutro
  {
    digitalWrite(MotorA_IN1, LOW);
    digitalWrite(MotorA_IN2, LOW);
    digitalWrite(MotorB_IN3, LOW);
    digitalWrite(MotorB_IN4, LOW);

    lcd.setCursor(0,1); //Posiciona para escrever
    lcd.print("          "); //escreve
    lcd.setCursor(0,1); //Posiciona para escrever
    lcd.print("parado"); //escreve

  }
  else
  {
    //Trás
    if (eixoY < 460 && (eixoX > 460 && eixoX < 564))
    {
      //Activa os motores
      digitalWrite(MotorA_IN1, LOW);
      digitalWrite(MotorA_IN2, HIGH);
      digitalWrite(MotorB_IN3, LOW);
      digitalWrite(MotorB_IN4, HIGH);

      lcd.setCursor(0,1); //Posiciona para escrever
      lcd.print("          "); //escreve
      lcd.setCursor(0,1); //Posiciona para escrever
      lcd.print("Tras"); //escreve


    }
    //Frente
    else if (eixoY > 564 && (eixoX > 400 && eixoX < 600))
    {
      digitalWrite(MotorA_IN1, HIGH);
      digitalWrite(MotorA_IN2, LOW);
      digitalWrite(MotorB_IN3, HIGH);
      digitalWrite(MotorB_IN4, LOW);
      lcd.setCursor(0,1); //Posiciona para escrever
      lcd.print("          "); //escreve
      lcd.setCursor(0,1); //Posiciona para escrever
      lcd.print("Frente"); //escreve
    }

    //Esquerda Frente
    else if (eixoX < 400 && eixoY > 512)
    {
      digitalWrite(MotorA_IN1, HIGH);
      digitalWrite(MotorA_IN2, LOW);
      digitalWrite(MotorB_IN3, LOW);
      digitalWrite(MotorB_IN4, LOW);
      lcd.setCursor(0,1); //Posiciona para escrever
      lcd.print("          "); //escreve
      lcd.setCursor(0,1); //Posiciona para escrever
      lcd.print("fr/es"); //escreve
    }

    //Direita Frente
    else if (eixoX > 564 && eixoY > 512)
    {
      digitalWrite(MotorA_IN1, LOW);
      digitalWrite(MotorA_IN2, LOW);
      digitalWrite(MotorB_IN3, HIGH);
      digitalWrite(MotorB_IN4, LOW);
      lcd.setCursor(0,1); //Posiciona para escrever
      lcd.print("      "); //escreve
      lcd.setCursor(0,1); //Posiciona para escrever
      lcd.print("fr/dr"); //escreve
    }

    //Esquerda Trás
    else if (eixoX < 400 && eixoY < 512)
    {
      digitalWrite(MotorA_IN1, LOW);
      digitalWrite(MotorA_IN2, HIGH);
      digitalWrite(MotorB_IN3, LOW);
      digitalWrite(MotorB_IN4, LOW);
      lcd.setCursor(0,1); //Posiciona para escrever
      lcd.print("          "); //escreve
      lcd.setCursor(0,1); //Posiciona para escrever
      lcd.print("tr/es"); //escreve
    }

    //Direita Trás
    else if (eixoX > 564 && eixoY < 512)
    {
      digitalWrite(MotorA_IN1, LOW);
      digitalWrite(MotorA_IN2, LOW);
      digitalWrite(MotorB_IN3, LOW);
      digitalWrite(MotorB_IN4, HIGH);
      lcd.setCursor(0,1); //Posiciona para escrever
      lcd.print("          "); //escreve
      lcd.setCursor(0,1); //Posiciona para escrever
      lcd.print("tr/dr"); //escreve
    }

  }

 
}

void Temperatura()
{
  sensors.requestTemperatures(); 
  Celcius=sensors.getTempCByIndex(0);
  if(Celcius != -127) //Acontece dar -127 quando os motores estao a trabalhar a alta velocidade
  {
  lcd.setCursor(11,0); //Posiciona para escrever
  lcd.print(Celcius);
  lcd.setCursor(15,0);
  lcd.print("C");
  }

}

//****** Funções da biblioteca do giroscopio, mais informação http://wiki.seeedstudio.com/Grove-IMU_9DOF_v2.0/
void Mxyz_init_calibrated ()
{

  Serial.println(F("Before using 9DOF,we need to calibrate the compass frist,It will takes about 2 minutes."));
  Serial.print("  ");
  Serial.println(F("During  calibratting ,you should rotate and turn the 9DOF all the time within 2 minutes."));
  Serial.print("  ");
  Serial.println(F("If you are ready ,please sent a command data 'ready' to start sample and calibrate."));
  while(!Serial.find("ready")); 
  Serial.println("  ");
  Serial.println("ready");
  Serial.println("Sample starting......");
  Serial.println("waiting ......");
  
  get_calibration_Data ();
  
  Serial.println("     ");
  Serial.println("compass calibration parameter ");
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
      /*
      Serial.print(mx_sample[2]);
      Serial.print(" ");
      Serial.print(my_sample[2]);                            //you can see the sample data here .
      Serial.print(" ");
      Serial.println(mz_sample[2]);
      */
      if (mx_sample[2]>=mx_sample[1])mx_sample[1] = mx_sample[2];     
      if (my_sample[2]>=my_sample[1])my_sample[1] = my_sample[2]; //find max value      
      if (mz_sample[2]>=mz_sample[1])mz_sample[1] = mz_sample[2];   
      
      if (mx_sample[2]<=mx_sample[0])mx_sample[0] = mx_sample[2];
      if (my_sample[2]<=my_sample[0])my_sample[0] = my_sample[2];//find min value
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
  Axyz[0] = (double) ax / 16384;//16384  LSB/g
  Axyz[1] = (double) ay / 16384;
  Axyz[2] = (double) az / 16384; 
}

void getGyro_Data(void)
{
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Gxyz[0] = (double) gx * 250 / 32768;//131 LSB(��/s)
  Gxyz[1] = (double) gy * 250 / 32768;
  Gxyz[2] = (double) gz * 250 / 32768;
}

void getCompass_Data(void)
{
  I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
  delay(10);
  I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);
  
    mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0] ;
  my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2] ;
  mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4] ;  
  
  //Mxyz[0] = (double) mx * 1200 / 4096;
  //Mxyz[1] = (double) my * 1200 / 4096;
  //Mxyz[2] = (double) mz * 1200 / 4096;
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
