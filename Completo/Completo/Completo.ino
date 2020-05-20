#include "Wire.h"
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>


// Define um display 2 linhas por 12 coluna
LiquidCrystal_I2C lcd(0x27, 16, 2);

//para o sensor de temperatura
#define temperatura_pin 13
OneWire oneWire(temperatura_pin);
DallasTemperature sensors(&oneWire);
float Celcius=0;


//Contantes
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
  sensors.requestTemperatures(); 

  

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


  delay(5000);

  
}

void loop()
{
  //Liga o led dos botões
  digitalWrite(buttonLed,HIGH);
  
  Temperatura();

  
  //Chama a função dos motores traseiros
  ControloMotoresTraseirosPotenciometro(analogRead(A1),  analogRead(A0));

  //Chama a função do motor central
  ControloMotorCentralPotenciometro(digitalRead(buttonUP),digitalRead(buttonDown));

  //Delay para obter dados
  delay(50);
}


void ControloMotorCentralPotenciometro (int estadoSubida, int estadoDescida)
{
  //Le o potenciometro
  velocidade = analogRead(A2);
  //Converte para as unidades do motor (0-255)
  velocidade = velocidade*0.23; 
  analogWrite(MotorC_ENA,velocidade);// injeta a velocidade no motor
  
  //Le o potenciometro
  velocidade = analogRead(A2);
  //Converte para as unidades do motor (0-255)
  velocidade = velocidade*0.23; 
    if (estadoSubida == LOW) {  //botão clicado
      digitalWrite(MotorC_IN1, LOW);
      digitalWrite(MotorC_IN2, HIGH);
      
      lcd.setCursor(0,0); //Posiciona para escrever
      lcd.print("          "); //escreve
      lcd.setCursor(0,0); //Posiciona para escrever
      lcd.print("Subir"); //escreve

      }
     if (estadoDescida == LOW) {  //botão clicado
      digitalWrite(MotorC_IN1, HIGH);
      digitalWrite(MotorC_IN2, LOW);
      
      lcd.setCursor(0,0); //Posiciona para escrever
      lcd.print("          "); //escreve
      lcd.setCursor(0,0); //Posiciona para escrever
      lcd.print("Descer"); //escreve
      }

   if  (estadoSubida != LOW && estadoDescida != LOW){ //nenhum clicado, motor desligado
      digitalWrite(MotorC_IN1, LOW);
      digitalWrite(MotorC_IN2, LOW);
      lcd.setCursor(0,0); //Posiciona para escrever
      lcd.print("          "); //escreve
      lcd.setCursor(0,0); //Posiciona para escrever
      lcd.print("Neutro"); //escreve
      
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

  if ((eixoX > 460 && eixoX < 564) && (eixoY > 460 && eixoY < 564)) // neutro
  {
    digitalWrite(MotorA_IN1, LOW);
    digitalWrite(MotorA_IN2, LOW);
    digitalWrite(MotorB_IN3, LOW);
    digitalWrite(MotorB_IN4, LOW);

    lcd.setCursor(0,1); //Posiciona para escrever
    lcd.print("            "); //escreve
    lcd.setCursor(0,1); //Posiciona para escrever
    lcd.print("Neutro"); //escreve

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
      lcd.print("Recuar"); //escreve


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
      lcd.print("Avançar"); //escreve
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
      lcd.print("Avançar / es"); //escreve
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
      lcd.print("Avançar / dr"); //escreve
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
      lcd.print("Recuar / es"); //escreve
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
      lcd.print("Recuar / dr"); //escreve
    }

  }

 
}

void Temperatura()
{
  sensors.requestTemperatures(); 
  Celcius=sensors.getTempCByIndex(0);
  if(Celcius != -127) //pode aparecer -127 quando os motores estão a trabalhar a alta velocidade
  {
  lcd.setCursor(11,0); //Posiciona para escrever
  lcd.print(Celcius);
  lcd.setCursor(15,0);
  lcd.print("C");
  }

}
