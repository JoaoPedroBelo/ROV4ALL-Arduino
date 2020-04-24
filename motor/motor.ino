
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
int velocidadeA=0; 
int velocidadeB=0; 

//botões
int buttonUP = 12;
int buttonDown = 11;


void setup()
{
  //Verificar sempre se a consola esta no mesmo valor que abaixo, neste caso é o 38400
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

}

void loop()
{
  
  //Chama a função da motores traseiros
  ControloMotoresTraseirosPotenciometro(analogRead(A1),  analogRead(A0));
  ControloMotorCentralPotenciometro(digitalRead(buttonUP),digitalRead(buttonDown));

    //** descomentar para controlar a velocidade pelo joysick**//
  //ControloMotoresTraseirosJoystick(analogRead(A1),  analogRead(A0));
  //ControloMotorCentralJoystick(digitalRead(buttonUP),digitalRead(buttonDown));  

  //Delay para obter dados
  delay(500);


}


void ControloMotorCentralPotenciometro (int estadoSubida, int estadoDescida)
{
   //Le o potenciometro
  velocidade = analogRead(A2);
  //Converte para as unidades do motor (0-255)
  velocidade = velocidade*0.23; 
  analogWrite(MotorC_ENA,velocidade);// injeta a velocidade no motor
  
  //Lê o potenciometro
  velocidade = analogRead(A2);
  //Converte para as unidades do motor (0-255)
  velocidade = velocidade*0.23; 
    if (estadoSubida == LOW) {  //button is pressed
      digitalWrite(MotorC_IN1, LOW);
      digitalWrite(MotorC_IN2, HIGH);
      }
     if (estadoDescida == LOW) {  //button is pressed
      digitalWrite(MotorC_IN1, HIGH);
      digitalWrite(MotorC_IN2, LOW);
      }
   if  (estadoSubida != LOW && estadoDescida != LOW){
          Serial.println("largado");

      digitalWrite(MotorC_IN1, LOW);
      digitalWrite(MotorC_IN2, LOW);
  }
  
}


void ControloMotoresTraseirosPotenciometro(int eixoX, int eixoY)
{
 
  //Lê o potenciometro
  velocidade = analogRead(A2);
  //Converte para as unidades do motor (0-255)
  velocidade = velocidade*0.23; 
  analogWrite(MotorA_ENA,velocidade);// injeta a velocidade no motor
  analogWrite(MotorB_ENB,velocidade);

    if ((eixoX > 460 && eixoX < 564) && (eixoY > 460 && eixoY < 564)) // neutro

    {
    digitalWrite(MotorA_IN1, LOW);
    digitalWrite(MotorA_IN2, LOW);
    digitalWrite(MotorB_IN3, LOW);
    digitalWrite(MotorB_IN4, LOW);
     }
  else
  {
    //Trás
    if (eixoY < 460 && (eixoX > 460 && eixoX < 564))
    {
      digitalWrite(MotorA_IN1, LOW);
      digitalWrite(MotorA_IN2, HIGH);
      digitalWrite(MotorB_IN3, LOW);
      digitalWrite(MotorB_IN4, HIGH);

    }
    //Frente
    else if (eixoY > 564 && (eixoX > 400 && eixoX < 600))
    {
      digitalWrite(MotorA_IN1, HIGH);
      digitalWrite(MotorA_IN2, LOW);
      digitalWrite(MotorB_IN3, HIGH);
      digitalWrite(MotorB_IN4, LOW);

    }

    //Esquerda Frente
    else if (eixoX < 400 && eixoY > 512)
    {
      digitalWrite(MotorA_IN1, HIGH);
      digitalWrite(MotorA_IN2, LOW);
      digitalWrite(MotorB_IN3, LOW);
      digitalWrite(MotorB_IN4, LOW);

    }

    //Direita Frente
    else if (eixoX > 564 && eixoY > 512)
    {
      digitalWrite(MotorA_IN1, LOW);
      digitalWrite(MotorA_IN2, LOW);
      digitalWrite(MotorB_IN3, HIGH);
      digitalWrite(MotorB_IN4, LOW);

    }

    //Esquerda Trás
    else if (eixoX < 400 && eixoY < 512)
    {
      digitalWrite(MotorA_IN1, LOW);
      digitalWrite(MotorA_IN2, HIGH);
      digitalWrite(MotorB_IN3, LOW);
      digitalWrite(MotorB_IN4, LOW);

    }

    //Direita Trás
    else if (eixoX > 564 && eixoY < 512)
    {
      digitalWrite(MotorA_IN1, LOW);
      digitalWrite(MotorA_IN2, LOW);
      digitalWrite(MotorB_IN3, LOW);
      digitalWrite(MotorB_IN4, HIGH);

    }

  }
}

void ControloMotorCentralJoystick(int estadoSubida, int estadoDescida){
   //Lê o potenciometro
  velocidade = 255; //não é possível ajustar a velocidade
  analogWrite(MotorC_ENA,velocidade);// injeta a velocidade no motor
     //Clicado para subir
    if (estadoSubida == LOW) { 
      digitalWrite(MotorC_IN1, LOW);
      digitalWrite(MotorC_IN2, HIGH);
      }
      //Clicado para descer
     if (estadoDescida == LOW) {  
      digitalWrite(MotorC_IN1, HIGH);
      digitalWrite(MotorC_IN2, LOW);
      }
   if  (estadoSubida != LOW && estadoDescida != LOW){

      digitalWrite(MotorC_IN1, LOW);
      digitalWrite(MotorC_IN2, LOW);
  }
  
}

void ControloMotoresTraseirosJoystick(int eixoX, int eixoY)
{

  if ((eixoX > 460 && eixoX < 564) && (eixoY > 460 && eixoY < 564)) // neutro
  {
    digitalWrite(MotorA_IN1, LOW);
    digitalWrite(MotorA_IN2, LOW);
    digitalWrite(MotorB_IN3, LOW);
    digitalWrite(MotorB_IN4, LOW);
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
      //Velocidade, como estamos a ir para trás temos de inverter as leituras
      eixoY = eixoY - 460; // Número fica negativo
      eixoY = eixoY * -1;  // Torna o número positivo, uma velocidade não é negativa.
      velocidadeA = map(eixoY, 0, 460, 0, 230);
      velocidadeB = map(eixoY, 0, 460, 0, 230);   
    }
    //Frente
    else if (eixoY > 564 && (eixoX > 400 && eixoX < 600))
    {
      digitalWrite(MotorA_IN1, HIGH);
      digitalWrite(MotorA_IN2, LOW);
      digitalWrite(MotorB_IN3, HIGH);
      digitalWrite(MotorB_IN4, LOW);
      velocidadeA = map(eixoY, 512, 790, 0, 230);
      velocidadeB = map(eixoY, 512, 790, 0, 230);

    }

    //Esquerda Frente
    else if (eixoX < 400 && eixoY > 512)
    {
      digitalWrite(MotorA_IN1, HIGH);
      digitalWrite(MotorA_IN2, LOW);
      digitalWrite(MotorB_IN3, HIGH);
      digitalWrite(MotorB_IN4, LOW);
      velocidade = map(eixoX, 0, 400, 0, 230);
      velocidadeA = velocidade + eixoX;
      velocidadeB = velocidade - eixoX;

      if (velocidadeA > 255)velocidadeA = 255;
      if (velocidadeB < 0) velocidadeB = 0;		  


    }

    //Direita Frente
    else if (eixoX > 564 && eixoY > 512)
    {
      digitalWrite(MotorA_IN1, HIGH);
      digitalWrite(MotorA_IN2, LOW);
      digitalWrite(MotorB_IN3, HIGH);
      digitalWrite(MotorB_IN4, LOW);
      velocidade = map(eixoX, 564, 790, 0, 230);
      velocidadeA = velocidade - eixoX;
      velocidadeB = velocidade + eixoX;

      if (velocidadeB > 255)velocidadeA = 255;
      if (velocidadeA < 0) velocidadeB = 0;		 

    }

    //Esquerda Trás
    else if (eixoX < 400 && eixoY < 512)
    {
      digitalWrite(MotorA_IN1, LOW);
      digitalWrite(MotorA_IN2, HIGH);
      digitalWrite(MotorB_IN3, LOW);
      digitalWrite(MotorB_IN4, HIGH);
      velocidade = map(eixoX, 0, 400, 0, 230);
      velocidadeA = velocidade + eixoX;
      velocidadeB = velocidade - eixoX;

      if (velocidadeA > 255)velocidadeA = 255;
      if (velocidadeB < 0) velocidadeB = 0;		 

    }

    //Direita Trás
    else if (eixoX > 564 && eixoY < 512)
    {
      digitalWrite(MotorA_IN1, LOW);
      digitalWrite(MotorA_IN2, HIGH);
      digitalWrite(MotorB_IN3, LOW);
      digitalWrite(MotorB_IN4, HIGH);
      velocidade = map(eixoX, 564, 790, 0, 230);
      velocidadeA = velocidade - eixoX;
      velocidadeB = velocidade + eixoX;

      if (velocidadeB > 255)velocidadeA = 255;
      if (velocidadeA < 0) velocidadeB = 0;	


    }
  }
      // injeta a velocidade no motor
        analogWrite(MotorA_ENA,velocidadeA);
        analogWrite(MotorB_ENB,velocidadeB);
}
