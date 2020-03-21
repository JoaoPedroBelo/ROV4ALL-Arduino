
//Motores
// Motor A -> Lado direito
int MotorA_IN1 = 4; //Cada motor tem 2 pins
int MotorA_IN2 = 5;
// Motor B -> Lado esquerdo
int MotorB_IN3 = 2;
int MotorB_IN4 = 3;

int PinSpeed = 6;// PIN PWM (~)
int velocidade=0;

void setup()
{
  // serial para a consola (testes)
  Serial.begin(38400);
  //Motores
  pinMode(MotorA_IN1, OUTPUT); //Motor A
  pinMode(MotorA_IN2, OUTPUT); //Motor A
  pinMode(MotorB_IN3, OUTPUT); //Motor B
  pinMode(MotorB_IN4, OUTPUT); //Motor B
  pinMode(PinSpeed, OUTPUT);  
}

void loop()
{
  //Chama a função da motores traseiros
  ControloMotoresTraseirosPotenciometro(analogRead(A1),  analogRead(A0));
  //ControloMotoresTraseirosJoystick(analogRead(A1),  analogRead(A0));
  //Delay para obter dados
  delay(500);
}


void ControloMotoresTraseirosPotenciometro(int eixoX, int eixoY)
{
 
  //Le o potenciometro
  velocidade = analogRead(A2);
  //Converte para as unidades do motor (0-255)
  velocidade = velocidade*0.23; 
  Serial.println(velocidade);
  analogWrite(PinSpeed,velocidade);// Then inject it to our motor
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
      eixoY = eixoY - 460; // Numero fica negativo
      eixoY = eixoY * -1;  // Torna o numero positivo
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
      eixoY = eixoY - 460; // Numero fica negativo
      eixoY = eixoY * -1;  // Torna o numero positivo
      velocidade = map(eixoY, 0, 460, 0, 230);

      
    }
    //Frente
    else if (eixoY > 564 && (eixoX > 400 && eixoX < 600))
    {
      digitalWrite(MotorA_IN1, HIGH);
      digitalWrite(MotorA_IN2, LOW);
      digitalWrite(MotorB_IN3, HIGH);
      digitalWrite(MotorB_IN4, LOW);
      velocidade = map(eixoY, 512, 790, 0, 230);

    }

    //Esquerda Frente
    else if (eixoX < 400 && eixoY > 512)
    {
      digitalWrite(MotorA_IN1, HIGH);
      digitalWrite(MotorA_IN2, LOW);
      digitalWrite(MotorB_IN3, LOW);
      digitalWrite(MotorB_IN4, LOW);
      velocidade = map(eixoX, 0, 400, 0, 230);
      velocidade = velocidade + eixoX;
      if (velocidade > 255)velocidade = 255;

    }

    //Direita Frente
    else if (eixoX > 564 && eixoY > 512)
    {
      digitalWrite(MotorA_IN1, LOW);
      digitalWrite(MotorA_IN2, LOW);
      digitalWrite(MotorB_IN3, HIGH);
      digitalWrite(MotorB_IN4, LOW);
      velocidade = map(eixoX, 564, 790, 0, 230);
      velocidade = velocidade + eixoX;
      if (velocidade > 255)velocidade = 255;

    }

    //Esquerda Trás
    else if (eixoX < 400 && eixoY < 512)
    {
      digitalWrite(MotorA_IN1, LOW);
      digitalWrite(MotorA_IN2, HIGH);
      digitalWrite(MotorB_IN3, LOW);
      digitalWrite(MotorB_IN4, LOW);
      velocidade = map(eixoX, 0, 400, 0, 230);
      velocidade = velocidade + eixoX;
      if (velocidade > 255)velocidade = 255;

    }

    //Direita Trás
    else if (eixoX > 564 && eixoY < 512)
    {
      digitalWrite(MotorA_IN1, LOW);
      digitalWrite(MotorA_IN2, LOW);
      digitalWrite(MotorB_IN3, LOW);
      digitalWrite(MotorB_IN4, HIGH);
      velocidade = map(eixoX, 564, 790, 0, 230);
      velocidade = velocidade + eixoX;
      if (velocidade > 255)velocidade = 255;


    }
  }
        analogWrite(PinSpeed,velocidade);// Then inject it to our motor
}

