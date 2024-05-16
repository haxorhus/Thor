#include <string.h>
#include "AccelStepper.h"

 // Definicion de los pines de motores

#define dirPin1 36
#define dirPin2 34
#define dirPin3 32
#define dirPin4 30
#define dirPin5 31
#define dirPin6 33
#define dirPin7 35
#define stepPin1 28
#define stepPin2 26
#define stepPin3 24
#define stepPin4 22
#define stepPin5 23
#define stepPin6 25
#define stepPin7 27

// Definicion de los sensores

#define sensor1 42
#define sensor2 44
#define sensor3 46
#define sensor4 48
#define sensor5 49
#define sensor6 47
#define sensor7 45

// Definicion de costantes

#define ID 2
#define DEFAULT_SPEED 200
#define MAX_JOINT_NUMBER 6
#define PARAMETER_AMMOUNT 4
#define motorInterfaceType 1
#define STEPS_PER_REVOLUTION 200*16
unsigned int CONV_SPEED 400/9
#define Motor_number 4

// Relaciones

#define R1 5
#define R2 30.88
#define R3 5.18

// Banderas

bool ONE = 0;
bool TWO = 0;
bool THREE = 0;

// Motores

AccelStepper pm1 = AccelStepper(motorInterfaceType, stepPin1, dirPin1);
AccelStepper pm2 = AccelStepper(motorInterfaceType, stepPin2, dirPin2);
AccelStepper pm3 = AccelStepper(motorInterfaceType, stepPin3, dirPin3);
AccelStepper pm4 = AccelStepper(motorInterfaceType, stepPin4, dirPin4);
// arreglo de motores
//NOTA: el array empieza en 0, por lo que pm1 = pm[0]
AccelStepper pm[] = {pm1, pm2, pm3, pm4};
// Parametros globales

int joint = 0;
int targetAngle = 0;
int speed = 0;

String data;

void setup()
{
  // Configuracion
  pm[0].setMaxSpeed(150U*CONV_SPEED);
  pm[1].setMaxSpeed(150U*CONV_SPEED);
  pm[2].setMaxSpeed(150U*CONV_SPEED);
  pm[3].setMaxSpeed(150U*CONV_SPEED);


  // Alimentacion
  // Esto no se si sirve lgm, voy a comentar y probamos dps
  //pinMode(38, OUTPUT);
  //pinMode(39, OUTPUT);
  //digitalWrite(38, HIGH);
  //digitalWrite(39, HIGH);

  // Enable
  pinMode(40, OUTPUT); 
  digitalWrite(40, LOW);

  Serial.begin(9600);
  Serial.print("THOR ");
  Serial.print(ID);
  Serial.println(" activado");

  home();

}

void loop()
{
  readSerialCommand();
  turn();
}

void readSerialCommand()
{
  // Verificar si hay datos disponibles en el puerto serial
  if(Serial.available() > 0)
  {
    data = Serial.readStringUntil('\n');

    char command[5];
    int newJoint = 0, newTargetAngle = 0, newSpeed = 0;
    int parameters = sscanf(data.c_str(), "%s %d %d %d", command, &newJoint, &newTargetAngle, &newSpeed);

    if (parameters != PARAMETER_AMMOUNT)
    {
      Serial.println("Error: Datos recibidos incorrectos");
    }
    else if (strcmp(command, "move") != 0)
    {
      Serial.println("Error: Comando no reconocido");
    }
    else
    {
      // Validar las variables recibidas
      if (newJoint < 1 || newJoint > MAX_JOINT_NUMBER)
      {
        Serial.print("Error: Las articulaciones van de 1 a ");
        Serial.println(MAX_JOINT_NUMBER);
      }
      else if (newTargetAngle < -180 || newTargetAngle > 180)
      {
        Serial.println("Error: La posición supera los límites");
      }
      else if (newSpeed < -1000 || newSpeed > 1000)
      {
        Serial.println("Error: La velocidad supera los límites");
      }
      else
      {
        // Asignar variables
        joint = newJoint;
        targetAngle = newTargetAngle;
        speed = newSpeed;

        // Imprimir los valores recibidos en el puerto serial
        Serial.print("Articulacion: ");
        Serial.print(joint);
        Serial.print("  Posicion: ");
        Serial.print(targetAngle);
        Serial.print(" °  Velocidad: ");
        Serial.print(speed);
        Serial.println(" °/s");

        move(joint, targetAngle, speed);
      }
    }
  }
}

void home()
{
  //Se mueve el motor 1 hasta encontrar el final de carrera
  pm[0].setSpeed(90U*CONV_SPEED);

  while(digitalRead(sensor1))
  {
    pm[0].runSpeed();
  }
  // Se definen que las posiciones actuales van a ser las iniciales
  for(int i = 0; i<Motor_number;i++){
    pm[i].setCurrentPosition(0);
  }
}

void move(int joint, int targetAngle, int speed)
{
  long targetSteps = targetAngle * (STEPS_PER_REVOLUTION / 360.0);

  if(joint == 1)
  {
    pm[0].moveTo(R1 * targetSteps);
    pm[0].setSpeed(speed * CONV_SPEED * R1);
    ONE = 0;
  }
  else if(joint == 2)
  {
    // Mover los motores 2 y 3
    for (int i = 1; i <= 2; i++) {
      pm[i].moveTo(R2 * targetSteps);
      pm[i].setSpeed(R2 * CONV_SPEED * speed);
    }
    TWO = 0;
  }
  else if(joint == 3)
  {
    pm[3].moveTo(R3 * targetSteps);
    pm[3].setSpeed(R3 * CONV_SPEED * speed);
    THREE = 0;
  }
}

void turn ()
{
// Ejecutar continuamente mientras alguna de las variables ONE, TWO o THREE sea igual a 0
  while (ONE == 0 || TWO == 0 || THREE == 0) {
    // Iterar sobre los motores y ejecutar runSpeed() si hay movimientos pendientes
    for (int i = 0; i < Motor_number; i++) {
      if (pm[i].distanceToGo() != 0) {
        pm[i].runSpeed();
      } else {
        // Actualizar el estado correspondiente cuando el movimiento haya finalizado
        if (i == 0) {
          ONE = 1;
        } else if (i == 1 || i == 2) {
          TWO = 1;
        } else {
          THREE = 1;
        }
      }
    }
  }
}
