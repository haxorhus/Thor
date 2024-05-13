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
#define DEFAULT_SPEED 90
#define MAX_JOINT_NUMBER 6
#define PARAMETER_AMMOUNT 4
#define motorInterfaceType 1
#define MAX_ACCELERATION 750
#define STEPS_PER_REVOLUTION 200*16
#define CONV_SPEED STEPS_PER_REVOLUTION/360
#define MAX_SPEED 150*CONV_SPEED

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

// Parametros globales

int joint = 0;
int targetAngle = 0;
int speed = 0;

String data;

void setup()
{
  // Configuracion
  pm1.setMaxSpeed(MAX_SPEED);
  pm2.setMaxSpeed(MAX_SPEED);
  pm3.setMaxSpeed(MAX_SPEED);
  pm4.setMaxSpeed(MAX_SPEED);
  pm1.setAcceleration(MAX_ACCELERATION);
  pm2.setAcceleration(MAX_ACCELERATION);
  pm3.setAcceleration(MAX_ACCELERATION);
  pm4.setAcceleration(MAX_ACCELERATION);

  // Alimentacion
  pinMode(38, OUTPUT);
  pinMode(39, OUTPUT);
  digitalWrite(38, HIGH);
  digitalWrite(39, HIGH);

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
  //pm1.runSpeedToPosition();
  //pm2.runSpeedToPosition();
  //pm3.runSpeedToPosition();
  //pm4.runSpeedToPosition();
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

  pm1.setSpeed(DEFAULT_SPEED * CONV_SPEED);

  while(digitalRead(sensor1))
  {
    pm1.runSpeed();
  }

  pm1.setCurrentPosition(0);
  pm2.setCurrentPosition(0);
  pm3.setCurrentPosition(0);
  pm4.setCurrentPosition(0);
}

void move(int joint, int targetAngle, int speed)
{
  long targetSteps = targetAngle * (STEPS_PER_REVOLUTION / 360.0);

  if(joint == 1)
  {
    pm1.moveTo(R1 * targetSteps);
    pm1.setSpeed(R1 * CONV_SPEED * speed);
    ONE = 0;
  }
  else if(joint == 2)
  {
    pm2.moveTo(R2 * targetSteps);
    pm2.setSpeed(R2 * CONV_SPEED * speed);
    pm3.moveTo(R2 * targetSteps);
    pm3.setSpeed(R2 * CONV_SPEED * speed);
    TWO = 0;
  }
  else if(joint == 3)
  {
    pm4.moveTo(R3 * targetSteps);
    pm4.setSpeed(R3 * CONV_SPEED * speed);
    THREE = 0;
  }
}

void turn ()
{
  while( ONE == 0 || TWO == 0 || THREE == 0)
  {
    if(pm1.distanceToGo() != 0)
    {
      pm1.runSpeed();
    }
    else
    {
      ONE = 1;
    }
    if(pm2.distanceToGo() != 0)
    {
      pm2.runSpeed();
      pm3.runSpeed();
    }
    else
    {
      TWO = 1;
    }
    if(pm4.distanceToGo() != 0)
    {
      pm4.runSpeed();
    }
    else
    {
      THREE = 1;
    }
  }
}
