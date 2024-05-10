#include <string.h>
#include "AccelStepper.h"

 // Definicion de los pines de motores

#define dirPin1 36
#define stepPin1 28
#define dirPin2 34
#define stepPin2 26
#define dirPin3 32
#define stepPin3 24
#define dirPin4 30
#define stepPin4 22
#define dirPin5 31
#define stepPin5 23
#define dirPin6 33
#define stepPin6 25
#define dirPin7 35
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
#define microsteps 1;
#define MAX_JOINT_NUMBER 6
#define PARAMETER_AMMOUNT 4
#define motorInterfaceType 1
#define STEPS_PER_REVOLUTION 200*16*5

AccelStepper pm1 = AccelStepper(motorInterfaceType, stepPin1, dirPin1);
AccelStepper pm2 = AccelStepper(motorInterfaceType,stepPin2,dirPin2);
AccelStepper pm3 = AccelStepper(motorInterfaceType,stepPin3,dirPin3);
AccelStepper pm4 = AccelStepper(motorInterfaceType,stepPin4,dirPin4);

// Parametros globales

uint8_t joint = 0;
int16_t targetAngle = 0;
int16_t speed = 0;

String data;

void setup() {
  pm1.setMaxSpeed(1000);
  pm1.setAcceleration(750);

  pinMode(40, OUTPUT); 
  digitalWrite(40, LOW);

  Serial.begin(9600);
  Serial.print("THOR ");
  Serial.print(ID);
  Serial.println(" activado");
}

void loop() {
  readSerialCommand();
  pm1.runSpeedToPosition();
  pm2.runSpeedToPosition();
  pm3.runSpeedToPosition();
  pm4.runSpeedToPosition();
}

void readSerialCommand() {
  // Verificar si hay datos disponibles en el puerto serial
  if(Serial.available() > 0)
  {
    data = Serial.readStringUntil('\n');

    char command[5];
    int newJoint = 0, newTargetAngle = 0, newSpeed = 0;
    int parameters = sscanf(data.c_str(), "%s %d %d %d", command, &newJoint, &newTargetAngle, &newSpeed);

    if (parameters != PARAMETER_AMMOUNT){
      Serial.println("Error: Datos recibidos incorrectos");
    }else if (strcmp(command, "move") != 0){
      if (strcmp(command, "start") == 0){
        Serial.println("buscando el cero \n");
        starter();
        return;
      }else{
      Serial.println("Error: Comando no reconocido");}
    }else{
      // Validar las variables recibidas
      if (joint < 1 || joint > MAX_JOINT_NUMBER){
        Serial.print("Error: Las articulaciones van de 1 a ");
        Serial.println(MAX_JOINT_NUMBER);
      }else if (targetAngle < -180 || targetAngle > 180){
        Serial.println("Error: La posición supera los límites");
      }else if (speed < -1000 || speed > 1000){
        Serial.println("Error: La velocidad supera los límites");
      }else{
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
      }
    }
      // Convertir la posición deseada de grados a pasos
      //numero magico pm1 = 3338
      long targetSteps = targetAngle * (STEPS_PER_REVOLUTION / 360.0);
      //long targetSteps = targetAngle * (3338);

      
      // Mover el motor correspondiente a la posición deseada con la velocidad especificada
      if (joint == 1) {
        // Mover el primer motor
        pm1.moveTo(targetSteps);
        pm1.setSpeed(speed);
        Serial.println("ok");
      } else if (joint == 2) {
        // Mover el segundo y tercer motor
        pm2.moveTo(targetSteps);
        pm2.setSpeed(speed);
        pm3.moveTo(targetSteps);
        pm3.setSpeed(speed);                
      } else if (joint == 3){
        pm4.moveTo(targetSteps);
        pm4.setSpeed(speed);
      }
  }
}

void starter(){
  while(digitalRead(sensor1)){
    pm1.setSpeed(2000);
    pm1.runSpeed();
  }
  pm1.setCurrentPosition(pm1.currentPosition());
}
