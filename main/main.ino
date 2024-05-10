#include "AccelStepper.h"
#include <string.h>

 
#define dirPin1 36
#define stepPin1 28
#define microsteps 1;
#define motorInterfaceType 1
#define ID 2
#define STEPS_PER_REVOLUTION 200*16
#define PARAMETER_AMMOUNT 4
#define MAX_JOINT_NUMBER 6

AccelStepper art1 = AccelStepper(motorInterfaceType, stepPin1, dirPin1);

// Parametros globales
uint8_t joint = 0;
int16_t targetAngle = 0;
int16_t speed = 0;

void readSerialCommand() {
  // Verificar si hay datos disponibles en el puerto serial
  if(Serial.available() > 0)
  {
    data = Serial.readStringUntil('\n');

    char command[5];
    int parameters = sscanf(data.c_str(), "%s %hhd %hd %hd", command, &joint, &targetAngle, &speed);

    if (parameters != PARAMETER_AMMOUNT){
      Serial.println("Error: Datos recibidos incorrectos");
    }else if (strcmp(command, "move") != 0){
      Serial.println("Error: Comando no reconocido");
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
      //numero magico art1 = 3338
      //long targetSteps = targetAngle * (STEPS_PER_REVOLUTION / 360.0);
      long targetSteps = targetAngle * (3338);

      
      // Mover el motor correspondiente a la posición deseada con la velocidad especificada
      if (joint == 1) {
        // Mover el primer motor
        art1.moveTo(targetSteps);
        art1.setSpeed(speed);
      } else if (joint == 2) {
        // Aquí puedes agregar el código para mover el segundo motor si tienes otro motor conectado
      }
    }
  }
}

void setup() {
  art1.setMaxSpeed(1000);
  art1.setAcceleration(750);

  pinMode(40,OUTPUT); 
  pinMode(36, OUTPUT);
  pinMode(26, OUTPUT);
  digitalWrite(40, LOW);
  digitalWrite(36, LOW);
  digitalWrite(26, LOW);

  Serial.begin(9600);
  Serial.print("THOR ");
  Serial.print(ID);
  Serial.println(" activado");
}

void loop() {
  readSerialCommand();
  art1.runSpeedToPosition();
}
