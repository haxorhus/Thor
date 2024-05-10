#include "AccelStepper.h"
 
#define dirPin1 36
#define stepPin1 28
#define microsteps 1;
#define motorInterfaceType 1
#define STEPS_PER_REVOLUTION 200*16

AccelStepper art1 = AccelStepper(motorInterfaceType, stepPin1, dirPin1);

// Variables para almacenar la posición deseada y la velocidad
float targetPosition = 0.0; // Posición deseada en grados
float speed = 0.0; // Velocidad en grados por segund

void readSerialCommand() {
  // Verificar si hay datos disponibles en el puerto serial
  if (Serial.available() > 0) {
    // Leer el comando enviado por el puerto serial
    String command = Serial.readStringUntil('\n');
    
    // Analizar el comando para extraer el número de motor, la posición y la velocidad
    int spaceIndex1 = command.indexOf(' ');
    int spaceIndex2 = command.indexOf(' ', spaceIndex1 + 1);
    
    // Verificar si el comando tiene dos espacios
    if (spaceIndex1 != -1 && spaceIndex2 != -1) {
      // Extraer el número de motor, la posición y la velocidad del comando
      int motorNumber = command.substring(0, spaceIndex1).toInt();
      float targetAngle = command.substring(spaceIndex1 + 1, spaceIndex2).toFloat();
      float speed = command.substring(spaceIndex2 + 1).toFloat();
      
      // Convertir la posición deseada de grados a pasos
      //numero magico art1 = 3338
      //long targetSteps = targetAngle * (STEPS_PER_REVOLUTION / 360.0);
      long targetSteps = targetAngle * (3338);

      
      // Mover el motor correspondiente a la posición deseada con la velocidad especificada
      if (motorNumber == 1) {
        // Mover el primer motor
        art1.moveTo(targetSteps);
        art1.setSpeed(speed);
      } else if (motorNumber == 2) {
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
    while(!Serial); //esperar a que se conecte
  Serial.flush();
}

void loop() {
  readSerialCommand();
  art1.runSpeedToPosition();
}
