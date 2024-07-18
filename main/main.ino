#include <string.h>
#include <EEPROM.h>
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
#define endStop1 42
#define endStop2 44
#define endStop3 46
#define endStop4 48
#define endStop5 49
#define endStop6 47
#define endStop7 45

// Definicion de costantes
#define ID 2
#define MOTOR_NUMBER 7
#define MAX_JOINT_NUMBER 6
#define BAUD_RATE 115200
#define DRIVER 1

// Relaciones
#define R1 44.5
#define R2 270
#define R3 265
#define R4 20
#define R5 25
#define R6 25
#define R56 25

// Banderas
bool ONE = true;
bool TWO = true;
bool THREE = true;
bool FOUR = true;
bool FIVE = true;
bool SIX = true;

// EEPROM
bool lastDirection;
const int address = 0;

// Articulacion 1
AccelStepper pm1 = AccelStepper(DRIVER, stepPin1, dirPin1);

// Articulacion 2
AccelStepper pm2 = AccelStepper(DRIVER, stepPin2, dirPin2);
AccelStepper pm3 = AccelStepper(DRIVER, stepPin3, dirPin3);

// Articulacion 3
AccelStepper pm4 = AccelStepper(DRIVER, stepPin4, dirPin4);

// Articulacion 4
AccelStepper pm5 = AccelStepper(DRIVER, stepPin5, dirPin5);

// Articulaciones 5 y 6
AccelStepper pm6 = AccelStepper(DRIVER, stepPin6, dirPin6);
AccelStepper pm7 = AccelStepper(DRIVER, stepPin7, dirPin7);

// Arreglo de motores
AccelStepper pm[] = {pm1, pm2, pm3, pm4, pm5, pm6, pm7}; // NOTA: el array empieza en 0, por lo que pm1 = pm[0]

// Parametros globales
int joint = 0;
float targetAngle = 0;
float speed = 0;
int isMoving = 0;


void setup() {
  
  // Leer el último sentido de giro de la EEPROM
  lastDirection = EEPROM.read(address);

  // Configuracion
  for (int i = 0; i < MOTOR_NUMBER; i++) {
    pm[i].setMaxSpeed(5000);
    pm[i].setAcceleration(500);
  }

  // Enable
  pinMode(40, OUTPUT);
  digitalWrite(40, LOW);

  // Enviar mensajes al Serial Monitor
  Serial.begin(BAUD_RATE);
  Serial.print("THOR ");
  Serial.print(ID);
  Serial.println(" activado");

  // Llamada a la función home
  home();
}

void loop() {
  isMoving = 0;
  readSerialCommand();
  turn();
}

void home() {

  if (lastDirection) {
    // Gira en sentido horario
    pm[0].setSpeed(R1*40);
  } else {
    // Gira en sentido antihorario
    pm[0].setSpeed(-R1*40);
  }

  // Alternar el sentido de giro
  lastDirection = !lastDirection;
  
  // Almacenar el nuevo sentido de giro en la EEPROM
  EEPROM.write(address, lastDirection);

  while(digitalRead(endStop1)) {
    pm[0].runSpeed();
  }

  // Se definen que las posiciones actuales van a ser las iniciales
  S00();
}

void readSerialCommand() {

  if (Serial.available() > 0) {
    
    String command = Serial.readStringUntil('\n');
    char *token = strtok((char*)command.c_str(), " ");

    if (strcmp(token, "G2") == 0) {
      int newJoint = atoi(strtok(NULL, " "));
      float newTargetAngle = atof(strtok(NULL, " "));
      float newSpeed = atof(strtok(NULL, " "));
      if (validArguments(newJoint, newTargetAngle, newSpeed)) {
        G2(newJoint, newTargetAngle, newSpeed);
      }
    } else if (strcmp(token, "G13") == 0) {
      int newJoint = atoi(strtok(NULL, " "));
      float newTargetAngle = atof(strtok(NULL, " "));
      float newSpeed = atof(strtok(NULL, " "));
      float startAngle = atof(strtok(NULL, " "));
      float stopAngle = atof(strtok(NULL, " "));
      if (validArguments(newJoint, newTargetAngle, newSpeed)) {
        G13(newJoint, newTargetAngle, newSpeed, startAngle, stopAngle);
      }
    } else if (strcmp(token, "G00") == 0) {
      G00();
    } else if (strcmp(token, "S00") == 0) {
      S00();
    } else if(strcmp(token, "wp") == 0) {
      float q1 = atof(strtok(NULL, " "));
      float q2 = atof(strtok(NULL, " "));
      float q3 = atof(strtok(NULL, " "));
      char *speedToken1 = strtok(NULL, " ");
      char *speedToken2 = strtok(NULL, " ");
      char *speedToken3 = strtok(NULL, " ");
      if (speedToken1 != nullptr && speedToken2 != nullptr && speedToken3 != nullptr) {
        float speed1 = atof(speedToken1);
        float speed2 = atof(speedToken2);
        float speed3 = atof(speedToken3);
        wp(q1, q2, q3, speed1, speed2, speed3);
      } else {
        wp(q1, q2, q3);
      }
    }else if (strcmp(token,"P1") == 0){
      float q1 = atof(strtok(NULL, " "));
      float q2 = atof(strtok(NULL, " "));
      float q3 = atof(strtok(NULL, " "));
      float q4 = atof(strtok(NULL, " "));
      float q5 = atof(strtok(NULL, " "));
      float q6 = atof(strtok(NULL, " "));
      char *speedToken1 = strtok(NULL, " ");
      char *speedToken2 = strtok(NULL, " ");
      char *speedToken3 = strtok(NULL, " ");
      char *speedToken4 = strtok(NULL, " ");
      char *speedToken5 = strtok(NULL, " ");
      char *speedToken6 = strtok(NULL, " ");
      if (speedToken1 != nullptr && speedToken2 != nullptr && speedToken3 != nullptr && speedToken4 != nullptr && speedToken5 != nullptr && speedToken6 != nullptr) {
        float v1 = atof(*speedToken1);
        float v2 = atof(*speedToken2);
        float v3 = atof(*speedToken3);
        float v4 = atof(*speedToken4);
        float v5 = atof(*speedToken5);
        float v6 = atof(*speedToken6);
        if (validAngle(1, q1) and validAngle(2, q2) and validAngle(3, q3) and validAngle(4, q4) and validAngle(5, q5) and validAngle(6, q6) and validSpeed(v1) and validSpeed(v2) and validSpeed(v3) and validSpeed(v4) and validSpeed(v5) and validSpeed(v6)) {
          P1(q1,q2,q3,q4,q5,q6,v1,v2,v3,v4,v5,v6);
        }
      } else {
        if (validAngle(1, q1) and validAngle(2, q2) and validAngle(3, q3) and validAngle(4, q4) and validAngle(5, q5) and validAngle(6, q6)) {
          P1(q1,q2,q3,q4,q5,q6);
        }
      }
    } else {
      Serial.println("Error: Comando no reconocido");
    }
  }
}

bool validArguments(int joint, float targetAngle, float speed) {
  // Realizar la validación de los argumentos según el comando
  if (joint < 1 || joint > MAX_JOINT_NUMBER) {
    Serial.print("Error: Las articulaciones van de 1 a 6");
    Serial.println(MAX_JOINT_NUMBER);
    return false;
  }

  if (joint == 1 || joint == 4 || joint == 6) {
    if (targetAngle < -180 || targetAngle > 180) {
      Serial.println("Error: La posición supera los límites");
      return false;
    }
  } else if (joint == 3) {
    if (targetAngle < -90 || targetAngle > 90) {
      Serial.println("Error: La posición supera los límites");
      return false;
    }
  } else {
    if (targetAngle < -75 || targetAngle > 75) {
      Serial.println("Error: La posición supera los límites");
      return false;
    }
  }

  if (speed < -50 || speed > 50) {
    Serial.println("Error: La velocidad supera los límites");
    return false;
  }

  // Todos los argumentos son válidos
  return true;
}

bool validAngle(int joint, float targetAngle) {
  // Realizar la validación de los argumentos según el comando
  if (joint < 1 || joint > MAX_JOINT_NUMBER) {
    Serial.print("Error: Las articulaciones van de 1 a 6");
    Serial.println(MAX_JOINT_NUMBER);
    return false;
  }

  if (joint == 1 || joint == 4 || joint == 6) {
    if (targetAngle < -180 || targetAngle > 180) {
      Serial.println("Error: La posición supera los límites");
      return false;
    }
  } else if (joint == 3 || joint == 5) {
    if (targetAngle < -90 || targetAngle > 90) {
      Serial.println("Error: La posición supera los límites");
      return false;
    }
  } else {
    if (targetAngle < -75 || targetAngle > 75) {
      Serial.println("Error: La posición supera los límites");
      return false;
    }
  }

  return true;

}

bool validSpeed(float speed) {

  if (speed < -50 || speed > 50) {
    Serial.println("Error: La velocidad supera los límites");
    return false;
  }

  return true;
}

// Funcion que devuelve a la posicion 0
void G00() {

  float target = 0;
  float DEFAULT_SPEED = 2000;

  for(int i = 0; i < MOTOR_NUMBER; i++) {
    float currentPos = pm[i].currentPosition();
    pm[i].moveTo(target);
    float adjustedSpeed = (target > currentPos) ? DEFAULT_SPEED : -DEFAULT_SPEED;
    pm[i].setSpeed(adjustedSpeed);
  }

  ONE = 0;
  TWO = 0;
  THREE = 0;
  FOUR = 0;
  FIVE = 0;
  SIX = 0;
}

// Funcion que marca una nueva posicion 0
void S00() {
  for(int i = 0; i < MOTOR_NUMBER; i++) {
    pm[i].setCurrentPosition(0);
  }
}


// Función que mueve una articulación a un ángulo objetivo con una velocidad dada
void G2(int joint, float targetAngle, float speed) {
  if (joint == 1) {
    float target = -R1 * targetAngle;
    float currentPos = pm[0].currentPosition();
    float adjustedSpeed = (target > currentPos) ? speed * R1 : -speed * R1;
    pm[0].moveTo(target);
    pm[0].setSpeed(adjustedSpeed);
    ONE = 0;
  } else if (joint == 2) {
    float target = R2 * targetAngle;
    float currentPos = pm[1].currentPosition();
    float adjustedSpeed = (target > currentPos) ? speed * R2 : -speed * R2;
    for (int i = 1; i <= 2; i++) {
      pm[i].moveTo(target);
      pm[i].setSpeed(adjustedSpeed);
    }
    TWO = 0;
  } else if (joint == 3) {
    float target = R3 * targetAngle;
    float currentPos = pm[3].currentPosition();
    float adjustedSpeed = (target > currentPos) ? speed * R3 : -speed * R3;
    pm[3].moveTo(target);
    pm[3].setSpeed(adjustedSpeed);
    THREE = 0;
  } else if (joint == 4) {
    float target = -R4 * targetAngle;
    float currentPos = pm[4].currentPosition();
    float adjustedSpeed = (target > currentPos) ? speed * R4 : -speed * R4;
    pm[4].moveTo(target);
    pm[4].setSpeed(adjustedSpeed);
    FOUR = 0;
  } else if (joint == 5) {
    float target = R56 * targetAngle;
    float currentPos = pm[5].currentPosition();
    float adjustedSpeed = (target > currentPos) ? speed * R56 : -speed * R56;
    pm[5].moveTo(target);
    pm[5].setSpeed(adjustedSpeed);
    pm[6].moveTo(-target);
    pm[6].setSpeed(-adjustedSpeed);
    FIVE = 0;
  } else if (joint == 6) {
    float target = R56 * targetAngle;
    float currentPos = pm[5].currentPosition();
    float adjustedSpeed = (target > currentPos) ? speed * R56 : -speed * R56;
    pm[5].moveTo(target);
    pm[5].setSpeed(adjustedSpeed);
    pm[6].moveTo(target);
    pm[6].setSpeed(adjustedSpeed);
    SIX = 0;
  }
}

void G13(int joint, float targetAngle, float speed, float startAngle, float stopAngle) {
  //se ve donde empieza la funcion
  float originAngle = 0;
  // caso por cada articulacion
  switch (joint) {
  case 1:
    originAngle = pm[0].currentPosition();
    pm[0].setSpeed(0);
    pm[0].setMaxSpeed(R1*speed);
    pm[0].setAcceleration(min(R1*speed*speed/(2*(fabs(startAngle-originAngle))),2*sq(speed*R1*0.676)));
    pm[0].moveTo(-R1*targetAngle);
    while(pm[0].distanceToGo() != 0){
      if( ((pm[0].currentPosition() - (long)(R1*startAngle))==0) ){
        pm[0].setAcceleration(min(R1*speed*speed/(2*fabs(startAngle-originAngle)),2*sq(speed*R1*0.676)));
        if(pm[0].distanceToGo()>0)
          pm[0].setSpeed(R1*speed);
        else
          pm[0].setSpeed(-R1*speed);
      }
      pm[0].run();
    }
    pm[0].setMaxSpeed(5000);
    break;

  case 2: 
    originAngle = pm[1].currentPosition();
    pm[1].setSpeed(0);
    pm[2].setSpeed(0);
    pm[1].setMaxSpeed(R2*speed);
    pm[2].setMaxSpeed(R2*speed);
    pm[1].setAcceleration(min(R2*speed*speed/(2*(fabs(startAngle-originAngle))),2*sq(speed*R2*0.676)));
    pm[2].setAcceleration(min(R2*speed*speed/(2*(fabs(startAngle-originAngle))),2*sq(speed*R2*0.676)));
    pm[1].moveTo(R2*targetAngle);
    pm[2].moveTo(R2*targetAngle);

    while(pm[1].distanceToGo() != 0){
      if( ((pm[1].currentPosition() - (long)(R2*startAngle))==0) ){
        pm[1].setAcceleration(min(R2*speed*speed/(2*fabs(startAngle-originAngle)),2*sq(speed*R2*0.676)));
        pm[2].setAcceleration(min(R2*speed*speed/(2*fabs(startAngle-originAngle)),2*sq(speed*R2*0.676)));
        if(pm[1].distanceToGo()>0){
          pm[1].setSpeed(R2*speed);
          pm[2].setSpeed(R2*speed);
        }else{
          pm[1].setSpeed(-R2*speed);
          pm[2].setSpeed(-R2*speed);
        }
      }
      pm[1].run();
      pm[2].run();
    }
    pm[1].setMaxSpeed(5000);
    pm[2].setMaxSpeed(5000);
    break;

  case 3:
    originAngle = pm[3].currentPosition();
    pm[3].setSpeed(0);
    pm[3].setMaxSpeed(R3*speed);
    pm[3].setAcceleration(min(R3*speed*speed/(2*(fabs(startAngle-originAngle))),2*sq(speed*R3*0.676)));
    pm[3].moveTo(R3*targetAngle);
    while(pm[3].distanceToGo() != 0){
      if( ((pm[3].currentPosition() - (long)(R3*startAngle))==0) ){
        pm[3].setAcceleration(min(R3*speed*speed/(2*fabs(startAngle-originAngle)),2*sq(speed*R3*0.676)));
        if(pm[3].distanceToGo()>0)
          pm[3].setSpeed(R3*speed);
        else
          pm[3].setSpeed(-R3*speed);
      }
      pm[3].run();
    }
    pm[3].setMaxSpeed(5000);
    break;
  case 4:
    originAngle = pm[4].currentPosition();
    pm[4].setSpeed(0);
    pm[4].setMaxSpeed(R4*speed);
    pm[4].setAcceleration(min(R4*speed*speed/(2*(fabs(startAngle-originAngle))),2*sq(speed*R4*0.676)));
    pm[4].moveTo(-R4*targetAngle);
    while(pm[4].distanceToGo() != 0){
      if( ((pm[4].currentPosition() - (long)(R4*startAngle))==0) ){
        pm[4].setAcceleration(min(R4*speed*speed/(2*fabs(startAngle-originAngle)),2*sq(speed*R4*0.676)));
        if(pm[4].distanceToGo()>0)
          pm[4].setSpeed(R4*speed);
        else
          pm[4].setSpeed(-R4*speed);
      }
      pm[4].run();
    }
    pm[4].setMaxSpeed(5000);
    break;

  case 5:
    originAngle = pm[5].currentPosition();
    pm[5].setSpeed(0);
    pm[6].setSpeed(0);
    pm[5].setMaxSpeed(R5*speed);
    pm[6].setMaxSpeed(R5*speed);
    pm[5].setAcceleration(min(R5*speed*speed/(2*(fabs(startAngle-originAngle))),2*sq(speed*R5*0.676)));
    pm[6].setAcceleration(min(R5*speed*speed/(2*(fabs(startAngle-originAngle))),2*sq(speed*R5*0.676)));
    pm[5].moveTo(R5*targetAngle);
    pm[6].moveTo(-R5*targetAngle);

    while(pm[5].distanceToGo() != 0){
      if( ((pm[5].currentPosition() - (long)(R5*startAngle))==0) ){
        pm[5].setAcceleration(min(R5*speed*speed/(2*fabs(startAngle-originAngle)),2*sq(speed*R5*0.676)));
        pm[6].setAcceleration(min(R5*speed*speed/(2*fabs(startAngle-originAngle)),2*sq(speed*R5*0.676)));
        if(pm[5].distanceToGo()>0){
          pm[5].setSpeed(R5*speed);
          pm[6].setSpeed(-R5*speed);
        }else{
          pm[5].setSpeed(-R5*speed);
          pm[6].setSpeed(R5*speed);
        }
      }
      pm[5].run();
      pm[6].run();
    }
    pm[5].setMaxSpeed(5000);
    pm[6].setMaxSpeed(5000);
    break;

  case 6:
    originAngle = pm[5].currentPosition();
    pm[5].setSpeed(0);
    pm[6].setSpeed(0);
    pm[5].setMaxSpeed(R5*speed);
    pm[6].setMaxSpeed(R5*speed);
    pm[5].setAcceleration(min(R5*speed*speed/(2*(fabs(startAngle-originAngle))),2*sq(speed*R5*0.676)));
    pm[6].setAcceleration(min(R5*speed*speed/(2*(fabs(startAngle-originAngle))),2*sq(speed*R5*0.676)));
    pm[5].moveTo(R5*targetAngle);
    pm[6].moveTo(R5*targetAngle);

    while(pm[5].distanceToGo() != 0){
      if( ((pm[5].currentPosition() - (long)(R5*startAngle))==0) ){
        pm[5].setAcceleration(min(R5*speed*speed/(2*fabs(startAngle-originAngle)),2*sq(speed*R5*0.676)));
        pm[6].setAcceleration(min(R5*speed*speed/(2*fabs(startAngle-originAngle)),2*sq(speed*R5*0.676)));
        if(pm[5].distanceToGo()>0){
          pm[5].setSpeed(R5*speed);
          pm[6].setSpeed(R5*speed);
        }else{
          pm[5].setSpeed(-R5*speed);
          pm[6].setSpeed(-R5*speed);
        }
      }
      pm[5].run();
      pm[6].run();
    }
    pm[5].setMaxSpeed(5000);
    pm[6].setMaxSpeed(5000);
    break;
  }

  // Indicar que el movimiento ha terminado
  Serial.println("done");
}

// Función que mueve el punto muñeca a un punto en el espacio
void wp(float q1, float q2, float q3) {
  float target1 = -R1 * q1;
  float target2 = R2 * q2;
  float target3 = R3 * q3;

  // Articulacion 1
  float currentPos1 = pm[0].currentPosition();
  float speed1 = (target1 > currentPos1) ? 10 * R1 : -10 * R1;
  pm[0].moveTo(target1);
  pm[0].setSpeed(speed1);

  // Articulacion 2
  float currentPos2 = pm[1].currentPosition();
  float speed2 = (target2 > currentPos2) ? 10 * R2 : -10 * R2;
  pm[1].moveTo(target2);
  pm[1].setSpeed(speed2);
  pm[2].moveTo(target2);
  pm[2].setSpeed(speed2);

  // Articulacion 3
  float currentPos3 = pm[3].currentPosition();
  float speed3 = (target3 > currentPos3) ? 10 * R3 : -10 * R3;
  pm[3].moveTo(target3);
  pm[3].setSpeed(speed3);

  ONE = 0;
  TWO = 0;
  THREE = 0;
}

// Función que mueve el punto muñeca a un punto en el espacio de forma coordinada
void wp(float q1, float q2, float q3, int v1, int v2, int v3) {
  
  float target1 = -R1 * q1;
  float target2 = R2 * q2;
  float target3 = R3 * q3;

  // Articulacion 1
  float currentPos1 = pm[0].currentPosition();
  float speed1 = (target1 > currentPos1) ? R1*v1 : -R1*v1;
  pm[0].moveTo(target1);
  pm[0].setSpeed(speed1);
  Serial.println(speed1);

  // Articulacion 2
  float currentPos2 = pm[1].currentPosition();
  float speed2 = (target2 > currentPos2) ? R2*v2 : -R2*v2;
  pm[1].moveTo(target2);
  pm[1].setSpeed(speed2);
  pm[2].moveTo(target2);
  pm[2].setSpeed(speed2);
  Serial.println(speed2);

  // Articulacion 3
  float currentPos3 = pm[3].currentPosition();
  float speed3 = (target3 > currentPos3) ? R3*v3 : -R3*v3;
  pm[3].moveTo(target3);
  pm[3].setSpeed(speed3);
  Serial.println(speed3);

  ONE = 0;
  TWO = 0;
  THREE = 0;
}

void P1 (float q1, float q2, float q3, float q4, float q5,float q6){
  float target1 = -q1*R1;
  float target2 = q2*R2;
  float target3 = q3*R3;
  float target4 = -q4*R4;
  float target5 = q5*R5;
  float target6 = q6*R6;

  // Articulacion 1
  float currentPos1 = pm[0].currentPosition();
  float speed1 = (target1 > currentPos1) ? R1*10 : -R1*10;
  pm[0].moveTo(target1);
  pm[0].setSpeed(speed1);

  // Articulacion 2
  float currentPos2 = pm[1].currentPosition();
  float speed2 = (target2 > currentPos2) ? R2*10 : -R2*10;
  pm[1].moveTo(target2);
  pm[1].setSpeed(speed2);
  pm[2].moveTo(target2);
  pm[2].setSpeed(speed2);

  // Articulacion 3
  float currentPos3 = pm[3].currentPosition();
  float speed3 = (target3 > currentPos3) ? R3*10 : -R3*10;
  pm[3].moveTo(target3);
  pm[3].setSpeed(speed3);

  // Articulacion 4
  float currentPos4 = pm[4].currentPosition();
  float speed4 = (target4 > currentPos4) ? R4*10 : -R4*10;
  pm[4].moveTo(target4);
  pm[4].setSpeed(speed4);

  // Articulacion 5 y 6
  float currentPos5 = pm[5].currentPosition();
  float currentPos6 = pm[6].currentPosition();
  float speed5 = (target5 > currentPos5) ? R5*10 : -R5*10;
  float speed6 = (target6 > currentPos6) ? R6*10 : -R6*10;
  float P1=q5 + q6*2.0;
  float P2=-q5 + q6*2.0;
  float vP1=speed5 + speed6*2.0;
  float vP2=fabs(-speed5 + speed6*2.0); 
  pm[5].moveTo(R5*P1);
  pm[5].setSpeed(vP1);
  pm[6].moveTo(R5*P2);
  pm[6].setSpeed(vP2);

  ONE = 0;
  TWO = 0;
  THREE = 0;
  FOUR = 0;
  FIVE = 0;
  SIX = 0;
}

void P1 (float q1, float q2, float q3, float q4, float q5,float q6, float v1, float v2, float v3, float v4, float v5,float v6){
  float target1 = -q1*R1;
  float target2 = q2*R2;
  float target3 = q3*R3;
  float target4 = -q4*R4;
  float target5 = q5*R5;
  float target6 = q6*R6;

  // Articulacion 1
  float currentPos1 = pm[0].currentPosition();
  float speed1 = (target1 > currentPos1) ? R1*v1 : -R1*v1;
  pm[0].moveTo(target1);
  pm[0].setSpeed(speed1);

  // Articulacion 2
  float currentPos2 = pm[1].currentPosition();
  float speed2 = (target2 > currentPos2) ? R2*v2 : -R2*v2;
  pm[1].moveTo(target2);
  pm[1].setSpeed(speed2);
  pm[2].moveTo(target2);
  pm[2].setSpeed(speed2);

  // Articulacion 3
  float currentPos3 = pm[3].currentPosition();
  float speed3 = (target3 > currentPos3) ? R3*v3 : -R3*v3;
  pm[3].moveTo(target3);
  pm[3].setSpeed(speed3);

  // Articulacion 4
  float currentPos4 = pm[4].currentPosition();
  float speed4 = (target4 > currentPos4) ? R4*v4 : -R4*v4;
  pm[4].moveTo(target4);
  pm[4].setSpeed(speed4);

  // Articulacion 5 y 6
  float currentPos5 = pm[5].currentPosition();
  float currentPos6 = pm[6].currentPosition();
  float speed5 = (target5 > currentPos5) ? R5*v5 : -R5*v5;
  float speed6 = (target6 > currentPos6) ? R6*v6 : -R6*v6;
  float P1=q5 + q6*2.0;
  float P2=-q5 + q6*2.0;
  float vP1=speed5 + speed6*2.0;
  float vP2=fabs(-speed5 + speed6*2.0); 
  pm[5].moveTo(R5*P1);
  pm[5].setSpeed(vP1);
  pm[6].moveTo(R5*P2);
  pm[6].setSpeed(vP2);

  ONE = 0;
  TWO = 0;
  THREE = 0;
  FOUR = 0;
  FIVE = 0;
  SIX = 0;
}

// Función que mueve los motores
void turn() {
  // Ejecutar continuamente mientras alguna de las variables ONE, TWO, THREE, FOUR, FIVE o SIX sean iguales a 1
  while (ONE == 0 || TWO == 0 || THREE == 0 || FOUR == 0 || FIVE == 0 || SIX == 0) {
    // Iterar sobre los motores y ejecutar runSpeed() si hay movimientos pendientes
    isMoving = 1;
    for (int i = 0; i < MOTOR_NUMBER; i++) {
      if (pm[i].distanceToGo() != 0) {
        pm[i].runSpeed();
      } else {
        // Actualizar el estado correspondiente cuando el movimiento haya finalizado
        if (i == 0) {
          ONE = 1;
        } else if (i == 1 || i == 2) {
          TWO = 1;
        } else if (i == 3) {
          THREE = 1;
        } else if (i == 4) {
          FOUR = 1;
        } else if (i == 5 || i == 6) {
          FIVE = 1;
          SIX = 1;
        }
      }
    }
  }
  
  // Mostrar mensaje cuando todos los motores hayan finalizado sus movimientos
  if (isMoving == 1) {
    Serial.println("done");
  }
}

