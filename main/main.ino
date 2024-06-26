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
#define sensor1 42
#define sensor2 44
#define sensor3 46
#define sensor4 48
#define sensor5 49
#define sensor6 47
#define sensor7 45

// Definicion de costantes
#define ID 2
#define MAX_JOINT_NUMBER 6
#define motorInterfaceType 1
#define Motor_number 7

// Relaciones
#define R1 44.5
#define R2 270
#define R3 265
#define R4 20
#define R5 250
#define R6 250

// Banderas
bool ONE = 1;
bool TWO = 1;
bool THREE = 1;
bool FOUR = 1;
bool FIVE = 1;
bool SIX = 1;

// EEPROM
bool lastDirection;
const int address = 0;

// Articulacion 1
AccelStepper pm1 = AccelStepper(motorInterfaceType, stepPin1, dirPin1);

// Articulacion 2
AccelStepper pm2 = AccelStepper(motorInterfaceType, stepPin2, dirPin2);
AccelStepper pm3 = AccelStepper(motorInterfaceType, stepPin3, dirPin3);

// Articulacion 3
AccelStepper pm4 = AccelStepper(motorInterfaceType, stepPin4, dirPin4);

// Articulacion 4
AccelStepper pm5 = AccelStepper(motorInterfaceType, stepPin5, dirPin5);

// Articulaciones 5 y 6
AccelStepper pm6 = AccelStepper(motorInterfaceType, stepPin6, dirPin6);
AccelStepper pm7 = AccelStepper(motorInterfaceType, stepPin7, dirPin7);

// Arreglo de motores
AccelStepper pm[] = {pm1, pm2, pm3, pm4, pm5, pm6, pm7}; // NOTA: el array empieza en 0, por lo que pm1 = pm[0]

// Parametros globales
int joint = 0;
int targetAngle = 0;
int speed = 0;
int isMoving = 0;


void setup() {
  // Leer el último sentido de giro de la EEPROM
  lastDirection = EEPROM.read(address);

  // Configuracion
  for (int i = 0; i < Motor_number; i++) {
    pm[i].setMaxSpeed(5000);
    pm[i].setAcceleration(500);
  }

  // Enable
  pinMode(40, OUTPUT); 
  digitalWrite(40, LOW);

  Serial.begin(115000);
  Serial.print(" THOR ");
  Serial.print(ID);
  Serial.println(" activado");
  delay(500);
  //home();
}


void loop() {
  isMoving = 0;
  readSerialCommand();
  turn();
}


void readSerialCommand() {
  // Verificar si hay datos disponibles en el puerto serial
  if (Serial.available() > 0) {
    // Leer la cadena recibida hasta el salto de línea
    String command = Serial.readStringUntil('\n');

    // Utilizar un separador para dividir la cadena en tokens
    char *token = strtok((char*)command.c_str(), " ");

    // Verificar el comando recibido
    if (strcmp(token, "G2") == 0) {
      // Comando "move"
      // Extraer los argumentos del comando
      int newJoint = atoi(strtok(NULL, " "));
      int newTargetAngle = atoi(strtok(NULL, " "));
      int newSpeed = atoi(strtok(NULL, " "));
      // Validar los argumentos y llamar a la función move()
      if (validArguments(newJoint, newTargetAngle, newSpeed)) {
        G2(newJoint, newTargetAngle, newSpeed);
      }
    } else if (strcmp(token, "G00") == 0) {
      G00();
    } else if (strcmp(token, "S00") == 0) {
      S00();
    } else if(strcmp(token, "wp") == 0) {
      int q1 = atoi(strtok(NULL, " "));
      int q2 = atoi(strtok(NULL, " "));
      int q3 = atoi(strtok(NULL, " "));      

      wp(q1,q2,q3);
    }else {
      // Comando no reconocido
      Serial.println("Error: Comando no reconocido");
    }
  }
}


bool validArguments(int joint, int targetAngle, int speed) {
  // Realizar la validación de los argumentos según el comando
  if (joint < 1 || joint > MAX_JOINT_NUMBER) {
    Serial.print("Error: Las articulaciones van de 1 a ");
    Serial.println(MAX_JOINT_NUMBER);
    return false;
  }

  if (targetAngle < -180 || targetAngle > 180) {
    Serial.println("Error: La posición supera los límites");
    return false;
  }

  if (speed < -1000 || speed > 1000) {
    Serial.println("Error: La velocidad supera los límites");
    return false;
  }

  // Todos los argumentos son válidos
  return true;
}

// Funcion que devuelve a la posicion 0
void G00(){
  for(int i = 0; i<Motor_number;i++){
    int target = 0;
    int currentPos = pm[i].currentPosition();
    pm[i].moveTo(target);
    int adjustedSpeed = (target > currentPos) ? 2000 : -2000;
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
void S00(){
  for(int i = 0; i<Motor_number;i++){
    pm[i].setCurrentPosition(0);
  }
}

void home()
{
  if (lastDirection) {
    // Gira en sentido horario
    pm[0].setSpeed(-R1*40);
  } else {
    // Gira en sentido antihorario
    pm[0].setSpeed(R1*40);
  }
  // Alternar el sentido de giro
  lastDirection = !lastDirection;
  
  // Almacenar el nuevo sentido de giro en la EEPROM
  EEPROM.write(address, lastDirection);

  while(digitalRead(sensor1))
  {
    pm[0].runSpeed();
  }
  // Se definen que las posiciones actuales van a ser las iniciales
  for(int i = 0; i<Motor_number;i++){
    pm[i].setCurrentPosition(0);
  }
}


// Función que mueve una articulación a un ángulo objetivo con una velocidad dada
void G2(int joint, int targetAngle, int speed)
{
  if (joint == 1)
  {
    int target = R1 * targetAngle;
    int currentPos = pm[0].currentPosition();
    int adjustedSpeed = (target > currentPos) ? speed * R1 : -speed * R1;
    pm[0].moveTo(target);
    pm[0].setSpeed(adjustedSpeed);
    Serial.println(adjustedSpeed);
    ONE = 0;
  }
  else if (joint == 2)
  {
    int target = R2 * targetAngle;
    int currentPos = pm[1].currentPosition();
    int adjustedSpeed = (target > currentPos) ? speed * R2 : -speed * R2;
    // Mover los motores 2 y 3
    for (int i = 1; i <= 2; i++) {
      pm[i].moveTo(target);
      pm[i].setSpeed(adjustedSpeed);
    }
    TWO = 0;
  }
  else if (joint == 3)
  {
    int target = R3 * targetAngle;
    int currentPos = pm[3].currentPosition();
    int adjustedSpeed = (target > currentPos) ? speed * R3 : -speed * R3;
    pm[3].moveTo(target);
    pm[3].setSpeed(adjustedSpeed);
    THREE = 0;
  }
  else if (joint == 4)
  {
    int target = R4 * targetAngle;
    int currentPos = pm[4].currentPosition();
    int adjustedSpeed = (target > currentPos) ? speed * R4 : -speed * R4;
    pm[4].moveTo(target);
    pm[4].setSpeed(adjustedSpeed);
    FOUR = 0;
  }
  else if (joint == 5)
  {
    int target = R5 * targetAngle;
    int currentPos = pm[5].currentPosition();
    int adjustedSpeed = (target > currentPos) ? speed * R5 : -speed * R5;
    pm[5].moveTo(target);
    pm[5].setSpeed(adjustedSpeed);
    pm[6].moveTo(target);
    pm[6].setSpeed(adjustedSpeed);    
    FIVE = 0;
  }
  else if (joint == 6)
  {
    int target = R6 * targetAngle;
    int currentPos = pm[6].currentPosition();
    int adjustedSpeed = (target > currentPos) ? speed * R6 : -speed * R6;
    pm[5].moveTo(target);
    pm[5].setSpeed(adjustedSpeed);
    pm[6].moveTo(target);
    pm[6].setSpeed(-adjustedSpeed);    
    SIX = 0;
  }
}



// Función que mueve el punto muñeca a un punto en el espacio
void wp(int q1, int q2, int q3){
  int target1 = R1 * q1;
  int target2 = R2 * q2;
  int target3 = R3 * q3;
  int speed = 1800;

  // art 1
  int currentPos1 = pm[0].currentPosition();
  int speed1 = (target1 > currentPos1) ? 10 * R1 : -10 * R1;
  pm[0].moveTo(target1);
  pm[0].setSpeed(speed1);

  // art 2
  int currentPos2 = pm[1].currentPosition();
  int speed2 = (target2 > currentPos2) ? 10 * R2 : -10 * R2;
  pm[1].moveTo(target2);
  pm[1].setSpeed(speed2);
  pm[2].moveTo(target2);
  pm[2].setSpeed(speed2);

  // art 3
  int currentPos3 = pm[3].currentPosition();
  int speed3 = (target3 > currentPos3) ? 10 * R3 : -10 * R3;
  pm[3].moveTo(target3);
  pm[3].setSpeed(speed3);

  ONE = 0;
  TWO = 0;
  THREE = 0;
}
// Función que mueve el punto muñeca a un punto en el espacio
void wp(int q1, int q2, int q3, int v1, int v2, int v3){
  int target1 = R1 * q1;
  int target2 = R2 * q2;
  int target3 = R3 * q3;
  

  // art 1
  int currentPos1 = pm[0].currentPosition();
  int speed1 = (target1 > currentPos1) ? R1*v1 : -R1*v1;
  pm[0].moveTo(target1);
  pm[0].setSpeed(speed1);

  // art 2
  int currentPos2 = pm[1].currentPosition();
  int speed2 = (target2 > currentPos2) ? R2*v2 : -R2*v2;
  pm[1].moveTo(target2);
  pm[1].setSpeed(speed2);
  pm[2].moveTo(target2);
  pm[2].setSpeed(speed2);

  // art 3
  int currentPos3 = pm[3].currentPosition();
  int speed3 = (target3 > currentPos3) ? R3*v3 : -R3*v3;
  pm[3].moveTo(target3);
  pm[3].setSpeed(speed3);

  ONE = 0;
  TWO = 0;
  THREE = 0;
}


//funcion que mueve los motores
void turn ()
{
// Ejecutar continuamente mientras alguna de las variables ONE, TWO o THREE sea igual a 0
  while (ONE == 0 || TWO == 0 || THREE == 0 || FOUR ==0 || FIVE==0 || SIX== 0) {
    // Iterar sobre los motores y ejecutar runSpeed() si hay movimientos pendientes
    // el 5 es por el numero de motores
    isMoving = 1;
    for (int i = 0; i < Motor_number; i++) {

      if (pm[i].distanceToGo() != 0) {
        pm[i].runSpeed();
      } else {
        // Actualizar el estado correspondiente cuando el movimiento haya finalizado
        if (i == 0) {
          ONE = 1;
        } else if (i == 1 || i == 2) {
          TWO = 1;
        } else if (i == 3){
          THREE = 1;
        } else if (i == 4){
          FOUR = 1;
        } else if (i == 5){
          FIVE = 1;
        }else if (i == 6){
          SIX = 1;
        }
      }
    }
  }
  if (isMoving == 1){
    Serial.println("done");
  }
}
