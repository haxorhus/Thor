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
#define DEFAULT_SPEED 200
#define MAX_JOINT_NUMBER 6
#define PARAMETER_AMMOUNT 4
#define motorInterfaceType 1
#define STEPS_PER_REVOLUTION  3200 // steps*16
#define Motor_number 5

// Relaciones

#define R1 44.5
#define R2 270
#define R3 265
#define R4 20


// Banderas

bool ONE = 1;
bool TWO = 1;
bool THREE = 1;
bool FOUR = 1;

// Direcciones de la memoria
const int address = 0;
bool lastDirection;

// Motores
//art 1
AccelStepper pm1 = AccelStepper(motorInterfaceType, stepPin1, dirPin1);
//art 2
AccelStepper pm2 = AccelStepper(motorInterfaceType, stepPin2, dirPin2);
AccelStepper pm3 = AccelStepper(motorInterfaceType, stepPin3, dirPin3);
//art 3
AccelStepper pm4 = AccelStepper(motorInterfaceType, stepPin4, dirPin4);
//art 4
AccelStepper pm5 = AccelStepper(motorInterfaceType, stepPin5, dirPin5);

// arreglo de motores
//NOTA: el array empieza en 0, por lo que pm1 = pm[0]
AccelStepper pm[] = {pm1, pm2, pm3, pm4, pm5};
// Parametros globales

int joint = 0;
int targetAngle = 0;
int speed = 0;

String data;

void setup()
{
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

  Serial.begin(9600);
  Serial.print(" THOR ");
  Serial.print(ID);
  Serial.println(" activado");
  delay(500);
  home();
}

void loop()
{
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
    if (strcmp(token, "move") == 0) {
      // Comando "move"
      // Extraer los argumentos del comando
      int newJoint = atoi(strtok(NULL, " "));
      int newTargetAngle = atoi(strtok(NULL, " "));
      int newSpeed = atoi(strtok(NULL, " "));

      // Validar los argumentos y llamar a la función move()
      if (validArguments(newJoint, newTargetAngle, newSpeed)) {
        move(newJoint, newTargetAngle, newSpeed);
      }
    } else if (strcmp(token, "home") == 0) {
      home();
      
    } else if(strcmp(token, "wp") == 0) {
      int q1 = atoi(strtok(NULL, " "));
      int q2 = atoi(strtok(NULL, " "));
      int q3 = atoi(strtok(NULL, " "));      

      Serial.println(" wp ");
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


void home()
{
  if (lastDirection) {
    // Gira en sentido horario
    pm[0].setSpeed(-4000);
  } else {
    // Gira en sentido antihorario
    pm[0].setSpeed(4000);
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
void move(int joint, int targetAngle, int speed)
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
}



// Función que mueve el punto muñeca a un punto en el espacio
void wp(int q1, int q2, int q3){
  int target1 = R1 * q1;
  int target2 = R2 * q2;
  int target3 = R3 * q3;

  // art 1
  int currentPos1 = pm[0].currentPosition();
  int speed1 = (target1 > currentPos1) ? 90 * R1 : -90 * R1;
  pm[0].moveTo(target1);
  pm[0].setSpeed(speed1);

  // art 2
  int currentPos2 = pm[1].currentPosition();
  int speed2 = (target2 > currentPos2) ? 90 * R2 : -90 * R2;
  pm[1].moveTo(target2);
  pm[1].setSpeed(speed2);
  pm[2].moveTo(target2);
  pm[2].setSpeed(speed2);

  // art 3
  int currentPos3 = pm[3].currentPosition();
  int speed3 = (target3 > currentPos3) ? 90 * R3 : -90 * R3;
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
  while (ONE == 0 || TWO == 0 || THREE == 0 || FOUR ==0) {
    // Iterar sobre los motores y ejecutar runSpeed() si hay movimientos pendientes
    // el 5 es por el numero de motores
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
        }
      }
    }
  }
}
