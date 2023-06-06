#include <Servo.h>

// Pines donde están conectados los servos
const int servoPin1 = 3;
const int servoPin2 = 5;
const int servoPin3 = 6;

// Objetos de los servos
Servo servoMotor1;
Servo servoMotor2;
Servo servoMotor3;

// Variables para controlar los ángulos de los servos
int angulo1 = 90;
int angulo2 = 90;
int angulo3 = 75;



int mR1 = 2;
int mR2 = 3; 
int mL1 = 4;
int mL2 = 5;
int pwmR = 6;
int pwmL = 7;

int vel = 80;

float x, z;

int manipulador;

void setup() {
    // Inicializar la comunicación serial
  Serial.begin(9600);
  
  // Esperar a que llegue un mensaje de ROS 2
  while (!Serial.available()) {
    delay(100);
  }


    // Inicializar los objetos de los servos
  servoMotor1.attach(servoPin1);
  servoMotor2.attach(servoPin2);
  servoMotor3.attach(servoPin3);
}

void loop() {
      
if (Serial.available() > 0) {

  String inputString = Serial.readString();
  int values[4]; // array to store the four parsed floats
  int valueIndex = 0; // index to keep track of which float is being parsed
  int commaIndex = 0; // index to keep track of the comma position

  while (commaIndex >= 0) {
    commaIndex = inputString.indexOf(","); // find the next comma in the input string
    if (commaIndex >= 0) {
      values[valueIndex] = atoi(inputString.substring(0, commaIndex).c_str()); // parse the float between the start of the string and the comma
      inputString = inputString.substring(commaIndex + 1); // remove the parsed float and the comma from the input string
      valueIndex++; // increment the index to parse the next float
    }
  }

  values[valueIndex] = atoi(inputString.c_str()); // parse the final float in the input string

  // destructure
  float x = values[0];
  float z = values[1];
  int manipulador = values[2];

  }

      if (x < 0) {
        digitalWrite(mL1, HIGH);
        digitalWrite(mL2, LOW);
        analogWrite(pwmL, -x*16);

        digitalWrite(mR1, HIGH);
        digitalWrite(mR2, LOW);
        analogWrite(pwmR, -x*16);
        
      }
      else if (x>0){
        digitalWrite(mL2, HIGH);
        digitalWrite(mL1, LOW);
        analogWrite(pwmL, x*16);

        digitalWrite(mR2, HIGH);
        digitalWrite(mR1, LOW);
        analogWrite(pwmR, x*16);
      }

      else if (z < 0){
        digitalWrite(mL2, HIGH);
        digitalWrite(mL1, LOW);
        analogWrite(pwmL, -z*16);

        digitalWrite(mR2, LOW);
        digitalWrite(mR1, HIGH);
        analogWrite(pwmR, -z*16);
       }

       else if (z > 0){
        digitalWrite(mL2, LOW);
        digitalWrite(mL1, HIGH);
        analogWrite(pwmL, z*16);

        digitalWrite(mR2, HIGH);
        digitalWrite(mR1, LOW);
        analogWrite(pwmR, z*16);
       }
      else if (x==0 || z==0){
        digitalWrite(mL1, LOW);
        digitalWrite(mL2, LOW);

        digitalWrite(mR1, LOW);
        digitalWrite(mR2, LOW);
      }

  
  if (manipulador ==1) {  // base de 15 cm
  
  // ------------------------------------ ABRIR ---------------------
  angulo1 = 110;
  angulo2 = 0;
  angulo3 = 50;
  // Posicionar los servos a los ángulos correspondientes
  servoMotor1.write(angulo1);
  servoMotor2.write(angulo2);
  delay(150);
  servoMotor3.write(angulo3);
  delay(700);
  // ------------------------------------ CERRAR ---------------------
  // Asignar los ángulos para cerrar
  angulo1 = 110;
  angulo2 =0;
  angulo3 = 115;

  // Posicionar los servos a los ángulos correspondientes
  servoMotor1.write(angulo1);
  servoMotor2.write(angulo2);
  servoMotor3.write(angulo3);
  delay(300);
  // ------------------------------------ SUBIR ---------------------
  // Asignar los ángulos para subir
  angulo1 = 90;
  angulo3 = 120;
  int objetivo = 50; // Ángulo objetivo a alcanzar
  int incremento = 5; // Incremento de ángulo

  // Incrementar el ángulo2 gradualmente hasta alcanzar el objetivo
  for (int angulo = angulo2; angulo <= objetivo; angulo += incremento) {
    angulo2 = angulo;

    // Verificar los límites del ángulo2
    if (angulo2 > 180) angulo2 = 180;

    // Posicionar el servo2 al ángulo correspondiente
      servoMotor1.write(angulo1);
      servoMotor2.write(angulo2);
      servoMotor3.write(angulo3);

    delay(100); // Tiempo de espera entre incrementos
  
  }
  }
  else if (manipulador==2){ // Base de 10 cm
      // ------------------------------------ ABRIR ---------------------
  angulo1 = 110;
  angulo2 = 0;
  angulo3 = 50;
  // Posicionar los servos a los ángulos correspondientes
  servoMotor1.write(angulo1);
  servoMotor2.write(angulo2);
  delay(150);
  servoMotor3.write(angulo3);
  delay(700);
  // ------------------------------------ CERRAR ---------------------
  // Asignar los ángulos para cerrar
  angulo1 = 110;
  angulo2 =0;
  angulo3 = 115;

  // Posicionar los servos a los ángulos correspondientes
  servoMotor1.write(angulo1);
  servoMotor2.write(angulo2);
  servoMotor3.write(angulo3);
  delay(300);
  // ------------------------------------ SUBIR ---------------------
  // Asignar los ángulos para subir
  angulo1 = 90;
  angulo3 = 120;
  int objetivo = 50; // Ángulo objetivo a alcanzar
  int incremento = 5; // Incremento de ángulo

  // Incrementar el ángulo2 gradualmente hasta alcanzar el objetivo
  for (int angulo = angulo2; angulo <= objetivo; angulo += incremento) {
    angulo2 = angulo;

    // Verificar los límites del ángulo2
    if (angulo2 > 180) angulo2 = 180;

    // Posicionar el servo2 al ángulo correspondiente
      servoMotor1.write(angulo1);
      servoMotor2.write(angulo2);
      servoMotor3.write(angulo3);

    delay(100); // Tiempo de espera entre incrementos
  
  }
  }
  
}
