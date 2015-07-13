//Code by  : Abhishek Mukhopadhyay and modified by Famibelle Médhi
//Created on : May 30,2010
//Time : 11.43 am

#include <Servo.h>
#include <SoftwareSerial.h>
#include <NewPing.h>

// Bluetooth done with http://www.wikidebrouillard.org/index.php/Android_et_arduino and http://eskimon.fr/2498-arduino-annexes-g-utiliser-module-bluetooth-hc-05

int motorPin1 = 7;    // pin 10 (Input A) on L293D
int motorPin2 = 8;    // pin 15 (Input B) on L293D
int enablePin = 6;    // PWM enable & pin 9 on L293D
int servoPin13 = 13;  // Servo Pin 

int Rx = 11;
int Tx = 10;
int key = 12;

// ultra sound detector
int trigger = 4;
int echo = 2;
long distance = 0;
long vitesse = 0;

long temps;

Servo monServo;
SoftwareSerial BTmavoieserie(Rx, Tx); // (RX, TX) (pin Rx BT, pin Tx BT)

// -------------------- BEGIN SETUP --------------
void setup() {
  //on initialise le temps
  temps = millis();
  
  // set all the other pins you're using as outputs:
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  // set the ultrasound detector
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);

  // set the servo motor
  monServo.attach(servoPin13, 1000, 2000);
  monServo.write(90);

  // basic control, verifiying is all motors are up & running
  Serial.println("Initialisation du systeme");
  //digitalWrite(enablePin, HIGH);
  //moveBackward(127);
  //moveForward(127);
  //moveStop();
  turn_left();
  turn_straight();
  turn_right();
  turn_straight();

  // Ouvre la voie série avec l'ordinateur
  Serial.begin(9600);
  // Ouvre la voie série avec le module BT
  BTmavoieserie.begin(9600);

  pinMode(key, OUTPUT);
  pinMode(key, LOW);
}

// -------------------- END SETUP --------------

// -------------------- BEGIN FUNCTIONS --------------
void turn_left() {
  // servo turn left
  monServo.write(0);
  Serial.println("Turning left");
//  delay(1000);
}

void turn_right() {
  // servo turn right
  monServo.write(180);
  Serial.println("Turning right");
//  delay(1000);
}

void turn_straight() {
  // servo goes ahead
  monServo.write(90);
//  delay(1000);
}

void moveForward(int vitesse) {
  // motor move forward
  digitalWrite(motorPin1, HIGH); // pin 10 (Input B) on L293D
  digitalWrite(motorPin2, LOW);  // pin 15 (Input A) on L293D
  analogWrite(enablePin, vitesse);
  Serial.print(vitesse);
  Serial.println(" blocks forward");
  //delay(2000);
}

void moveBackward(int vitesse) {
  //motor moves backward
  digitalWrite(motorPin1, LOW);  // pin 10 (Input B) on L293D
  digitalWrite(motorPin2, HIGH); // pin 15 (Input A) on L293D
  analogWrite(enablePin, vitesse);
  Serial.print(vitesse);
  Serial.println(" blocks backward");
  //delay(2000);
}

void moveStop() {
  //motor stops :
  digitalWrite(motorPin1, LOW);   // pin 10 (Input B) on L293D
  digitalWrite(motorPin2, LOW);  // pin 15 (Input A) on L293D
  Serial.println("Stop");
  //delay(2000);
}

int mesure_distance() {
  long time = 0;
  long distance = 0; 
  
  digitalWrite(trigger, LOW);
  delay(5);
  digitalWrite(trigger, HIGH);
  delay(10);
  digitalWrite(trigger, LOW);
  time = pulseIn(echo, HIGH);
  
  distance = (time/2) / 29.1;
  if ( distance >= 255 || distance <= 0)
  {
    Serial.println("No measurement");
    distance = 255;
  }
  return distance;
}

void DroneMode() {
  int aleatoire_vitesse;
  int aleatoire_angle;
  int aleas;
  distance = mesure_distance();
  vitesse = distance/1;

  aleas = random(2);
  
  if (distance<20) {
    moveBackward(127);
    if (aleas == 1) turn_left();
    else turn_right();
  }  
  else {
    if((millis() - temps) > 2000) {
      aleatoire_vitesse = random(127,256);
      aleatoire_angle = random(181);
      moveForward(aleatoire_vitesse);
      monServo.write(aleatoire_angle);
      Serial.print(aleatoire_angle);
      Serial.println(" degrees");
      temps = millis(); //on stocke la nouvelle heure
    }  
  }
}
// -------------------- END FUNCTIONS --------------

// -------------------- BEGIN LOOP --------------
void loop() {
  
  if (BTmavoieserie.available()) {
    Serial.print(BTmavoieserie.read());
  }
  switch (BTmavoieserie.read()) {
    case 8:
      moveForward(vitesse);
      break;
    case 2:
      moveBackward(vitesse);
      break;
    case 0:
      moveStop();
      break;
    case 4:
      turn_left();
      break;
    case 6:
      turn_right();
      break;

    default:
      // if nothing matches, go into drone mode
      DroneMode() ;
      break;
  }
}
