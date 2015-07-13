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
int OctetRecu = 0;

// ultra sound detector
int  trigger = 4;
int  echo = 2;
long distance = 0;
long vitesse = 0;

long temps;
long temps_recul;
int aleas; 

Servo monServo;
SoftwareSerial BTmavoieserie(Rx, Tx); // (RX, TX) (pin Rx BT, pin Tx BT)
NewPing sonar(trigger, echo, 255);

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
  long distance = 0; 
  long echoTime;

  echoTime = sonar.ping_median(5); //Do multiple pings (default=5), discard out of range pings and return median in microseconds 
  distance = sonar.convert_cm(echoTime);

  if ( distance >= 255 || distance <= 0)
  {
    Serial.println("Pas de mesure...");
    distance = 255;
  }
  else {
    Serial.print(distance);
    Serial.println(" cm de l'obstacle");
  }
  return distance;
}

void DroneMode() {
  int aleatoire_vitesse;
  int aleatoire_angle;
  
  distance = mesure_distance();
  vitesse = distance/1;

  if (distance<20) {
    moveBackward(127);

    if((millis() - temps_recul) > 5000) {
      aleas = random(2); // on tire un nouveau chiffre pour savoir si on tourne à gauche ou à droite.
      temps_recul = millis(); //on stocke la nouvelle heure
    }
    
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
    OctetRecu = BTmavoieserie.read();
    Serial.print(OctetRecu);
    Serial.println(" : par le canal BlueTooth");
  
    switch (OctetRecu) {
    // source https://play.google.com/store/apps/details?id=com.inex.BlueStickControl&hl=en
    //This application use bluetooth connection in Serial Port Profile (SPP).Send hex code to robot as follows :
    //0x30 = 0d48 = '0' = Stop
    //0x38 = 0d56 = '8' = Up
    //0x32 = 0d50 = '2' = Down
    //0x34 = 0d52 = '4' = Left
    //0x36 = 0d54 = '6' = Right
    //0x41 = 0d65 = 'A' = Auto Grab
    //0x42 = 0d66 = 'B' = Auto Release
    //0x43 = 0d67 = 'C' = Grab
    //0x44 = 0d68 = 'D' = Release
    //0x45 = 0d69 = 'E' = Rotate Left
    //0x46 = 0d70 = 'F' = Rotate Right
      case 38:
        moveForward(vitesse);
        Serial.println("en avant toute");        
        break;
      case 50:
        moveBackward(vitesse);
        Serial.println("en arrière toute");        
        break;
      case 48:
        digitalWrite(enablePin, LOW); // le moteur passe en roue libre
        break;
      case 0x52:
        turn_left();
        Serial.println("à gauche toute");
        break;
      case 6:
        turn_right();
        Serial.println("à droite toute");
        break;
  
      default:
        // if nothing matches, go into drone mode
        DroneMode() ;
        break;
    }
  }
}
