// Mesure l'intensité lumineuse reçue par les deux photodiodes connectées sur les sorties analogiques A01 et A1

#include <Servo.h>
#include <SoftwareSerial.h>
#include <NewPing.h>


#define START_CMD_CHAR '>'
#define END_CMD_CHAR '\n'
#define DIV_CMD_CHAR ','

int PhotoDIODEGauche = A0;
int PhotoDIODEDroite = A1;

int LumiereGauche;
int LumiereDroite;

float value0, value1, value2;

// Center servos
int tiltVal = 90; 
int panVal =90; 

// Bluetooth done with http://www.wikidebrouillard.org/index.php/Android_et_arduino and http://eskimon.fr/2498-arduino-annexes-g-utiliser-module-bluetooth-hc-05
// inpiration comes also from http://blog.miguelgrinberg.com/post/building-an-arduino-robot-part-vi-remote-control
// and http://www.techbitar.com/bluetooth-controlled-pan-tilt-servo.html

int motorPin1 = 7;    // pin 10 (Input A) on L293D
int motorPin2 = 8;    // pin 15 (Input B) on L293D
int enablePin = 6;    // PWM enable & pin 9 on L293D
int servoPin13 = 13;  // Servo Pin 

int Rx = 11;
int Tx = 10;
char OctetRecu;

// ultra sound detector
int  trigger = 4;
int  echo = 2;
long distance = 0;
long vitesse = 0;

long temps;
long temps_recul;
int aleas; 

int Drone = 12;
boolean DroneModeActivated = true;

Servo monServo;
SoftwareSerial BTmavoieserie(Rx, Tx); // (RX, TX) (pin Rx BT, pin Tx BT)
NewPing sonar(trigger, echo, 255);

// -------------------- BEGIN FUNCTIONS --------------

void tourne(int angle_rotation = 90) {
  monServo.write(angle_rotation); // 90 degré le drone va tout droit
}
void turn_left(int angle=0) {
  // servo turn left
  monServo.write(0);
  Serial.println("Turning left");
//  delay(1000);
}

void turn_right(int angle=180) {
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
  digitalWrite(Drone, HIGH);
  int aleatoire_vitesse;
  int aleatoire_angle;
  float angle_fuite = 0;
  int min_lum;
  
  distance = mesure_distance();
  vitesse = distance/1;

  if (distance<20) {
    while (distance <80) {
      distance = mesure_distance();
      moveBackward(255);
      if((millis() - temps_recul) > 5000) {
        aleas = random(2); // on tire un nouveau chiffre pour savoir si on tourne à gauche ou à droite.
        temps_recul = millis(); //on stocke la nouvelle heure
      }
      if (aleas == 1) turn_left();
      else turn_right();
      }
  }
  else {
    if((millis() - temps) > 1000) {
      aleatoire_vitesse = random(127,256);
      moveForward(aleatoire_vitesse);
      angle_fuite = ( LumiereGauche > (LumiereDroite + 100)) ? 180 : ((LumiereDroite > (LumiereGauche + 100)) ? 0 : 90);
      // angle_fuite = 90*(1+((float)LumiereGauche-(float)LumiereDroite)/(1024- (float)min_lum));
      tourne(angle_fuite); // tourne prend des valeurs entre 0 et 180, 0 => tourne à droite, 180 tourne à gauche, 90 va tout droit
      Serial.print("PhotoCapteurGauche = "); // Prints the text inside the quotes.
      Serial.print(LumiereGauche); // Prints the value of the Left Sensor.
      Serial.print("\t"); // Prints a tab (space).
      
      Serial.print("PhotoCapteurDroite = "); // Prints the text inside the quotes.
      Serial.print(LumiereDroite); // Prints the value of the Right Sensor.
      //Serial.print("\t"); // Prints a tab (space).
      Serial.print("\n"); // Prints a new line after all the necessary data is displayed.      
      Serial.print("Angle de Rotation = "); // Prints the text inside the quotes.
      Serial.print(angle_fuite); // Prints the value of the Right Sensor.
      Serial.print("\n"); // Prints a new line after all the necessary data is displayed.
      temps = millis(); //on stocke la nouvelle heure
    }
  }
}
// -------------------- END FUNCTIONS --------------


// -------------------- BEGIN SETUP --------------
void setup() {
  pinMode(PhotoDIODEGauche, INPUT);
  digitalWrite(PhotoDIODEGauche, HIGH); // on passe la sortie en mode pullup pour faire l'économie d'une résistance
  
  pinMode(PhotoDIODEDroite, INPUT);
  digitalWrite(PhotoDIODEDroite, HIGH); // on passe la sortie en mode pullup pour faire l'économie d'une résistance

  Serial.begin(9600);
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

  pinMode(Drone, OUTPUT);
  }

// -------------------- END SETUP --------------

void loop()
{
	LumiereGauche = 1024 - analogRead(PhotoDIODEGauche); // Plus l'éclairage est fort, plus la valeur sera elevée
	LumiereDroite = 1024 - analogRead(PhotoDIODEDroite);

  int sensorType = 0;
  unsigned long logCount = 0L;
   
  if (DroneModeActivated) digitalWrite(Drone, HIGH); else digitalWrite(Drone, LOW);
  
  if (BTmavoieserie.available()) {
    DroneModeActivated = false;
    OctetRecu = BTmavoieserie.read();
    //Serial.print(OctetRecu);
    //Serial.println(" : par le canal BlueTooth");
    if (OctetRecu == 'C') DroneModeActivated = !DroneModeActivated; //0x43 = 0d67 = 'C' = Grab

    if (OctetRecu != START_CMD_CHAR) {
      return;
    } // if no command start flag, return to loop().

    // parse incoming pin# and value 
    sensorType = BTmavoieserie.parseInt(); // read sensor typr
    logCount = BTmavoieserie.parseInt();  // read total logged sensor readings
    value0 = BTmavoieserie.parseFloat();  // 1st sensor value - value0 = X sensor reading - valeur variant de -10 à +10
    value1 = BTmavoieserie.parseFloat();  // 2rd sensor value - value1 = Y sensor reading - valeur variant de -10 à +10
    value2 = BTmavoieserie.parseFloat();  // 3rd sensor value

    panVal = ceil(value0*9/5)*5+90;    // a faire varier entre 0 et 180 par palier de 5 degres
    tiltVal = value1/10*255; // à faire varier entre 0 et 255
    Serial.print(panVal); 
    Serial.println(" : Pan Val (entre 0 et 180 degres)");
    Serial.print(tiltVal);
    Serial.println(" : Tilt Val");
    if (tiltVal>0) {moveForward(tiltVal);}
    if (tiltVal<0) {moveBackward(-tiltVal);}
    monServo.write(panVal);
  }
  if (DroneModeActivated) {
    DroneMode();
  }
  else {
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
      case '8':
        moveForward(255);
        Serial.println("en avant toute");        
        break;

      case '2':
        moveBackward(255);
        Serial.println("en arrière toute");        
        break;

      case '0':
        digitalWrite(enablePin, LOW); // le moteur passe en roue libre
        turn_straight(); // les roues se mettent droite
        break;

      case '4':
        turn_left();
        moveForward(255);
        Serial.println("à gauche toute");
        break;

      case '6':
        turn_right();
        moveForward(255);
        Serial.println("à droite toute");
        break;
    }
  }
}
