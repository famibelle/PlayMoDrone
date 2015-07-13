// Mesure l'intensité lumineuse reçue par les deux photodiodes connectées sur les sorties analogiques A01 et A1

int PhotoDIODEGauche = A0;
int PhotoDIODEDroite = A1;

int LumiereGauche;
int LumiereDroite;
void setup()
{
  pinMode(PhotoDIODEGauche, INPUT);
  digitalWrite(PhotoDIODEGauche, HIGH); // on passe la sortie en mode pullup pour faire l'économie d'une résistance
  
  pinMode(PhotoDIODEDroite, INPUT);
  digitalWrite(PhotoDIODEDroite, HIGH); // on passe la sortie en mode pullup pour faire l'économie d'une résistance

  Serial.begin(9600);
}

void loop()
{
	LumiereGauche = 1024 - analogRead(PhotoDIODEGauche); // Plus l'éclairage est fort, plus la valeur sera elevée
	LumiereDroite = 1024 - analogRead(PhotoDIODEDroite);
	
	Serial.print("PhotoCapteurGauche = "); // Prints the text inside the quotes.
	Serial.print(LumiereGauche); // Prints the value of the Left Sensor.
	Serial.print("\t"); // Prints a tab (space).
	Serial.print("PhotoCapteurDroite = "); // Prints the text inside the quotes.
	Serial.print(LumiereDroite); // Prints the value of the Right Sensor.
	Serial.print("\n"); // Prints a new line after all the necessary data is displayed.
}
