#include <PowerFunctions.h>
#include <LCD.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Stepper.h>
//Libaries Ende
//Stepmotor
#define STEPS_PER_MOTOR_REVOLUTION 32
#define STEPS_PER_OUTPUT_REVOLUTION 32 * 64
Stepper small_stepper(STEPS_PER_MOTOR_REVOLUTION, 3, 5, 4, 6);
int  Steps2Take;

//State MAchine
//Variablen zum Speicher der vergangene Zeit
unsigned long previousMillisScannen = 0;
unsigned long previousMillisServo = 0;
unsigned long previousMillisMG = 0;
unsigned long previousMillisTurm = 0;
//Zeiten
long zeitScannen = 300;
long zeitServo = 500;
long zeitMG = 500;
long zeitTurm = 15000;
//Zustände
String zustandServo = "lmitte";
String zustandMG = "lmitte";
bool zustandFahren = false;

//Pin Setup
//LCD
LiquidCrystal_I2C  lcd(0x3F, 2, 1, 0, 4, 5, 6, 7);
//LED
int LEDt = 8;
//Servos
Servo sensorServo; //Servo um den Sensor zu drehen
Servo mgServo; //Servo um das MG zu drehen
//Powerfunctions
PowerFunctions pf( 12, 0);

//Ultraschallsensor
const int trigPin = 9;
const int echoPin = 10;
long duration = 0;
int distance = 150;

//Ende Pin Setup

void setup() {
  //Ultraschallsensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //LED
  pinMode(LEDt, OUTPUT);

  //LCD Display
  lcd.begin(16, 2);
  lcd.setBacklightPin(3, POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.print("Panzer 16 OS 1.0");

  //Servo
  sensorServo.attach(A5);
  sensorServo.write(90);
  mgServo.attach(A4);
  mgServo.write(90);
}



void loop() {
  unsigned long currentMillis = millis(); //

  if (!zustandFahren) {
    fahre(5);
  }

  //State Machine Scannen
  if (currentMillis - previousMillisScannen >= zeitScannen)
  {
    messen();
    if (di   stance < 120) {
      fahre(0);
      delay(2000);
      ausweichen();
    }
    previousMillisScannen = currentMillis;  // Remember the time

  }

  //State Machine Servo für Sensor
  if (currentMillis - previousMillisServo >= zeitServo && zustandServo == "rechts")
  {
    sensorServo.write(90); //Setze Servo auf Position Links
    zustandServo = "rmitte";
    previousMillisServo = currentMillis; // Remember the time
  }
  else if (currentMillis - previousMillisServo >= zeitServo && zustandServo == "links") {
    sensorServo.write(90); //Setze Servo auf Position Rechts
    zustandServo = "lmitte";
    previousMillisServo = currentMillis; // Remember the time
  }
  else if (currentMillis - previousMillisServo >= zeitServo && zustandServo == "lmitte") {
    sensorServo.write(150); //Setze Servo auf Position Rechts
    zustandServo = "rechts";
    previousMillisServo = currentMillis; // Remember the time
  }
  else if (currentMillis - previousMillisServo >= zeitServo && zustandServo == "rmitte") {
    sensorServo.write(50); //Setze Servo auf Position Rechts
    zustandServo = "links";
    previousMillisServo = currentMillis; // Remember the time
  }

  /*/State Machine MG
    if (currentMillis - previousMillisMG >= zeitMG && zustandMG == "rechts")
    {
    mgServo.write(90); //Setze Servo auf Position Links
    zustandMG = "rmitte";
    previousMillisMG = currentMillis; // Remember the time
    }
    else if (currentMillis - previousMillisMG >= zeitMG && zustandMG == "links") {
    sensorServo.write(90); //Setze Servo auf Position Rechts
    mgServo = "lmitte";
    previousMillisMG = currentMillis; // Remember the time
    }
    else if (currentMillis - previousMillisMG >= zeitMG && zustandMG == "lmitte") {
    sensorServo.write(150); //Setze Servo auf Position Rechts
    mgServo = "rechts";
    previousMillisMG = currentMillis; // Remember the time
    }
    else if (currentMillis - previousMillisMG >= zeitMG && zustandMG == "rmitte") {
    sensorServo.write(50); //Setze Servo auf Position Rechts
    mgServo = "links";
    previousMillisMG = currentMillis; // Remember the time
    }


    //State Machine Turm
    if (currentMillis - previousMillisTurm >= zeitTurm)
    {
    dreheTurm();
    previousMillisTurm = currentMillis; // Remember the time
    }
  */
}

void messen() { //Misst die Distanz zu Hinderniss
  char line[16];
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.34 / 2 - 5 ;
  // Prints the distance on the Serial Monitor
  sprintf(line, "Dist:    % 7d", distance);
  lcd.setCursor (0, 0);
  lcd.print(line);
}

//fahren
void fahre(int geschwindigkeit) {


  int red[] = { PWM_REV1, PWM_REV2, PWM_REV3, PWM_REV4, PWM_REV5, PWM_REV6, PWM_REV7 }; //Arays um die Geschwindigkeit zu  definieren
  int blu[] = { PWM_FWD1, PWM_FWD2, PWM_FWD3, PWM_FWD4, PWM_FWD5, PWM_FWD6, PWM_FWD7 };

  char line2[16];

  if (geschwindigkeit == 0) { //Bremsen
    Brake(BLUE);
    Brake(RED);
    zustandFahren = false;
    lcd.setCursor (0, 1);
    sprintf(line2, "Speed: % 3d", geschwindigkeit);
    lcd.print(line2);
  }
  else if (-7 <= geschwindigkeit && geschwindigkeit <= 7 ) { //Vorwärts oder Rückwärts fahren
    zustandFahren = true;
    Drive(RED,  (geschwindigkeit > 0) ? (red[geschwindigkeit - 1]) : ( blu[abs(geschwindigkeit) - 1]) );
    Drive(BLUE, (geschwindigkeit > 0) ? (blu[geschwindigkeit - 1]) : ( red[abs(geschwindigkeit) - 1]) );
    lcd.setCursor (0, 1);
    sprintf(line2, "Speed: % 3d", geschwindigkeit);
    lcd.print(line2);
  }
  else { //Fehler
    lcd.setCursor (0, 1);
    lcd.print("Stop            ");
  }

}

//Drehen um circa 90 Grad
void dreheRechts() {
  Drive(RED, PWM_FWD4);
  Drive(BLUE, PWM_FWD4);
  delay(2000);
  Brake(RED);
  Brake(BLUE);
}

void dreheLinks() {
  Drive(BLUE, PWM_FWD4);
  Drive(RED, PWM_FWD4);
  delay(2000);
  Brake(RED);
  Brake(BLUE);
}

void ausweichen() { //Setzt zurück und dreht um 90 Grad
  fahre(-3);
  delay(1000);
  fahre(0);
  dreheLinks();

}
/*
  void dreheTurm() { //Dreht den Turm um 90 Grad im Uhrzeigersinn, 180Grad gegen den Uhrzeigersinn und dann nochmal 90 Grad im Uhrzeigersinn
  fahre(0);
  delay(500);
  Steps2Take  =  STEPS_PER_OUTPUT_REVOLUTION / 2;
  small_stepper.setSpeed(700);
  small_stepper.step(Steps2Take);
  delay(1000);
  Steps2Take  = -2 * STEPS_PER_OUTPUT_REVOLUTION ;
  small_stepper.setSpeed(700);
  small_stepper.step(Steps2Take);
  delay(1000);
  Steps2Take  =  STEPS_PER_OUTPUT_REVOLUTION / 2;
  small_stepper.setSpeed(700);
  small_stepper.step(Steps2Take);
  delay(1000);
  }
*/


void Drive(uint8_t output, uint8_t pwm) { //Methode welche die Lego Motoren drehen lässt
  pf.single_pwm(output, pwm);
}


void Brake(uint8_t output) {  //Methode um die Lego Motor zu bremsen
  pf.single_pwm(output, PWM_BRK);
  delay(30);
  pf.single_pwm(output, PWM_FLT);
}


void ledein() {
  digitalWrite(LEDt, HIGH); //Schaltet den TurmScheinwerfer ein
}
void ledaus() {
  digitalWrite(LEDt, LOW); //Schaltet den TurmScheinwerfer aus
}



