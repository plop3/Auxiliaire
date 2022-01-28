/*
  Auxiliaire: gestion de la position Park et des limites du télescope
  # Serge CLAUS
  # GPL V3
  # Version 6.0
  # 02/11/2018 / 27/01/2022

  Précision de MPU9250 ~ +/- 0.5°
*/

/* TODO
  Problème quand WiFi non disponible
  Mise au propre du code
 *  */

#include <Wire.h>

#include <EEPROM.h>

#define TELPOS    -1  // Sens de montage du télescope (1, -1)
#define TOLPARK   3   // Tolérance de park en degrés
#define TCAL      0.7 // Axe calibré
#define TOLALT    -30 // Tolérance limites (télescope baissé)
#define TOLROT    -20 //-11  // Tolérance AD, télescope horizontal TELESCOPE
#define TEMPOPARK 30  // Temporisation avant arret de la LED une fois le télescope parqué

#define D 8      // Taille en octets d'un double
#define LUM       4   // Luminosité des LEDs (Luminosité/LUM)
#define BOUTON  D7  // Bouton de calibrage de position park
#define PARK    D3  // Télescope parqué (Connecté à l'abri)
#define LIMIT   D4  // Limites
#define LED     D6  // LED APA106

// APA106
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel pixels(1, LED, NEO_RGB + NEO_KHZ400);

// Timer
#include <SimpleTimer.h>
SimpleTimer timer;

// OTA
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// Configuration WiFi
#include "WiFiP.h"
const char* ssid = STASSID;
const char* password = STAPSK;

// MPU6050
#include <math.h>
#include "mpu9250.h" // https://github.com/bolderflight/mpu9250
bfs::Mpu9250 imu(&Wire, 0x68);
int status;

// Telnet
#include "ESPTelnet.h"  
ESPTelnet telnet;

// Variables globales
double AltOffset, RotOffset = 0;
double AltPark, RotPark = 0;

double ALT; // Altitude télescope
double ROT; // Rotation de l'axe AD

int ETATB = 0;    // Etat du bouton poussoir
int TIMER;        // N° du timer
unsigned long TimerDebut;
bool Limites=false; // Limites atteintes

void setup() {
  Serial.begin(9600);

  EEPROM.begin(512);
  // Récupération des valeurs sauvegardées
  EEPROM.get(0 * D, AltPark);
  EEPROM.get(1 * D, RotPark);
  EEPROM.get(2 * D, AltOffset);
  EEPROM.get(3 * D, RotOffset);

  //Initialisation des sorties
  pinMode(BOUTON, INPUT_PULLUP);
  pinMode(PARK, OUTPUT);
  digitalWrite(PARK, LOW);
  digitalWrite(LIMIT, LOW);
  pinMode(LIMIT, INPUT);

  // MPU9250
  // start communication with IMU
  Wire.begin();
  Wire.setClock(400000);
  status = imu.Begin();
  if (status < 0) {
    Serial.print("IMU initialization unsuccessful: ");
    Serial.println(status);
  }
  imu.ConfigSrd(19);

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // OTA
  ArduinoOTA.setHostname("auxiliaire");
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // APA106
  pixels.begin();
  pixels.clear();
  TimerDebut = millis();  // Initialisation du timer pour la LED

delay(15000);
  // Telnet
  telnet.begin();
}

void loop() {
  timer.run();
  telnet.loop();
  String rep=String(ALT)+" "+String(ROT)+" offset: "+String(AltOffset)+" "+String(RotOffset)+" Park: "+String(AltPark)+" "+String(RotPark);
  telnet.println(rep);
  //telnet.print(ALT); telnet.print(" ");telnet.print(ROT);telnet.print(" Offset: ");
  //telnet.print(AltOffset); telnet.print(" ");telnet.print(RotOffset);telnet.print(" Park: ");
  //telnet.print(AltPark); telnet.print(" ");telnet.println(RotPark);

  if (ETATB == 0) {       // Mode surveillance
    ArduinoOTA.handle();
    if (!telPark()) telLimit();
    // Lecture des boutons
    if (!digitalRead(BOUTON)) { // Bouton appuyé
      if (!ETATB) {
        timer.setTimeout(5000, bouton5s);
        RVB(0, 0, 255);   // Bleu
        ETATB = 1;
      }
    }
    delay(100);
  }
  else {                  // Mode boutons
    gereBoutons();
  }
}


//convert the accel data to pitch/roll
void getAngle() {
  while (imu.Read() != 1 ); // TODO boucle bloquante, à modifier si possible
  //Lecture accéléromètres
  // Carte en position verticale: Z->X, X->Z)
  double z = imu.accel_x_mps2();
  double y = imu.accel_y_mps2();
  double x = imu.accel_z_mps2();
  ALT = -TELPOS * 57.3 * atan(x / sqrt((y * y) + (z * z)));
  ROT = TELPOS * 57.3 * atan(y / sqrt((x * x) + (z * z))); // TODO Tester si TELPOS est nécessaire sur cette ligne
}

void RVB(int R, int V, int B) {
  pixels.setPixelColor(0, pixels.Color(R / LUM, V / LUM, B / LUM));
  pixels.show();
}

void Clignote() {
  for (int i = 1; i < 5; i++) {
    RVB(255, 0, 0);
    delay(200);
    RVB(0, 0, 0);
    delay(200);
  }
}

void bouton5s() {
  if (!digitalRead(BOUTON)) {     // Bouton toujours appuyé
    ETATB++;
  }
}

void telLimit() {
  for (int i=0; i<2; i++) {
    // Tests de position du télescope
    if ((ROT - RotOffset) < TOLROT || (ALT - AltOffset) < TOLALT) {
    // Limites atteintes, 2e test
    getAngle();
    delay(200);
    }
    else {
      // Limites non atteintes
      if (Limites) {
        RVB(0,0,0); // Extinction de la LED limites
        Limites=false;
      }
      digitalWrite(LIMIT, LOW);
      pinMode(LIMIT, INPUT);
      
      return;
    }
  }
  telnet.println("*** LIMITES ***");
  Limites=true;
  digitalWrite(LIMIT, LOW);
  pinMode(LIMIT, OUTPUT);
  RVB(255, 0, 0); // Rouge
}

bool telPark() {
  // Test du park du télescope
  for (int i=0; i<2; i++) {
      getAngle();
    if (ALT <= (AltPark + TOLPARK) && ALT >= (AltPark - TOLPARK)  && ROT <= (RotPark + TOLPARK) && ROT >= (RotPark - TOLPARK)) {
      // Télescope parqué
      telnet.println("Park");
      digitalWrite(PARK, HIGH);
      if (millis() - TimerDebut >= TEMPOPARK * 1000L) {
        RVB(0, 0, 0); // LEDs éteintes
      }
      else {
        if (abs(ROT-RotPark) < TCAL && abs(ALT-AltPark) < TCAL) {
          // Deux axes calibrés
          RVB(0, 255, 0); // Vert
        }
        else if (abs(ALT-AltPark) < TCAL) {
          // Axe X calibré
          RVB(255, 0, 255);; // Orange
        }
        else if (abs(ROT-RotPark) < TCAL) {
          // Axe Y calibré
          RVB(255, 255, 0);  //Jaune
        }
        else {
          // Position Park
          RVB(0, 0, 255); // Bleu
        }
      }
      return true;
    }
    delay(200);
  }
  if (!Limites) RVB(0,0,0);
  TimerDebut = millis(); // Réinitialisation du timer pour la LED
  return false;
}

void gereBoutons() {
  // Lecture des boutons
  if (!digitalRead(BOUTON)) { // Bouton appuyé
    switch (ETATB) {
      case 1:
        ETATB++;
        break;
      case 3:   // Bouton appuyé depuis plus de 5s
        TIMER = timer.setTimeout(5000, bouton5s);
        RVB(237, 127, 16); // Orange
        ETATB++;
        break;
      case 5:   // Bouton appuyé depuis plus de 10s
        TIMER = timer.setTimeout(5000, bouton5s);
        ETATB++;
        RVB(255, 0, 0);
        break;
      case 7: // Bouton appuyé plus de 15s: annulation
        ETATB++;
        RVB(0, 0, 255);
        delay(3000);
        break;
    }
  }
  else {    // Bouton relaché
    if (timer.getNumTimers()) {
      timer.deleteTimer(TIMER);
    }
    switch (ETATB) {
      case 2:   // Bouton relaché avant 5s
        ETATB = 0;
        break;
      case 4:   // Position Park
        ETATB = 0;
        EEPROM.put(0 * D, ALT);
        EEPROM.put(1 * D, ROT);
        EEPROM.commit();
        AltPark = ALT;
        RotPark = ROT;
        Clignote();
        break;
      case 6:   // Calibrage
        ETATB = 0;
        EEPROM.put(2 * D, ALT);
        EEPROM.put(3 * D, ROT);
        EEPROM.commit();
        AltOffset = ALT;
        RotOffset = ROT;
        Clignote();
        break;
      case 8u: // Annulation
        ETATB = 0;
        break;
    }
  }
}
