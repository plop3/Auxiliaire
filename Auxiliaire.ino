/*
  # Serge CLAUS
  # GPL V3
  # Version 4.1
  # 02/11/2018 / 24/07/2019
*/

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

//Serveur Web
#include <ESP8266WebServer.h>
ESP8266WebServer server ( 80 );

#ifndef STASSID
#define STASSID "astro"
#define STAPSK  "B546546AF0"
#endif

const char* ssid = STASSID;
const char* password = STAPSK;

#include <Wire.h>
#include <EEPROM.h>

// MPU6050
#include <math.h>
#include "MPU9250.h"

MPU9250 IMU(Wire, 0x68);
int status;
double AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; //int16_t
double pitch, roll, orient;

#define BOUTON D3   // Bouton de calibrage de position park
#define BOFFSET D4  // Bouton Offset (position horizontale)
#define PARK D0     // Télescope parqué (Connecté à l'abri)
//#define LIMIT D0    // Limites (option)
#define LEDR D5     // LED indicateur position home
#define LEDV D6     // LED indicateur postion park de précision
#define LEDB D7     // LED indicateur position park OK

// Variables globales

double X;
double Y;
double Z;

int XOK;
int YOK;
int ZOK;

int OFX = 0;      // Offset sur l'axe X
int OFY = 0;      // Offset sur l'axe Y

int TOL = 3;      // Tolérance angle en degrés (accéléromètres
int TOLZ = 20;    // Tolérance rotation en degrés (magnétomètre)

int ETATB = 0;    // Etat du bouton poussoir

void setup() {
  Serial.begin(9600);
  Serial.println("Booting");
  EEPROM.begin(512);
  // Récupération des valeurs sauvegardées
  XOK = EEPROM.read(0) - 100;
  YOK = EEPROM.read(1) - 100;
  ZOK = EEPROM.read(2) - 100;
  OFX = EEPROM.read(3) - 100;
  OFY = EEPROM.read(4) - 100;

  //Initialisation des sorties
  pinMode(BOUTON, INPUT_PULLUP);
  pinMode(BOFFSET, INPUT_PULLUP);
  pinMode(PARK, OUTPUT);
  digitalWrite(PARK, LOW);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDV, OUTPUT);
  pinMode(LEDB, OUTPUT);
  RVB(0, 0, 0);

  // MPU9250
  // start communication with IMU
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
  }
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
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
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  // Serveur Web
  server.on("/axis", showAxis);
  server.begin();
}

void loop() {
  ArduinoOTA.handle();
  server.handleClient();
  if (ETATB == 0) {
    // MPU9250
    IMU.readSensor();
    double AcXoff, AcYoff, AcZoff, GyXoff, GyYoff, GyZoff;
    int temp, toff;
    double t, tx, tf;

    //read accel data
    AcX = IMU.getAccelX_mss();
    AcY = IMU.getAccelY_mss();
    AcZ = IMU.getAccelZ_mss();

    //read temperature data
    temp = IMU.getTemperature_C();

    //read gyro data
    GyX = IMU.getGyroX_rads();
    GyY = IMU.getGyroY_rads();
    GyZ = IMU.getGyroZ_rads();

    //get pitch/roll
    getAngle(AcX, AcY, AcZ);
    //X = -(pitch + OFX);
    //Y = -(roll + OFY); //-10;
    X = - pitch;
    Y = - roll;
    orient = IMU.getMagX_uT();
    Z = orient;
    /*
    //send the data out the serial port
    Serial.print("Orientation: "); Serial.print(IMU.getMagX_uT(), 6);
    Serial.print(" Angle: ");
    Serial.print("Pitch = "); Serial.print(pitch);
    Serial.print(" | Roll = "); Serial.println(roll);
    Serial.print("XOK: "); Serial.print(XOK);
    Serial.print(" YOK: "); Serial.print(YOK);
    Serial.print(" ZOK: "); Serial.print(ZOK);
    Serial.print(" XOF: "); Serial.print(OFX);
    Serial.print(" YOF: "); Serial.println(OFY);
    */
    // Orientation correcte dans les tolérances
    if (X > (XOK - TOL) && X < (XOK + TOL) && Y > (YOK - TOL) && Y < (YOK + TOL) && Z > (ZOK - TOLZ) && Z < (ZOK + TOLZ)) {
      bool ok = false;
      if (X > (XOK - 0.5) && X < (XOK + 0.5)) {
        ok = true;
        // Axe X parfait
        analogWrite(LEDV, 255);
      } else {
        analogWrite(LEDV, 0);
      }
      if (Y > (YOK - 0.5) && Y < (YOK + 0.5)) {
        ok = true;
        // Axe Y parfait
        analogWrite(LEDR, 255);
      } else {
        analogWrite(LEDR, 0);
      }
      Serial.println("Telescope parque");
      digitalWrite(PARK, HIGH);
      if (!ok) {
        analogWrite(LEDB, 255);
      } else {
        analogWrite(LEDB, 0);
      }

    }
    /*
    // Télescope en position "home"
    else if (X > (0 - 2) && X < (0 + 2) && Y > (-45 - 2) && Y < (-45 + 2) ) {
      RVB(255, 0, 0);
      digitalWrite(PARK, LOW);
    }
    */
    // Télescope non parqué
    else {
      RVB(0, 0, 0);
      digitalWrite(PARK, LOW);
    }
    if (!digitalRead(BOUTON)) { // Bouton de calibrage park
      ETATB = 1;
    }
    if (!digitalRead(BOFFSET)) {  // Bouton de calibrage offset
      ETATB = 2;
    }
    delay(500);
  }
  if (ETATB == 1) {
    Serial.println("Bouton");
    RVB(255, 255, 255);
    delay(1000);
    // Bouton toujours appuyé ?
    //On passe en orange
    if (!digitalRead(BOUTON)) {
      RVB(237, 127, 16);
      delay(3000);
      // Bouton relaché, on valide la position de park
      if (digitalRead(BOUTON)) {
        // Validation de la position de Park
        EEPROM.write(0,  - pitch + 100);
        EEPROM.write(1, - roll + 100);
        EEPROM.write(2, orient + 100);
        EEPROM.commit();
        XOK = - pitch;
        YOK = - roll;
        ZOK = orient;
        Clignote();
        ETATB = 0;
      }
    }
    // Relaché, on repasse en mode normal
    RVB(0, 0, 0);
    ETATB = 0;
    delay(2000);
  }
  if (ETATB == 2 ) {
    // Cablibrage des offsets
    EEPROM.write(3, pitch + 100);
    EEPROM.write(4, roll + 100);
    EEPROM.commit();
    OFX = pitch;
    OFY = roll;
    Clignote();
    RVB(0, 0, 0);
    ETATB = 0;
    delay(1000);
  }
}

void showAxis() {
  Serial.println("Affichage des axes ");
  server.send(200, "text/plain", String(X)+" "+String(Y)+" "+String(Z)+"\n");
}

//convert the accel data to pitch/roll
void getAngle(double Vx, double Vy, double Vz) {
  double x = Vx;
  double y = Vy;
  double z = Vz;

  pitch = atan(x / sqrt((y * y) + (z * z)));
  roll = atan(y / sqrt((x * x) + (z * z)));
  //convert radians into degrees
  pitch = pitch * (180.0 / 3.14);
  roll = roll * (180.0 / 3.14) ;
}

void RVB(int R, int V, int B) {
  analogWrite(LEDR, R);
  analogWrite(LEDV, V);
  analogWrite(LEDB, B);
}

void Clignote() {
  for (int i = 1; i < 5; i++) {
    RVB(255, 0, 0);
    delay(200);
    RVB(0, 0, 0);
    delay(200);
  }
}
