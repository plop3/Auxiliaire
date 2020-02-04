/*
  # Serge CLAUS
  # GPL V3
  # Version 4.2
  # 02/11/2018 / 29/01/2020
*/

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

//Serveur Web
#include <ESP8266WebServer.h>
ESP8266WebServer server ( 80 );

//#ifndef STASSID
#define STASSID "astro"
#define STAPSK  "B546546AF0"
//#endif

const char* ssid = STASSID;
const char* password = STAPSK;

#include <Wire.h>
#include <EEPROM.h>

// MPU6050
#include <math.h>
#include "MPU9250.h"

MPU9250 IMU(Wire, 0x68);

#define BOUTON D3   // Bouton de calibrage de position park
#define BOFFSET D4  // Bouton Offset (position horizontale)
#define PARK D8     // Télescope parqué (Connecté à l'abri)
#define LIMIT D0    // Limites (option)
#define LEDR D5     // LED indicateur position home
#define LEDV D6     // LED indicateur postion park de précision
#define LEDB D7     // LED indicateur position park OK

#define TOL 3       // Tolérance de park en degrés
#define TCAL 0.5    // Axe calibré
#define TOLH 7      // Tolérance home
#define TOLZ 50     // Tolérance boussole en degrés
#define TOLAZ 1     // Tolérance magnétomètre Z en g
#define TOLLIMX   -15 // Tolérance limites (télescope baissé)
//#define TOLLIMYH  -11 // Tolérance AD, télescope horizontal TELESCOPE
//#define TOLLIMYV  -4.5  // Tolérance AD, télescope vertical TELESCOPE
#define TOLLIMYH  -20 // Tolérance AD, télescope horizontal   LUNETTE
#define TOLLIMYV  -20  // Tolérance AD, télescope vertical    LUNETTE

#define TVERT     11  // Angle (90+/- TVERT) télescope considéré vertical  
#define VMIN      58  // Hauteur mini vertical
#define VMAX      80  // Hauteur maxi vertical

#define D 8			// Taille en octets d'un double

// Variables globales

int status;
double AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; //int16_t
double pitch, roll, cote, angle, yaw;

double X;
double Y;
double Z;
double AZ;

double XOK;
double YOK;
double ZOK;
double AOK;

double OFX = 0;      // Offset sur l'axe X
double OFY = 0;      // Offset sur l'axe Y

bool park, xx, yy, limit, home = false;
bool LimitStatus = true;

int ETATB = 0;    // Etat du bouton poussoir

void setup() {
  Serial.begin(9600);
  Serial.println("Booting");
  EEPROM.begin(512);
  // Récupération des valeurs sauvegardées
  EEPROM.get(0 * D, XOK);
  EEPROM.get(1 * D, YOK);
  EEPROM.get(2 * D, ZOK);
  EEPROM.get(3 * D, OFX);
  EEPROM.get(4 * D, OFY);
  EEPROM.get(5 * D, AOK);

  //Initialisation des sorties
  pinMode(BOUTON, INPUT_PULLUP);
  pinMode(BOFFSET, INPUT_PULLUP);
  pinMode(PARK, OUTPUT);
  digitalWrite(PARK, LOW);
  pinMode(LIMIT, INPUT);
  digitalWrite(LIMIT, LOW);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDV, OUTPUT);
  pinMode(LEDB, OUTPUT);
  RVB(255, 255, 255);

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
  int nb = 5;
  while ((WiFi.waitForConnectResult() != WL_CONNECTED) && (nb > 0)) {
    nb--;
    Serial.println("Connection Failed! Restart...");
    delay(5000);
    //ESP.restart();
  }

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
  server.on("/status", showStatus);
  server.on("/set", setPark);
  server.on("/lim", setLimit);
  server.begin();

}

void loop() {
  ArduinoOTA.handle();
  server.handleClient();

  if (ETATB == 0) {
    // MPU9250
    IMU.readSensor();
    double AcXoff, AcYoff, AcZoff, GyXoff, GyYoff, GyZoff, MagX, MagY, MagZ;
    int temp, toff;
    double t, tx, tf;

    //read accel data
    AcX = IMU.getAccelX_mss();
    AcY = IMU.getAccelY_mss();
    AcZ = IMU.getAccelZ_mss();
    AZ = AcZ;

    //read temperature data
    temp = IMU.getTemperature_C();

    //read gyro data
    GyX = IMU.getGyroX_rads();
    GyY = IMU.getGyroY_rads();
    GyZ = IMU.getGyroZ_rads();

    //get pitch/roll
    getAngle(AcX, AcY, AcZ); 

    MagX = IMU.getMagX_uT();
    MagY = IMU.getMagY_uT();
    MagZ = IMU.getMagZ_uT();
    yaw = getYaw(MagX, MagY, MagZ);
    angle=AcZ;
    cote=MagZ;
    
    X = pitch - OFX;
    Y = roll - OFY;
    Z = yaw;
    Serial.print(String(X) + ',' + String(Y));
    Serial.print(" " + String(AcZ));

    Serial.println();
    /*
        //send the data out the serial port
        Serial.print("Orientation: "); Serial.print(Z, 6);
        Serial.print(" Angle: ");
        Serial.print("Pitch = "); Serial.print(X);
        Serial.print(" | Roll = "); Serial.println(Y);
        Serial.print("XOK: "); Serial.print(XOK);
        Serial.print(" YOK: "); Serial.print(YOK);
        Serial.print(" ZOK: "); Serial.print(ZOK);
        Serial.print(" XOF: "); Serial.print(OFX);
        Serial.print(" YOF: "); Serial.println(OFY);
        delay(1000);
    */
    // Park Ok
    park = (AZ > (AOK - TOLAZ) && (AZ < (AOK + TOLAZ) && X > (XOK - TOL) && X < (XOK + TOL) && Y > (YOK - TOL) && Y < (YOK + TOL) && Z > (ZOK - TOLZ) && Z < (ZOK + TOLZ)) ? true : false);
    // Valeur X OK
    xx = ((X > (XOK - TCAL) && X < (XOK + TCAL)) ? true : false);
    yy = ((Y > (YOK - TCAL) && Y < (YOK + TCAL)) ? true : false);

    // Position home
    home = ((Y > (50 - TOLH) && Y < (50 + TOLH) && X > (45 - TOLH) && X < (45 + TOLH) ) ? true : false);

    
    // Hors limites
    
    if (LimitStatus) {
      if (X>VMIN && X<VMAX && (((cote<-50) && (angle >0)) || ((cote>-30 && angle<0)))) {
        limit = ((X < TOLLIMX) || (Y < TOLLIMYV) ? true : false );
      }
      else {
        limit = ((X < TOLLIMX) || (Y < TOLLIMYH) ? true : false );
      }
      if (limit) {
        // Limites
        Serial.println("Limites");
        digitalWrite(LIMIT, LOW);
        pinMode(LIMIT, OUTPUT);
        RVB(255, 0, 0); // Rouge
      }
      else {
        pinMode(LIMIT, INPUT);
        digitalWrite(LIMIT, LOW);
      }
    }
    else {
      limit=false;
    }
    if (home) {
      // Home
      Serial.println("Home");
      //RVB(237, 127, 16);
    }
    if (park) {
      Serial.println("Park");
      digitalWrite(PARK, HIGH);
      if (xx && yy) {
        // Deux axes calibrés
        RVB(0, 255, 0); // Vert
      }
      else if (xx) {
        // Axe X calibré
        RVB(255, 0, 255);; // Orange
      }
      else if (yy) {
        // Axe Y calibré
        RVB(255, 255, 0);  //Jaune

      }
      else {
        // Position Park
        RVB(0, 0, 255); // Bleu
      }
    }
    else {
      // Télescope non parqué
      digitalWrite(PARK, LOW);
      if (!home && !limit) RVB(0, 0, 0); // LEDs éteintes
    }



    // Lecture des boutons
    if (!digitalRead(BOUTON)) { // Bouton de calibrage park
      ETATB = 1;
    }
    if (!digitalRead(BOFFSET)) {  // Bouton de calibrage offset
      ETATB = 2;
    }
    delay(200);
  }
  if (ETATB == 1) {
    Serial.println("Bouton");
    RVB(237, 237, 237);
    delay(1000);
    // Bouton toujours appuyé ?
    //On passe en orange
    if (!digitalRead(BOUTON)) {
      RVB(102, 0, 153);
      delay(3000);
      // Bouton relaché, on valide la position de park
      if (digitalRead(BOUTON)) {
        // Validation de la position de Park
        EEPROM.put(0 * D, X);
        EEPROM.put(1 * D, Y);
        EEPROM.put(2 * D, Z);
        EEPROM.put(5 * D, AZ);
        EEPROM.commit();
        XOK = X;
        YOK = Y;
        ZOK = Z;
        AOK = AZ;
        Clignote();
        ETATB = 0;
      }
    }
    // Relaché, on repasse en mode normal
    RVB(0, 0, 0);
    ETATB = 0;
    delay(500);
  }
  if (ETATB == 2 ) {
    // Cablibrage des offsets
    EEPROM.put(3 * D, pitch);
    EEPROM.put(4 * D, roll);
    EEPROM.commit();
    OFX = pitch;
    OFY = roll;
    Clignote();
    ETATB = 0;
    delay(500);
  }
}

void showAxis() {
  Serial.println("Affichage des axes ");
  server.send(200, "text/plain", String(X) + " " + String(Y) + " " + String(Z) + "\t"+String(angle)+","+String(cote)+"\n");
}

void showStatus() {
  Serial.println("Affichage du status du télescope ");
  String status = "N";
  if (limit) status = "L";
  if (home) status = "H";
  if (park) status = "P";
  server.send(200, "text/plain", status + "\n");
}

void setLimit() {
  // Active/désactive les limites
  String state;
  state = server.arg("enable");
  if (state == "0") {
    LimitStatus = false;
    server.send ( 200, "text/html", "inactif");
  }
  else if (state == "1") {
    LimitStatus = true;
    server.send ( 200, "text/html", "actif");
  }
}

void setPark() {
  // Valide la nouvelle position de park
  EEPROM.put(0 * D, X);
  EEPROM.put(1 * D, Y);
  EEPROM.put(2 * D, Z);
  EEPROM.put(5 * D, AZ);
  EEPROM.commit();
  XOK = X;
  YOK = Y;
  ZOK = Z;
  AOK = AZ;
  server.send(200, "text/plain", "set" + String(XOK) + "," + String(YOK) + "," + String(ZOK) + "\t" + String(AcY) + "\n");
  Clignote();
}
//convert the accel data to pitch/roll
void getAngle(double Vx, double Vy, double Vz) {
  double x = Vx;
  double y = Vy;
  double z = Vz;

  pitch = atan(x / sqrt((y * y) + (z * z)));
  roll = atan(y / sqrt((x * x) + (z * z)));
  //convert radians into degrees
  pitch = - pitch * (180.0 / 3.14);
  roll = - roll * (180.0 / 3.14) ;
}

double getYaw(double magX, double magY, double magZ) {
  float Yh = (magY * cos(radians(roll))) - (magZ * sin(radians(roll)));
  float Xh = (magX * cos(radians(pitch))) + (magY * sin(radians(roll)) * sin(radians(pitch))) + (magZ * cos(radians(roll)) * sin(radians(pitch)));
  //return  57.3*(atan2(Yh, Xh));
  //return magX * 3.6;
  return  abs(57.3 * (atan2(Yh, Xh)));
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
