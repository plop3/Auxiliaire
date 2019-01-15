/*
  # Serge CLAUS
  # GPL V3
  # Version 0.2
  # 02/11/2018
*/

#include <Wire.h>
// MPU6050
#include <math.h>
const int MPU = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
double pitch, roll;

#define DHTPIN D5    // Pin sur lequel est branché le DHT
#define PARK D7      // Télescope parqué
#define WIRE D3      // 1-wire (T° miroir) /!\ D3 ne fonctionne pas (T° erronée)
#define CHAUF D6     // Chauffage (MOSFET)
#define LED D3       // LED indicateur de chauffage (Pour l'instant pas utilisé.)

// DHT22
#include "DHT.h"          // Librairie des capteurs DHT
#define DHTTYPE DHT22         // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE);

// 1wire
#include <OneWire.h>
#include <DallasTemperature.h>
OneWire oneWire(WIRE);
DallasTemperature sensors(&oneWire);
DeviceAddress therMir; //sensorDeviceAddress;

// Json
#include <ArduinoJson.h>

// OTA
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
const char* ssid = "astro";
const char* password = "1234567890";

WiFiClient espClient;

// MQTT
#include <PubSubClient.h>
#define mqtt_server "192.168.0.7"
bool debug = true;  //Affiche sur la console si True
int value = 0;
char msg[MQTT_MAX_PACKET_SIZE + 1];
char msgToPublish[MQTT_MAX_PACKET_SIZE + 1];
PubSubClient client(espClient);
#define mqtt_user ""  //s'il a été configuré sur Mosquitto
#define mqtt_password "" //idem

// Remote debug
#include "RemoteDebug.h"        //https://github.com/JoaoLopesF/RemoteDebug
RemoteDebug Debug;

// Variables globales
int XOK = 0;
int YOK = 0;
int TOL = 10;

void setup() {
  Serial.begin(9600);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    //ESP.restart();
  }
  ArduinoOTA.setHostname("auxiliaire");
  // ArduinoOTA.setPassword("admin");
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

  // Remote debug
  Debug.begin("auxiliaire.local"); // Initiaze the telnet server
  Debug.setResetCmdEnabled(true); // Enable the reset command
  //Debug.showTime(true); // To show time
  // Debug.showProfiler(true); // To show profiler - time between messages of Debug
  // Good to "begin ...." and "end ...." messages

  //Wire.begin();
  pinMode(CHAUF, OUTPUT);
  pinMode(PARK, OUTPUT);
  digitalWrite(PARK, LOW);

  // MPU6050
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);


  // 1wire
  sensors.begin();
  sensors.getAddress(therMir, 0);
  //sensors.setResolution(therMir, 9);

  // MQTT
  client.setServer(mqtt_server, 1883);    //Configuration de la connexion au serveur MQTT
  client.setCallback(callback);  //La fonction de callback qui est executée à chaque réception de message
  if (!client.connected()) {
    reconnect();
  }
  sendMsg("{\"command\": \"switchlight\", \"idx\":  823, \"switchcmd\": \"Off\"}");
  sendMsg("{\"command\": \"switchlight\", \"idx\":  824, \"switchcmd\": \"Off\"}");
  sendMsg("{\"command\": \"switchlight\", \"idx\":  846, \"switchcmd\": \"Off\"}");
}

// Timers
unsigned long prevPark = 0;
unsigned long prevTemp = 0;

#define intervPark 1000
#define intervTemp 10000

void loop() {
  ArduinoOTA.handle();

  // Remote debug
  Debug.handle();

  // MQTT
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  // Timers
  unsigned long curMillis = millis();

  if (curMillis - prevPark >= intervPark) {
    // Toutes les secondes
    // MPU6050
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 14, true);

    int AcXoff, AcYoff, AcZoff, GyXoff, GyYoff, GyZoff;
    int temp, toff;
    double t, tx, tf;

    //Acceleration data correction
    AcXoff = -950;
    AcYoff = -300;
    AcZoff = 0;

    //Temperature correction
    toff = -1600;

    //Gyro correction
    GyXoff = 480;
    GyYoff = 170;
    GyZoff = 210;

    //read accel data
    AcX = (Wire.read() << 8 | Wire.read()) + AcXoff;
    AcZ = (Wire.read() << 8 | Wire.read()) + AcZoff;
    AcY = (Wire.read() << 8 | Wire.read()) + AcYoff;

    //read temperature data
    temp = (Wire.read() << 8 | Wire.read()) + toff;
    tx = temp;
    t = tx / 340 + 36.53;
    tf = (t * 9 / 5) + 32;

    //read gyro data
    GyX = (Wire.read() << 8 | Wire.read()) + GyXoff;
    GyY = (Wire.read() << 8 | Wire.read()) + GyYoff;
    GyZ = (Wire.read() << 8 | Wire.read()) + GyZoff;

    //get pitch/roll
    getAngle(AcX, AcY, AcZ);
    int X=pitch;
    int Y=roll-10;
    //send the data out the serial port
    //Serial.print("Angle: ");
    //Serial.print("Pitch = "); Serial.print(pitch);
    //Serial.print(" | Roll = "); Serial.println(roll);
    //    Vector normAccel = mpu.readNormalizeAccel();
    //int X = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis * normAccel.YAxis + normAccel.ZAxis * normAccel.ZAxis)) * 180.0) / M_PI; // pitch
    //int  Y = (atan2(normAccel.YAxis, normAccel.ZAxis) * 180.0) / M_PI;                                                      // roll
    rdebugVln("X: %i Y: %i", X, Y);
    if (X > (XOK - TOL) && X < (XOK + TOL) && Y > (YOK - TOL) && Y < (YOK + TOL)) {
      rdebugVln("Telescope parque");
      if (!digitalRead(PARK)) {
        digitalWrite(PARK, HIGH);
        sendMsg("{\"command\": \"switchlight\", \"idx\":  824, \"switchcmd\": \"On\"}");
      }
    }
    else {
      if (digitalRead(PARK)) {
        digitalWrite(PARK, LOW);
        sendMsg("{\"command\": \"switchlight\", \"idx\":  824, \"switchcmd\": \"Off\"}");
      }
    }
    prevPark = curMillis;
  }
  if (curMillis - prevTemp >= intervTemp) {
    // Toutes les 10 secondes
    // Mesure T° / humidité
    float h = dht.readHumidity();
    // Lecture de la température en Celcius
    float t = dht.readTemperature();
    if (!isnan(h) && !isnan(t)) {
      String msgtel = "{\"command\": \"udevice\", \"idx\": 822, \"nvalue\": 0, \"svalue\": \"";
      msgtel = msgtel + t;
      String msg2 = ";";
      msg2 = msg2 + h;
      msg2 = msg2 + ";0\"}";
      msgtel = msgtel + msg2;
      //msgtel="{\"command\": \"udevice\", \"idx\": 822, \"nvalue\": 0, \"svalue\": \"12;50;0\"}";
      sendMsg(msgtel);
    }
    // T° miroir
    sensors.requestTemperatures();
    float tmir = sensors.getTempC(therMir);
    //float tmir = 12.50;
    if (tmir != -127 && tmir != 85) {
      String msgmir = "{\"command\": \"udevice\", \"idx\": 821, \"nvalue\": 0, \"svalue\": \"";
      msgmir = msgmir + tmir;
      msgmir = msgmir + "\"}";
      sendMsg(msgmir);
      rdebugVln("Temp: %f Hum: %f T miroir: %f", t, h, tmir);

      // Calcul du point de rosée
      float ptRosee = CalculRosee(t, h);
      //ptRosee=25.0;
      rdebugV(" Point de rosee: %f", ptRosee);
      // T° miroir <= point de rosée +1° ?
      if (tmir <= (ptRosee + 1.5)) {
        rdebugVln(" ON CHAUFFE");
        if (!digitalRead(CHAUF)) {
          digitalWrite(CHAUF, HIGH);
          sendMsg("{\"command\": \"switchlight\", \"idx\":  823, \"switchcmd\": \"On\"}");
        }
      }
      else {
        if (digitalRead(CHAUF)) {
          digitalWrite(CHAUF, LOW);
          sendMsg("{\"command\": \"switchlight\", \"idx\":  823, \"switchcmd\": \"Off\"}");
        }
      }
    }
    else {
      rdebugVln("Problème température miroir");
      if (digitalRead(CHAUF)) {
        digitalWrite(CHAUF, LOW);
        sendMsg("{\"command\": \"switchlight\", \"idx\":  823, \"switchcmd\": \"Off\"}");
      }
    }
    prevTemp = curMillis;

  }

}

float CalculRosee(float t, float h) {
  float ptr = (17.27 * t) / (237.7 + t) + log(h * 0.01);
  return ((237.7 * ptr) / (17.27 - ptr));
  // return t-((100.0-h)/5.0);  Autre calcul plus approximatif...
}


//sendMsg("{\"command\": \"udevice\", \"idx\": 795, \"nvalue\": 2, \"svalue\": \"Initialisation...\"}");
//sendMsg("{\"command\": \"switchlight\", \"idx\":  818, \"switchcmd\": \"Off\"}");

void sendMsg(String message) {
  rdebugVln("Envoi message");
  message.toCharArray( msgToPublish, MQTT_MAX_PACKET_SIZE);
  int r = client.publish("domoticz/in", msgToPublish );
  rdebugVln("Retour: %i", r);
}

//Reconnexion
void reconnect() {
  //Boucle jusqu'à obtenur une reconnexion
  while (!client.connected()) {
    rdebugVln("Connexion au serveur MQTT...");
    if (client.connect("Auxiliaire", mqtt_user, mqtt_password)) {
      rdebugVln("OK");
      client.subscribe("domoticz/out");
    } else {
      rdebugVln("KO, erreur : ");
      Serial.print(client.state());
      rdebugVln(" On attend 5 secondes avant de recommencer");
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String messageReceived = "";
  //rdebugVln("**** MESSAGE ****");
  DynamicJsonBuffer jsonBuffer( MQTT_MAX_PACKET_SIZE );
  for (int i = 0; i < length; i++) {
    messageReceived += ((char)payload[i]);
  }
  // if domoticz message
  if ( strcmp(topic, "domoticz/out") == 0 ) {
    JsonObject& root = jsonBuffer.parseObject(messageReceived);
    if (!root.success()) {
      Serial.println("parsing Domoticz/out JSON Received Message failed");
      return;
    }
    String idx = root["idx"];
    /*
      if ( idx == "846" ) {       // index du message
      const char* cmde = root["nvalue"];
      if ( strcmp(cmde, "1") == 0 ) { // 0 means we have to switch OFF the lamps
      // Switch OFF
          delay(500);
        }
      }
      }
    */
  }
}


//convert the accel data to pitch/roll
void getAngle(int Vx,int Vy,int Vz) {
double x = Vx;
double y = Vy;
double z = Vz;

pitch = atan(x/sqrt((y*y) + (z*z)));
roll = atan(y/sqrt((x*x) + (z*z)));
//convert radians into degrees
pitch = pitch * (180.0/3.14);
roll = roll * (180.0/3.14) ;

}
