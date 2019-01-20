/*
  # Serge CLAUS
  # GPL V3
  # Version 2.1
  # 02/11/2018
*/

#include <Wire.h>
// MPU6050
#include <math.h>
#include "MPU9250.h"
#include <math.h>
MPU9250 IMU(Wire,0x68);
int status;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
double pitch, roll;

#define DHTPIN 12    // Pin sur lequel est branché le DHT
#define PARK A0      // Télescope parqué
#define WIRE 2      // 1-wire (T° miroir) /!\ D3 ne fonctionne pas (T° erronée)
#define CHAUF 11     // Chauffage (MOSFET)
#define LED 9       // LED indicateur de chauffage (Pour l'instant pas utilisé.)

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


// Variables globales
int XOK = 0;
int YOK = 0;
int TOL = 10;

void setup() {
  Serial.begin(9600);
  Serial.println("Booting");

  //Wire.begin();
  pinMode(CHAUF, OUTPUT);
  pinMode(PARK, OUTPUT);
  digitalWrite(PARK, LOW);

  // MPU9250
  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
   // while(1) {}
  }


  // 1wire
  sensors.begin();
  sensors.getAddress(therMir, 0);
  //sensors.setResolution(therMir, 9);
}

// Timers
unsigned long prevPark = 0;
unsigned long prevTemp = 0;

#define intervPark 1000
#define intervTemp 10000

void loop() {
  // Timers
  unsigned long curMillis = millis();

  if (curMillis - prevPark >= intervPark) {
    // Toutes les secondes
   // MPU9250
    IMU.readSensor();
    int AcXoff, AcYoff, AcZoff, GyXoff, GyYoff, GyZoff;
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
    int X=pitch;
    int Y=roll; //-10;
    //send the data out the serial port
    Serial.print("Angle: ");
    Serial.print("Pitch = "); Serial.print(pitch);
    Serial.print(" | Roll = "); Serial.println(roll);
    //    Vector normAccel = mpu.readNormalizeAccel();
    //int X = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis * normAccel.YAxis + normAccel.ZAxis * normAccel.ZAxis)) * 180.0) / M_PI; // pitch
    //int  Y = (atan2(normAccel.YAxis, normAccel.ZAxis) * 180.0) / M_PI;                                                      // roll
    if (X > (XOK - TOL) && X < (XOK + TOL) && Y > (YOK - TOL) && Y < (YOK + TOL)) {
      Serial.println("Telescope parque");
      if (!digitalRead(PARK)) {
        digitalWrite(PARK, HIGH);
      }
    }
    else {
      if (digitalRead(PARK)) {
        digitalWrite(PARK, LOW);
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
    Serial.print("T° ext: ");Serial.print(t);Serial.print(" H%: ");Serial.println(h);
//    if (!isnan(h) && !isnan(t)) {
      
//    }
    // T° miroir
    sensors.requestTemperatures();
    float tmir = sensors.getTempC(therMir);
    //float tmir = 12.50;
    Serial.print("T° miroir: ");Serial.println(tmir);
    if (tmir != -127 && tmir != 85) {
//      Serial.println("Temp: %f Hum: %f T miroir: %f", t, h, tmir);

      // Calcul du point de rosée
      float ptRosee = CalculRosee(t, h);
      //ptRosee=25.0;
//      Serial.println(" Point de rosee: %f", ptRosee);
      // T° miroir <= point de rosée +1° ?
      if (tmir <= (ptRosee + 1.5)) {
        Serial.println(" ON CHAUFFE");
        if (!digitalRead(CHAUF)) {
          digitalWrite(CHAUF, HIGH);
        }
      }
      else {
        if (digitalRead(CHAUF)) {
          digitalWrite(CHAUF, LOW);
        }
      }
    }
    else {
      Serial.println("Problème température miroir");
      if (digitalRead(CHAUF)) {
        digitalWrite(CHAUF, LOW);
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
