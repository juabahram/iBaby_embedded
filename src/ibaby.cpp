#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include "TinyGPS++.h"
#include "HardwareSerial.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "VL53L1X.h"

const char* ssid = "Juanito";
const char* password = "olelaferia";
//const char* mqtt_server = "mqtt.eclipseprojects.io";
const char* mqtt_server = "10.164.114.170";

const int STROLLERID = 1;

WiFiClient wifiClientTx;

PubSubClient client(wifiClientTx);

String buzz="";

//DHT11
#define DHT_PIN 17
#define DHTTYPE DHT11

float temp;
float hum;

DHT dht(DHT_PIN, DHTTYPE);

//GPS
#define TXD2 2
#define RXD2 4

#define GPS_BAUD 9600

TinyGPSPlus gps;

//pressure
#define PRESS_PIN 33

//MPU6050 Accel/gyro
MPU6050 sensor;

int16_t ax, ay, az;
int16_t gx, gy, gz;
float deg_x, deg_y;
float x_offset, y_offset;

//air particles
byte dataH, dataL;
int data;

//TOF
VL53L1X tof;

//BUZZER
#define BUZZER_PIN 12

//LUZ
#define LDR_PIN 32
int inputLDR;

//LED
#define LED_PIN 27

static const uint64_t UPDATE_INTERVAL = 4000;
unsigned long lastUpdate =0;

bool isFirst = true;


void actuatorsLogic(){
  String payload;

  if(inputLDR > 350 || temp > 33 || hum > 90){
    digitalWrite(LED_PIN, HIGH);
    sleep(1);
    digitalWrite(LED_PIN, LOW);
    sleep(1);
    digitalWrite(LED_PIN, HIGH);
    sleep(1);
    digitalWrite(LED_PIN, LOW);
    sleep(1);
    digitalWrite(LED_PIN, HIGH);
    sleep(1);
    digitalWrite(LED_PIN, LOW);
    payload = "led:ON:1:"+String(STROLLERID);

  } else{
    digitalWrite(LED_PIN, LOW);
    payload = "led:OFF:1:"+String(STROLLERID);
  }
    client.publish("ibaby/actuators/LED", payload.c_str());
}

void publishSensors(){
  //TEMP
  temp = dht.readTemperature();
  // Publish data on theme
  String payload = "temperatura:"+String(temp)+":1"+":"+STROLLERID;
  client.publish("ibaby/sensors/temperatura", payload.c_str());

  //HUM
  hum = dht.readHumidity();
  payload = "humedad:"+String(hum)+":2"+":"+STROLLERID;
  client.publish("ibaby/sensors/humedad", payload.c_str());

  //GYRO/ACCEL
  sensor.getAcceleration(&ax, &ay, &az);

  float deg_x=atan(ax/sqrt(pow(ay,2) + pow(az,2)))*(180.0/3.14) - x_offset;
  float deg_y=atan(ay/sqrt(pow(ax,2) + pow(az,2)))*(180.0/3.14) - y_offset;


  payload = "angular:"+String(deg_x)+","+String(deg_y)+":3"+":"+STROLLERID;
  client.publish("ibaby/sensors/angular", payload.c_str());

  //GPS
  while(Serial2.available()){
    gps.encode(Serial2.read());
  }
  float lat=gps.location.lat();
  float lon=gps.location.lng();
  float spd=gps.speed.kmph();

  if(gps.location.isUpdated()){
    Serial.println("updated");
    lat=gps.location.lat();
    lon=gps.location.lng();
    spd=gps.speed.kmph();
  }
  payload = "gps:"+String(lat)+","+String(lon)+","+String(spd)+":4"+":"+STROLLERID;
  client.publish("ibaby/sensors/gps", payload.c_str());

  //Air particle
  while(Serial1.available()>=4){
    byte charac= Serial1.read();
    if(charac== 0xA5){
      dataH= Serial1.read();
      dataL= Serial1.read();
    }
  }
  data= int(dataH)*128 + int(dataL);
  payload = "airPart:"+String(data)+":5"+":"+STROLLERID;
  client.publish("ibaby/sensors/airPart", payload.c_str()); //µg/m³

  //Pressure
  /*analogRead(PRESS_PIN);
  Serial.println(analogRead(PRESS_PIN));*/

  //TOF
  uint16_t distance = tof.read();

  payload="proximity:"+String(distance)+":6"+":"+STROLLERID;
  client.publish("ibaby/sensors/proximity", payload.c_str());

  //BUZZER
  noTone(BUZZER_PIN);
  const int tonos[] = {261, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494};
  const int countTonos = 10;
  String pld="buzzer:"+buzz+":1"+":"+STROLLERID;
  client.publish("ibaby/actuators/buzzer", pld.c_str());

  //LDR
  inputLDR = analogRead(LDR_PIN);
  payload="light:"+String(inputLDR)+":7"+":"+STROLLERID;
  client.publish("ibaby/sensors/LDR", payload.c_str());
}

void reconnect(){
  //MQTT
  client.setServer(mqtt_server, 1883);
  while (!client.connected()) {
    if (client.connect("ESP32Client")) {
      Serial.println("Conectado al broker MQTT");
    } else {
      Serial.println("Error de conexión MQTT, rc=");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(GPS_BAUD, SERIAL_8N1, 25, 16);
  Serial2.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  ledcSetup(0, 1000, 8);
  ledcAttachPin(BUZZER_PIN, 0);
  //WIFI CONN
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando a WiFi...");
  }
  Serial.println("Conectado a WiFi");

  dht.begin(); //DHT
  Wire.begin(); //I2C
  
  tof.setTimeout(1000);
  tof.init();
  tof.setDistanceMode(VL53L1X::Long);  
  tof.setMeasurementTimingBudget(50000);
  tof.startContinuous(50);

  sensor.initialize(); //ACCEL/GYRO

  /*for(byte addr= 1; addr <127; addr++){
    Wire.beginTransmission(addr);
    if(Wire.endTransmission()==0){
      Serial.print("I2C device found at: 0x");
      Serial.println(addr, HEX);
    }
  }*/

  //offset calibration
  sensor.getAcceleration(&ax, &ay, &az);

  float x_offset=atan(ax/sqrt(pow(ay,2) + pow(az,2)))*(180.0/3.14);
  float y_offset=atan(ay/sqrt(pow(ax,2) + pow(az,2)))*(180.0/3.14);

  //pressure
  pinMode(PRESS_PIN, INPUT);

  //LDR
  pinMode(LDR_PIN, INPUT);

  //LED
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  unsigned long currentMillis = millis();
  if (!client.connected()) {
    Serial.print(client.state());
    reconnect();
  }

  client.loop();
  
  if (currentMillis - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = currentMillis;

    if(isFirst){
      //Registrar carrito en base de datos
      String payload = String(STROLLERID);
      client.publish("ibaby/strollers", payload.c_str());
      isFirst = false;
      Serial.println(payload);
    }

    publishSensors();

    actuatorsLogic();
  }
}