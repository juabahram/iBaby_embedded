#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include "TinyGPS++.h"
#include "HardwareSerial.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "Adafruit_VL53L0X.h"

const char* ssid = "Juanito";
const char* password = "olelaferia";
const char* mqtt_server = "192.168.175.99";

WiFiClient espClient;
PubSubClient client(espClient);

//DHT11
#define DHT_PIN 17
#define DHTTYPE DHT11

DHT dht(DHT_PIN, DHTTYPE);

//GPS
#define TXD2 2
#define RXD2 4

#define GPS_BAUD 9600

TinyGPSPlus gps;

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
Adafruit_VL53L0X tof = Adafruit_VL53L0X();

static const uint64_t UPDATE_INTERVAL = 4000;

void setup() {
  Serial.begin(115200);
  Serial1.begin(GPS_BAUD, SERIAL_8N1, 25, 16);
  Serial2.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  //WIFI CONN
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando a WiFi...");
  }
  Serial.println("Conectado a WiFi");

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

  dht.begin(); //DHT
  Wire.begin(); //I2C

/*   for(byte addr= 1; addr <127; addr++){
    Wire.beginTransmission(addr);
    if(Wire.endTransmission()==0){
      Serial.print("I2C device found at: 0x");
      Serial.println(addr, HEX);
    }
  } */

  sensor.initialize(); //ACCEL/GYRO
  //tof.begin();
  //offset calibration
  sensor.getAcceleration(&ax, &ay, &az);

  float x_offset=atan(ax/sqrt(pow(ay,2) + pow(az,2)))*(180.0/3.14);
  float y_offset=atan(ay/sqrt(pow(ax,2) + pow(az,2)))*(180.0/3.14);
}

void loop() {

  //TEMP
  float temp = dht.readTemperature();
  // Publish data on theme
  String payload = "temperatura:"+String(temp)+":1";
  client.publish("sensors/temperatura", payload.c_str());

  //HUM
  float hum = dht.readHumidity();
  payload = "humedad:"+String(hum)+":2";
  client.publish("sensors/humedad", payload.c_str());

  //GYRO/ACCEL
  sensor.getAcceleration(&ax, &ay, &az);

  float deg_x=atan(ax/sqrt(pow(ay,2) + pow(az,2)))*(180.0/3.14) - x_offset;
  float deg_y=atan(ay/sqrt(pow(ax,2) + pow(az,2)))*(180.0/3.14) - y_offset;


  payload = "angular:"+String(deg_x)+","+String(deg_y)+":3";
  client.publish("sensors/angular", payload.c_str());

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
  payload = "gps:"+String(lat)+","+String(lon)+","+String(spd)+":4";
  client.publish("sensors/gps", payload.c_str());

  //Air particle
  while(Serial1.available()>=4){
    byte charac= Serial1.read();
    if(charac== 0xA5){
      dataH= Serial1.read();
      dataL= Serial1.read();
    }
  }
  data= int(dataH)*128 + int(dataL);
  payload = "airPart:"+String(data)+":5";
  client.publish("sensors/airPart", payload.c_str()); //µg/m³
/*   Serial.println("here");
  //TOF
  VL53L0X_RangingMeasurementData_t measure;

  tof.rangingTest(&measure, false);
  Serial.println(String(measure.RangeMilliMeter)+"mm");
  
  if(measure.RangeStatus!=4){
    payload="proximity:"+String(measure.RangeMilliMeter)+":1";
    client.publish("sensors/proximity", payload.c_str());

  }
  Serial.println("done"); */
  delay(UPDATE_INTERVAL);
}