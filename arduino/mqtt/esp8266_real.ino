#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <ESP8266WiFi.h>  // Enables the ESP8266 to connect to the local network (via WiFi)
#include <PubSubClient.h> // Connect and publish to the MQTT broker

Adafruit_MPU6050 mpu;

sensors_event_t a, g, tmp;
float Gyro[3];
float Acc[3];

float gyroX, gyroY, gyroZ;
float accX, accY, accZ;
//float temperature;

//Gyroscope sensor deviation
float gyroXerror = 0.00;
float gyroYerror = 0.03;
float gyroZerror = 0.01;

void MPU_init()
{
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1);
  }
}

void getGyro_read()
{
  mpu.getEvent(&a, &g, &tmp);

  float gyroX_tmp = g.gyro.x;
  float gyroY_tmp = g.gyro.y;
  float gyroZ_tmp = g.gyro.z;
  
  if (abs(gyroX_tmp) > gyroXerror)
    gyroX += gyroX_tmp;//50.00;
  if (abs(gyroY_tmp) > gyroYerror)
    gyroY += gyroY_tmp;//70.00;
  if (abs(gyroZ_tmp) > gyroZerror)
    gyroZ += gyroZ_tmp;//90;

  Gyro[0] = gyroX;
  Gyro[1] = gyroY;
  Gyro[2] = gyroZ;
}

void getAcc_read()
{
  mpu.getEvent(&a, &g, &tmp);
  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;

  Acc[0] = accX;
  Acc[1] = accY;
  Acc[2] = accZ;
}
// WiFi
const char* ssid = "leek3_101(2)_2.4G";                 // Your personal network SSID
const char* wifi_password = "101101101"; // Your personal network password

// MQTT
const char* mqtt_server = "192.9.64.120";  // IP of the MQTT broker
const char* gyro_topic = "lab/mpu6050/gyro";
const char* acc_topic = "lab/mpu6050/acc";
const char* mqtt_username = "dohlee"; // MQTT username
const char* mqtt_password = "dohlee"; // MQTT password
const char* clientID = "client_lab"; // MQTT client ID

// Initialise the WiFi and MQTT Client objects
WiFiClient wifiClient;
// 1883 is the listener port for the Broker
PubSubClient client(mqtt_server, 1883, wifiClient); 


// Custom function to connet to the MQTT broker via WiFi
void connect_MQTT(){
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Connect to the WiFi
  WiFi.begin(ssid, wifi_password);

  // Wait until the connection has been confirmed before continuing
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Debugging - Output the IP Address of the ESP8266
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Connect to MQTT Broker
  // client.connect returns a boolean value to let us know if the connection was successful.
  // If the connection is failing, make sure you are using the correct MQTT Username and Password (Setup Earlier in the Instructable)
  if (client.connect(clientID, mqtt_username, mqtt_password)) {
    Serial.println("Connected to MQTT Broker!");
  }
  else {
    Serial.println("Connection to MQTT Broker failed...");
  }

}

void setup() {
  Serial.begin(9600);
  MPU_init();
}

void loop() {

  connect_MQTT();
  Serial.setTimeout(2000);
  getGyro_read();
  getAcc_read();
  
  Serial.print("Gyro_x: ");
  Serial.println(Gyro[0]);
  Serial.print(", ");
  Serial.print("Gyro_y: ");
  Serial.println(Gyro[1]);
  Serial.print(", ");
  Serial.print("Gyro_z: ");
  Serial.println(Gyro[2]);
  Serial.print(", ");
  Serial.print("\n");
  Serial.print("Acc_x: ");
  Serial.println(Acc[0]);
  Serial.print(", ");
  Serial.print("Acc_y: ");
  Serial.println(Acc[1]);
  Serial.print(", ");
  Serial.print("Acc_z: ");
  Serial.println(Acc[2]);

  
  if (client.publish(gyro_topic, String(Gyro[0]).c_str()) && client.publish(gyro_topic, String(Gyro[1]).c_str()) && client.publish(gyro_topic, String(Gyro[2]).c_str())) {
    Serial.println("Gyro sent!");
  }
  else {
    Serial.println("Gyro data failed to send. Reconnecting to MQTT Broker and trying again");
    client.connect(clientID, mqtt_username, mqtt_password);
    delay(10);
    client.publish(gyro_topic, String(Gyro[0]).c_str());
    client.publish(gyro_topic, String(Gyro[1]).c_str());
    client.publish(gyro_topic, String(Gyro[2]).c_str());
  }

  if (client.publish(acc_topic, String(Acc[0]).c_str()) && client.publish(acc_topic, String(Acc[1]).c_str()) && client.publish(acc_topic, String(Acc[2]).c_str())) {
    Serial.println("Acc sent!");
  }
  else {
    Serial.println("Acc data failed to send. Reconnecting to MQTT Broker and trying again");
    client.connect(clientID, mqtt_username, mqtt_password);
    delay(10);
    client.publish(acc_topic, String(Acc[0]).c_str());
    client.publish(acc_topic, String(Acc[1]).c_str());
    client.publish(acc_topic, String(Acc[2]).c_str());
  }

  // if (client.publish(humidity_topic, String(h).c_str())) {
  //   Serial.println("Humidity sent!");
  // }
  // // Again, client.publish will return a boolean value depending on whether it succeded or not.
  // // If the message failed to send, we will try again, as the connection may have broken.
  // else {
  //   Serial.println("Humidity failed to send. Reconnecting to MQTT Broker and trying again");
  //   client.connect(clientID, mqtt_username, mqtt_password);
  //   delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
  //   client.publish(humidity_topic, String(h).c_str());
  // }
  client.disconnect();  // disconnect from MQTT broker
  delay(1);      // print new values every 1 Minute
}
