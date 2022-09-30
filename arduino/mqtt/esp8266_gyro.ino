/*
  Adapted from WriteSingleField Example from ThingSpeak Library (Mathworks)
  
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp8266-nodemcu-thingspeak-publish-arduino/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include "ThingSpeak.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>

const char* ssid = "leek3_101(2)_2.4G";   // your network SSID (name) 
const char* password = "101101101";   // your network password

WiFiClient  client;

unsigned long myChannel = 1;
const char * myWriteAPIKey = "IGHSXR5KAGTOB59C";

// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 10;

// Variable to hold temperature readings

//uncomment if you want to get temperature in Fahrenheit
//float temperatureF;

// Create a sensor object
Adafruit_MPU6050 mpu;

sensors_event_t a, g, tmp;
String Gyro[3];
String Acc[3];

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

  Gyro[0] = String(gyroX);
  Gyro[1] = String(gyroY);
  Gyro[2] = String(gyroZ);
}

void getAcc_read()
{
  mpu.getEvent(&a, &g, &tmp);
  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;

  Acc[0] = String(accX);
  Acc[1] = String(accY);
  Acc[2] = String(accZ);
}

/*String getTemperature()
{
  mpu.getEvent(&a, &g, &tmp);
  temperature = tmp.temperature;
  return String(temperature);
}*/

void setup() {
  Serial.begin(115200);  //Initialize serial
  MPU_init();
  
  WiFi.mode(WIFI_STA);   
  
  ThingSpeak.begin(client);  // Initialize ThingSpeak

}

void loop() {
  if ((millis() - lastTime) > timerDelay) {
     //Serial.printSerial.print("Connect or reconnect to WiFi");
    if(WiFi.status() != WL_CONNECTED){
      Serial.print("Attempting to connect");
      while(WiFi.status() != WL_CONNECTED){
        WiFi.begin(ssid, password); 
        delay(5000);     
      } 
      Serial.println("\nConnected.");
    }
    getGyro_read();
    getAcc_read();
    

    // Get a new temperature reading
    Serial.print("Gyro_x: ");
    Serial.println(Gyro[0]);
    Serial.print(", ");
    Serial.print("Gyro_y: ");
    Serial.println(Gyro[1]);
    Serial.print(", ");
    Serial.print("Gyro_z: ");
    Serial.println(Gyro[2]);
    Serial.print(", ");
    Serial.print("Acc_x: ");
    Serial.println(Acc[0]);
    Serial.print("\n");
    Serial.print("Acc_y: ");
    Serial.println(Acc[1]);
    Serial.print("Acc_z: ");
    Serial.println(Acc[2]);
  


    
    //uncomment if you want to get temperature in Fahrenheit
    /*temperatureF = 1.8 * bme.readTemperature() + 32;
    Serial.print("Temperature (ºC): ");
    Serial.println(temperatureF);*/
    ThingSpeak.setField(1, Gyro[0]);
    ThingSpeak.setField(2, Gyro[1]);
    ThingSpeak.setField(3, Gyro[2]);
    ThingSpeak.setField(4, Acc[0]);
    ThingSpeak.setField(5, Acc[1]);
    ThingSpeak.setField(6, Acc[2]);
    
    // Write to ThingSpeak. There are up to 8 fields in a channel, allowing you to store up to 8 different
    // pieces of information in a channel.  Here, we write to field 1.
    //int x = ThingSpeak.writeField(myChannelNumber, 1, temperatureC, myWriteAPIKey);
    int x = ThingSpeak.writeFields(myChannel, myWriteAPIKey);
    /*int Gyro_r_x = ThingSpeak.writeField(myCh_Gyro_x, 1, Gyro[0], myWriteAPIKey);
    int Gyro_r_y = ThingSpeak.writeField(myCh_Gyro_y, 2, Gyro[1], myWriteAPIKey);
    int Gyro_r_z = ThingSpeak.writeField(myCh_Gyro_z, 3, Gyro[2], myWriteAPIKey);
    int Acc_r_x = ThingSpeak.writeField(myCh_Acc_x, 4, Acc[0], myWriteAPIKey);
    int Acc_r_y = ThingSpeak.writeField(myCh_Acc_y, 5, Acc[1], myWriteAPIKey);
    int Acc_r_z = ThingSpeak.writeField(myCh_Acc_z, 6, Acc[2], myWriteAPIKey);*/
    //uncomment if you want to get temperature in Fahrenheit
    //int x = ThingSpeak.writeField(myChannelNumber, 1, temperatureF, myWriteAPIKey);

    //if(Gyro_r_x == 200 && Gyro_r_y == 200 && Gyro_r_z == 200 && Acc_r_x == 200 && Acc_r_y == 200 && Acc_r_z == 200){
    if(x == 200){
      Serial.println("Channel update successful.");
    }
    else{
      Serial.println("Problem updating channel. HTTP error code " + String(x));
//      Serial.println("Problem updating channel. HTTP error code " + String(Gyro_r_y));
//      Serial.println("Problem updating channel. HTTP error code " + String(Gyro_r_z));
//      Serial.println("Problem updating channel. HTTP error code " + String(Acc_r_x));
//      Serial.println("Problem updating channel. HTTP error code " + String(Acc_r_z));
//      Serial.println("Problem updating channel. HTTP error code " + String(Acc_r_y));
    }
    lastTime = millis();
  }
 }
