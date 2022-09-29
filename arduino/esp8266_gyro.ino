#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Arduino_JSON.h>

const char* ssid    = "dohlee_5G";
const char* password = "dohlee0528";

const char* serverName = "http://<라즈베리파이 아이피>/post-esp-data.php"

String apiKeyValue = "tPmAT5Ab3j7f9";

String sensorName = "mpu6050";

Adafruit_MPU6050 mpu;

sensors_event_t a, g, tmp;
String Gyro[3];
String Acc[3];

float gyroX, gyroY, gyroZ;
float accX, accY, accZ;
float temperature;

//Gyroscope sensor deviation
float gyroXerror = 0.07;
float gyroYerror = 0.03;
float gyroZerror = 0.01;

void wifi_init()
{
    WiFi.begin(ssid, password);
    Serial.println("Connectng");
    while (Wifi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.print("Connected to WiFi network with IP Address: ");
    Serial.println(WiFi.localIP());
}

void MPU_init()
{
	if (!mpu.begin())
	{
		Serial.println("Failed to find MPU6050 chip");
		while (1)
			delay(10);
	}
	Serial.println("MPU6050 Found!");
}

void getGyro_read()
{
	mpu.getEvent(&a, &g, &tmp);

	float gyroX_tmp = g.gyro.x;
	float gyroY_tmp = g.gyro.y;
	float gyroZ_tmp = g.gyro.z;
	
	if (abs(gyroX_tmp) > gyroXerror)
		gyroX += gyroX_tmp/50.00;
	if (abs(gyroY_tmp) > gyroYerror)
		gyroY += gyroY_tmp/70.00;
	if (abs(gyroZ_tmp) > gyroZerror)
		gyroZ += gyroZ_tmp/90;

	Gyro[0] = String(gyroX);
	Gyro[1] = String(gyroY);
	Gyro[2] = String[gyroZ];
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

String getTemperature()
{
	mpu.getEvent(&a, &g, &tmp);
	temperature = tmp.temperature;
	return String(temperature);
}


void setup() 
{
	Serial.begin(115200);
	wifi_init();
	MPU_init();
}

void loop() 
{
	if (!(WiFi.status() == WL_CONNECTED))
		Serial.println("WiFi Disconnected");
	else {
		WiFiClient client;
		HTTPClient http;

		http.begin(client, serverName);
		getGyro_read();
		getAcc_read();
		http.addHeader("Content-Type", "application/x-www-form-urlencoded");
		String httpRequestData = "api_key=" + apiKeyValue + "&sensor=" + sensorName
								+ "&temperature=" + getTemperature() + "&valgx" + Gyro[0]
								+ "&valgy=" + Gyro[1] + "&valgz=" + Gyro[2]; + "&valax=" + Acc[0]
								+ "&valay=" + Acc[1] + "&valaz" + Acc[2] + "";
		Serial.print("httpRequestData: ");
    	Serial.println(httpRequestData);	

		int httpResponseCode = http.POST(httpRequestData);
		if (httpResponseCode>0) {
      		Serial.print("HTTP Response code: ");
      		Serial.println(httpResponseCode);
    	}
   		else {
      		Serial.print("Error code: ");
      		Serial.println(httpResponseCode);
    	}
    	// Free resources
    	http.end();
	}
	//delay(30000);
}

