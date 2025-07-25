#include <WiFi.h>
#include "ThingSpeak.h"

#define SCOUNT 30
#define VREF 3.3

// WiFi credentials
const char* ssid = "Olympus";
const char* password = "7707888119";

//ThingSpeak Info
const char* APIKey = "U38K516VO6W02UWU";
unsigned long channel = 3017403;
unsigned long lastUpload = 0;
const unsigned long uploadInterval = 15000;

int analogPin1 = 32;
int analogPin2 = 33;
int analogPin3 = 35;
int switchPin = 23;
int rawValue1 = 0;
int rawValue2 = 0;
int rawValue3 = 0;
float voltage1 = 0.0;
float voltage2 = 0.0;
float voltage3 = 0.0;

int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, temperature = 25;

float kalmanEstimate = 0.0; // Current estimate
float kalmanErrorEstimate = 1.0; // Uncertainty estimation
float kalmanErrorMeasurement = 0.3; // Sensor noise
float gain = 0.8;

WiFiClient client;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(2000);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected.");
  ThingSpeak.begin(client);  // Initialize ThingSpeak

  analogReadResolution(12); // 12-bit ADC
  analogSetAttenuation(ADC_11db); // Full 0-3.3V range

  // Set LED GPIO pins to output low
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(switchPin, INPUT);
  pinMode(switchPin, OUTPUT);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
  digitalWrite(14, LOW);
  digitalWrite(27, LOW);
}

void loop() {

  int state = digitalRead(switchPin);

  // Set LEDs to off
  digitalWrite(27, LOW);
  digitalWrite(14, LOW);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
  
  rawValue1 = analogRead(analogPin1); // Read input value from ADC
  rawValue2 = analogRead(analogPin2); // Read input value from ADC
  rawValue3 = analogRead(analogPin3); // Read input value from ADC
  
  voltage1 = (rawValue1 / 4095.0) * 3.3; // Convert ADC value to input voltage
  voltage2 = (rawValue2 / 4095.0) * 3.3; // Convert ADC value to input voltage
  voltage3 = (rawValue3 / 4095.0) * 3.3; // Convert ADC value to input voltage
  

  float brightness = 0.0;

  // Piecewise transfer function, finds brightness from input voltage
  if(voltage1 == 0) {
    brightness = 0;
  } else if(voltage1 > 0 && voltage1 <= 1.9439) {
    brightness = (voltage1 - 0.0019) / 0.0096;
  } else if(voltage1 >= 1.9439 && voltage1 < 3.2) {
    brightness = (voltage1 * voltage1 - 0.3186) / 0.0173;
  } else {brightness = 601;}
  
  float hmClarity = 1 - brightness / 601; // Clarity of water according to homemade sensor
  float ppClarity = voltage2 / 3.1; // Clarity of water according to prepackaged sensor
  
  float weight = constrain((voltage1 - 1.5), 0.0, 1.0); // Weight based on sensor 1 voltage
  float clarity = (1.0 - weight) * hmClarity + weight * ppClarity;
  clarity = constrain(clarity, 0.0, 1.0);

  if(state == HIGH) {

    static unsigned long analogSampleTimepoint = millis();
    if(millis()-analogSampleTimepoint > 40U) { //every 40 milliseconds,read the analog value from the ADC
      analogSampleTimepoint = millis();
      analogBuffer[analogBufferIndex] = analogRead(analogPin3); //read the analog value and store into the buffer
      analogBufferIndex++;
      if(analogBufferIndex == SCOUNT) {
        analogBufferIndex = 0;
      }
      static unsigned long printTimepoint = millis();
      if(millis()-printTimepoint > 800U) {
        printTimepoint = millis();
        for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
          analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
          // read the analog value more stable by the median filtering algorithm, and convert to voltage value
          averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF/ 1024.0; 
          //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
          float compensationCoefficient=1.0+0.02*(temperature-25.0); 
          float compensationVoltage=averageVoltage/compensationCoefficient; //temperature compensation
          tdsValue=(133.42*compensationVoltage*compensationVoltage*compensationVoltage - 
            255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage)*0.5; //convert voltage value to tds value
          Serial.print("TDS Value:");
          Serial.print(tdsValue, 0);
          Serial.println("ppm");
      }
    }

    Serial.print("Clarity Sensor 1 Voltage: ");
    Serial.println(voltage1, 3);
    Serial.print("Clarity Sensor 2 Voltage: ");
    Serial.println(voltage2, 3);
    Serial.print("Water Clarity: ");
    Serial.println(clarity * 100, 3);
    
    float scaledTDS = constrain(1.0 - (tdsValue / 2000.0), 0.0, 1.0);
    float purity = (0.6 * clarity) + (0.4 * scaledTDS);

    // Kalman filtering
    gain = kalmanErrorEstimate / (kalmanErrorEstimate + kalmanErrorMeasurement);
    kalmanEstimate = kalmanEstimate + gain * (purity - kalmanEstimate);
    kalmanErrorEstimate = (1 - gain) * kalmanErrorEstimate;

    Serial.print("Kalman Filtered Purity: ");
    Serial.println(kalmanEstimate * 100, 3);


    // Turn on indicator LED depending on water purity
    if (kalmanEstimate > 0.98) {
      digitalWrite(27, HIGH); // Blue
    } else if(kalmanEstimate >= 0.9) {
      digitalWrite(14, HIGH); // Green
    } else if(kalmanEstimate >= 0.6) {
      digitalWrite(12, HIGH); // Yellow
    } else {
      digitalWrite(13, HIGH); // Red
    }

    
    if(millis() - lastUpload >= uploadInterval) {
      ThingSpeakUpload(hmClarity, ppClarity, kalmanEstimate * 100, tdsValue);
      lastUpload = millis();
    }
    delay(500);
  }
}

  int getMedianNum(int bArray[], int iFilterLen) {
    int bTab[iFilterLen];
    for (byte i = 0; i<iFilterLen; i++)
      bTab[i] = bArray[i];
      int i, j, bTemp;
    for (j = 0; j < iFilterLen - 1; j++) {
      for (i = 0; i < iFilterLen - j - 1; i++) {
        if (bTab[i] > bTab[i + 1]) {
          bTemp = bTab[i];
          bTab[i] = bTab[i + 1];
          bTab[i + 1] = bTemp;
        }
      }
    }
    if ((iFilterLen & 1) > 0)
      bTemp = bTab[(iFilterLen - 1) / 2];
    else
      bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
    return bTemp;
  }

  void ThingSpeakUpload(float hmClarity, float ppClarity, float purity, float tdsValue) {
    ThingSpeak.setField(1, hmClarity * 100);
    ThingSpeak.setField(2, ppClarity * 100);
    ThingSpeak.setField(3, tdsValue);
    ThingSpeak.setField(4, purity);
    int response = ThingSpeak.writeFields(channel, APIKey);
    if (response == 200) {
      Serial.println("Channel update successful.");
    } else {
      Serial.print("Problem updating channel. HTTP error code ");
      Serial.println(response);
    }
  }
