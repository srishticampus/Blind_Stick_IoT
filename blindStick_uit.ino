#include <TinyGPS++.h>
#include <HardwareSerial.h>

#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>

//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
//  #define WIFI_SSID "iPhone"
//  #define WIFI_PASSWORD "ayswarya"
#define WIFI_SSID "Galaxy F62"
#define WIFI_PASSWORD "vbii6922"
// #define WIFI_SSID "Galaxy A14 0C7B"
// #define WIFI_PASSWORD "cnt4zfz2j3rnndn"

// Insert Firebase project API Key
#define API_KEY "AIzaSyDYMdZ-nPFZBJjhP6r-jq_vIWAvwpdcye0"

// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "https://blindstickuit-default-rtdb.firebaseio.com/" 

//Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

// Define the serial interface with which the GPS module is connected
#define GPS_SERIAL Serial2 // Change this to the serial port you are using

// Define the pins connected to the GPS module
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define SENSOR_PIN 32  // Example: Analog pin connected to the heart rate sensor
#define TRIG_PIN 5
#define ECHO_PIN 18
#define BUZZER_PIN 12 // Changed to D12
#define BUTTON_PIN 13  // GIOP21 pin connected to button


// Create a TinyGPS++ object
TinyGPSPlus gps;
bool signupOK = false;
int lastState = LOW;  // the previous state from the input pin
int currentState;     // the current reading from the input pin
void setup() {
  // Start serial communication
  Serial.begin(115200);
  GPS_SERIAL.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
   WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Sign up */
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("ok");
    signupOK = true;
  }
  else {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void loop() {
  objectDetection();

if (Firebase.ready() && signupOK ) {

  HeartRateCal();
  DangerButton();
  
  // Keep reading data from the GPS module
  while (GPS_SERIAL.available() > 0) {
    // Read one byte from the GPS module
    if (gps.encode(GPS_SERIAL.read())) {
      // If a valid GPS data is received, print it
      if (gps.location.isValid()) {
        Serial.print("Latitude: ");
        Serial.print(gps.location.lat(), 6);
        if (Firebase.RTDB.setString(&fbdo, "/lat:",gps.location.lat())){
          Serial.println("Latitude PASSED"); 
        }
        Serial.print(", Longitude: ");
        Serial.println(gps.location.lng(), 6);
         if (Firebase.RTDB.setString(&fbdo, "/long:",gps.location.lng())){
          Serial.println("Longitude PASSED"); 
        }
        HeartRateCal();
        DangerButton();
        objectDetection();
      } else {
        Serial.println("Waiting for GPS fix...");
      }
    }
  }

}
}
void objectDetection(){
long duration, distance;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = (duration / 2) / 29.1; // Convert duration to distance in cm

  Serial.print("Distance: ");
  Serial.println(distance);
  if (distance < 50) { // If an object is within 50cm
    tone(BUZZER_PIN, 1000); // Turn on the buzzer
  } else {
    noTone(BUZZER_PIN); // Turn off the buzzer
  }
}
void DangerButton(){
// read the state of the switch/button:
  
Serial.println(digitalRead(BUTTON_PIN));
  if (digitalRead(BUTTON_PIN) == LOW){
    Serial.println("The button is pressed");
    Firebase.RTDB.setString(&fbdo, "/Alert",1);
     //if (Firebase.RTDB.setString(&fbdo, "/Alert",1)){
          Serial.println("The button is pressed 1 PASSED"); 
      //  }
  }
  else if (digitalRead(BUTTON_PIN) == HIGH){
    Serial.println("The button is released");
    Firebase.RTDB.setString(&fbdo, "/Alert",0);
  //  if (Firebase.RTDB.setString(&fbdo, "/Alert",0)){
          Serial.println("The button is pressed 0 PASSED"); 
    //    }
  }

}
void HeartRateCal(){
int sensorValue = analogRead(SENSOR_PIN);
  
  // Convert sensor value to beats per minute (BPM) using your sensor's calibration
  int heartRate = map(sensorValue, 0, 4095, 50, 150); // Example calibration
  
  Serial.print("Heart Rate: ");
  Serial.println(heartRate-60);
  if (Firebase.RTDB.setString(&fbdo, "/hrate:",heartRate-60)){
          Serial.println("Heart Rate PASSED"); 
        }

}