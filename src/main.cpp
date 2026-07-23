#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <cmath>
#include <HTTPClient.h>
#include <WiFi.h>
#include "Config.h"
#include "Sensors/MPUInterface.h"
#include "Processing/MotionEngine.h"
#include "Detection/TremorDetector.h"
#include "Detection/FallDetector.h"
#include "Detection/BradykinesiaDetector.h"
#include "UI/DisplayManager.h"
#include "Data/Logger.h"

// ==============================
// System Components
// ==============================
Sensors::MPUInterface imu;
Processing::MotionEngine motionEngine;
Detection::TremorDetector tremorDetector;
Detection::FallDetector fallDetector;
Detection::BradykinesiaDetector bradykinesiaDetector;
UI::DisplayManager display;
Data::Logger telemetryLogger;

// ==============================
// Timing
// ==============================
unsigned long nextSampleUs = 0;
bool sensorReady = false;

// Latest results (for display refresh)
MotionFeatures latestFeatures;
Detection::TremorResult latestTremor;
Detection::FallResult latestFall;

// WiFi
const char* ssid = "Joe's Galaxy";
const char* password = "password";


// Discord webhook
String discordWebhook = "https://discord.com/api/webhooks/1529183469106630728/pmZB1b9pc034vfcY9z6yQhSg4QbG6wlc4JSY7xBUBwmgFiO5P9GOnH23-EqzQJ5Q7DRY";


// Step counter
bool isPeakDetected = false;
int stepcount = 0;
unsigned long lastStepTime = 0;

// Turn / FOG detection
int turnThreshold = 55;
int FOGFlag = 0;
bool monitoringTurn = false;
unsigned long turnStartTime = 0;
const unsigned long turnCheckTime = 5000;
int highImpactThreshold = 70;
int outputDecision = 2;

float Magni(float x, float y, float z) {
  return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}
//Function to find magnitude

float rotationMagni(float x, float y, float z) {
  return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}
//Function to find rotation magnitude

void calibrateTurnThreshold() {

  float totalTurnSpeed = 0;

  Serial.println("====================");
  Serial.println("TURN CALIBRATION");
  Serial.println("Perform 3 normal turns");
  Serial.println("====================");


  for(int i = 0; i < 3; i++) {

    Serial.print("Prepare for turn ");
    Serial.println(i + 1);


    // Countdown
    delay(1000);
    Serial.println("3");

    delay(1000);
    Serial.println("2");

    delay(1000);
    Serial.println("1");

    Serial.println("TURN NOW");


    float maxTurnSpeed = 0;

    unsigned long startTime = millis();


    // Collect turn data for 3 seconds
    while(millis() - startTime < 3000) {


      sensors_event_t a, g, temp;

      if (!imu.readEvents(a, g, temp)) {
          continue;
      }


      // Convert gyro values to degrees/sec
      float gx = g.gyro.x * 57.2958;
      float gy = g.gyro.y * 57.2958;
      float gz = g.gyro.z * 57.2958;


      // Total rotation speed
      float turnSpeed = sqrt(
        gx*gx +
        gy*gy +
        gz*gz
      );


      // Save highest speed
      if(turnSpeed > maxTurnSpeed) {
        maxTurnSpeed = turnSpeed;
      }

      //Serial.print("Current speed: ");
      //Serial.println(turnSpeed);

    }


    Serial.print("Turn ");
    Serial.print(i+1);
    Serial.print(" max speed: ");
    Serial.println(maxTurnSpeed);


    totalTurnSpeed += maxTurnSpeed;


    delay(2000);

  }


  // Average normal turn speed
  float averageTurnSpeed = totalTurnSpeed / 3;


  // Add safety margin
  turnThreshold = averageTurnSpeed * 0.7;


  Serial.println("====================");
  Serial.print("Average turn speed: ");
  Serial.println(averageTurnSpeed);

  Serial.print("New turn threshold: ");
  Serial.println(turnThreshold);

  Serial.println("====================");

}

void resting() {
    if (!sensorReady) return;

    unsigned long nowUs = micros();
    if (nowUs < nextSampleUs) return;

    // Schedule next sample
    nextSampleUs += SAMPLE_PERIOD_US;
    if (micros() > nextSampleUs + SAMPLE_PERIOD_US) {
        nextSampleUs = micros() + SAMPLE_PERIOD_US;
    }

    SensorSample sample;
    if (!imu.readSample(sample)) {
        return;
    }

    // Feed sample to motion engine (handles calibration internally)
    motionEngine.addSample(sample);

    // During calibration, update display progress
    if (!motionEngine.isCalibrated()) {
        // Show calibration screen — estimate progress from internal state
        static uint32_t calDisplayCounter = 0;
        calDisplayCounter++;
        if (calDisplayCounter % 10 == 0) { // Update display every 10 samples
            display.showCalibration(calDisplayCounter, CALIBRATION_SAMPLES);
        }
        return;
    }

    // Announce calibration completion once
    static bool calibrationAnnounced = false;
    if (!calibrationAnnounced) {
        Serial.println("[+] Calibration complete!");
        Serial.print("[*] Gyro offsets: ");
        Serial.print(motionEngine.getGyroBiasX(), 4); Serial.print(", ");
        Serial.print(motionEngine.getGyroBiasY(), 4); Serial.print(", ");
        Serial.println(motionEngine.getGyroBiasZ(), 4);
        Serial.print("[*] Rest Gyro Noise Floor (RMS): ");
        Serial.print(motionEngine.getRestingNoise(), 4);
        Serial.println(" rad/s");
        calibrationAnnounced = true;
    }

    // Feed sample to fall detector (needs per-sample tracking)
    fallDetector.updateSample(sample, motionEngine.getRestingNoise());

    // Feed sample to bradykinesia detector if test is active
    if (bradykinesiaDetector.isTestActive()) {
        bradykinesiaDetector.updateSample(sample);
    }

    // When a full window is ready, extract features and run analysis
    if (motionEngine.hasWindow()) {
        latestFeatures = motionEngine.extract();

        // Run tremor analysis
        latestTremor = tremorDetector.analyze(latestFeatures);

        // Run fall evaluation
        latestFall = fallDetector.evaluate(latestFeatures);

        // Log telemetry
        telemetryLogger.logFeatures(latestFeatures, motionEngine.getState());

        // Update display
        if (latestFall.fallSuspected) {
            display.showFallWarning(latestFall);
        } else {
            display.showMonitoring(latestFeatures, latestTremor, latestFall, motionEngine.getState());
        }
    }
}

void highImpactCheck(float MAGNITUDE) {
  if (MAGNITUDE > highImpactThreshold) {
    Serial.println("ALERTING ALERTING ALERTING EXCEEDED THE NUMBER OR SMTH");
    HTTPClient http;

    http.begin(discordWebhook);


    http.addHeader(
      "Content-Type",
      "application/json"
    );


    String message =
    "{\"content\":\"⚠️ High impact detected!\"}";


    int response = http.POST(message);


    Serial.print("Discord response: ");
    Serial.println(response);


    http.end();
  }
}
//Detects for high impact, should be put prior to every event

bool checkForStepsOnce(float HIGH_THRESHOLD_INPUT, float LOW_THRESHOLD_INPUT) {
  sensors_event_t a, g, temp;
  if (!imu.readEvents(a, g, temp)) {
      return false;
  }

  float HIGH_THRESHOLD = HIGH_THRESHOLD_INPUT;
  float LOW_THRESHOLD = LOW_THRESHOLD_INPUT;
  //These should be parameters that are specified absed on how accurate we're trying to detect the stuff

  float magnitude = Magni(
    a.acceleration.x,
    a.acceleration.y,
    a.acceleration.z
  );


  if(magnitude > HIGH_THRESHOLD && !isPeakDetected) {
    isPeakDetected = true;
    stepcount++;
    lastStepTime = millis();

    Serial.print("Steps: ");
    Serial.println(stepcount);

    return true;
  }

  if(magnitude < LOW_THRESHOLD) {
    isPeakDetected = false;
  }
  return false;
}

void walkingState() {
  sensors_event_t a, g, temp;
  if (!imu.readEvents(a, g, temp)) {
      return;
  }

  float HIGH_THRESHOLD = 10.7;
  float LOW_THRESHOLD = 9.3;

  float magnitude = Magni(
    a.acceleration.x,
    a.acceleration.y,
    a.acceleration.z
  );

  if(magnitude > HIGH_THRESHOLD && !isPeakDetected) {
    isPeakDetected = true;
    stepcount++;
    lastStepTime = millis();

    Serial.print("Steps: ");
    Serial.println(stepcount);
  }

  if(magnitude < LOW_THRESHOLD) {
    isPeakDetected = false;
  }

  

  float gx = g.gyro.x * 57.2958;
  float gy = g.gyro.y * 57.2958;
  float gz = g.gyro.z * 57.2958;
  float turnRate = rotationMagni(gx, gy, gz);

  if(turnRate > turnThreshold) {
    outputDecision = 3;
    return; 
  }

   if(millis() - lastStepTime > 9000){
    outputDecision = 2;
    return;
  }
}

void walkingorRestingorRepeat(){
  sensors_event_t a, g, temp;
  unsigned long decisionStart = millis();
  //Begins the first timer to check for resting
  float maxMag = 0.0f;
  float minMag = 100.0f;
  //Helps find the range of magnitudes to decide whether they are resting or walking
  unsigned long stepCounterStart;
  //Begins the second timer to check for steps
  
  while(millis() - decisionStart < 5000){
    if (!imu.readEvents(a, g, temp)) {
        continue;
    }
    float mag = Magni(a.acceleration.x, a.acceleration.y, a.acceleration.z);
    if(mag > maxMag){
      maxMag = mag;
    }
    else if(mag < minMag){
      minMag = mag;
    }
  }
  float differenceMag = maxMag - minMag;
  Serial.println(String(differenceMag));
  //Last like 10 lines spend 5 seconds just checking the difference
  if(differenceMag < 1.5 || differenceMag > 5){
    outputDecision = 1;
    Serial.println(String(outputDecision));
    Serial.println("Flag 1");
    
    return;
  }
  //If the range of magnitude is greater than 5, they are probably having a tremor not walking,
  //if their range of magnitude is really small they are probably resting
  //we could also implement tremor function which would be a little more accurate and stuff

  else{
    stepCounterStart = millis();
    while(millis() - stepCounterStart < 2000){
      if(checkForStepsOnce(10.8, 9.3) == true){
        outputDecision = 0;
        outputDecision = 1;
        Serial.println(String(outputDecision));
        Serial.println("FLAG 2");
        return;
      }
      //Returns stepcounter as either true or false
      //stepCounter(false) <- The parameter false tells it not to worry about stopping the function 
    }
    Serial.println("Flag 3");
    outputDecision = 2;
    outputDecision = 1;
    return;
    //If the decision didnt already get returned with either a 0 or 1 it just repeats
  }
}

void checkForTurn() {
  sensors_event_t a, g, temp;
  if (!imu.readEvents(a, g, temp)) {
      return;
  }

  if (!monitoringTurn){
    Serial.println("TURN DETECTED");
    turnStartTime = millis();
    monitoringTurn = true;
  }

  if(monitoringTurn) {

    if(checkForStepsOnce(10.71, 9.4)){
      Serial.println("Recovered walking");
      monitoringTurn = false;
      outputDecision = 0;
      return;
    }

    if(millis() - turnStartTime > 5000) {

      if(millis() - lastStepTime > 5000) {
        Serial.println("POSSIBLE FREEZING OF GAIT");

        FOGFlag++;

        Serial.print("FOG EVENTS: ");
        Serial.println(FOGFlag);
        outputDecision = 2;
      }
      else{
        outputDecision = 0;
      }
      monitoringTurn = false;
      return;
    }
  }
}

void setup() {
  Serial.begin(115200);

  delay(1000);

    Serial.println("==================================================");
    Serial.println("NeuroMotion Movement Analysis Platform");
    Serial.println("==================================================");

    // Initialize display first so we can show calibration progress
    display.begin();
    display.showCalibration(0, CALIBRATION_SAMPLES);

    if (!imu.begin()) {
        Serial.println("[-] CRITICAL ERROR: MPU6050 sensor not found!");
        while (true) {
            delay(100);
        }
    }

    sensorReady = true;
    Serial.println("[+] MPU6050 initialized successfully.");
    Serial.println("[*] Calibrating... keep hand still for 3 seconds.");

    // Initialize latest results to safe defaults
    latestTremor.detected = false;
    latestTremor.confidence = 0.0f;
    latestTremor.severity = Detection::TremorResult::NONE;
    latestTremor.dominantFrequency = 0.0f;
    latestTremor.dominantAxis = 0;

    latestFall.fallSuspected = false;
    latestFall.confidence = 0.0f;
    latestFall.stage = Detection::FallResult::SAFE;

    nextSampleUs = micros();

    // Wait for motion calibration to complete before running turn calibration
    while (!motionEngine.isCalibrated()) {
        resting();
        delay(1);
    }

    calibrateTurnThreshold();
    Serial.println("Turn calibration complete");

    // Start in resting state after calibration
    outputDecision = 1;

    delay(100);

    WiFi.begin(
        ssid,
        password
    );

    Serial.print("Connecting to WiFi");

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.println("WiFi connected");
    Serial.println("please work");
    Serial.println("MPU6050 Starting");
}



void loop() {
  switch(outputDecision){
    case 0: {
      walkingState();
      break;
    }
    case 1:{
      resting();
      break;
    }
    case 2:{
      walkingorRestingorRepeat();
      break;
    }
    case 3:{
      checkForTurn();
      break;
    }
  }
}