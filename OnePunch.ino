#include <SoftwareSerial.h>
#include <Adafruit_Soundboard.h>
#include "RunningAverageFilter.h"
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_Simple_AHRS.h>
#include <BasicLinearAlgebra.h>
#include <Geometry.h>

#define USE_SS

// "local" pinouts
// recieve on rx
// transmit on tx
#define SFX_TX 3
#define SFX_RX 2
#define SFX_RST 4
#define SFX_ACT 5

#define USER_INT_BTN 10

#define FLEX_PIN A0
#define FLEX_THRESHOLD 25

#define FILTER_TYPE float

#define PUNCH_THRESHOLD 3

typedef enum {
  IDLE,
  INTRO,
  LOOP,
  HOOK,
  PLAY
} State;

State state;

#ifdef USE_SS
// RX, TX
SoftwareSerial ss(SFX_RX, SFX_TX);
Adafruit_Soundboard sfx(&ss, NULL, SFX_RST);
#else
Adafruit_Soundboard sfx(&Serial, NULL, SFX_RST);
#endif

Adafruit_LSM9DS1 imu;
Adafruit_Simple_AHRS ahrs(&imu.getAccel(), &imu.getMag());

RunningAverageFilter<FILTER_TYPE>* filter = NULL;
FILTER_TYPE lastFlexReading;

uint32_t timer;

bool hookQueued;

float localGravity;

sensors_vec_t orientation;

void setup() {
  // put your setup code here, to run once:
  #ifdef USE_SS
  Serial.begin(115200);
  ss.begin(9600);
  #else
  Serial.begin(9600);
  #endif

  #ifdef USE_SS
  Serial.println("Initializing...");
  Serial.print("[SFX] -> ");
  #endif
  if (!sfx.reset()) {
    #ifdef USE_SS
    Serial.println("ERROR");
    #endif
    while (1) {
      digitalWrite(13, HIGH);
      delay(200);
      digitalWrite(13, LOW);
      delay(200);
      digitalWrite(13, HIGH);
      delay(200);
      digitalWrite(13, LOW);
      delay(500);
    }
  }
  #ifdef USE_SS
  Serial.println("OK!");

  Serial.print("[IMU] -> ");
  #endif
  if (!imu.begin()) {
    #ifdef USE_SS
    Serial.println("ERROR");
    #endif
    while (1) {
      digitalWrite(13, HIGH);
      delay(200);
      digitalWrite(13, LOW);
      delay(500);
    }
  }
  #ifdef USE_SS
  Serial.println("OK!");
  #endif

  localGravity = SENSORS_GRAVITY_EARTH;

  imu.setupAccel(imu.LSM9DS1_ACCELRANGE_4G);
  imu.setupMag(imu.LSM9DS1_MAGGAIN_4GAUSS);
  imu.setupGyro(imu.LSM9DS1_GYROSCALE_500DPS);

  pinMode(FLEX_PIN, INPUT);
  filter = new RunningAverageFilter<FILTER_TYPE>((FILTER_TYPE) analogRead(FLEX_PIN), 10);
  lastFlexReading = filter->read();

  pinMode(USER_INT_BTN, INPUT_PULLUP);
  pinMode(SFX_ACT, INPUT);
  pinMode(13, OUTPUT);

  state = State::IDLE;
  
  uint8_t files = sfx.listFiles();

  #ifdef USE_SS
  Serial.println("File Listing");
  Serial.println("========================");
  Serial.print("Found "); Serial.print(files); Serial.println(" Files");
  for (uint8_t f=0; f<files; f++) {
    Serial.print(f); 
    Serial.print("\tname: "); Serial.print(sfx.fileName(f));
    Serial.print("\tsize: "); Serial.println(sfx.fileSize(f));
  }
  Serial.println("========================");
  #endif

  digitalWrite(13, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  FILTER_TYPE unfilteredFlexReading = (FILTER_TYPE) analogRead(FLEX_PIN);
  FILTER_TYPE currentFlexReading = filter->update(unfilteredFlexReading);
  bool flex = currentFlexReading <= FLEX_THRESHOLD;// && lastFlexReading > FLEX_THRESHOLD;
  bool unflex = currentFlexReading > FLEX_THRESHOLD;// && lastFlexReading <= FLEX_THRESHOLD;
  uint32_t timeElapsed = millis() - timer;
  bool userButtonPressed = digitalRead(USER_INT_BTN) == LOW;
  bool sfxActive = digitalRead(SFX_ACT) == LOW;
  sensors_event_t a, m, g, temp;
  imu.getEvent(&a, &m, &g, &temp);

  Serial.print("Current Flex Reading: ");
  Serial.println(currentFlexReading);

  ahrs.getOrientation(&orientation);

  Point gravity, accel;

  gravity.X() = gravity.Y() = 0;
  gravity.Z() = localGravity;

  Rotation rot;
  rot.RotateY(radians(-orientation.pitch));
  rot.RotateX(radians(-orientation.roll));

  gravity = rot * gravity;

  accel.X() = a.acceleration.x - gravity.X();
  accel.Y() = a.acceleration.y - gravity.Y();
  accel.Z() = a.acceleration.z - gravity.Z();
  
  switch (state) {
    case State::IDLE:
      if (userButtonPressed) {
        localGravity = a.acceleration.z;
      }
      hookQueued = false;
      if (flex) {
        transitionState(State::INTRO);
        sfx.playTrack((uint8_t) 1);
      }
      break;
    case State::INTRO:
      if (timeElapsed > 2032) {
        transitionState(State::LOOP);
        if (sfxActive) sfx.stop();
        sfx.playTrack((uint8_t) 0);
      }
      break;
    case State::LOOP:
      if (timeElapsed > 3692) {
        if (hookQueued) {
          transitionState(State::HOOK);
          if (sfxActive) sfx.stop();
          sfx.playTrack((uint8_t) 2);
        } else {
          if (sfxActive) sfx.stop();
          sfx.playTrack((uint8_t) 0);
          timer = millis();
        }
      }
      if (unflex) {
        hookQueued = true;
      }
      break;
    case State::HOOK:
      if (timeElapsed > 20000) {
        transitionState(State::IDLE);
      }
      if (userButtonPressed || accel.Y() > PUNCH_THRESHOLD) {
        transitionState(State::PLAY);
        if (sfxActive) sfx.stop();
        sfx.playTrack((uint8_t) 3);
      }
      break;
    case State::PLAY:
      if (timeElapsed > 76587 || (timeElapsed > 500 && userButtonPressed)) {
        if (sfxActive) sfx.stop();
        transitionState(State::IDLE);
      }
      break;
  }
  lastFlexReading = currentFlexReading;
  delay(50);
}

void transitionState(State to) {
  #ifdef USE_SS
  switch (state) {
    case State::IDLE:
      Serial.print("IDLE");
      break;
    case State::INTRO:
      Serial.print("INTRO");
      break;
    case State::LOOP:
      Serial.print("LOOP");
      break;
    case State::HOOK:
      Serial.print("HOOK");
      break;
    case State::PLAY:
      Serial.print("PLAY");
      break;
  }
  Serial.print(" -> ");
  switch (to) {
    case State::IDLE:
      Serial.println("IDLE");
      break;
    case State::INTRO:
      Serial.println("INTRO");
      break;
    case State::LOOP:
      Serial.println("LOOP");
      break;
    case State::HOOK:
      Serial.println("HOOK");
      break;
    case State::PLAY:
      Serial.println("PLAY");
      break;
  }
  Serial.print("ACT: ");
  Serial.println(digitalRead(5));
  #endif
  state = to;
  timer = millis();
}
