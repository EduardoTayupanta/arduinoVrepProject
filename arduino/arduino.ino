/*
    MPU6050 Triple Axis Gyroscope & Accelerometer. Pitch & Roll & Yaw Gyroscope Example.
    Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-zyroskop-i-akcelerometr-mpu6050.html
    GIT: https://github.com/jarzebski/Arduino-MPU6050
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/

#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu_0;
MPU6050 mpu_1;
MPU6050 mpu_2;
MPU6050 mpu_3;

// Timers
unsigned long timer_0 = 0;
unsigned long timer_1 = 0;
unsigned long timer_2 = 0;
unsigned long timer_3 = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float ypr_0[3];
float ypr_1[3];
float ypr_2[3];
float ypr_3[3];

#define LED_PIN 13
bool blinkState = false;

//Mux control pins
int s0 = 5;
int s1 = 6;
int control = 0;

void setup()
{
  Serial.begin(115200);

  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);

  // ================= MPU: 0 =================
  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  // Initialize MPU6050
  Serial.println("MPU: 0");
  while (!mpu_0.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu_0.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu_0.setThreshold(3);

  // ================= MPU: 1 =================
  digitalWrite(s0, HIGH);
  digitalWrite(s1, LOW);
  // Initialize MPU6050
  Serial.println("MPU: 1");
  while (!mpu_1.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu_1.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu_1.setThreshold(3);

  // ================= MPU: 2 =================
  digitalWrite(s0, LOW);
  digitalWrite(s1, HIGH);
  // Initialize MPU6050
  Serial.println("MPU: 2");
  while (!mpu_2.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu_2.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu_2.setThreshold(3);

  // ================= MPU: 3 =================
  digitalWrite(s0, HIGH);
  digitalWrite(s1, HIGH);
  // Initialize MPU6050
  Serial.println("MPU: 3");
  while (!mpu_3.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu_3.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu_3.setThreshold(3);

  // configure Arduino LED for
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  if (control == 0) {
    // ================= MPU: 0 =================
    digitalWrite(s0, LOW);
    digitalWrite(s1, LOW);
    timer_0 = millis();

    // Read normalized values
    Vector norm_0 = mpu_0.readNormalizeGyro();

    // Calculate Pitch, Roll and Yaw
    ypr_0[1] = ypr_0[1] + norm_0.YAxis * timeStep * 3.5;
    ypr_0[2] = ypr_0[2] + norm_0.XAxis * timeStep * 3.5;
    ypr_0[0] = ypr_0[0] + norm_0.ZAxis * timeStep * 3.5;

    // Output raw
    Serial.print("#0#");
    Serial.print(ypr_0[0]); Serial.print("#");
    Serial.print(ypr_0[1]); Serial.print("#");
    Serial.println(ypr_0[2]);

    // Wait to full timeStep period
    delay((timeStep * 1000) - (millis() - timer_0));
    control = 1;
  } else if (control == 1) {
    // ================= MPU: 1 =================
    digitalWrite(s0, HIGH);
    digitalWrite(s1, LOW);
    timer_1 = millis();

    // Read normalized values
    Vector norm_1 = mpu_1.readNormalizeGyro();

    // Calculate Pitch, Roll and Yaw
    ypr_1[1] = ypr_1[1] + norm_1.YAxis * timeStep * 3.5;
    ypr_1[2] = ypr_1[2] + norm_1.XAxis * timeStep * 3.5;
    ypr_1[0] = ypr_1[0] + norm_1.ZAxis * timeStep * 3.5;

    // Output raw
    Serial.print("#1#");
    Serial.print(ypr_1[0]); Serial.print("#");
    Serial.print(ypr_1[1]); Serial.print("#");
    Serial.println(ypr_1[2]);

    // Wait to full timeStep period
    delay((timeStep * 1000) - (millis() - timer_1));
    control = 2;
  } else if (control == 2) {
    // ================= MPU: 2 =================
    digitalWrite(s0, LOW);
    digitalWrite(s1, HIGH);
    timer_2 = millis();

    // Read normalized values
    Vector norm_2 = mpu_2.readNormalizeGyro();

    // Calculate Pitch, Roll and Yaw
    ypr_2[1] = ypr_2[1] + norm_2.YAxis * timeStep * 3.5;
    ypr_2[2] = ypr_2[2] + norm_2.XAxis * timeStep * 3.5;
    ypr_2[0] = ypr_2[0] + norm_2.ZAxis * timeStep * 3.5;

    // Output raw
    Serial.print("#2#");
    Serial.print(ypr_2[0]); Serial.print("#");
    Serial.print(ypr_2[1]); Serial.print("#");
    Serial.println(ypr_2[2]);

    // Wait to full timeStep period
    delay((timeStep * 1000) - (millis() - timer_2));
    control = 3;
  } else if (control == 3) {
    // ================= MPU: 3 =================
    digitalWrite(s0, HIGH);
    digitalWrite(s1, HIGH);
    timer_3 = millis();

    // Read normalized values
    Vector norm_3 = mpu_3.readNormalizeGyro();

    // Calculate Pitch, Roll and Yaw
    ypr_3[1] = ypr_3[1] + norm_3.YAxis * timeStep * 3.5;
    ypr_3[2] = ypr_3[2] + norm_3.XAxis * timeStep * 3.5;
    ypr_3[0] = ypr_3[0] + norm_3.ZAxis * timeStep * 3.5;

    // Output raw
    Serial.print("#3#");
    Serial.print(ypr_3[0]); Serial.print("#");
    Serial.print(ypr_3[1]); Serial.print("#");
    Serial.println(ypr_3[2]);

    // Wait to full timeStep period
    delay((timeStep * 1000) - (millis() - timer_3));
    control = 0;
  }

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}
