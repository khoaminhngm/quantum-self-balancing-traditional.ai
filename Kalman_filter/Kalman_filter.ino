#include <Wire.h>
#include <MPU6050.h>
#include <math.h>


float accelPitch, gyroPitch, pitchAngle;
float kalmanPitch = 0.0; // Kalmanstate
float kalmanPitchUncertainty = 0.1; // initial uncertainty
unsigned long timeCurrent = 0;
unsigned long timePrev = 0;
unsigned long timeDelta = 0;

// Gyroscope drift (needs to be determined by calibration)
float gyroXDrift = 0.0;
float gyroYDrift = 0.0;
float gyroZDrift = 0.0;

// Kalman filter noise settings (tune these)
float accelUncertainty = 0.7;  // higher = trust accel less
float gyroUncertainty = 1.0;   // higher = trust gyro less

MPU6050 mpu;

void calibrateGyro() {
  const int numSamples = 500;
  long sumX = 0, sumY = 0, sumZ = 0;

  int16_t gx, gy, gz;

  for (int i = 0; i < numSamples; i++) {
    mpu.getRotation(&gx, &gy, &gz);
    sumX += gx;
    sumY += gy;
    sumZ += gz;
    delay(5);
  }

  gyroXDrift = sumX / (float)numSamples;
  gyroYDrift = sumY / (float)numSamples;
  gyroZDrift = sumZ / (float)numSamples;
}


void setup() {
  Serial.begin(115200);
  Wire.begin();  // uses default SDA = 21, SCL = 22
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
  Serial.println("MPU6050 ready");

  // Initialize timing
  timePrev = millis();

  calibrateGyro();

}

void loop() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  calculateAngles(ax, ay, az, gx, gy, gz);
  delay(10);

}

void calculateAngles(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz)
{
  float accelX = ax / 16384.0;  // assuming ±2g range
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;

  float gyroX = (gx - gyroXDrift) / 131.0;  // assuming ±250°/s
  float gyroY = (gy - gyroYDrift) / 131.0;
  float gyroZ = (gz - gyroZDrift) / 131.0;

  //Keep track of loop times in milliseconds
  timeCurrent = millis();
  timeDelta = timeCurrent - timePrev;
  timePrev = timeCurrent;

  //obtain pitch and roll from accelerometer alone in degrees
  accelPitch = atan2(accelZ, sqrt(accelX*accelX + accelY*accelY))*(180/M_PI);
  //accelRoll = atan2(accelX, sqrt(accelY*accelY + accelZ*accelZ))*(180/M_PI); //comment this out for final code. This is just for testing purposes.

  //obtain pitch and roll from gyroscope alone in degrees (comment this out for final code. This is just for testing purposes)
  // gyroPitch += gyroX/1000*timeDelta;


  //Calculate the pitch angle using Kalman filter. Function automatically updates kalmanPitch variable
  float prediction = kalmanFilter(kalmanPitch, kalmanPitchUncertainty, gyroY, accelPitch, accelUncertainty, gyroUncertainty, timeDelta);
  pitchAngle = kalmanPitch; //update the dashboard

  //Serial monitor friendly format
  Serial.print("accelPitch:");
  Serial.print(accelPitch);
  Serial.print(",prediction:"); //comment this out for final code. This is just for testing purposes.
  Serial.print(prediction);
  // Serial.print(",accelX:");
  // Serial.print(accelX);
  // Serial.print(",accelY:");
  // Serial.print(accelY);
  // Serial.print(",accelZ:");
  // Serial.print(accelZ);
  // Serial.print(", accelRoll: "); //comment this out for final code. This is just for testing purposes.
  // Serial.print(accelRoll); //comment this out for final code. This is just for testing purposes.
  // Serial.print(",gyroX:"); //comment this out for final code. This is just for testing purposes.
  // Serial.print(gyroX); //comment this out for final code. This is just for testing purposes.
  Serial.print(",gyroY:"); //comment this out for final code. This is just for testing purposes.
  Serial.print(gyroY);
  // Serial.print(",gyroZ:"); //comment this out for final code. This is just for testing purposes.
  // Serial.print(gyroZ);
  // Serial.print(", gyroRoll: "); //comment this out for final code. This is just for testing purposes.
  // Serial.print(gyroRoll); //comment this out for final code. This is just for testing purposes.
  // Serial.print(", gyroYaw: "); //comment this out for final code. This is just for testing purposes.
  // Serial.println(gyroYaw); //comment this out for final code. This is just for testing purposes.
  

  //Serial plotter friendly format
  Serial.print(",filteredAngle:"); 
  Serial.println(pitchAngle); //comment this out for final code. This is just for testing purposes.
}

float kalmanFilter(float &kalmanState, float &kalmanUncertainty, float kalmanInput, float kalmanMeasurement, float accelUncertainty, float gyroUncertainty, float timeStepMillis)
{
  float kalmanGain;
  kalmanState = kalmanState + timeStepMillis/1000*kalmanInput; //Calculate the initial prediction of the pitch angle
  float prediction = kalmanState;
  kalmanUncertainty = kalmanUncertainty + pow((timeStepMillis/1000*gyroUncertainty), 2); //calculate the initial uncertainty prediction
  kalmanGain = kalmanUncertainty/(kalmanUncertainty + pow(accelUncertainty,2)); //calculate the kalman gain
  kalmanState = kalmanState + kalmanGain*(kalmanMeasurement - kalmanState); //Calculate the pitch angle from the kalman filter
  kalmanUncertainty = (1 - kalmanGain)*kalmanUncertainty; //Prepare for the next iteration step by calculating current uncertainty
  return prediction;
}