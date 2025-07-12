#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

float accelPitch, gyroPitch, pitchAngle;
float kalmanPitch = 0.0; // Kalmanstate
float setPoint;

unsigned long timeCurrent = 0;
unsigned long timePrev = 0;
unsigned long timeDelta = 0;
unsigned int timePrev2, timeCurrent2, timeDelta2; //used to keep track of loop times for PID control

// Gyroscope drift (needs to be determined by calibration)
float gyroXDrift = 0.0;
float gyroYDrift = 0.0;
float gyroZDrift = 0.0;

// Kalman filter noise settings (tune these)
float kalmanPitchUncertainty = 0.1; // initial uncertainty
float accelUncertainty = 0.7;  // higher = trust accel less
float gyroUncertainty = 1.0;   // higher = trust gyro less

MPU6050 mpu;

// Motor pins
const int Motor1_Ena1 = 2;   // AIN1
const int Motor1_PWM1 = 5;   // PWMA
const int Motor1_Ena2 = 4;  // AIN2
// const int STBY = 4;

// PWM settings
const int PWM_Channel = 0;
const int PWM_Frequency = 7000;
const int PWM_Resolution = 12; // 0 - 4095

// PID coefficients
float kP = 5;
float kD = 0.0;
float kI = 0.0;


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
  delay(1000);
  Wire.begin(21, 22);  // uses default SDA = 21, SCL = 22
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
  Serial.println("MPU6050 ready");

  // Initialize timing
  timePrev = millis();

  calibrateGyro();

  pinMode(Motor1_Ena1, OUTPUT);
  pinMode(Motor1_Ena2, OUTPUT);
  // pinMode(STBY, OUTPUT);

  // Setup PWM
  ledcAttach(Motor1_PWM1, PWM_Frequency, PWM_Resolution);
  
  // Spin motor forward at 50% speed
  digitalWrite(Motor1_Ena1, LOW);
  digitalWrite(Motor1_Ena2, HIGH);
  // digitalWrite(STBY, HIGH);

  // ledcWrite(Motor1_PWM1, 2048);  // 50% of 4095

}

void loop() {
  while (1)
  {
    balanceControls();
  }
  // Motor_1_Speed(manualControl);
  // Motor_2_Speed(manualControl);

}

//Controls for balance as well as user input for movement
void balanceControls()
{
  static float proportional;
  static float integral;
  static float derivative;
  static float signalOutput;
  static float currentAngle;
  static float previousAngle;

  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  calculateAngles(ax, ay, az, gx, gy, gz);
  currentAngle = kalmanPitch;

  timeCurrent2 = millis();
  timeDelta2 = timeCurrent2 - timePrev2;
  timePrev2 = millis();

  //PID control signal calculation
  proportional = kP*(setPoint - currentAngle); //when tilted forward, angle is negative, so we want a positive output (forward signal)
  integral = constrain(integral + (kI*(setPoint - currentAngle)*timeDelta2), -4095, 4095); //when tilted forward, angle is negative, so we want to accumulate positive output (forward signal)
  derivative = -kD*(currentAngle - previousAngle)/timeDelta2; //when tilting towards level from foward tilt, angle difference is positive (pitching up), so want a negative signal to counteract P and I
 
  
  signalOutput = constrain(proportional+integral+derivative, -3895, 3895); //reserve 200 values for non-balancing movements
  //Only have outputs above 1500 or below -1500
  // if (signalOutput < 1000 && signalOutput >= 0)
  // {
  //   signalOutput = 1000;
  // }
  // if (signalOutput > -1000 && signalOutput <= 0)
  // {
  //   signalOutput = -1000;
  // }
  previousAngle = currentAngle;

  //Motor control
  Serial.println(signalOutput);
  Motor_1_Speed(signalOutput);
  // Motor_2_Speed(signalOutput);

  // if (forwardInput == 1)
  // {
  //   Motor_1_Speed(signalOutput + 200);
  //   // Motor_2_Speed(signalOutput + 200);
  // }
  // else if (reverseInput == 1)
  // {
  //   Motor_1_Speed(signalOutput - 200);
  //   // Motor_2_Speed(signalOutput - 200);
  // }
  // else if (rotateLeftInput == 1)
  // {
  //   Motor_1_Speed(signalOutput - 200);
  //   // Motor_2_Speed(signalOutput + 200);
  // }
  // else if (rotateRightInput == 1)
  // {
  //   Motor_1_Speed(signalOutput + 200);
  //   // Motor_2_Speed(signalOutput - 200);
  // }
  // else
  // {
  //   Motor_1_Speed(signalOutput);
  //   Motor_2_Speed(signalOutput);
  // }

  //if balance mode turned off, reset values
  // if (balanceMode == 0)
  // {
  //   proportional = 0;
  //   integral = 0;
  //   derivative = 0;
  //   signalOutput = 0;
  //   currentAngle = 0;
  //   previousAngle = 0;
  // }
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
  Serial.print(",gyroY:"); //comment this out for final code. This is just for testing purposes.
  Serial.print(gyroY);
  
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

void Motor_1_Speed(int speed) //speed ranging from -4095 to 4095 (negative is reverse, positive is forward)
{
  static int prev_Speed; //to keep track of motor direction. Saved across function calls

  if (prev_Speed*speed < 0 || speed == 0) //Only negative when speed and previous speed are different signs. If speed = 0, turn off MOSFETs
  {
    //turn off MOSFETs for 500 uS to prevent MOSFET shoot through
    digitalWrite(Motor1_Ena1, LOW);
    ledcWrite(Motor1_PWM1, 0);
    digitalWrite(Motor1_Ena2, LOW);
    delayMicroseconds(500);
  }

  if (speed < 0)
  {
    digitalWrite(Motor1_Ena1, HIGH);
    digitalWrite(Motor1_Ena2, LOW);
    ledcWrite(Motor1_PWM1, constrain(abs(speed), 0, 4095));
  }

  if (speed > 0)
  {
    digitalWrite(Motor1_Ena1, LOW);
    digitalWrite(Motor1_Ena2, HIGH);
    ledcWrite(Motor1_PWM1, constrain(speed, 0, 4095));
  }
  prev_Speed = speed;
}