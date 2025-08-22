#include <Wire.h>
#include <MPU6050.h>
#include <math.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoOTA.h>

const char* ssid = "Vinaheim2.0_EXT";
const char* password = "KTXaachen20";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

float accelPitch, gyroPitch, pitchAngle;
float kalmanPitch = 0.0; // Kalmanstate
float setPoint = 0.0;

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
const int AIN1 = 18;
const int PWMA = 23;
const int AIN2 = 19;

const int BIN1 = 2;
const int PWMB = 5;
const int BIN2 = 4;

// PWM settings
const int PWM_Channel = 0;
const int PWM_Frequency = 7000;
const int PWM_Resolution = 12;

// PID coefficients
float kP = 50.0;
float kD = 0.0;
float kI = 0.0;

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>PID Tuning</title>
  <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
  <style>
    body { font-family: sans-serif; text-align: center; padding: 20px; }
    input[type=range] { width: 80%; }
    .slider-container { margin: 20px; }
    canvas { max-width: 100%; }
  </style>
</head>
<body>
  <h2>PID Tuning</h2>

  <div class="slider-container">
    <label>K<sub>P</sub>: <span id="kpVal">50</span></label><br>
    <input type="range" id="kp" min="0" max="1000" value="50">
  </div>

  <div class="slider-container">
    <label>K<sub>I</sub>: <span id="kiVal">0</span></label><br>
    <input type="range" id="ki" min="0" max="4" value="0" step="0.01">
  </div>

  <div class="slider-container">
    <label>K<sub>D</sub>: <span id="kdVal">0</span></label><br>
    <input type="range" id="kd" min="0" max="70" value="0" step="0.1">
  </div>

  <div class="slider-container">
    <label>setpoint: <span id="spVal">0</span></label><br>
    <input type="range" id="sp" min="-10" max="10" value="0" step="0.1">
  </div>

  <canvas id="plot"></canvas>

  <script>
    const ws = new WebSocket(`ws://${location.hostname}/ws`);

    ws.onopen = () => {
      console.log("Connection established");
    };

    ws.onclose = () => {
      console.log('Connection closed');
    }

    const sliders = ['kp', 'ki', 'kd', 'sp'];

    sliders.forEach(id => {
      const slider = document.getElementById(id);
      const display = document.getElementById(id + 'Val');

      slider.oninput = () => {
        display.textContent = slider.value;

        if (ws.readyState === WebSocket.OPEN) {
          // Send format: "P:100,I:1.5,D:20"
          const data = `P:${document.getElementById('kp').value},` +
                       `I:${document.getElementById('ki').value},` +
                       `D:${document.getElementById('kd').value},` +
                       `S:${document.getElementById('sp').value},`;
          console.log(data);
          ws.send(data);
        }
      };
    });

    const ctx = document.getElementById('plot').getContext('2d');

    const chart = new Chart(ctx, {
      type: 'line',
      data: {
        labels: [],
        datasets: [{
          label: 'Pitch',
          borderColor: 'blue',
          data: [],
          tension: 0.3,
        }]
      },
      options: {
        animation: false,
        scales: {
          x: { display: false },
          y: { suggestedMin: -45, suggestedMax: 45 }
        }
      }
    });

    ws.onmessage = (event) => {
      const data = JSON.parse(event.data);
      chart.data.labels.push('');
      chart.data.datasets[0].data.push(data.pitch);

      const maxPoints = 100;
      if (chart.data.labels.length > maxPoints) {
        chart.data.labels.shift();
        chart.data.datasets[0].data.shift();
      }
      chart.update();
    };

  </script>
</body>
</html>
)rawliteral";

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.println("WebSocket connected");
  }
  if (type == WS_EVT_DISCONNECT) {
    Serial.println("WebSocket disconnected");
  }
  if (type == WS_EVT_DATA) {
    String msg = "";
    for (size_t i = 0; i < len; i++) msg += (char)data[i];

    Serial.println("Received: " + msg);

    int pIndex = msg.indexOf("P:");
    int iIndex = msg.indexOf("I:");
    int dIndex = msg.indexOf("D:");
    int spIndex = msg.indexOf("S:");

    if (pIndex >= 0 && iIndex >= 0 && dIndex >= 0 && spIndex >= 0) {
      float newP = msg.substring(pIndex + 2, iIndex - 1).toFloat();
      float newI = msg.substring(iIndex + 2, dIndex - 1).toFloat();
      float newD = msg.substring(dIndex + 2, spIndex - 1).toFloat();
      float newS = msg.substring(spIndex + 2).toFloat();

      kP = newP;
      kI = newI;
      kD = newD;
      setPoint = newS;

      // Serial.printf("Updated PID: P=%.2f, I=%.2f, D=%.2f\n", kP, kI, kD);
    }
  }
}

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

void balanceControls();
void calculateAngles(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);
float kalmanFilter(float &kalmanState, float &kalmanUncertainty, float kalmanInput, float kalmanMeasurement, float accelUncertainty, float gyroUncertainty, float timeStepMillis);
void Motor_1_Speed(int speed);
void Motor_2_Speed(int speed);

void setup() {

  Serial.begin(115200);
  delay(1000);

  // WiFi part
  WiFi.begin(ssid, password);

  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());

  // OTA setup for uploading code through wifi
  ArduinoOTA.setHostname("esp32-ota");  // Optional
  ArduinoOTA.begin();

  Serial.println("Ready for OTA updates");

  // Websocket part
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", index_html);
  });

  server.begin();

  // Gyroscope part
  Wire.begin(21, 22);  // uses default SDA = 21, SCL = 22
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
  Serial.println("MPU6050 ready");

  timePrev = millis();

  calibrateGyro();  // hold the robot still

  // Motor part
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  ledcAttach(PWMA, PWM_Frequency, PWM_Resolution);
  ledcAttach(PWMB, PWM_Frequency, PWM_Resolution);

}

void loop() {
  balanceControls();
  ArduinoOTA.handle();
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
  static unsigned long lastSend = 0;

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
  derivative = -kD*10*(currentAngle - previousAngle)/timeDelta2; //when tilting towards level from foward tilt, angle difference is positive (pitching up), so want a negative signal to counteract P and I
 
  if (abs(currentAngle) > 30) {
    integral = 0;
  }
  if (abs(setPoint - currentAngle) < 1.0) {
    integral *= 0.9; // slowly decay
  }

  // if (millis() - lastSend > 50) {  // send every 50ms (20Hz)
  //   lastSend = millis();
  //   String json = "{\"time\":" + String(millis()) +
  //                 ",\"pitch\":" + String(derivative) + "}";
  //   ws.textAll(json);
  // }
  // Serial.print("Proportional:");
  // Serial.print(proportional);
  // Serial.print(",Integral:");
  // Serial.print(integral);
  // Serial.print(",Derivative:");
  // Serial.println(derivative);
  
  signalOutput = constrain(proportional+integral+derivative, -4095, 4095);
  previousAngle = currentAngle;

  //Motor control
  // Serial.print("MotorSpeed:");
  // Serial.println(signalOutput);
  Motor_1_Speed(signalOutput);
  Motor_2_Speed(signalOutput);
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

  accelPitch = atan2(accelY, sqrt(accelZ*accelZ + accelX*accelX))*(180/M_PI);

  //Calculate the pitch angle using Kalman filter. Function automatically updates kalmanPitch variable
  float prediction = kalmanFilter(kalmanPitch, kalmanPitchUncertainty, gyroX, accelPitch, accelUncertainty, gyroUncertainty, timeDelta);
  pitchAngle = kalmanPitch; //update the dashboard

  // Serial monitor friendly format
  // Serial.print("accelPitch:");
  // Serial.print(accelPitch);
  // Serial.print(",gyroX:"); //comment this out for final code. This is just for testing purposes.
  // Serial.print(gyroX);
  
  // Serial.print(",filteredAngle:"); 
  // Serial.println(pitchAngle); //comment this out for final code. This is just for testing purposes.
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
  static int prev_Speed_1; //to keep track of motor direction. Saved across function calls

  if (prev_Speed_1*speed < 0 || speed == 0) //Only negative when speed and previous speed are different signs. If speed = 0, turn off MOSFETs
  {
    //turn off MOSFETs for 500 uS to prevent MOSFET shoot through
    digitalWrite(AIN1, LOW);
    ledcWrite(PWMA, 0);
    digitalWrite(AIN2, LOW);
    delayMicroseconds(500);
  }

  if (speed < 0)
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    ledcWrite(PWMA, constrain(abs(speed), 0, 4095));
  }

  if (speed > 0)
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    ledcWrite(PWMA, constrain(speed, 0, 4095));
  }
  prev_Speed_1 = speed;
}

void Motor_2_Speed(int speed) //speed ranging from -4095 to 4095 (negative is reverse, positive is forward)
{
  static int prev_Speed_2; //to keep track of motor direction. Saved across function calls

  if (prev_Speed_2*speed < 0 || speed == 0) //Only negative when speed and previous speed are different signs. If speed = 0, turn off MOSFETs
  {
    //turn off MOSFETs for 500 uS to prevent MOSFET shoot through
    digitalWrite(BIN1, LOW);
    ledcWrite(PWMB, 0);
    digitalWrite(BIN2, LOW);
    delayMicroseconds(500);
  }

  if (speed < 0)
  {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    ledcWrite(PWMB, constrain(abs(speed), 0, 4095));
  }

  if (speed > 0)
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    ledcWrite(PWMB, constrain(speed, 0, 4095));
  }
  prev_Speed_2 = speed;
}