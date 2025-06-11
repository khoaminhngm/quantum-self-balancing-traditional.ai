// Motor pins
const int Motor1_Ena1 = 23;   // AIN1
const int Motor1_PWM1 = 2;   // PWMA
const int Motor1_Ena2 = 22;  // AIN2

// PWM settings
const int PWM_Channel = 0;
const int PWM_Frequency = 7000;
const int PWM_Resolution = 12; // 0 - 4095

void setup() {
  delay(1000);
  Serial.begin(115200);
  delay(1500);
  // Serial.println("Started.");
  // Serial.println("Started.");
  // Serial.println("Started.");

  pinMode(Motor1_Ena1, OUTPUT);
  pinMode(Motor1_Ena2, OUTPUT);

  // Setup PWM
  bool a = ledcAttach(Motor1_PWM1, PWM_Frequency, PWM_Resolution);
  if (a) {
    Serial.println("Success");
  } else {
    Serial.println("Not successful");
  }
  
  // ledcAttachChannel(Motor1_PWM1, PWM_Frequency, PWM_Resolution, PWM_Channel);
  // Spin motor forward at 50% speed
  digitalWrite(Motor1_Ena1, LOW);
  digitalWrite(Motor1_Ena2, HIGH);
  bool c = ledcWrite(Motor1_PWM1, 2048);  // 50% of 4095
  if (c) {
    Serial.println("Successfully ledcwrite.");
  } else {
    Serial.println("Did not write");
  }
}

void loop() {
  Motor_1_Speed(2000);
  delay(1000);
}


//Motor functions
void Motor_1_Speed(int speed) //speed ranging from -4095 to 4095 (negative is reverse, positive is forward)
{
  static int prev_Speed; //to keep track of motor direction. Saved across function calls

  if (prev_Speed*speed < 0 || speed == 0) //Only negative when speed and previous speed are different signs. If speed = 0, turn off MOSFETs
  {
    //turn off MOSFETs for 500 uS to prevent MOSFET shoot through
    digitalWrite(Motor1_Ena1, LOW);
    ledcWrite(Motor1_PWM1, 0); //Motor1_PWM1
    // ledcWrite(1, 0); //Motor1_PWM2
    digitalWrite(Motor1_Ena2, LOW);
    delayMicroseconds(500);
  }

  if (speed < 0)
  {
    digitalWrite(Motor1_Ena1, HIGH);
    digitalWrite(Motor1_Ena2, LOW);
    ledcWrite(Motor1_PWM1, constrain(abs(speed), 0, 4095)); //Motor1_PWM2
  }

  if (speed > 0)
  {
    digitalWrite(Motor1_Ena1, LOW);
    digitalWrite(Motor1_Ena2, HIGH);
    bool ok = ledcWrite(Motor1_PWM1, constrain(speed, 0, 4095)); //Motor1_PWM1
    if (ok) {
      Serial.println("Written in loop.");
    } else {
      Serial.println("Did not write in loop.");
    }
  }
  prev_Speed = speed; //save the current speed for next time function is called
}

// void Motor_2_Speed(int speed) //speed ranging from -4095 to 4095 (negative is reverse, positive is forward)
// {
//   static int prev_Speed; //to keep track of motor direction. Saved across function calls

//   if (prev_Speed*speed < 0 || speed == 0) //Only negative when speed and previous speed are different signs. If speed = 0, turn off MOSFETs
//   {
//     //turn off MOSFETs for 500 uS to prevent MOSFET shoot through
//     digitalWrite(Motor2_Ena1, LOW);
//     ledcWrite(2, 0); //Motor2_PWM1
//     ledcWrite(3, 0); //Motor2_PWM2
//     digitalWrite(Motor2_Ena2, LOW);
//     delayMicroseconds(500);
//   }

//   if (speed > 0)
//   {
//     digitalWrite(Motor2_Ena1, HIGH);
//     ledcWrite(3, constrain(speed, 0, 4095)); //Motor2_PWM2
//   }

//   if (speed < 0)
//   {
//     digitalWrite(Motor2_Ena2, HIGH);
//     ledcWrite(2, constrain(abs(speed), 0, 4095)); //Motor2_PWM1
//   }
//   prev_Speed = speed; //save the current speed for next time function is called
// }