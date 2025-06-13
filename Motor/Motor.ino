// Motor pins
const int Motor1_Ena1 = 14;   // AIN1
const int Motor1_PWM1 = 26;   // PWMA
const int Motor1_Ena2 = 27;  // AIN2
// const int STBY = 4;

// PWM settings
const int PWM_Channel = 0;
const int PWM_Frequency = 7000;
const int PWM_Resolution = 12; // 0 - 4095

void setup() {
  delay(1000);
  Serial.begin(115200);
  delay(1500);
  // Serial.println("Started.");

  pinMode(Motor1_Ena1, OUTPUT);
  pinMode(Motor1_Ena2, OUTPUT);
  // pinMode(STBY, OUTPUT);

  // Setup PWM
  ledcAttach(Motor1_PWM1, PWM_Frequency, PWM_Resolution);
  
  // ledcAttachChannel(Motor1_PWM1, PWM_Frequency, PWM_Resolution, PWM_Channel);
  // Spin motor forward at 50% speed
  digitalWrite(Motor1_Ena1, LOW);
  digitalWrite(Motor1_Ena2, HIGH);
  // digitalWrite(STBY, HIGH);

  ledcWrite(Motor1_PWM1, 2048);  // 50% of 4095
}

int i = 0;
void loop() {
  
  if (i%2 == 0) {
    Motor_1_Speed(4000);
    delay(1000);
  } else {
    Motor_1_Speed(4000);
    delay(1000);
  }
  i++;
  Serial.println(i);
}


//Motor functions
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