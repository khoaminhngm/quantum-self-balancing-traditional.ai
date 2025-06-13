int pulses;                    // Count encoder pulses
int encoderA = 19;             // Encoder signal A (interrupt pin)
int encoderB = 18;             // Encoder signal B
int pulsesChanged = 0;         // Flag to indicate pulse count change

void setup() {
  Serial.begin(9600);
  delay(1000);

  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderA), A_CHANGE, CHANGE);
  
}

void loop() {
  if (pulsesChanged != 0) {
    pulsesChanged = 0;
    Serial.println(pulses);   // Print pulses when updated
  }

  // Serial.print("EncoderA:");
  // Serial.print(digitalRead(encoderA));
  // Serial.print(",");
  // Serial.print("EncoderB:");
  // Serial.println(digitalRead(encoderB));
}

void A_CHANGE() {
  // Called on any change of encoderA signal

  if (digitalRead(encoderB) == 0) {
    if (digitalRead(encoderA) == 0) {
      pulses--;  // Encoder rotating one direction
    } else {
      pulses++;  // Encoder rotating opposite direction
    }
  } else {
    if (digitalRead(encoderA) == 0) {
      pulses++;  // Opposite direction
    } else {
      pulses--;  // Original direction
    }
  }
  pulsesChanged = 1;  // Set flag to update Serial/LCD
}



// Below is ChatGPT code

// volatile long encoderCount = 0;
// int lastEncoded = 0;

// void setup() {
//   Serial.begin(9600);

//   pinMode(2, INPUT_PULLUP);  // Encoder A
//   pinMode(3, INPUT_PULLUP);  // Encoder B

//   attachInterrupt(digitalPinToInterrupt(2), updateEncoder, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(3), updateEncoder, CHANGE);
// }

// void loop() {
//   static long lastCount = 0;
//   static unsigned long lastTime = 0;

//   unsigned long now = millis();
//   if (now - lastTime >= 1000) {
//     long count = encoderCount;
//     long delta = count - lastCount;
//     Serial.print("Pulses/sec: ");
//     Serial.println(delta);

//     lastCount = count;
//     lastTime = now;
//   }
// }

// void updateEncoder() {
//   int MSB = digitalRead(2); // A
//   int LSB = digitalRead(3); // B

//   int encoded = (MSB << 1) | LSB;
//   int sum = (lastEncoded << 2) | encoded;

//   // This lookup table implements a Gray code state machine
//   if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderCount++;
//   if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderCount--;

//   lastEncoded = encoded;
// }
