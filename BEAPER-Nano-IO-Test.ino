 /*
 Project:   BEAPER-Nano-IO-Test
 Date:      August 12, 2024
 
 Test on-board BEAPER Nano I/O devices, plus starter robot functions.
*/

// BEAPER I/O (input/output) pin settings
// Define BEAPER pins used for human interface devices
const int SW2 = 0;            // Pushbuttons
const int SW3 = 1;
const int SW4 = 2;
const int SW5 = 3;

const int LED2 = 4;           // LEDs
const int LED3 = 5;
const int LED4 = 6;
const int LED5 = 7;

const int BEEPER = 8;         // Piezo beeper LS1

const int M1A = 4;            // Motor driver outputs (I/O pins shared with LEDs)
const int M1B = 5;
const int M2A = 6;
const int M2B = 7;

const int H1 = 21;            // 3.3V Analog header pins (shared with 5V digital output pins)
const int H2 = 23;
const int TRIG = 23;          // SONAR module TRIG pin
const int H3 = 24;
const int ECHO = 24;          // SONAR module ECHO pin
const int H4 = 22;

const int H5OUT = 21;         // 5V Digital/Servo output header pins (Same as 3.3V Analog header pins)
const int H6OUT = 22;
const int H7OUT = 23;
const int H8OUT = 24;

// Define BEAPER Analog input devices. Each analog input device is defined by both a mnemonic and
// its part reference: e.g. ANLIGHT/ANQ4 both refer to ambient light sensor Q4.
// Use jumpers JP1-JP4 to secect between pairs of analog input devices.
const int ANLIGHT = A0;       // ANLIGHT = AN(alog) ambient LIGHT sensor Q4
const int ANQ4 = A0;          // Analog phototransistor Q4 (same as above)
const int ANLFLOOR = A0;      // ANLFLOOR = AN(alog) L(eft) FLOOR sensor Q1
const int ANQ1 = A0;          // Floor sensor phototransistor Q1 on left sensor module (same as above)

const int ANTEMP = A1;        // ANTEMP = AN(alog) TEMP(erature) sensor U4
const int ANU4 = A1;          // Analog temperature sensor U4
const int ANLLINE  = A1;      // ANLLINE = AN(alog) L(eft) LINE sensor Q2
const int ANQ2 = A1;          // Left line sensor phototransistor Q2 on right sensor module

const int ANLPOT = A2;        // ANLPOT = AN(alaog) L(eft) POT(entiometer) RV1
const int ANRV1 = A2;         // Analog left potentiometer RV1
const int ANRFLOOR = A2;      // ANRFLOOR = AN(alog) R(ight) FLOOR sensor Q3
const int ANRLINE = A2;       // ANRLINE = AN(alog) R(ight) LINE sensor Q3
const int ANQ3 = A2;          // Right line/floor sensor phototransistor Q3 on right sensor module

const int ANRPOT = A3;        // ANRPOT = AN(alog) R(ight) POT(entiometer) RV2
const int ANRV2 = A3;         // Analog right potentiometer RV2
const int ANVDIV = A3;        // ANVDIV = AN(alog) battery V(oltage) DIV(ider) - R25 & R26
const int ANVBATT = A3;       // ANVBATT = AN(alog) V(oltage), BATT(ery) - from R25 & R26

// Pre-defined Arduino Nano ESP32 LEDS
// LED_BUILTIN (D13)          // Yellow LED
// LED_BLUE                   // RGB LED blue element (active LOW)
// LED_GREEN                  // RGB LED green element (active LOW)
// LED_RED                    // RGB LED red element(acive LOW)

// Define input variables
int SW2State;                 // Pushbutton states
int SW3State;
int SW4State;
int SW5State;

int rawLight;                 // Analog input levels
int rawTemp;
int posRV1;
int posRV2;

void setup(void) {
  Serial.begin(115200);       // Start serial port

  // Initialize I/O pin directions/types
  pinMode(SW2, INPUT_PULLUP);
  pinMode(SW3, INPUT_PULLUP);
  pinMode(SW4, INPUT_PULLUP);
  pinMode(SW5, INPUT_PULLUP);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(LED5, OUTPUT);
  pinMode(BEEPER, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(H5OUT, OUTPUT);
  pinMode(H6OUT, OUTPUT);

  analogWriteResolution(12);  // Match analogRead() resolution for ESP32
  Serial.println(F("Starting BEAPER"));
}

void loop() {
  SW2State = digitalRead(SW2);
  SW3State = digitalRead(SW3);
  SW4State = digitalRead(SW4);
  SW5State = digitalRead(SW5);

  if(SW2State == LOW) {
    digitalWrite(LED2, HIGH);
    Serial.println("SW2 pressed.");
    tone(BEEPER, 1047, 200);
    while(SW2State == LOW) {
      delay(50);
      SW2State = digitalRead(SW2);
    }
    digitalWrite(LED2, LOW);
    Serial.println("SW2 released.");
  }

  if(SW3State == LOW) {
    digitalWrite(LED3, HIGH);
    Serial.println("SW3 pressed.");
    tone(BEEPER, 1319, 200);
    while(SW3State == LOW) {
      delay(50);
      SW3State = digitalRead(SW3);
    }
    digitalWrite(LED3, LOW);
    Serial.println("SW3 released.");
  }

  if(SW4State == LOW) {
    digitalWrite(LED4, HIGH);
    Serial.println("SW4 pressed.");
    tone(BEEPER, 1568, 200);
    while(SW4State == LOW) {
      delay(50);
      SW4State = digitalRead(SW4);
    }
    digitalWrite(LED4, LOW);
    Serial.println("SW4 released.");
  }

  if(SW5State == LOW) {
    digitalWrite(LED5, HIGH);
    Serial.println("SW5 pressed.");
    tone(BEEPER, 2093, 200);
    while(SW5State == LOW) {
      delay(50);
      SW5State = digitalRead(SW5);
    }
    digitalWrite(LED5, LOW);
    Serial.println("SW5 released.");
  }

  // Read analog input levels
  rawLight = analogRead(ANLIGHT);
  rawTemp = analogRead(ANTEMP);
  posRV1 = analogRead(ANRV1);
  posRV2 = analogRead(ANRV2);

  Serial.print("Light:");
  Serial.println(rawLight);
  Serial.print("Temp:");
  Serial.println(rawTemp);
  Serial.print("RV1 position:");
  Serial.println(posRV1);
  Serial.print("RV2 position:");
  Serial.println(posRV2);
  Serial.println();

  delay(200);

  // Analog output can be 8-bit, or 12 bit when using 'analogWriteResolution(12);'
  // analogWrite(H5OUT, map(posRV1, 0, 4095, 0, 255)); // Map to 8 bits, or
  // analogWrite(H5OUT, posRV1);     // Output as 12 bits

  // Make periodic servo pulses without interrupts or PWM
  //if(millis() - servoTimer > 15) {
  //  servoTimer = millis();
  //  servoWrite(H5OUT, map(posRV1, 0, 4095, 0, 90));
  //  servoWrite(H6OUT, map(posRV2, 0, 4095, 0, 90));
  //}
}

// Sample robot-related functions

// Create a servo pulse on the specified servo pin. Map pulse duration to degrees.
// Example use: servoWrite(H5OUT, 45);
// -> creates a 1500us servo pulse on BEAPER Nano header pin H5 (using 90 deg servos).
void servoWrite(int servo, int deg) {
  int pulse = map(deg, 0, 90, 1000, 2000);  // For 90 degree standard servos
  // int pulse = map(deg, 0, 180, 500, 2500);  // For 180 degree servos
  digitalWrite(servo, HIGH);
  delayMicroseconds(pulse);
  digitalWrite(servo, LOW);
}

// Return range to the closest target in cm. Limit search range to max distance (cm).
// Example usage: cm = sonarRange(100);  -> returns distance to closest target
// within 100cm, or returns 0 if a target is beyond the specified range or if no
// target is found. Returns -1 if the SONAR module is not ready.
int sonarRange(int max) {
  if(digitalRead(ECHO) == HIGH) {
    return -1;                // ECHO in progress. Return error code.
  }
  digitalWrite(TRIG, HIGH);   // All clear? Trigger a new SONAR ping
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  // Time ECHO pulse duration. Includes TRIG setup and transmit time in the time-out value.
  // Some 3.3V HC-SR04P modules have 2x longer setup time than 5V HC-SR04 modules.
  unsigned long duration = pulseIn(ECHO, HIGH, max * 58 + 2320);
  if(duration == 0) {
    return 0;                 // Return 0 if no target is within max range
  }
  return (int(duration / 58));	// Return distance to target
}

// Motor stop function
void stop() {
  digitalWrite(M1A, LOW);
  digitalWrite(M1B, LOW);
  digitalWrite(M2A, LOW);
  digitalWrite(M2B, LOW);
}

// Motor forward function
void goFwd() {
  digitalWrite(M1A, HIGH);
  digitalWrite(M1B, LOW);
  digitalWrite(M2A, LOW);
  digitalWrite(M2B, HIGH);
}
