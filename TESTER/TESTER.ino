/*

TESTER

Connect 
IMAGE_SEL0 - IMAGE_SEL1
UNIT_SEL1 - UNIT_SEL2
UNIT_SEL0 - ACTIVITY_LED
SCSI DBP (18) - SCSI ATN (32)
SCSI DB7 (16) - SCSI BSY (36)
SCSI DB6 (14) - SCSI ACK (38)
SCSI DB5 (12) - SCSI RST (40)
SCSI DB4 (10) - SCSI MSG (42)
SCSI DB3 (8)  - SCSI SEL (44)
SCSI DB2 (6)  - SCSI CD  (46)
SCSI DB1 (4) -  SCSI IO  (50)
SCSI DB0 (2) -  SCSI REQ (48)

*/

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  //pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);
}

bool testPins(int pinOne, int pinTwo, char * str, bool debug) {
  pinMode(pinOne, OUTPUT);
  pinMode(pinTwo, INPUT);
  digitalWrite(pinOne, HIGH);
  delay(1);  
  if (digitalRead(pinTwo) == HIGH) { 
    if (debug) {
      Serial.print(str);
      Serial.println(" HIGH OK");
    }
  } else {
    Serial.print(str);
    Serial.println(" HIGH NOT OK");
  }
  digitalWrite(pinOne, LOW);
  delay(1); 
  if (digitalRead(pinTwo) == LOW) {
    if (debug) { 
      Serial.print(str);
      Serial.println(" LOW OK");
    }
  } else {
    Serial.print(str);
    Serial.println(" LOW NOT OK");
  }
  pinMode(pinTwo, OUTPUT);
  pinMode(pinOne, INPUT);
  digitalWrite(pinTwo, HIGH);
  delay(1);
  
  if (digitalRead(pinOne) == HIGH) {
    if (debug) { 
      Serial.print(str);
      Serial.println(" HIGH OK");
    }
  } else {
    Serial.print(str);
    Serial.println(" HIGH NOT OK");
  }
  digitalWrite(pinTwo, LOW);
  delay(1);
  
  if (digitalRead(pinOne) == LOW) {
    if (debug) { 
      Serial.print(str);
      Serial.println(" LOW OK");
    }
  } else {
    Serial.print(str);
    Serial.println(" LOW NOT OK");
  }
}

// the loop function runs over and over again forever
void loop() {
  Serial.println("START");
  //digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  testPins(PB9, PA0, "PA0/PB9", false);
  testPins(PB8, PA1, "PA1/PB8", false);
  testPins(PB10, PA2, "PA2/PB10", false);
  testPins(PB11, PB7, "PB7/PB11", false);
  testPins(PB12, PB6, "PB12/PB6", false);
  testPins(PB13, PB5, "PB13/PB5", false);
  testPins(PB14, PB4, "PB14/PB4", false);
  testPins(PB15, PB3, "PB15/PB3", false);
  testPins(PB0, PA15, "PB0/PA15", false);
  testPins(PB1, PA8, "PB1/PA8", false);
  testPins(PA3, PC15, "PA3/PC15", false);
  testPins(PC13, PC14, "PC13/PC14", false);
  Serial.println("STOP");
  //digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}
