/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Blink
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
