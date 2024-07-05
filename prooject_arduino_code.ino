// Motor driver pins
const int ENA = 3; // Motor speed control (PWM)
const int IN1 = 2; // Motor direction pin 1
const int IN2 = 4; // Motor direction pin 2

// Buzzer and LED pins
const int buzzerPin = 7;
const int ledPin = 5;
const int g = 8;
const int tx = 12;
const int rx = 11;


void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(g, OUTPUT);
  Serial.begin(9600); // Start serial communication
}

void loop() {
  digitalWrite(g,HIGH);
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    if (command == "right") {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 255);
    } else if (command == "left") {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, 255);
    } else if (command == "up") {
      analogWrite(ENA, 255);
    } else if (command == "down") {
      analogWrite(ENA, 127);
    } else if (command == "stop") {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 0);
    } else if (command == "child") {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(g,LOW);
      analogWrite(ENA, 0);
      digitalWrite(buzzerPin, HIGH);
      digitalWrite(ledPin, HIGH);
      delay(200); // Buzzer and LED on for 2 seconds
      digitalWrite(buzzerPin, LOW);
      digitalWrite(ledPin, LOW);
    }
  }
}