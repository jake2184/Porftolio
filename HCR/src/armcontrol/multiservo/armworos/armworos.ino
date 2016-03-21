#include <Servo.h>

int func = 0;

Servo myservo1;
Servo myservo2;
Servo myservo3;

void setup() {
  Serial.begin(9600);

  myservo1.attach(9);
  myservo2.attach(10);
  myservo3.attach(11);
}

void loop() {
  
  if (Serial.available() > 0) {
    func = Serial.read();
  }
  
  switch (func) {
    case 49:
      wave();
      break;
    case 50:
      straight();
      break;
    case 51:
      left();
      break;
    case 52:
      right();
      break;
    default:
      stp();
  }

  func = 0;
  
}

void stp() {
  myservo1.writeMicroseconds(1500);
  myservo2.write(90);
  myservo3.write(90);
}

void straight() {

}

void left() {
  int pos = 0;
  myservo1.writeMicroseconds(1350);
  delay(900);
  myservo1.writeMicroseconds(1500);
  for (pos = 90; pos >= 45; pos -= 1) { 
    myservo2.write(pos);
    delay(10);
  }
  for (pos = 90; pos <= 150; pos += 1) { 
    myservo3.write(pos);
    delay(10);
  }
  delay(1000);
  for (pos = 150; pos >= 90; pos -= 1) { 
    myservo3.write(pos);
    delay(10);
  }
  for (pos = 45; pos <= 90; pos += 1) { 
    myservo2.write(pos);
    delay(10);
  }
  myservo1.writeMicroseconds(1600);
  delay(900);
  myservo1.writeMicroseconds(1500);
}


void right() {  
  int pos = 0;
  myservo1.writeMicroseconds(1350);
  delay(900);
  myservo1.writeMicroseconds(1500);
  for (pos = 90; pos >= 45; pos -= 1) { 
    myservo2.write(pos);
    delay(10);
  }
  for (pos = 90; pos >= 30; pos -= 1) { 
    myservo3.write(pos);
    delay(10);
  }
  delay(1000);
  for (pos = 30; pos <= 90; pos += 1) { 
    myservo3.write(pos);
    delay(10);
  }
  for (pos = 45; pos <= 90; pos += 1) { 
    myservo2.write(pos);
    delay(10);
  }
  myservo1.writeMicroseconds(1600);
  delay(900);
  myservo1.writeMicroseconds(1500);
}


void wave() {

  int pos = 90;
  int i = 0;
  myservo1.writeMicroseconds(1500);
  myservo3.write(90);

  while (i != 3) {
    for (pos = 90; pos >= 45; pos -= 1) { 
      myservo2.write(pos);
      delay(10);
    }
    for (pos = 45; pos <= 135; pos += 1) { 
      myservo2.write(pos);
      delay(10);
    }
    for (pos = 135; pos >= 90; pos -= 1) {
      myservo2.write(pos);
      delay(15);
    }
    i++;
  }
  
}


