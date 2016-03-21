#include <ros.h>
#include <std_msgs/UInt16.h>
#include <Servo.h>

ros::NodeHandle nh;

Servo myservo1;
Servo myservo2;
Servo myservo3;

void straight() {
  int pos = 0;
  myservo1.writeMicroseconds(1350);
  delay(800);
  myservo1.writeMicroseconds(1500);
  for (pos = 90; pos >= 45; pos -= 1) { 
    myservo2.write(pos);
    delay(10);
  }
  delay(1000);
  for (pos = 45; pos <= 90; pos += 1) { 
    myservo2.write(pos);
    delay(10);
  }
  myservo1.writeMicroseconds(1600);
  delay(900);
  myservo1.writeMicroseconds(1500);

}

void stp() {
  myservo1.writeMicroseconds(1500);
  myservo2.write(90);
  myservo3.write(90);
}

void right() {
  int pos = 0;
  myservo1.writeMicroseconds(1350);
  delay(800);
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

void left() {  
  int pos = 0;
  myservo1.writeMicroseconds(1350);
  delay(800);
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

  while (i != 2) {
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


void servo_cb (const std_msgs::UInt16& cmd_msg) {
  switch (cmd_msg.data) {
    case 1:
      wave();
      break;
    case 2:
      straight();
      break;
    case 3:
      left();
      break;
    case 4:
      right();
      break;
    default:
      stp();
  }
}

ros::Subscriber<std_msgs::UInt16> sub("servo",servo_cb);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  
  myservo1.attach(9);
  myservo2.attach(10);
  myservo3.attach(11);
}

void loop() {
  nh.spinOnce();
}

