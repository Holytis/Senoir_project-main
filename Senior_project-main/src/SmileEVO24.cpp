/*
  SmileEVO24.cpp - Implementation
  Control Logic based on Table in Page 1 and Example in Page 4 [cite: 22, 101, 102]
*/

#include "SmileEVO24.h"

SmileEVO24::SmileEVO24(int pwmPin, int inaPin, int inbPin, int csPin) {
  _pwmPin = pwmPin;
  _inaPin = inaPin;
  _inbPin = inbPin;
  _csPin = csPin;
}

void SmileEVO24::begin() {
  pinMode(_pwmPin, OUTPUT);
  pinMode(_inaPin, OUTPUT);
  pinMode(_inbPin, OUTPUT);
  
  if (_csPin != -1) {
    pinMode(_csPin, INPUT);
  }

  // เริ่มต้นด้วยการหยุดมอเตอร์
  brake();
}

void SmileEVO24::drive(int speed) {
  // ป้องกันค่าเกินขอบเขต -255 ถึง 255
  if (speed > 255) speed = 255;
  if (speed < -255) speed = -255;

  if (speed > 0) {
    // ทิศทางที่ 1: INA=HIGH, INB=LOW (อ้างอิง Code ตัวอย่างหน้า 4 [cite: 101, 102])
    digitalWrite(_inaPin, HIGH);
    digitalWrite(_inbPin, LOW);
    analogWrite(_pwmPin, speed);
  } 
  else if (speed < 0) {
    // ทิศทางที่ 2: INA=LOW, INB=HIGH [cite: 107]
    digitalWrite(_inaPin, LOW);
    digitalWrite(_inbPin, HIGH);
    analogWrite(_pwmPin, -speed); // แปลงค่าลบเป็นบวกสำหรับ PWM
  } 
  else {
    // Speed = 0 ให้ทำการ Brake
    brake();
  }
}

void SmileEVO24::brake() {
  // การเบรก: PWM Low, INA Low, INB Low [cite: 22]
  // ตามตารางหน้า 1 แถวที่ 3
  digitalWrite(_inaPin, LOW);
  digitalWrite(_inbPin, LOW);
  analogWrite(_pwmPin, 0); // PWM Low
}

void SmileEVO24::coast() {
  // การปล่อยไหล (Free): INA High, INB High [cite: 23]
  // ตามตารางหน้า 1 แถวที่ 4 (X FREE)
  digitalWrite(_inaPin, HIGH);
  digitalWrite(_inbPin, HIGH);
  analogWrite(_pwmPin, 0); 
}

int SmileEVO24::readCurrentRaw() {
  if (_csPin != -1) {
    return analogRead(_csPin);
  }
  return 0;
}