/*
  SmileEVO24.h - Library for Smile Robotics EVO24X50.0 Motor Driver
  Based on datasheet EVO24X50.0 [cite: 1, 5, 25]
*/

#ifndef SmileEVO24_h
#define SmileEVO24_h

#include "Arduino.h"

class SmileEVO24 {
  public:
    // Constructor: กำหนดขา PWM, INA, INB และขา CS (ถ้ามี)
    SmileEVO24(int pwmPin, int inaPin, int inbPin, int csPin = -1);

    // เริ่มต้นการทำงาน (ใส่ใน setup)
    void begin();

    // ขับมอเตอร์: speed มีค่าตั้งแต่ -255 ถึง 255
    // บวก (+) = หมุนทิศทางที่ 1 (Clockwise)
    // ลบ (-) = หมุนทิศทางที่ 2 (Counter-Clockwise)
    // 0 = หยุด
    void drive(int speed);

    // หยุดมอเตอร์แบบ Brake (หน่วง)
    void brake();

    // หยุดมอเตอร์แบบ Free (ปล่อยไหล)
    void coast();

    // อ่านค่ากระแส (คืนค่าเป็น Analog 0-1023)
    // หมายเหตุ: เอกสารระบุว่า Scale factor ยังรอการปรับปรุง 
    int readCurrentRaw();

  private:
    int _pwmPin;
    int _inaPin;
    int _inbPin;
    int _csPin;
};

#endif