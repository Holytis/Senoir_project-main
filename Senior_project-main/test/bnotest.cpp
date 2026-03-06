#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <math.h>

// กำหนดขา Hardware Serial ของ ESP32
#define RX_PIN 16
#define TX_PIN 17
#define BNO08X_RESET_PIN 4 

Adafruit_BNO08x bno08x(BNO08X_RESET_PIN);
sh2_SensorValue_t sensorValue;

// --- 1. สร้างตัวแปร Global ไว้เก็บค่าล่าสุดของเซ็นเซอร์แต่ละตัว ---
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
float mx = 0, my = 0, mz = 0;
float lx = 0, ly = 0, lz = 0;
float roll = 0, pitch = 0, yaw = 0;
uint32_t stepCount = 0;

// --- 2. ตัวแปรสำหรับตั้งเวลา (Timer) ---
unsigned long lastPrintTime = 0;
const int printInterval = 10; // อัปเดตหน้าจอทุกๆ 250 มิลลิวินาที (4 ครั้งต่อวินาที)

// ฟังก์ชันสำหรับตั้งค่า Report (เหมือนเดิม)
void setReports() {
  bno08x.enableReport(SH2_ACCELEROMETER, 50000);           
  bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 50000);    
  bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 50000); 
  bno08x.enableReport(SH2_LINEAR_ACCELERATION, 50000);     
  bno08x.enableReport(SH2_ROTATION_VECTOR, 50000);         
  bno08x.enableReport(SH2_STEP_COUNTER, 500000);           
}

// ฟังก์ชันแปลง Quaternion เป็นมุม Euler (เหมือนเดิม)
void quaternionToEuler(float qr, float qi, float qj, float qk, float* r, float* p, float* y) {
  float sqr = sq(qr); float sqi = sq(qi); float sqj = sq(qj); float sqk = sq(qk);
  *r = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr)) * 180.0 / M_PI;
  float sinp = 2.0 * (qi * qk - qj * qr);
  if (abs(sinp) >= 1) *p = copysign(M_PI / 2, sinp) * 180.0 / M_PI;
  else *p = asin(sinp) * 180.0 / M_PI;
  *y = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr)) * 180.0 / M_PI;
}

// ฟังก์ชันสำหรับจัดรูปแบบการแสดงผลให้เป็นตาราง
void printDashboard() {
  // สั่งขึ้นบรรทัดใหม่หลายๆ รอบเพื่อดันข้อมูลเก่าขึ้นไป (จำลองการเคลียร์หน้าจอ)
  Serial.println("\n\n\n\n"); 
  
  Serial.println("========================== BNO086 DASHBOARD ==========================");
  // ใช้ %7.2f หมายถึง จองพื้นที่ตัวอักษรความกว้าง 7 ตัว และแสดงทศนิยม 2 ตำแหน่ง ช่วยให้คอลัมน์ตรงกัน
  Serial.printf(" 📐 Angle (deg)  | Roll: %7.2f | Pitch: %7.2f | Yaw: %7.2f\n", roll, pitch, yaw);
  Serial.println("----------------------------------------------------------------------");
  Serial.printf(" 🚀 Accel (m/s2) | X:    %7.2f | Y:     %7.2f | Z:   %7.2f\n", ax, ay, az);
  Serial.printf(" 🏎️ LinAc (m/s2) | X:    %7.2f | Y:     %7.2f | Z:   %7.2f\n", lx, ly, lz);
  Serial.printf(" 🌀 Gyro (rad/s) | X:    %7.2f | Y:     %7.2f | Z:   %7.2f\n", gx, gy, gz);
  Serial.printf(" 🧲 Mag (uT)     | X:    %7.2f | Y:     %7.2f | Z:   %7.2f\n", mx, my, mz);
  Serial.println("----------------------------------------------------------------------");
  Serial.printf(" 🚶 Step Counter | %d ก้าว\n", stepCount);
  Serial.println("======================================================================");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial2.begin(3000000, SERIAL_8N1, RX_PIN, TX_PIN);

  if (!bno08x.begin_UART(&Serial2)) {
    Serial.println("ไม่พบเซ็นเซอร์! ตรวจสอบสายไฟและขา PS0=LOW, PS1=HIGH");
    while (1) { delay(10); }
  }
  setReports();
}

void loop() {
  // --- 3. ดักจับข้อมูลที่เข้ามา แล้วนำไปอัปเดตในตัวแปร (ยังไม่ Print) ---
  if (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ACCELEROMETER:
        ax = sensorValue.un.accelerometer.x;
        ay = sensorValue.un.accelerometer.y;
        az = sensorValue.un.accelerometer.z;
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        gx = sensorValue.un.gyroscope.x;
        gy = sensorValue.un.gyroscope.y;
        gz = sensorValue.un.gyroscope.z;
        break;
      case SH2_MAGNETIC_FIELD_CALIBRATED:
        mx = sensorValue.un.magneticField.x;
        my = sensorValue.un.magneticField.y;
        mz = sensorValue.un.magneticField.z;
        break;
      case SH2_LINEAR_ACCELERATION:
        lx = sensorValue.un.linearAcceleration.x;
        ly = sensorValue.un.linearAcceleration.y;
        lz = sensorValue.un.linearAcceleration.z;
        break;
      case SH2_ROTATION_VECTOR: {
        float r = sensorValue.un.rotationVector.real;
        float i = sensorValue.un.rotationVector.i;
        float j = sensorValue.un.rotationVector.j;
        float k = sensorValue.un.rotationVector.k;
        quaternionToEuler(r, i, j, k, &roll, &pitch, &yaw);
        break;
      }
      case SH2_STEP_COUNTER:
        stepCount = sensorValue.un.stepCounter.steps;
        break;
    }
  }

  // --- 4. ใช้ Timer ตรวจสอบเวลา หากครบ 250ms ค่อย Print ตารางออกมาทีเดียว ---
  if (millis() - lastPrintTime >= printInterval) {
    lastPrintTime = millis();
    printDashboard();
  }
}

