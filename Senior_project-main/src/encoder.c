#include "encoder.h"

// ISR: ทำงานทันทีเมื่อมีสัญญาณ Pulse
void IRAM_ATTR encoder_isr(void* arg) {
    Encoder_t* enc = (Encoder_t*)arg;
    
    // ตรวจสอบทิศทาง ถ้า PinB HIGH ให้บวก ถ้า LOW ให้ลบ
    if (digitalRead(enc->pinB)) {
        enc->count++;
    } else {
        enc->count--;
    }
}

void Encoder_Init(Encoder_t *enc, int a, int b) {
    enc->pinA = a;
    enc->pinB = b;
    enc->count = 0;
    enc->prevCount = 0; // กำหนดค่าเริ่มต้นตัวแปรจำค่าเก่า

    // จัดการ Pullup สำหรับขา ESP32 (ขา >= 34 ไม่มี Pullup ภายใน)
    if (a < 34) pinMode(a, INPUT_PULLUP);
    else pinMode(a, INPUT);
    
    if (b < 34) pinMode(b, INPUT_PULLUP);
    else pinMode(b, INPUT);

    // Attach Interrupt
    attachInterruptArg(digitalPinToInterrupt(enc->pinA), encoder_isr, (void*)enc, RISING);
}

long Encoder_GetCount(Encoder_t *enc) {
    long temp;
    noInterrupts(); // ปิด Interrupt ชั่วคราวเพื่ออ่านค่าให้ครบถ้วน (Atomic)
    temp = enc->count;
    interrupts();
    return temp;
}

// ** ไฮไลท์สำคัญ: ฟังก์ชันหาค่า Delta **
long Encoder_GetDelta(Encoder_t *enc) {
    long current_count;
    
    // 1. อ่านค่าปัจจุบันแบบปลอดภัย (Atomic Read)
    noInterrupts();
    current_count = enc->count;
    interrupts();

    // 2. คำนวณความต่าง (ปัจจุบัน - อดีต)
    long delta = current_count - enc->prevCount;

    // 3. อัปเดตค่าอดีต ให้เท่ากับปัจจุบัน เพื่อใช้รอบหน้า
    enc->prevCount = current_count;

    return delta;
}

void Encoder_Reset(Encoder_t *enc) {
    noInterrupts();
    enc->count = 0;
    enc->prevCount = 0;
    interrupts();
}