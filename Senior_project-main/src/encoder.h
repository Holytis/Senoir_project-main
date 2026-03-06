#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

// เพิ่มส่วนนี้เพื่อรองรับการเรียกใช้จากไฟล์ .cpp
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int pinA;
    int pinB;
    volatile long count; 
    long prevCount;      
} Encoder_t;

void Encoder_Init(Encoder_t *enc, int pinA, int pinB);
long Encoder_GetCount(Encoder_t *enc);
long Encoder_GetDelta(Encoder_t *enc);
void Encoder_Reset(Encoder_t *enc);

#ifdef __cplusplus
}
#endif

#endif