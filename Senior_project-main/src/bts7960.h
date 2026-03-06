#ifndef BTS7960_H
#define BTS7960_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t RPWM_Pin;
    uint16_t LPWM_Pin;
    int32_t max_pwm;
} BTS7960_t;

void BTS7960_Init(BTS7960_t *motor);
void BTS7960_SetSpeed(BTS7960_t *motor, int32_t speed);
void BTS7960_Stop(BTS7960_t *motor);

#ifdef __cplusplus
}
#endif

#endif