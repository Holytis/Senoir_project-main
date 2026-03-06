#include "bts7960.h"
#include <Arduino.h>  // <--- เพิ่มบรรทัดนี้

// void MCU_PWM_Write(uint16_t pin, uint32_t value) {
//     analogWrite(pin, value); // ตอนนี้คอมไพเลอร์จะรู้จักฟังก์ชันนี้แล้ว
// }
/* * หมายเหตุ: ฟังก์ชัน analogWrite ต้องปรับตาม Library ของ MCU ที่คุณใช้ 
 * เช่น analogWrite() ของ Arduino หรือ HAL_TIM_PWM_Start() ของ STM32
 */
// extern void analogWrite(uint16_t pin, uint32_t value);

// void analogWrite(uint16_t pin, uint32_t value) {
//     analogWrite(pin, value);
// }

void BTS7960_Init(BTS7960_t *motor) {
    BTS7960_Stop(motor);
}

void BTS7960_SetSpeed(BTS7960_t *motor, int32_t speed) {
    // ป้องกันค่าเกินช่วง Max PWM
    if (speed > motor->max_pwm) speed = motor->max_pwm;
    if (speed < -motor->max_pwm) speed = -motor->max_pwm;

    if (speed > 0) {
        // Forward: จ่าย PWM ที่ RPWM, LPWM เป็น 0
        analogWrite(motor->RPWM_Pin, (uint32_t)speed);
        analogWrite(motor->LPWM_Pin, 0);
    } 
    else if (speed < 0) {
        // Backward: จ่าย PWM ที่ LPWM, RPWM เป็น 0
        analogWrite(motor->RPWM_Pin, 0);
        analogWrite(motor->LPWM_Pin, (uint32_t)(-speed));
    } 
    else {
        BTS7960_Stop(motor);
    }
}

void BTS7960_Stop(BTS7960_t *motor) {
    analogWrite(motor->RPWM_Pin, 0);
    analogWrite(motor->LPWM_Pin, 0);
}