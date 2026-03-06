#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_BMP280.h>

#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h> 
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/vector3.h>

#include "config.h"
#include "driver/twai.h"

///////////////////////////////////////////////////////////////////////////////
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X) do { static volatile int64_t init = -1; if (init == -1) { init = uxr_millis(); } if (uxr_millis() - init > MS) { X; init = uxr_millis(); } } while (0)
///////////////////////////////////////////////////////////////////////////////

unsigned long long time_offset = 0;

rcl_publisher_t acc_pub;
rcl_publisher_t euler_pub;
rcl_publisher_t airpress_pub;

geometry_msgs__msg__Vector3 acc_msg;
geometry_msgs__msg__Vector3 euler_msg;
geometry_msgs__msg__Vector3 airpress_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_init_options_t init_options;

enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

void rclErrorLoop();
void syncTime();
bool createEntities();
bool destroyEntities();
struct timespec getTime();
void imu_sensor();
void bmp_sensor();

// --- การตั้งค่าขา ---
// *** UART สำหรับ BNO08x ***
#define BNO08X_RX_PIN  16
#define BNO08X_TX_PIN  17

// *** UART สำหรับ micro-ROS (ต้องแยกจาก BNO08x) ***
// ใช้ Serial0 (UART0) หรือเชื่อมต่อ USB-to-UART ที่ขา GPIO กำหนดเอง
// ตัวอย่าง: ใช้ UART0 (GPIO1=TX, GPIO3=RX) ซึ่งปกติคือ USB Serial บน ESP32
// หรือกำหนดขาใหม่สำหรับ UART2 ถ้าต้องการ
#define UROS_SERIAL   Serial   // ใช้ USB Serial (UART0) สำหรับ micro-ROS

#define BNO08X_RESET_PIN 4 

Adafruit_BMP280 bmp;
Adafruit_BNO08x bno08x(BNO08X_RESET_PIN);
sh2_SensorValue_t sensorValue;

float roll, pitch, yaw;
float ax, ay, az;

void quaternionToEuler(float qr, float qi, float qj, float qk, float* r, float* p, float* y) {
    float sqr = sq(qr); float sqi = sq(qi); float sqj = sq(qj); float sqk = sq(qk);
    *r = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr)) * 180.0 / M_PI;
    float sinp = 2.0 * (qi * qk - qj * qr);
    if (abs(sinp) >= 1) *p = copysign(M_PI / 2, sinp) * 180.0 / M_PI;
    else *p = asin(sinp) * 180.0 / M_PI;
    *y = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr)) * 180.0 / M_PI;
}

void setup() {
    Serial.begin(921600);
    delay(100);  // รอให้ Serial พร้อม
    // BNO08x ใช้ Serial2 (UART2) ที่ขา RX=16, TX=17
    Serial2.begin(3000000, SERIAL_8N1, BNO08X_RX_PIN, BNO08X_TX_PIN);
    if (!bno08x.begin_UART(&Serial2)) {
        while (1) delay(10);
    }

    bno08x.enableReport(SH2_LINEAR_ACCELERATION, 5000);
    bno08x.enableReport(SH2_ROTATION_VECTOR, 5000);

    Wire.begin(21, 22);
    if (!bmp.begin(0x76)) {
        // BMP280 ไม่ตอบสนอง — ข้ามไปก็ได้หรือ blink LED แจ้งเตือน
    }

    // *** ตั้งค่า micro-ROS ผ่าน UART (USB Serial / UART0) ***
    // Baud rate 115200 เป็นค่า default ของ micro-ROS UART transport
    set_microros_serial_transports(UROS_SERIAL);  // <-- เปลี่ยนจาก WiFi เป็น UART

    state = WAITING_AGENT;
}

void loop() {
    switch (state) {
    case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        break;
    case AGENT_AVAILABLE:
        state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT) destroyEntities();
        break;
    case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (state == AGENT_CONNECTED) {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        }
        break;
    case AGENT_DISCONNECTED:
        destroyEntities();
        state = WAITING_AGENT;
        break;
    default:
        break;
    }
}

void controlCallback(rcl_timer_t *timer, int64_t last_call_time) {
    if (timer != NULL) {
        imu_sensor();
        bmp_sensor();
    }
}

void bmp_sensor() {
    float temp  = bmp.readTemperature();
    float press = bmp.readPressure() / 100.0F;

    airpress_msg.x = temp;
    airpress_msg.y = press;
    airpress_msg.z = 0.0;

    rcl_publish(&airpress_pub, &airpress_msg, NULL);
}

void imu_sensor() {
    while (bno08x.getSensorEvent(&sensorValue)) {
        switch (sensorValue.sensorId) {
            case SH2_LINEAR_ACCELERATION:                        // เปลี่ยนจาก SH2_ACCELEROMETER
                ax = sensorValue.un.linearAcceleration.x;        // เปลี่ยนจาก .accelerometer
                ay = sensorValue.un.linearAcceleration.y;
                az = sensorValue.un.linearAcceleration.z;

                acc_msg.x = ax;
                acc_msg.y = ay;
                acc_msg.z = az;
                rcl_publish(&acc_pub, &acc_msg, NULL);
                break;

            case SH2_ROTATION_VECTOR:
                quaternionToEuler(
                    sensorValue.un.rotationVector.real,
                    sensorValue.un.rotationVector.i,
                    sensorValue.un.rotationVector.j,
                    sensorValue.un.rotationVector.k,
                    &roll, &pitch, &yaw
                );

                euler_msg.x = roll;
                euler_msg.y = pitch;
                euler_msg.z = yaw;
                rcl_publish(&euler_pub, &euler_msg, NULL);
                break;
        }
    }
}

bool createEntities() {
    allocator = rcl_get_default_allocator();
    init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 22);

    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    RCCHECK(rclc_node_init_default(&node, "bno08x_node", "", &support));

    RCCHECK(rclc_publisher_init_best_effort(&acc_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "/bno/accel"));
    RCCHECK(rclc_publisher_init_best_effort(&euler_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "/bno/euler"));
    RCCHECK(rclc_publisher_init_best_effort(&airpress_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "/air/airpress"));

    const unsigned int control_timeout = 10; // 50 Hz
    RCCHECK(rclc_timer_init_default(&control_timer, &support,
        RCL_MS_TO_NS(control_timeout), controlCallback));

    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    syncTime();
    return true;
}

bool destroyEntities() {
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&acc_pub, &node);
    rcl_publisher_fini(&euler_pub, &node);
    rcl_publisher_fini(&airpress_pub, &node);
    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);
    return true;
}

void syncTime() {
    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    time_offset = ros_time_ms - now;
}

struct timespec getTime() {
    struct timespec tp = {0};
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
    return tp;
}

void rclErrorLoop() {
    while (true) {}
}