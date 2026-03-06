#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_BMP280.h>
#include <esp_wifi.h>          // เพิ่ม

#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h> 
#include <rclc/executor.h>
#include <geometry_msgs/msg/vector3.h>

#include "config.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X) do { static volatile int64_t init = -1; if (init == -1) { init = uxr_millis(); } if (uxr_millis() - init > MS) { X; init = uxr_millis(); } } while (0)

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

char ssid[] = "Net_robot";
char pass[] = "0620355575";
IPAddress agent_ip(172, 20, 10, 4);
size_t agent_port = 8888;

unsigned long long time_offset = 0;

#define RX_PIN 16
#define TX_PIN 17
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
    Serial.begin(115200);
    Wire.begin(21, 22);
    Serial2.begin(3000000, SERIAL_8N1, RX_PIN, TX_PIN);

    if (!bno08x.begin_UART(&Serial2)) {
        while (1) delay(10);
    }
    bno08x.enableReport(SH2_ACCELEROMETER, 10000);
    bno08x.enableReport(SH2_ROTATION_VECTOR, 10000);

    WiFi.setAutoReconnect(true);       // reconnect WiFi อัตโนมัติ
    WiFi.persistent(true);

    set_microros_wifi_transports(ssid, pass, agent_ip, agent_port);

    WiFi.setSleep(false);              // ปิด WiFi sleep
    esp_wifi_set_ps(WIFI_PS_NONE);     // บังคับ no power saving

    state = WAITING_AGENT;
}

void loop() {
    switch (state) {
    case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(200, 3)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        break;
    case AGENT_AVAILABLE:
        state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT) destroyEntities();
        break;
    case AGENT_CONNECTED:
        // ✅ เพิ่ม interval และ retry เพื่อทนต่อ Hotspot latency
        EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(200, 3)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
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
    airpress_msg.x = bmp.readTemperature();
    airpress_msg.y = bmp.readPressure() / 100.0F;
    airpress_msg.z = 0.0;
    rcl_publish(&airpress_pub, &airpress_msg, NULL);
}

void imu_sensor() {
    if (bno08x.getSensorEvent(&sensorValue)) {
        switch (sensorValue.sensorId) {
            case SH2_ACCELEROMETER:
                acc_msg.x = sensorValue.un.accelerometer.x;
                acc_msg.y = sensorValue.un.accelerometer.y;
                acc_msg.z = sensorValue.un.accelerometer.z;
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

    // ✅ ลด rate เหลือ 20Hz เพื่อลด UDP flood บน Hotspot
    const unsigned int control_timeout = 10;
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
    RCSOFTCHECK(rmw_uros_sync_session(10));  // ✅ SOFT — ไม่ตายถ้า sync fail
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

// ✅ ไม่ตายแล้ว — reset กลับไป reconnect ใหม่แทน
void rclErrorLoop() {
    destroyEntities();
    state = WAITING_AGENT;
}