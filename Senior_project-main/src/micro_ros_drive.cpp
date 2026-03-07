#include <Arduino.h>
#include <math.h>
#include "bts7960.h"
#include "encoder.h"
#include "pid.h"
#include "driver/twai.h"

// micro-ROS Headers
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>

// ==========================================
// 1. CONFIGURATION & PINS
// ==========================================
BTS7960_t motor_L = { 22, 23, 255 }; 
BTS7960_t motor_R = { 25, 26, 255 };
Encoder_t encLeft, encRight;

// ค่า PID สำหรับควบคุมล้อ
PID PIDMotorL(-255, 255, 200.0, 0.0, 0.0);
PID PIDMotorR(-255, 255, 200.0, 0.0, 0.0);

const float WHEEL_PPR    = 16.0 * 99.5;
const float WHEEL_RADIUS = 0.04;
const float TRACK_WIDTH  = 0.37;
const float DIST_PER_TICK = (2.0 * M_PI * WHEEL_RADIUS) / WHEEL_PPR;

// ==========================================
// 2. SHARED STATE (Core 0 ↔ Core 1)
// ==========================================
portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;

volatile float g_totalDist    = 0.0f;   // z: ระยะทางสะสม (m)
volatile float g_vel_left     = 0.0f;   // vL จาก encoder (m/s)
volatile float g_vel_right    = 0.0f;   // vR จาก encoder (m/s)
volatile float target_v       = 0.0f;   // รับจาก Pi
volatile float target_omega   = 0.0f;   // รับจาก Pi

// micro-ROS Variables
rcl_subscription_t cmd_sub;
rcl_publisher_t    drive_pub;
geometry_msgs__msg__Twist   msg_cmd;
geometry_msgs__msg__Vector3 msg_drive;
rclc_executor_t  executor;
rclc_support_t   support;
rcl_allocator_t  allocator;
rcl_node_t       node;

#define UROS_SERIAL Serial
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ return false; }}
#define EXECUTE_EVERY_N_MS(MS, X) do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis(); } \
    if (uxr_millis() - init > MS) { X; init = uxr_millis(); } \
} while (0)

enum states { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } uros_state;

// ==========================================
// 3. CALLBACKS
// ==========================================
void cmd_vel_callback(const void * msvin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msvin;
    portENTER_CRITICAL(&gMux);
    target_v     = msg->linear.x;
    target_omega = msg->angular.z;
    portEXIT_CRITICAL(&gMux);
}

// ==========================================
// 4. CONTROL LOOP (Core 1 — 1000 Hz)
// ==========================================
void ControlLoopTask(void * pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1);

    unsigned long prevMicros = micros();
    float filtered_v_L = 0.0f;
    float filtered_v_R = 0.0f;
    const float LPF_ALPHA = 0.05f;
    float local_total_dist = 0.0f;

    while (1) {
        unsigned long currentMicros = micros();
        double dt = (currentMicros - prevMicros) / 1000000.0;
        if (dt <= 0.0) dt = 0.001;
        prevMicros = currentMicros;

        float tv, tom;
        portENTER_CRITICAL(&gMux);
        tv  = target_v;
        tom = target_omega;
        portEXIT_CRITICAL(&gMux);

        // Inverse Kinematics (Target v,w -> Target vL, vR)
        float left_target_ms  = tv - (tom * TRACK_WIDTH / 2.0f);
        float right_target_ms = tv + (tom * TRACK_WIDTH / 2.0f);

        // Forward Kinematics (Encoder -> Measured vL, vR)
        long delta_L = Encoder_GetDelta(&encLeft);
        long delta_R = Encoder_GetDelta(&encRight);
        float stepL = delta_L * DIST_PER_TICK;
        float stepR = delta_R * DIST_PER_TICK;
        
        local_total_dist += (stepL + stepR) / 2.0f;

        float raw_v_L = stepL / dt;
        float raw_v_R = stepR / dt;
        filtered_v_L = (LPF_ALPHA * raw_v_L) + ((1.0f - LPF_ALPHA) * filtered_v_L);
        filtered_v_R = (LPF_ALPHA * raw_v_R) + ((1.0f - LPF_ALPHA) * filtered_v_R);

        // อัปเดตข้อมูลให้ Core 0
        portENTER_CRITICAL(&gMux);
        g_vel_left   = filtered_v_L;
        g_vel_right  = filtered_v_R;
        g_totalDist  = local_total_dist;
        portEXIT_CRITICAL(&gMux);

        // PID Control
        int pwm_L = (int)PIDMotorL.compute(left_target_ms, filtered_v_L);
        int pwm_R = (int)PIDMotorR.compute(right_target_ms, filtered_v_R);

        if (abs(left_target_ms) < 0.001f && abs(right_target_ms) < 0.001f && abs(filtered_v_L) < 0.01f) {
            BTS7960_Stop(&motor_L);
            BTS7960_Stop(&motor_R);
        } else {
            BTS7960_SetSpeed(&motor_L, pwm_L);
            BTS7960_SetSpeed(&motor_R, pwm_R);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ==========================================
// 5. PUBLISH HELPER
// ==========================================
void publishDriveStatus() {
    float vl, vr, td;
    portENTER_CRITICAL(&gMux);
    vl = g_vel_left;
    vr = g_vel_right;
    td = g_totalDist;
    portEXIT_CRITICAL(&gMux);

    // Forward Kinematics เพื่อหาค่าความเร็วรวมของหุ่นยนต์
    float measured_v = (vr + vl) / 2.0f;
    float measured_omega = (vr - vl) / TRACK_WIDTH;

    msg_drive.x = measured_v;      // ความเร็วเส้นตรงปัจจุบัน (m/s)
    msg_drive.y = measured_omega;  // ความเร็วเชิงมุมปัจจุบัน (rad/s)
    msg_drive.z = td;              // ระยะทางสะสมรวม (m)
    
    rcl_publish(&drive_pub, &msg_drive, NULL);
}

// ==========================================
// 6. MICRO-ROS ENTITIES & SETUP
// ==========================================
bool createEntities() {
    allocator = rcl_get_default_allocator();
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, 22));
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    RCCHECK(rclc_node_init_default(&node, "drive_node", "", &support));

    RCCHECK(rclc_subscription_init_default(&cmd_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
    RCCHECK(rclc_publisher_init_best_effort(&drive_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "/drive_status"));

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_sub, &msg_cmd, &cmd_vel_callback, ON_NEW_DATA));
    return true;
}

void destroyEntities() {
    rcl_subscription_fini(&cmd_sub, &node);
    rcl_publisher_fini(&drive_pub, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

void setup() {
    UROS_SERIAL.begin(921600);
    BTS7960_Init(&motor_L);
    BTS7960_Init(&motor_R);
    Encoder_Init(&encLeft, 19, 18);
    Encoder_Init(&encRight, 13, 14);
    set_microros_serial_transports(UROS_SERIAL);
    uros_state = WAITING_AGENT;
    xTaskCreatePinnedToCore(ControlLoopTask, "Control", 8192, NULL, 10, NULL, 1);
}

void loop() {
    switch (uros_state) {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(500, uros_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            uros_state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (uros_state == WAITING_AGENT) destroyEntities();
            break;
        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(200, uros_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if (uros_state == AGENT_CONNECTED) {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
                EXECUTE_EVERY_N_MS(20, publishDriveStatus()); // ส่งข้อมูล 50Hz
            }
            break;
        case AGENT_DISCONNECTED:
            destroyEntities();
            uros_state = WAITING_AGENT;
            break;
    }
}