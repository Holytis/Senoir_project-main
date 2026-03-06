#include <Arduino.h>
#include <WiFi.h>
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
BTS7960_t motor_L = { 23, 22, 255 }; 
BTS7960_t motor_R = { 26, 25, 255 };
Encoder_t encLeft, encRight;

// แก้ไข: ประกาศ Object PID ให้ตรงกับใน main.cpp เดิม
PID PIDMotorL(-255, 255, 250.0, 0.0, 0.0); 
PID PIDMotorR(-255, 255, 250.0, 0.0, 0.0);

const float WHEEL_PPR = 16.0 * 99.5; 
const float WHEEL_RADIUS = 0.04;
const float TRACK_WIDTH = 0.37;
volatile float totalDistance = 0;

// micro-ROS Variables
rcl_subscription_t cmd_sub;
rcl_publisher_t drive_pub;
geometry_msgs__msg__Twist msg_cmd;
geometry_msgs__msg__Vector3 msg_drive;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

char ssid[] = "TIIS";
char pass[] = "08092003";
IPAddress agent_ip(172, 20, 10, 4);
size_t agent_port = 8888;

volatile float target_v = 0;
volatile float target_omega = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ return false; }}
#define EXECUTE_EVERY_N_MS(MS, X) do { static volatile int64_t init = -1; if (init == -1) { init = uxr_millis(); } if (uxr_millis() - init > MS) { X; init = uxr_millis(); } } while (0)

enum states { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } uros_state;

// ==========================================
// 2. CALLBACKS & HELPERS
// ==========================================

void cmd_vel_callback(const void * msvin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msvin;
    target_v = msg->linear.x;
    target_omega = msg->angular.z;
}

// ==========================================
// 3. CONTROL LOOP (Core 1 - 1000Hz)
// ==========================================
void ControlLoopTask(void * pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1); 
    
    unsigned long prevMicros = micros();
    float filtered_v_L = 0;
    float filtered_v_R = 0;
    const float LPF_ALPHA = 0.05;

    while (1) {
        unsigned long currentMicros = micros();
        double dt = (currentMicros - prevMicros) / 1000000.0;
        if (dt <= 0.0) dt = 0.001;
        prevMicros = currentMicros;

        // Kinematics
        float left_target_ms = target_v - (target_omega * TRACK_WIDTH / 2.0);
        float right_target_ms = target_v + (target_omega * TRACK_WIDTH / 2.0);

        // แก้ไข: ใช้ฟังก์ชันจาก Library ของคุณ (อ้างอิงจาก main.cpp เดิม)
        long delta_L = Encoder_GetDelta(&encLeft);
        long delta_R = Encoder_GetDelta(&encRight);
        
        float dist_per_tick = (2.0 * PI * WHEEL_RADIUS) / WHEEL_PPR;
        float stepL = delta_L * dist_per_tick;
        float stepR = delta_R * dist_per_tick;
        totalDistance += (stepL + stepR) / 2.0;

        float raw_v_L = stepL / dt;
        float raw_v_R = stepR / dt;
        filtered_v_L = (LPF_ALPHA * raw_v_L) + ((1.0f - LPF_ALPHA) * filtered_v_L);
        filtered_v_R = (LPF_ALPHA * raw_v_R) + ((1.0f - LPF_ALPHA) * filtered_v_R);

        // แก้ไข: เรียกใช้ PID และ Motor Driver ให้ตรงตาม Class/Struct ของคุณ
        int final_L = (int)PIDMotorL.compute(left_target_ms, filtered_v_L);
        int final_R = (int)PIDMotorR.compute(right_target_ms, filtered_v_R);

        if (abs(left_target_ms) < 0.001 && abs(right_target_ms) < 0.001 && abs(filtered_v_L) < 0.01) {
            BTS7960_Stop(&motor_L);
            BTS7960_Stop(&motor_R);
        } else {
            BTS7960_SetSpeed(&motor_L, final_L);
            BTS7960_SetSpeed(&motor_R, final_R);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ==========================================
// 4. MICRO-ROS ENTITIES
// ==========================================

bool createEntities() {
    allocator = rcl_get_default_allocator();
    
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, 22));

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    RCCHECK(rclc_node_init_default(&node, "drive_node", "", &support));

    RCCHECK(rclc_subscription_init_default(
        &cmd_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

    RCCHECK(rclc_publisher_init_best_effort(
        &drive_pub, &node,
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

// ==========================================
// 5. SETUP & LOOP
// ==========================================

void setup() {
    Serial.begin(115200);
    
    BTS7960_Init(&motor_L);
    BTS7960_Init(&motor_R);
    Encoder_Init(&encLeft, 19, 18); 
    Encoder_Init(&encRight, 13, 14);

    set_microros_wifi_transports(ssid, pass, agent_ip, agent_port);
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
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
                
                EXECUTE_EVERY_N_MS(50, {
                    float dist_per_tick = (2.0 * PI * WHEEL_RADIUS) / WHEEL_PPR;
                    msg_drive.x = (float)encLeft.count * dist_per_tick;
                    msg_drive.y = (float)encRight.count * dist_per_tick;
                    msg_drive.z = totalDistance;
                    rcl_publish(&drive_pub, &msg_drive, NULL);
                });
            }
            break;
        case AGENT_DISCONNECTED:
            destroyEntities();
            uros_state = WAITING_AGENT;
            break;
    }
}