#include "AnglePid.h"
#include "IncrementalPid.h"
#include "Pid.h"
#include "constants.hpp"
#include "localization.hpp"
#include "nnct/interfaces/interfaces.hpp"
#include "swerve_drive.hpp"
#include "trapezoid.h"
#include "utility.h"
#include <Arduino.h>
#include <peer_link.h>

using namespace nnct::interfaces;

IncrementalEncoder enc_1(ENCODER_A_1, ENCODER_B_1);
IncrementalEncoder enc_2(ENCODER_A_2, ENCODER_B_2);
IncrementalEncoder enc_3(ENCODER_A_3, ENCODER_B_3);

Odometry odometry(enc_1, enc_2, enc_3);

constexpr uint8_t WIFI_CHANNEL          = 14;
const peer_id_t   FROM_PEER_ID          = 0x11;
const peer_id_t   TO_PEER_ID            = 0x12;
constexpr uint8_t POSITION_MESSAGE_TYPE = 0x01;

int16_t target_x   = 0;
int16_t target_y   = 0;
int16_t target_deg = 0;

unsigned long last = micros();

Position_deg now_pos_deg;
Position_deg target_pos;

volatile uint32_t last_target_received_ms = 0;

void peer_link_recv_cb(const peer_id_t peer_id, const std::vector<struct Message>& messages) {

    for (const Message& message : messages) {
        if (message.type != POSITION_MESSAGE_TYPE) {
            continue;
        }

        if (message.data.size() < 6) {
            Serial.println("invalid target data");
            continue;
        }

        target_pos.x            = (double)readInt16(message.data, 0);
        target_pos.y            = (double)readInt16(message.data, 2);
        target_pos.deg          = (double)readInt16(message.data, 4);
        last_target_received_ms = millis();
    }
}

TaskHandle_t control_loop_task_handle;

Motor          steering_motor_1(STEERING_MOTOR_DIR_1, STEERING_MOTOR_PWM_1);
AnglePID       steering_pid_1(STEERING_PID_PARAM.p_gain, STEERING_PID_PARAM.i_gain, STEERING_PID_PARAM.d_gain,
                              -STEER_MOTOR_POWER_LIMIT, STEER_MOTOR_POWER_LIMIT, -STEER_INTEGRAL_LIMIT, STEER_INTEGRAL_LIMIT, RANGE);
nnct::Amt223dv steering_encoder_1(STEERING_ABS_ENCODER_CS_1);
Steering       steering_1(&steering_motor_1, &steering_encoder_1, &steering_pid_1, OFFSET_DEG_1, 1);

Motor          steering_motor_2(STEERING_MOTOR_DIR_2, STEERING_MOTOR_PWM_2);
nnct::Amt223dv steering_encoder_2(STEERING_ABS_ENCODER_CS_2);
AnglePID       steering_pid_2(STEERING_PID_PARAM.p_gain, STEERING_PID_PARAM.i_gain, STEERING_PID_PARAM.d_gain,
                              -STEER_MOTOR_POWER_LIMIT, STEER_MOTOR_POWER_LIMIT, -STEER_INTEGRAL_LIMIT, STEER_INTEGRAL_LIMIT, RANGE);
Steering       steering_2(&steering_motor_2, &steering_encoder_2, &steering_pid_2, OFFSET_DEG_2, 2);

Motor          steering_motor_3(STEERING_MOTOR_DIR_3, STEERING_MOTOR_PWM_3);
nnct::Amt223dv steering_encoder_3(STEERING_ABS_ENCODER_CS_3);
AnglePID       steering_pid_3(STEERING_PID_PARAM.p_gain, STEERING_PID_PARAM.i_gain, STEERING_PID_PARAM.d_gain,
                              -STEER_MOTOR_POWER_LIMIT, STEER_MOTOR_POWER_LIMIT, -STEER_INTEGRAL_LIMIT, STEER_INTEGRAL_LIMIT, RANGE);
Steering       steering_3(&steering_motor_3, &steering_encoder_3, &steering_pid_3, OFFSET_DEG_3, 3);

RobomasMotor drive_motor_1(DRIVE_MOTOR_ID_1);
RobomasMotor drive_motor_2(DRIVE_MOTOR_ID_2);
RobomasMotor drive_motor_3(DRIVE_MOTOR_ID_3);

target_vec_data target_data;
CAN             can(CAN_RX_PIN, CAN_TX_PIN);

PID drive_pid_1(DRIVE_PID_PARAM.p_gain, DRIVE_PID_PARAM.i_gain, DRIVE_PID_PARAM.d_gain, -DRIVE_MOTOR_POWER_LIMIT,
                DRIVE_MOTOR_POWER_LIMIT, -DRIVE_INTEGRAL_LIMIT, DRIVE_INTEGRAL_LIMIT);
PID drive_pid_2(DRIVE_PID_PARAM.p_gain, DRIVE_PID_PARAM.i_gain, DRIVE_PID_PARAM.d_gain, -DRIVE_MOTOR_POWER_LIMIT,
                DRIVE_MOTOR_POWER_LIMIT, -DRIVE_INTEGRAL_LIMIT, DRIVE_INTEGRAL_LIMIT);
PID drive_pid_3(DRIVE_PID_PARAM.p_gain, DRIVE_PID_PARAM.i_gain, DRIVE_PID_PARAM.d_gain, -DRIVE_MOTOR_POWER_LIMIT,
                DRIVE_MOTOR_POWER_LIMIT, -DRIVE_INTEGRAL_LIMIT, DRIVE_INTEGRAL_LIMIT);

// 後ろのはID printのときなどに使用
Drive drive_1(&drive_motor_1, &drive_pid_1, 1);
Drive drive_2(&drive_motor_2, &drive_pid_2, 2);
Drive drive_3(&drive_motor_3, &drive_pid_3, 3);

SwerveDrive swerve_drive_1(&drive_1, &steering_1, 1);
SwerveDrive swerve_drive_2(&drive_2, &steering_2, 2);
SwerveDrive swerve_drive_3(&drive_3, &steering_3, 3);

SwerveDrive*     swerve_drives[]    = {&swerve_drive_1, &swerve_drive_2, &swerve_drive_3};
constexpr size_t NUM_SWERVE_MODULES = 3;

double x_ref_speed   = 0.0;
double y_ref_speed   = 0.0;
double deg_ref_speed = 0.0;

void drive_pid_reset() {
    drive_pid_1.reset();
    drive_pid_2.reset();
    drive_pid_3.reset();
}

void stop_swerve_drives() {
    for (size_t i = 0; i < NUM_SWERVE_MODULES; ++i) {
        swerve_drives[i]->stop_drive();
    }
    steering_1.set_target(90);
    steering_2.set_target(210);
    steering_3.set_target(330);

    drive_pid_reset();
}

void set_robot_velocity(double vx_mm_s, double vy_mm_s, double omega_deg_s) {

    // 小さい速度指令を0にする
    if (hypot(vx_mm_s, vy_mm_s) < TRANSLATION_DEADZONE_MM_S) {
        vx_mm_s = 0.0;
        vy_mm_s = 0.0;
    }

    if (fabs(omega_deg_s) < ROTATION_DEADZONE_DEG_S) {
        omega_deg_s = 0.0;
    }

    const double omega_rad_s = omega_deg_s * M_PI / 180.0;

    double        wheel_speed[NUM_SWERVE_MODULES];
    static double wheel_angle[NUM_SWERVE_MODULES];
    double        max_speed = 0.0;

    for (size_t i = 0; i < NUM_SWERVE_MODULES; ++i) {
        const double x = MODULE_POSITIONS[i].x_mm;
        const double y = MODULE_POSITIONS[i].y_mm;

        // 各車輪位置での速度ベクトル
        const double wheel_vx = vx_mm_s - omega_rad_s * y;
        const double wheel_vy = vy_mm_s + omega_rad_s * x;

        wheel_speed[i] = hypot(wheel_vx, wheel_vy);

        if (!(vx_mm_s == 0.0 && vy_mm_s == 0.0 && omega_deg_s == 0.0)) {
            wheel_angle[i] = atan2(wheel_vy, wheel_vx) * 180.0 / M_PI;
        }

        // 車輪で一番早いものを探す
        if (wheel_speed[i] > max_speed) {
            max_speed = wheel_speed[i];
        }
    }

    // 最大速度を超えないように全輪を同じ比率でスケーリング
    const double scale = max_speed > WHEEL_MAX_SPEED_MM_S ? WHEEL_MAX_SPEED_MM_S / max_speed : 1.0;

    for (size_t i = 0; i < NUM_SWERVE_MODULES; ++i) {
        const double speed = wheel_speed[i] * scale;

        swerve_drives[i]->set_target_mm_s(wheel_angle[i], speed);
    }
}

void handle_controller_input_deg_vec(int x_vec, int y_vec, uint8_t drive_power) {
    double magnitude = hypot((double)x_vec, (double)y_vec);

    if (magnitude <= MAGNITUDE_DEADZONE) {
        stop_swerve_drives();
        return;
    }

    double degree            = atan2((double)y_vec, (double)x_vec) * 180.0 / M_PI;
    double drive_target_duty = drive_power;

    for (size_t i = 0; i < NUM_SWERVE_MODULES; i++) {
        swerve_drives[i]->set_target_duty(degree, drive_target_duty);
    }
}

void handle_controller_input(int x_vec, int y_vec, uint8_t l2_value, uint8_t r2_value) {
    constexpr double MAX_TRANSLATION_SPEED_MM_S = WHEEL_MAX_SPEED_MM_S;
    constexpr double MAX_ROTATION_SPEED_DEG_S   = MAX_ROTATE_SPEED_DEG_S;

    constexpr double STICK_MAX   = 127.0;
    constexpr double TRIGGER_MAX = 255.0;

    // 右スティック：並進
    const double vx = static_cast<double>(x_vec) / STICK_MAX * MAX_TRANSLATION_SPEED_MM_S;
    const double vy = static_cast<double>(y_vec) / STICK_MAX * MAX_TRANSLATION_SPEED_MM_S;

    // R2 - L2：旋回
    // 符号が逆なら l2_value と r2_value を入れ替える
    const double omega =
        (static_cast<double>(l2_value) - static_cast<double>(r2_value)) / TRIGGER_MAX * MAX_ROTATION_SPEED_DEG_S;

    set_robot_velocity(vx, vy, omega);
}

void update_swerve_drives() {
    for (size_t i = 0; i < NUM_SWERVE_MODULES; i++) {
        swerve_drives[i]->update(CONTROL_CYCLE_S);
    }
}

void can_send() {
    // ドライブモータの指令値を CAN で送信
    if (!can.send(&drive_motor_1, &drive_motor_2, 0, &drive_motor_3)) {
        Serial.println("Drive CAN send failed");
    }
}

void control_loop_task(void* args) {
    TickType_t wake_time = xTaskGetTickCount();

    while (true) {
        update_swerve_drives();

        can_send();

        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(CONTROL_CYCLE_MS));
    }
}

bool initialize_swerve_drives() {
    for (size_t i = 0; i < NUM_SWERVE_MODULES; i++) {
        if (!swerve_drives[i]->init()) {
            return false;
        }
    }

    return true;
}

void setup() {
    Serial.begin(115200);
    odometry.begin();
    SPI.begin();

    if (!can.begin()) {
        Serial.println("CAN begin failed");
        while (true) {
        }
    }
    can.setOdometryData(&target_data);
    nnct::Amt223dv::beginStatic(&SPI, ABS_ENCODER_READ_PERIOD_MS);

    if (!initialize_swerve_drives()) {
        while (true) {
        }
    }
    peer_link_task_init(WIFI_CHANNEL, FROM_PEER_ID);

    xTaskCreate(control_loop_task, "ControlLoopTask", CONTROL_LOOP_TASK_STACK_SIZE, NULL, CONTROL_LOOP_TASK_PRIORITY,
                &control_loop_task_handle);
}

void loop() {
    unsigned long now = micros();

    double dt = (now - last) * 1.e-6;
    last      = now;

    if (millis() - last_target_received_ms > 500) {
        x_ref_speed   = 0.0;
        y_ref_speed   = 0.0;
        deg_ref_speed = 0.0;
        stop_swerve_drives();
        return;
    }

    odometry.update(dt);

    now_pos_deg = odometry.get_position_deg();

    if (peer_link_is_peer_exist(TO_PEER_ID)) {
        Message message;
        message.type = POSITION_MESSAGE_TYPE;
        message.data = positionToPayload(now_pos_deg);

        std::vector<Message> messages{message};

        const esp_err_t result = peer_link_send(TO_PEER_ID, messages);

        if (result != ESP_OK) {
            Serial.printf("send error: %d\n", result);
        }
    }

    x_ref_speed =
        updateVelocityProfile(target_pos.x, now_pos_deg.x, x_ref_speed, SHIFT_MAX_SPEED_MM_S, MAX_SHIFT_ACCELERATION, dt);

    y_ref_speed =
        updateVelocityProfile(target_pos.y, now_pos_deg.y, y_ref_speed, SHIFT_MAX_SPEED_MM_S, MAX_SHIFT_ACCELERATION, dt);

    deg_ref_speed = updateAngleVelocityProfile(target_pos.deg, now_pos_deg.deg, deg_ref_speed, MAX_ROTATE_SPEED_DEG_S,
                                               MAX_ROTATE_ACCELERATION, dt);

    Position_deg now_velocity = odometry.get_velocity_deg();

    int16_t x_vec   = x_ref_speed;
    int16_t y_vec   = y_ref_speed;
    int16_t deg_vec = deg_ref_speed;

    // int     rx     = PS4.RStickX();
    // int     ry     = PS4.RStickY();
    // uint8_t r2_val = PS4.R2Value();
    // uint8_t l2_val = PS4.L2Value();

    // handle_controller_input_deg_vec(rx, ry, r2_val);
    // handle_controller_input(rx, ry, l2_val, r2_val);
    set_robot_velocity(x_vec, y_vec, deg_vec);
    // set_robot_velocity(1000, 0, 0);

    // static uint32_t last_print_time = 0;
    // const uint32_t  now             = millis();

    // if (now - last_print_time >= 1000) {
    //     last_print_time = now;
    //     Serial.printf("x:%d y:%d deg:%d receive:%d\r\n", target_data.x_mm_s, target_data.y_mm_s, target_data.theta_deg_s,
    //                   target_data.received);
    // }

    delay(LOOP_DELAY_MS);
}