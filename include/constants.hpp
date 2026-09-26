#pragma once
#include <Arduino.h>
#include <array>
// units: mm, deg, rad, mm/s

struct PidParam {
        double p_gain;
        double i_gain;
        double d_gain;
};
struct Position_rad {
        double x;
        double y;
        double rad;

        Position_rad() : x(0.0), y(0.0), rad(0.0) {}
        Position_rad(double x_, double y_, double yaw_) : x(x_), y(y_), rad(yaw_) {}
};

struct x_y_theta_deg_s {
        double x;
        double y;
        double deg;

        x_y_theta_deg_s() : x(0.0), y(0.0), deg(0.0) {}
        x_y_theta_deg_s(double x_, double y_, double yaw_) : x(x_), y(y_), deg(yaw_) {}
};

struct ModulePosition {
        double x_mm;
        double y_mm;
};

struct TargetCommand {
        x_y_theta_deg_s position;
        uint32_t        received_ms;
};

struct Position {
        int16_t x;
        int16_t y;
        int16_t deg;
};

using pin_t = uint8_t;
using ch_t  = uint8_t;

// ステアリング1
const pin_t  STEERING_MOTOR_DIR_1      = 39;
const pin_t  STEERING_MOTOR_PWM_1      = 40;
const ch_t   STEERING_MOTOR_CH_1       = 0;
const id_t   DRIVE_MOTOR_ID_1          = 0x01; // TODO:変更
const pin_t  STEERING_ABS_ENCODER_CS_1 = 8;
const pin_t  STEERING_LIMIT_SW_1       = 4;
const double OFFSET_DEG_1              = 294.95;

// ステアリング2
const pin_t  STEERING_MOTOR_DIR_2      = 41;
const pin_t  STEERING_MOTOR_PWM_2      = 42;
const ch_t   STEERING_MOTOR_CH_2       = 1;
const id_t   DRIVE_MOTOR_ID_2          = 0x02;
const pin_t  STEERING_ABS_ENCODER_CS_2 = 18;
const pin_t  STEERING_LIMIT_SW_2       = 5;
const double OFFSET_DEG_2              = 353.48;

// ステアリング3
const pin_t  STEERING_MOTOR_DIR_3      = 1;
const pin_t  STEERING_MOTOR_PWM_3      = 2;
const ch_t   STEERING_MOTOR_CH_3       = 2;
const id_t   DRIVE_MOTOR_ID_3          = 0x04;
const pin_t  STEERING_ABS_ENCODER_CS_3 = 17;
const pin_t  STEERING_LIMIT_SW_3       = 6;
const double OFFSET_DEG_3              = 42.75;

const pin_t       CAN_CS_PIN  = 9; // 実際の配線に合わせて変更
constexpr uint8_t CAN_INT_PIN = -1;

constexpr uint8_t SPI_SCK_PIN  = 7;
constexpr uint8_t SPI_MISO_PIN = 15;
constexpr uint8_t SPI_MOSI_PIN = 16;

const size_t NUM_SWERVE_MODULES = 3;

// ステア制御パラメータ
const int16_t STEER_MOTOR_POWER_LIMIT = 255.;
const int16_t STEER_INTEGRAL_LIMIT    = 10;
const int16_t RANGE                   = 360;

// ドライブ制御パラメータ
const int16_t DRIVE_MOTOR_POWER_LIMIT = 255.;
const int16_t DRIVE_INTEGRAL_LIMIT    = 10.;

// コントローラ
const double MAGNITUDE_DEADZONE = 15.0;
const double CONTROL_CYCLE_MS   = 2.0;
const double CONTROL_CYCLE_S    = CONTROL_CYCLE_MS / 1000.0;

constexpr double TRANSLATION_DEADZONE_MM_S = 0.0;
constexpr double ROTATION_DEADZONE_DEG_S   = 5.0;

// PIDパラメータ
const struct PidParam STEERING_PID_PARAM = {.p_gain = 25., .i_gain = 0.0, .d_gain = 0.0};
const struct PidParam DRIVE_PID_PARAM    = {.p_gain = 0.5, .i_gain = 0.0, .d_gain = 0.0};

// FreeRTOS
constexpr uint32_t CONTROL_LOOP_TASK_STACK_SIZE = 8192;
constexpr uint8_t  CONTROL_LOOP_TASK_PRIORITY   = 10;
constexpr uint32_t LOOP_DELAY_MS                = 10;

const uint8_t ABS_ENCODER_READ_PERIOD_MS = 2;

constexpr double DRIVE_RADIUS = 50;

// constexpr double GEAR_RATIO = 1.0;
static constexpr double DRIVE_GEAR_RATIO = 19.0 / 1.0; // モーター:ホイールの速度比
// static constexpr double DRIVE_GEAR_RATIO = 1.0; // モーター:ホイールの速度比

static constexpr double STEER_GEAR_RATIO_MOTOR_TO_STEER = 65.0 / 27.0;

// オドメトリ

const pin_t ENCODER_A_1 = 11;
const pin_t ENCODER_B_1 = 12;

const pin_t ENCODER_A_2 = 47;
const pin_t ENCODER_B_2 = 21;

const pin_t ENCODER_A_3 = 38;
const pin_t ENCODER_B_3 = 48;

const pin_t can_tx = 5;
const pin_t can_rx = 4;

constexpr double OD_RADIUS = 30.0;

constexpr double ROBOT_TO_ODO_RADIUS = 250.0;

constexpr int8_t   ENCODER_SIGN_1     = 1;
constexpr int8_t   ENCODER_SIGN_2     = 1;
constexpr int8_t   ENCODER_SIGN_3     = 1;
constexpr uint32_t ENCODER_RESOLUTION = 8192;

constexpr double GEAR_RATIO = 1.0;

constexpr double COUNTS_PER_MM = (ENCODER_RESOLUTION * GEAR_RATIO) / (M_PI * OD_RADIUS * 2.0);

const double SPEED_EPS = 1e-3; // 1 mm/s 程度のノイズは角度更新を行わない

constexpr double ANGLE_DEAD_ZONE_DEG = 0.1;

// TODO:実機の車輪位置に合わせて変更してください
// x: 前後方向、y: 左右方向
const ModulePosition MODULE_POSITIONS[NUM_SWERVE_MODULES] = {
    {0,        -338.623}, // module 1
    {394.405,  344.506 }, // module 2
    {-394.405, 344.506 }, // module 3
};

constexpr double ODO_CENTER_OFFSET_X = 0.0;
constexpr double ODO_CENTER_OFFSET_Y = 116.796;

constexpr int16_t STEER_TURN_LIMIT  = 2;
constexpr double  STEER_RETURN_DUTY = 255.0;

// 最低速度
constexpr double MIN_TRANSLATION_SPEED_MM_S = 300.0;
constexpr double MIN_STOP_SPEED_MM_S        = 70.0;

// ホイール物理最大速度（実機に合わせて調整）  482rpmなので2500mm/s
constexpr double MAX_SHIFT_SPEED_MM_S = 2400.0;

constexpr double MAX_ROTATE_SPEED_DEG_S = 500.0;

constexpr double MAX_SHIFT_ACCELERATION  = 2000.;
constexpr double MAX_SHIFT_DECELERATION  = 1500.;
constexpr double MAX_ROTATE_ACCELERATION = 300.;

// 停止判定(速度)
constexpr double POSITION_STOP_SPEED_MM_S = 40.0;
constexpr double ANGLE_STOP_SPEED_DEG_S   = 5.0;
// 許容誤差
constexpr double POSITION_TOLERANCE_MM = 3.0;
constexpr double ANGLE_TOLERANCE_DEG   = 1.0;

// constexpr uint8_t GAMEPAD_MESSAGE_TYPE  = static_cast<uint8_t>(MessageType::Gamepad);
// constexpr uint8_t POSITION_MESSAGE_TYPE = static_cast<uint8_t>(MessageType::RobotState);

constexpr uint32_t GAMEPAD_TIMEOUT_MS = 250;
