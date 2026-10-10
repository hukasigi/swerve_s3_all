#include "AnglePid.h"
#include "IncrementalPid.h"
#include "Pid.h"
#include "constants.hpp"
#include "localization.hpp"
#include "message.h"
#include "nnct/interfaces/interfaces.hpp"
#include "nnct/interfaces/spi_mutex.hpp"
#include "peer_link.h"
#include "swerve_drive.h"
#include "trapezoid.h"
#include "utility.h"
#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>

using namespace nnct::interfaces;

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

CAN can(CAN_CS_PIN, -1, &SPI);

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

SwerveDrive* swerve_drives[] = {&swerve_drive_1, &swerve_drive_2, &swerve_drive_3};

IncrementalEncoder enc_1(ENCODER_A_1, ENCODER_B_1);
IncrementalEncoder enc_2(ENCODER_A_2, ENCODER_B_2);
IncrementalEncoder enc_3(ENCODER_A_3, ENCODER_B_3);

Odometry odometry(enc_1, enc_2, enc_3);

TabletData_Pos  now_pos;
TabletData_Pos  target_pos;
x_y_theta_deg_s ref_speed;

StateData now_state;

x_y_theta_deg_s now_vel_deg;

GamepadData received_gamepad_data{};
PS4Data     received_ps4_data{};

BeltData   can_belt;
RollerData can_roller;

volatile bool belt_order_received   = false;
volatile bool roller_order_received = false;

volatile bool     gamepad_data_received   = false;
volatile uint32_t last_gamepad_receive_ms = 0;

portMUX_TYPE gamepad_data_mux = portMUX_INITIALIZER_UNLOCKED;

bool is_stopped = false;

static double filtered_vx   = 0.0;
static double filtered_vy   = 0.0;
static double filtered_vdeg = 0.0;

constexpr double VELOCITY_FILTER_ALPHA = 0.2;

bool translation_decelerating = false;

bool gamepad_use = false;

void drive_pid_reset() {
    drive_pid_1.reset();
    drive_pid_2.reset();
    drive_pid_3.reset();
}

void stop_swerve_drives() {
    for (size_t i = 0; i < NUM_SWERVE_MODULES; ++i) {
        swerve_drives[i]->stop_drive();
    }
    drive_pid_reset();
}

void set_outward_steering_targets() {
    for (size_t i = 0; i < NUM_SWERVE_MODULES; ++i) {
        const double outward_angle = atan2(MODULE_POSITIONS[i].y_mm, MODULE_POSITIONS[i].x_mm) * 180.0 / M_PI;
        if (i != 0) {
            if (swerve_drives[i]->needsTwistRelease()) {
                swerve_drives[i]->releaseTwist();
            }
        }
        swerve_drives[i]->set_deg(outward_angle);
    }
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

    if (vx_mm_s == 0.0 && vy_mm_s == 0.0 && omega_deg_s == 0.0) {

        if (!is_stopped) {
            stop_swerve_drives();
            set_outward_steering_targets();
            is_stopped = true;
        }

        return;
    }

    is_stopped = false;

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
    const double scale = max_speed > MAX_SHIFT_SPEED_MM_S ? MAX_SHIFT_SPEED_MM_S / max_speed : 1.0;

    for (size_t i = 0; i < NUM_SWERVE_MODULES; ++i) {
        const double speed = wheel_speed[i] * scale;

        swerve_drives[i]->set_target_mm_s(wheel_angle[i], speed);
        // Serial.printf("%f wheel_angle%d ", wheel_angle[i], i);
    }
    // Serial.println();
}

void handle_controller_input(int x_vec, int y_vec, int8_t l_stick_x) {
    constexpr double STICK_MAX = 127.0;

    const double magnitude = hypot(static_cast<double>(x_vec), static_cast<double>(y_vec));

    if (magnitude <= MAGNITUDE_DEADZONE) {
        x_vec = 0;
        y_vec = 0;
    } else {
        // デッドゾーン後を0～最大値へ再マッピング
        const double remapped = std::min((magnitude - MAGNITUDE_DEADZONE) / (STICK_MAX - MAGNITUDE_DEADZONE), 1.0);

        const double scale = remapped / magnitude;

        x_vec = static_cast<int>(x_vec * scale);
        y_vec = static_cast<int>(y_vec * scale);
    }

    const double body_vx = static_cast<double>(x_vec) / STICK_MAX * STICK_SHIFT_SPEED_MM_S;
    const double body_vy = -static_cast<double>(y_vec) / STICK_MAX * STICK_SHIFT_SPEED_MM_S;

    const double omega = static_cast<double>(-l_stick_x) / STICK_MAX * MAX_ROTATE_SPEED_DEG_S;

    set_robot_velocity(body_vx, body_vy, omega);
}

void sendCommand(uint32_t id) {
    CanFrame frame = {};

    frame.identifier       = id;
    frame.extd             = 0;
    frame.data_length_code = 0;

    ESP32Can.writeFrame(&frame);
}
void sendCommand(uint32_t id, uint8_t command) {
    CanFrame frame = {};

    frame.identifier       = id;
    frame.extd             = 0;
    frame.data_length_code = 1;

    frame.data[0] = command;

    ESP32Can.writeFrame(frame, 10);
}

void sendCommand(uint32_t id, uint16_t command) {
    CanFrame frame = {};

    frame.identifier       = id;
    frame.extd             = 0;
    frame.data_length_code = 2;

    Serial.println(command);

    // Big Endian
    frame.data[0] = static_cast<uint8_t>((command >> 8) & 0xFF); // 上位バイト
    frame.data[1] = static_cast<uint8_t>(command & 0xFF);        // 下位バイト

    ESP32Can.writeFrame(frame, 10);
}

void gamepad(GamepadData gamepad, PS4Data ps4_data) {
    static uint16_t previous_actions = 0;
    static bool     roller_started   = false;

    constexpr uint16_t ACTION_SQUARE        = 1 << 0;
    constexpr uint16_t ACTION_CROSS         = 1 << 1;
    constexpr uint16_t ACTION_TRIANGLE      = 1 << 2;
    constexpr uint16_t ACTION_TOUCHPAD      = 1 << 3;
    constexpr uint16_t ACTION_DPAD_UP       = 1 << 4;
    constexpr uint16_t ACTION_DPAD_DOWN     = 1 << 5;
    constexpr uint16_t ACTION_DPAD_LEFT     = 1 << 6;
    constexpr uint16_t ACTION_START         = 1 << 7;
    constexpr uint16_t ACTION_SELECT        = 1 << 8;
    constexpr uint16_t ACTION_ROLLER_LAUNCH = 1 << 9;

    constexpr uint16_t ELEVATION_ACTIONS = ACTION_DPAD_UP | ACTION_DPAD_DOWN;

    constexpr uint16_t START_SELECT_MASK = ACTION_START | ACTION_SELECT;

    const bool enable = gamepad.buttons.bits.shoulder_left && gamepad.buttons.bits.shoulder_right;

    uint16_t actions = 0;

    // 通常操作：L1 + R1を押している間だけ有効
    if (enable) {
        if (gamepad.buttons.bits.west) {
            actions |= ACTION_SQUARE;
        }
        if (gamepad.buttons.bits.south) {
            actions |= ACTION_CROSS;
        }
        if (gamepad.buttons.bits.north) {
            actions |= ACTION_TRIANGLE;
        }
        if (ps4_data.buttons.bits.touchpad) {
            actions |= ACTION_TOUCHPAD;
        }
        if (gamepad.dpad == Dpad::Up) {
            actions |= ACTION_DPAD_UP;
        }
        if (gamepad.dpad == Dpad::Down) {
            actions |= ACTION_DPAD_DOWN;
        }
        if (gamepad.dpad == Dpad::Left) {
            actions |= ACTION_DPAD_LEFT;
        }
    }

    // START / SELECTはL1 + R1に関係なく取得
    if (gamepad.buttons.bits.start) {
        actions |= ACTION_START;
    }
    if (gamepad.buttons.bits.select) {
        actions |= ACTION_SELECT;
    }

    // STARTまたはSELECTを押しながらタッチパッド
    if ((actions & START_SELECT_MASK) != 0 && ps4_data.buttons.bits.touchpad) {
        actions |= ACTION_ROLLER_LAUNCH;
    }

    // 前回から今回への変化を計算
    const uint16_t pressed = actions & ~previous_actions;

    // 昇降操作：押下・離上・方向転換
    const uint16_t previous_elevation = previous_actions & ELEVATION_ACTIONS;

    const uint16_t current_elevation = actions & ELEVATION_ACTIONS;

    if (current_elevation != previous_elevation) {
        if (previous_elevation != 0) {
            sendCommand(CAN_CMD_ELEVATION_STOP);
        }

        if (current_elevation & ACTION_DPAD_UP) {
            sendCommand(CAN_CMD_ELEVATION_UP);
        } else if (current_elevation & ACTION_DPAD_DOWN) {
            sendCommand(CAN_CMD_ELEVATION_DOWN);
        }
    }

    // ベルト操作
    if (pressed & ACTION_SQUARE) {
        can_belt.load_chamber = true;
        belt_order_received   = true;
    }

    if (pressed & ACTION_CROSS) {
        can_belt.unload_mag = true;
        belt_order_received = true;
    }

    if (pressed & ACTION_TRIANGLE) {
        can_belt.load_mag   = true;
        belt_order_received = true;
    }

    if (pressed & ACTION_TOUCHPAD) {
        can_belt.launch     = true;
        can_belt.acc        = 0;
        belt_order_received = true;
    }

    if (pressed & ACTION_DPAD_LEFT) {
        can_belt.unload_chamber = true;
        belt_order_received     = true;
    }

    // START + SELECTの同時押しでローラーを切り替え
    const bool start_select_triggered =
        (actions & START_SELECT_MASK) == START_SELECT_MASK && (previous_actions & START_SELECT_MASK) != START_SELECT_MASK;

    if (start_select_triggered) {
        roller_started = !roller_started;

        if (roller_started) {
            can_roller.acc_start = true;
        } else {
            can_roller.stop = true;
        }

        roller_order_received = true;
    }

    // STARTまたはSELECTを押しながらタッチパッドでローラー発射
    if (pressed & ACTION_ROLLER_LAUNCH) {
        can_roller.launch     = true;
        roller_order_received = true;
    }

    // 前回の状態を最後に一度だけ更新
    previous_actions = actions;
}

void can_receive() {
    CanFrame frame;

    // 受信バッファにあるフレームをすべて処理
    while (ESP32Can.readFrame(frame, 0)) {
        // Serial.printf("CAN RX: ID=0x%03lX DLC=%d DATA=", static_cast<unsigned long>(frame.identifier),
        // frame.data_length_code);

        // for (uint8_t i = 0; i < frame.data_length_code && i < 8; ++i) {
        //     Serial.printf("%02X ", frame.data[i]);
        // }

        now_state.roller_reach = false;
        if (frame.identifier == 0x102) {
            now_state.roller_reach = true;
        }
    }
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

bool initialize_swerve_drives() {
    for (size_t i = 0; i < NUM_SWERVE_MODULES; i++) {
        if (!swerve_drives[i]->init()) {
            return false;
        }
    }

    return true;
}

void can_send_belt_data() {

    if (can_belt.load_chamber) {
        sendCommand(CAN_CMD_LOAD_CHAMBER);
        can_belt.load_chamber = false;
    }

    if (can_belt.unload_mag) {
        sendCommand(CAN_CMD_UNLOAD_MAG);
        can_belt.unload_mag = false;
    }

    if (can_belt.load_mag) {
        sendCommand(CAN_CMD_LOAD_MAG);
        can_belt.load_mag = false;
    }

    if (can_belt.unload_chamber) {
        sendCommand(CAN_CMD_UNLOAD_CHAMBER);
        can_belt.unload_chamber = false;
    }

    if (can_belt.launch) {
        sendCommand(CAN_CMD_LAUNCH_BELT, can_belt.acc);
        can_belt.launch = false;
    }

    if (can_belt.elevation_change) {
        sendCommand(CAN_CMD_LAUNCH_ELEVATION_BELT, can_belt.elevation_pos);
        can_belt.elevation_change = false;
    }
}
void can_send_roller_data() {

    if (can_roller.stop) {
        sendCommand(CAN_CMD_STOP_ROLLER);
        can_roller.stop = false;
    }
    if (can_roller.acc_start) {
        sendCommand(CAN_CMD_LAUNCH_START);
        can_roller.acc_start = false;
    }
    if (can_roller.launch) {
        sendCommand(CAN_CMD_LAUNCH_ROLLER);
        can_roller.launch = false;
    }
}
void peer_link_recv_cb(const peer_id_t peer_id, const std::vector<Message>& messages) {
    (void)peer_id;

    for (const Message& message : messages) {
        // Serial.println(message.type);
        switch (static_cast<MessageType>(message.type)) {

        case MessageType::Stop: sendCommand(CAN_CMD_STOP); break;

        case MessageType::STM_RESET: sendCommand(CAN_RESET); break;

        case MessageType::Reboot: ESP.restart(); break;

        case MessageType::Position:
            if (message.data.size() != sizeof(TabletData_Pos)) {
                Serial.println("invalid target data");
                break;
            }

            memcpy(&target_pos, message.data.data(), sizeof(TabletData_Pos));
            break;

        case MessageType::Gamepad:
            if (message.data.size() != sizeof(GamepadData)) {
                Serial.println("invalid gamepad data");
                break;
            }

            portENTER_CRITICAL(&gamepad_data_mux);
            memcpy(&received_gamepad_data, message.data.data(), sizeof(GamepadData));
            portEXIT_CRITICAL(&gamepad_data_mux);
            break;

        case MessageType::PS4Data:
            if (message.data.size() != sizeof(PS4Data)) {
                Serial.println("invalid PS4 data");
                break;
            }

            portENTER_CRITICAL(&gamepad_data_mux);
            memcpy(&received_ps4_data, message.data.data(), sizeof(PS4Data));
            portEXIT_CRITICAL(&gamepad_data_mux);
            break;
        case MessageType::GamePadUse: gamepad_use = true; break;

        case MessageType::TabletUse: gamepad_use = false; break;

        case MessageType::ELEVATION_UP:
        case MessageType::ELEVATION_DOWN: {
            if (message.data.size() != sizeof(uint16_t)) {
                Serial.println("invalid elevation duration");
                break;
            }

            uint16_t duration_ms = 0;
            memcpy(&duration_ms, message.data.data(), sizeof(duration_ms));

            const uint32_t command = static_cast<MessageType>(message.type) == MessageType::ELEVATION_UP
                                         ? CAN_CMD_ELEVATION_UP
                                         : CAN_CMD_ELEVATION_DOWN;

            // uint16_tのpayload分だけ動作させる
            sendCommand(command, duration_ms);
            break;
        }

        case MessageType::LoadChamber:
            can_belt.load_chamber = true;
            belt_order_received   = true;
            break;

        case MessageType::UnloadMag:
            can_belt.unload_mag = true;
            belt_order_received = true;
            break;

        case MessageType::LoadMag:
            can_belt.load_mag   = true;
            belt_order_received = true;
            break;

        case MessageType::UnloadChanber:
            can_belt.unload_chamber = true;
            belt_order_received     = true;
            break;

        case MessageType::BeltBucket_High:
            can_belt.elevation_pos    = 2;
            can_belt.elevation_change = true;
            belt_order_received       = true;

            break;
        case MessageType::BeltBucket_Middle:
            can_belt.elevation_pos    = 2;
            can_belt.elevation_change = true;
            belt_order_received       = true;

            break;
        case MessageType::BeltBucket_Low:
            can_belt.elevation_pos    = 2;
            can_belt.elevation_change = true;
            belt_order_received       = true;

            break;
        case MessageType::BeltDesk:
            can_belt.elevation_pos    = 3;
            can_belt.elevation_change = true;
            belt_order_received       = true;

            break;
        case MessageType::BeltFlag:
            can_belt.elevation_pos    = 1;
            can_belt.elevation_change = true;
            belt_order_received       = true;

            break;

        case MessageType::BeltLaunch:
            can_belt.launch     = true;
            belt_order_received = true;

            memcpy(&can_belt.acc, message.data.data(), sizeof(can_belt.acc));

            break;

        case MessageType::RollerStart:
            can_roller.acc_start  = true;
            roller_order_received = true;
            break;

        case MessageType::RollerLaunch:
            can_roller.launch     = true;
            roller_order_received = true;
            break;

        case MessageType::RollerStop:
            can_roller.stop       = true;
            roller_order_received = true;
            break;

        default: Serial.printf("unknown message type: 0x%02X\n", message.type); break;
        }
    }
}

void control_loop_task(void* args) {
    TickType_t wake_time = xTaskGetTickCount();

    TabletData_Pos previous_target = target_pos;

    while (true) {
        const double dt = CONTROL_CYCLE_MS / 1000.0;

        GamepadData gamepad_data{};
        PS4Data     ps4_data{};

        portENTER_CRITICAL(&gamepad_data_mux);
        memcpy(&gamepad_data, &received_gamepad_data, sizeof(GamepadData));
        memcpy(&ps4_data, &received_ps4_data, sizeof(PS4Data));
        portEXIT_CRITICAL(&gamepad_data_mux);

        update_swerve_drives();

        odometry.update(dt);

        const Position_deg current_position = odometry.get_position_deg();

        now_pos.x   = current_position.x;
        now_pos.y   = current_position.y;
        now_pos.deg = current_position.deg;

        // Gamepad制御との切り替え時に、現在位置へ目標を更新
        if (gamepad_use != now_state.gamepad_used) {
            now_state.gamepad_used = gamepad_use;

            target_pos.x   = now_pos.x;
            target_pos.y   = now_pos.y;
            target_pos.deg = now_pos.deg;

            ref_speed.x   = 0.0;
            ref_speed.y   = 0.0;
            ref_speed.deg = 0.0;

            translation_decelerating = false;
            previous_target          = target_pos;
        }

        now_vel_deg = odometry.get_velocity_deg();

        // 目標座標が変更されたら速度プロファイルを初期化
        if (target_pos.x != previous_target.x || target_pos.y != previous_target.y || target_pos.deg != previous_target.deg) {
            translation_decelerating = false;

            ref_speed.x   = 0.0;
            ref_speed.y   = 0.0;
            ref_speed.deg = 0.0;

            previous_target = target_pos;
        }

        const double dx = target_pos.x - now_pos.x;
        const double dy = target_pos.y - now_pos.y;

        const double distance = hypot(dx, dy);

        if (distance > 0.0) {
            const double direction_x = dx / distance;
            const double direction_y = dy / distance;

            const double current_speed = now_vel_deg.x * direction_x + now_vel_deg.y * direction_y;

            const double calculate_speed = ref_speed.x * direction_x + ref_speed.y * direction_y;

            const double linear_speed =
                updateDistanceVelocityProfile(distance, calculate_speed, current_speed, MAX_SHIFT_SPEED_MM_S,
                                              MAX_SHIFT_ACCELERATION, MAX_SHIFT_DECELERATION, dt, translation_decelerating);

            ref_speed.x = direction_x * linear_speed;
            ref_speed.y = direction_y * linear_speed;
        } else {
            ref_speed.x              = 0.0;
            ref_speed.y              = 0.0;
            translation_decelerating = false;
        }
        const double angle_error = wrapAngle(static_cast<double>(target_pos.deg) - static_cast<double>(now_pos.deg));

        const bool angle_reached = std::fabs(angle_error) <= ANGLE_TOLERANCE_DEG;

        const bool angle_stopped = std::fabs(now_vel_deg.deg) <= ANGLE_STOP_SPEED_DEG_S;

        if (angle_reached && angle_stopped) {
            ref_speed.deg = 0.0;
        } else {
            ref_speed.deg = updateAngleVelocityProfile(target_pos.deg, now_pos.deg, ref_speed.deg, MAX_ROTATE_SPEED_DEG_S,
                                                       MAX_ROTATE_ACCELERATION, dt);
        }

        const double theta = now_pos.deg * M_PI / 180.0;

        const double c = cos(theta);
        const double s = sin(theta);

        const double body_vx = c * ref_speed.x + s * ref_speed.y;

        const double body_vy = -s * ref_speed.x + c * ref_speed.y;

        static uint32_t last_print = 0;

        if (millis() - last_print >= 200) {
            last_print = millis();

            // Serial.printf("target:(%d, %d) pos:(%d, %d) dist:%.1f "
            //               "now_v:(%.1f, %.1f) ref_v:(%.1f, %.1f) body:(%.1f, %.1f)\n",
            //               target_pos.x, target_pos.y, n, now_status.y, distance, now_vel_deg.x, now_vel_deg.y,
            //               ref_speed.x, ref_speed.y, body_vx, body_vy);
            // Serial.printf("target:(%d, %d,%d)\r\n", target_pos.x, target_pos.y, target_pos.deg);
            // Serial.printf("steer1 turns = %d steer2 turns = %dsteer3 turns = %d  steer_deg%f  steer_deg%f steer_deg%f\n",
            //               steering_1.get_turns(), steering_2.get_turns(), steering_3.get_turns(),
            //               steering_1.get_current_degree(), steering_2.get_current_degree(),
            //               steering_3.get_current_degree());
            // Serial.printf("steer1 turns = %d steer2 turns = %dsteer3 turns = %d \n", steering_1.get_turns(),
            //               steering_2.get_turns(), steering_3.get_turns());
            // Serial.printf("Gamepad received: x=%d y=%d\n", received_gamepad_data.joystick_left.x,
            //               received_gamepad_data.joystick_left.y);
            // Serial.printf("order: gamepad=%d load=%d reload=%d finish=%d launch=%d pos=%u acc=%u\n", gamepad_use,
            //               can_belt.load_belt, can_belt.reload_belt, can_belt.reload_finish_belt, can_belt.launch,
            //               can_belt.elevation_pos, can_belt.acc);
        }
        if (now_state.gamepad_used) {
            handle_controller_input(gamepad_data.joystick_right.x, gamepad_data.joystick_right.y, gamepad_data.joystick_left.x);
            gamepad(gamepad_data, ps4_data);
        } else {
            set_robot_velocity(body_vx, body_vy, ref_speed.deg);
        }

        can.update();

        can_send();

        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(CONTROL_CYCLE_MS));
    }
}

void setup() {
    Serial.begin(115200);
    can_belt.elevation_pos = 1;
    delay(500);
    Serial.println("setup start");

    pinMode(CAN_CS_PIN, OUTPUT);
    digitalWrite(CAN_CS_PIN, HIGH);

    SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);

    spi_mutex_init();
    Serial.println("SPI initialized");

    can.begin();

    ESP32Can.setPins(TX_PIN, RX_PIN);

    if (!ESP32Can.begin(ESP32Can.convertSpeed(1000))) {
        Serial.println("CAN begin FAILED");
        while (1) {
            delay(1000);
        }
    }

    odometry.begin();
    Serial.println("odometry initialized");

    if (!initialize_swerve_drives()) {
        Serial.println("swerve initialization failed");
        while (true) {
            delay(1000);
        }
    }

    Serial.println("swerve initialized");

    nnct::Amt223dv::beginStatic(&SPI, ABS_ENCODER_READ_PERIOD_MS);
    Serial.println("encoder task started");

    peer_link_task_init(WIFI_CHANNEL, SWERVE_S3_ID);
    Serial.println("peer link initialized");

    xTaskCreate(control_loop_task, "ControlLoopTask", CONTROL_LOOP_TASK_STACK_SIZE, NULL, CONTROL_LOOP_TASK_PRIORITY,
                &control_loop_task_handle);

    Serial.printf("can_order");
    belt_order_received = false;
    can_send_belt_data();

    Serial.println("setup complete");
}

void loop() {
    can_receive();
    static bool previous_roller_reach = false;
    const bool  roller_reach_trigger  = now_state.roller_reach && !previous_roller_reach;

    previous_roller_reach = now_state.roller_reach;

    static uint32_t last_print = 0;

    if (belt_order_received) {
        Serial.printf("can_order");
        belt_order_received = false;
        can_send_belt_data();
    }
    if (roller_order_received) {
        Serial.printf("can_order");
        roller_order_received = false;
        can_send_roller_data();
    }
    if (peer_link_is_peer_exist(TABLET_ESP_ID)) {
        std::vector<Message> messages;

        Message pos_message;
        pos_message.type = static_cast<uint8_t>(MessageType::Position);
        pos_message.data.resize(sizeof(TabletData_Pos));

        memcpy(pos_message.data.data(), &now_pos, sizeof(TabletData_Pos));

        messages.push_back(pos_message);

        Message state_message;
        state_message.type = static_cast<uint8_t>(MessageType::RobotState);
        state_message.data.resize(sizeof(StateData));

        memcpy(state_message.data.data(), &now_state, sizeof(StateData));

        messages.push_back(state_message);

        const esp_err_t result = peer_link_send(TABLET_ESP_ID, messages);

        if (result != ESP_OK) {
            Serial.printf("send error: %d\n", result);
        }
    }

    if (roller_reach_trigger && peer_link_is_peer_exist(Gamepad_ESP_ID)) {
        // if (true) {
        std::vector<Message> rumble_messages;

        Message rumble_message;
        rumble_message.type = static_cast<uint8_t>(MessageType::PS4SetRumble);
        rumble_message.data.resize(sizeof(PS4RumbleData));

        PS4RumbleData data{};
        data.small_rumble = 0;
        data.big_rumble   = 255;
        data.duration     = 1000;

        memcpy(rumble_message.data.data(), &data, sizeof(data));
        rumble_messages.push_back(rumble_message);
        Serial.println("send");

        const esp_err_t rumble_result = peer_link_send(Gamepad_ESP_ID, rumble_messages);

        if (rumble_result != ESP_OK) {
            Serial.printf("rumble send error: %d\n", rumble_result);
        }
    }
    delay(LOOP_DELAY_MS);
}