#pragma once

#include "Arduino.h"
#include <algorithm>
#include <cmath>

double sign(double value) {
    if (value > 0.0) return 1.0;
    if (value < 0.0) return -1.0;
    return 0.0;
}
// 角度を -180 ～ +180 にする
double wrapAngle(double angle) {
    while (angle > 180.0) {
        angle -= 360.0;
    }

    while (angle < -180.0) {
        angle += 360.0;
    }

    return angle;
}

double updateAngleVelocityProfile(int16_t target_deg, int16_t now_deg, double current_speed, double max_speed,
                                  double max_acceleration, double dt) {
    const double error    = wrapAngle(target_deg - now_deg);
    const double distance = std::fabs(error);

    constexpr double ANGLE_THRESHOLD = 1.0;
    constexpr double SPEED_THRESHOLD = 5.0;

    // 角度・速度ともに十分小さい場合だけ停止
    if (distance < ANGLE_THRESHOLD && std::fabs(current_speed) < SPEED_THRESHOLD) {
        return 0.0;
    }

    // 目標角度付近に到達しているが、速度が残っている場合
    // 現在の速度を0にする
    if (distance < ANGLE_THRESHOLD) {
        double next_speed;

        if (current_speed > 0.0) {
            next_speed = current_speed - max_acceleration * dt;

            if (next_speed < 0.0) {
                next_speed = 0.0;
            }
        } else {
            next_speed = current_speed + max_acceleration * dt;

            if (next_speed > 0.0) {
                next_speed = 0.0;
            }
        }

        return next_speed;
    }

    const double direction = sign(error);

    // 現在速度が目標方向と逆なら、まず停止する
    if (current_speed * direction < 0.0) {
        double next_speed = current_speed + direction * max_acceleration * dt;

        // 0を跨がないようにする
        if (next_speed * current_speed < 0.0) {
            next_speed = 0.0;
        }

        return next_speed;
    }

    // 現在速度から停止するまでに必要な角度
    const double speed_abs = std::fabs(current_speed);

    const double stop_angle = (speed_abs * speed_abs) / (2.0 * max_acceleration);

    double next_speed = current_speed;

    if (distance > stop_angle) {
        // 加速
        next_speed += direction * max_acceleration * dt;
    } else {
        // 減速
        next_speed -= direction * max_acceleration * dt;
    }

    next_speed = constrain(next_speed, -max_speed, max_speed);

    return next_speed;
}

double updateDistanceVelocityProfile(double distance, double ref_speed, double measured_speed, double max_speed,
                                     double acceleration, double deceleration, double dt, bool& decelerating) {

    // パラメータが不正な場合は現在の目標速度をそのまま維持する
    if (acceleration <= 0.0 || deceleration <= 0.0 || dt <= 0.0) {
        return ref_speed;
    }

    // 目標位置（許容誤差内）に到達した場合の処理
    if (distance <= POSITION_TOLERANCE_MM) {
        // ほぼ停止していれば停止完了とする
        if (std::fabs(measured_speed) <= POSITION_STOP_SPEED_MM_S) {
            decelerating = false;
            return 0.0;
        }
        // 停止中でなければ減速を継続する
        return std::max(0.0, ref_speed - deceleration * dt);
    }

    // まだ減速中でない場合、停止距離を計算して減速開始タイミングを判定する
    if (!decelerating) {
        const double speed_for_stopping = std::max(std::fabs(measured_speed), std::fabs(ref_speed));

        const double stop_distance = (speed_for_stopping * speed_for_stopping) / (2.0 * deceleration);

        if (distance <= stop_distance) {
            decelerating = true;
        }
    }

    // 減速中の処理
    if (decelerating) {
        const double available_distance = distance - POSITION_TOLERANCE_MM;

        // 残り距離から算出される理想的な制動速度
        const double braking_speed = std::sqrt(std::max(0.0, 2.0 * deceleration * available_distance));

        if (braking_speed >= MIN_TRANSLATION_SPEED_MM_S) {
            ref_speed -= deceleration * dt;

            ref_speed = std::max(MIN_TRANSLATION_SPEED_MM_S, ref_speed);

            ref_speed = std::min(ref_speed, braking_speed);

        } else {
            // 低速域では最低停止速度に維持する
            ref_speed = MIN_STOP_SPEED_MM_S;
        }

    } else {
        // 減速中でない（加速または等速）場合の処理
        ref_speed += acceleration * dt;
    }

    // 最高速度を超えないように制限して返す
    return std::min(ref_speed, max_speed);
}