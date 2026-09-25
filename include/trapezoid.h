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

double updateVelocityProfile(double target_pos, double now_pos, double current_speed, double max_speed, double acceleration,
                             double dt) {
    const double error    = target_pos - now_pos;
    const double distance = std::fabs(error);

    constexpr double POSITION_THRESHOLD = 3.0;
    constexpr double SPEED_THRESHOLD    = 2.0;

    // 位置・速度ともに十分小さい場合だけ停止
    if (distance < POSITION_THRESHOLD && std::fabs(current_speed) < SPEED_THRESHOLD) {
        return 0.0;
    }

    // 目標位置付近に到達しているが、速度が残っている場合
    // 現在の速度を0にする
    if (distance < POSITION_THRESHOLD) {
        double next_speed;

        if (current_speed > 0.0) {
            next_speed = current_speed - acceleration * dt;

            if (next_speed < 0.0) {
                next_speed = 0.0;
            }
        } else {
            next_speed = current_speed + acceleration * dt;

            if (next_speed > 0.0) {
                next_speed = 0.0;
            }
        }

        return next_speed;
    }

    const double direction = sign(error);

    // 現在速度が目標方向と逆なら、まず停止する
    if (current_speed * direction < 0.0) {
        double next_speed = current_speed + direction * acceleration * dt;

        // 0を跨がないようにする
        if (next_speed * current_speed < 0.0) {
            next_speed = 0.0;
        }

        return next_speed;
    }

    // 目標方向に進んでいる場合
    const double speed_abs = std::fabs(current_speed);

    const double stop_distance = (speed_abs * speed_abs) / (2.0 * acceleration);

    double next_speed = current_speed;

    if (distance > stop_distance) {
        // 加速
        next_speed += direction * acceleration * dt;
    } else {
        // 減速
        next_speed -= direction * acceleration * dt;
    }

    next_speed = constrain(next_speed, -max_speed, max_speed);

    return next_speed;
}

double updateAngleVelocityProfile(double target_deg, double now_deg, double current_speed, double max_speed,
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
                                     double acceleration, double dt, bool& decelerating) {
    if (distance <= POSITION_TOLERANCE_MM) {
        decelerating = false;
        return 0.0;
    }

    if (acceleration <= 0.0 || dt <= 0.0) {
        return ref_speed;
    }

    // 実測速度が目標方向と逆
    if (measured_speed < -5.0) {
        ref_speed -= acceleration * dt;
        return std::max(0.0, ref_speed);
    }

    // まだ減速に入っていない場合だけ、
    // 実測速度から減速開始を判定
    if (!decelerating) {

        const double speed_for_stopping = std::max(std::fabs(measured_speed), std::fabs(ref_speed));

        const double stop_distance = (speed_for_stopping * speed_for_stopping) / (2.0 * acceleration);

        if (distance <= stop_distance) {
            decelerating = true;
        }
    }

    if (decelerating) {
        // 減速中はref_speedだけを使う
        ref_speed -= acceleration * dt;

        // 20 mmより遠い場合は最低100 mm/s、
        // 20 mm以内では最低40 mm/s
        const double minimum_speed =
            (distance > APPROACH_DISTANCE_MM) ? FAR_MIN_TRANSLATION_SPEED_MM_S : MIN_TRANSLATION_SPEED_MM_S;

        ref_speed = std::max(ref_speed, minimum_speed);
    } else {
        // 通常はref_speedを加速
        ref_speed += acceleration * dt;
    }

    return std::min(ref_speed, max_speed);
}