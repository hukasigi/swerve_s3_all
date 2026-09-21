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