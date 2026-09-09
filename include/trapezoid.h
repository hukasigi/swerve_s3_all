#pragma once

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
    const double stop_distance = (current_speed * current_speed) / (2.0 * acceleration);

    double next_speed = current_speed;

    if (fabs(error) > stop_distance) {
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

    constexpr double ANGLE_THRESHOLD = 1.;
    constexpr double SPEED_THRESHOLD = 5.;

    if (distance < ANGLE_THRESHOLD && std::fabs(current_speed) < SPEED_THRESHOLD) {
        return 0.0;
    }

    const double stop_angle = (current_speed * current_speed) / (2.0 * max_acceleration);

    double next_speed = current_speed;

    if (fabs(error) > stop_angle) {

        next_speed += sign(error) * max_acceleration * dt;

    } else {

        next_speed -= sign(current_speed) * max_acceleration * dt;
    }

    next_speed = constrain(next_speed, -max_speed, max_speed);

    return next_speed;
}