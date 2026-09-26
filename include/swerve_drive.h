#pragma once

#include "AnglePid.h"
#include "IncrementalPid.h"
#include "Pid.h"
#include "constants.hpp"
#include "nnct/interfaces/interfaces.hpp"
#include <Arduino.h>

using namespace nnct::interfaces;

class Steering {
    public:
        Steering(Motor* motor, nnct::Amt223dv* encoder, AnglePID* pid, double offset_deg, uint8_t ID)
            : motor(motor), encoder(encoder), pid(pid), target_degree(0), offset_degree(offset_deg), ID(ID) {}

        void begin() {
            encoder->begin();
            motor->stop();
        }

        void   set_target(double degree) { this->target_degree = degree; }
        double get_current_degree() {
            // 絶対角度からオフセットを減算
            const double absolute_degree = static_cast<double>(encoder->positionDeg());
            return normalizeAngleDeg(absolute_degree - offset_degree);
        }
        void update(double dt) {
            const double current_degree = get_current_degree();
            double       duty           = pid->update(target_degree, current_degree, dt);
            const double error          = pid->getError();

            if (fabs(error) < ANGLE_DEAD_ZONE_DEG) {
                duty = 0.0;
            }

            motor->run(duty, -1);
        }

    private:
        static double normalizeAngleDeg(double a) {
            while (a > 180.0)
                a -= 360.0;
            while (a < -180.0)
                a += 360.0;
            return a;
        }

        Motor*          motor;
        nnct::Amt223dv* encoder;
        AnglePID*       pid;

        double        target_degree;
        double        offset_degree;
        const uint8_t ID;
};

class Drive {
    public:
        enum class ControlMode {
            Duty,
            Speed
        };
        Drive(RobomasMotor* motor, PID* pid, uint8_t ID)
            : motor(motor), pid(pid), mode(ControlMode::Duty), target_duty(0.0), target_mm_s(0.0), ID(ID) {}

        // duty指定
        void set_target_duty(double duty) {
            mode        = ControlMode::Duty;
            target_duty = duty;
        }

        // 速度指定[mm/s]
        void set_target_mm_s(double speed_mm_s) {
            mode        = ControlMode::Speed;
            target_mm_s = speed_mm_s;
        }

        double get_current_mm_s() { // rpm -> mm/s
            double       motor_rpm = this->motor->rpm();
            const double wheel_rpm = motor_rpm / DRIVE_GEAR_RATIO;
            return wheel_rpm * 2.0 * M_PI * DRIVE_RADIUS / 60.0;
        }

        void update(double dt) {
            double drive_command;

            if (mode == ControlMode::Speed) {
                const double current_mm_s = get_current_mm_s();

                // static uint32_t last_print_time = 0;
                // const uint32_t  now             = millis();

                // if (now - last_print_time >= 1000) {
                //     last_print_time = now;
                //     Serial.printf("%f\r\n", current_mm_s);
                // }

                drive_command = pid->update(target_mm_s, current_mm_s, dt);

            } else {
                drive_command = target_duty;
            }

            motor->run(drive_command);
        }

        void stop() {
            mode        = ControlMode::Duty;
            target_duty = 0.0;
            target_mm_s = 0.0;
            motor->stop();
        }

    private:
        RobomasMotor* motor;
        PID*          pid;

        ControlMode mode;
        double      target_duty;
        double      target_mm_s;

        const uint8_t ID;
};

class SwerveDrive {
    public:
        SwerveDrive(Drive* drive, Steering* steering, uint8_t ID) : drive(drive), steering(steering), ID(ID) {}
        bool init() {
            steering->begin();
            return true;
        }

        void set_target_duty(double degree, double drive_target_duty) {
            double          current_degree = steering->get_current_degree();
            OptimizedParams params         = optimizeSteerAngle(degree, current_degree);

            steering->set_target(params.degree);
            drive->set_target_duty(drive_target_duty * params.drive_dir);
        }

        void set_target_mm_s(double degree, double drive_target_mm_s) {
            double          current_degree = steering->get_current_degree();
            OptimizedParams params         = optimizeSteerAngle(degree, current_degree);

            // Serial.printf("OPT target=%.1f current=%.1f count=%ld\n", degree, current_degree, steering->get_encoder_count());

            steering->set_target(params.degree);
            drive->set_target_mm_s(drive_target_mm_s * params.drive_dir);
        }

        void set_deg(double degree) { steering->set_target(degree); }

        void stop_drive() { drive->stop(); }
        void update(double dt) {
            this->steering->update(dt);
            this->drive->update(dt);
        }

    private:
        Drive*        drive;
        Steering*     steering;
        const uint8_t ID;

        struct OptimizedParams {
                double degree;
                int8_t drive_dir;
        };

        static double normalizeAngleDeg(double a) {
            while (a > 180.0)
                a -= 360.0;
            while (a < -180.0)
                a += 360.0;
            return a;
        }
        static OptimizedParams optimizeSteerAngle(double steer_target, double currentAngleDeg) {
            double error = normalizeAngleDeg(steer_target - currentAngleDeg);

            OptimizedParams result;
            result.drive_dir = 1;

            // 90°を超えるなら、車輪を180°反転して
            // ドライブ方向を逆にする
            if (error > 90.0) {
                error -= 180.0;
                result.drive_dir = -1;
            } else if (error < -90.0) {
                error += 180.0;
                result.drive_dir = -1;
            }

            // 現在角度の近くにある目標角度を作る
            result.degree = currentAngleDeg + error;

            return result;
        }
};