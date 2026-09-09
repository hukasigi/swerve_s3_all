#pragma once

#include <cmath>

#include "constants.hpp"
#include "nnct/interfaces/incremental_encoder.hpp"

struct Position_rad {
        double x;
        double y;
        double rad;

        Position_rad() : x(0.0), y(0.0), rad(0.0) {}
        Position_rad(double x_, double y_, double yaw_) : x(x_), y(y_), rad(yaw_) {}
};

struct Position_deg {
        double x;
        double y;
        double deg;

        Position_deg() : x(0.0), y(0.0), deg(0.0) {}
        Position_deg(double x_, double y_, double yaw_) : x(x_), y(y_), deg(yaw_) {}
};

class Odometry {
    public:
        Odometry(nnct::interfaces::IncrementalEncoder& encoder1, nnct::interfaces::IncrementalEncoder& encoder2,
                 nnct::interfaces::IncrementalEncoder& encoder3);

        void         begin();
        Position_rad get_position_rad() const;
        Position_rad get_velocity_rad() const;
        Position_deg get_position_deg() const;
        Position_deg get_velocity_deg() const;
        void         update(double dt);

    private:
        static bool Invert3x3(const double A[3][3], double invA[3][3]);
        void        _buildInverse();

        nnct::interfaces::IncrementalEncoder& encoder_1_;
        nnct::interfaces::IncrementalEncoder& encoder_2_;
        nnct::interfaces::IncrementalEncoder& encoder_3_;

        int32_t prev_count1_{0};
        int32_t prev_count2_{0};
        int32_t prev_count3_{0};

        double last_dc1_{0.0};
        double last_dc2_{0.0};
        double last_dc3_{0.0};

        Position_rad position_{};
        Position_rad velocity_{};

        bool   inv_ok_{false};
        double invA_[3][3]{};
};