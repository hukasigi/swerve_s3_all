#include "localization.hpp"

double wrapPi(double rad) {
    while (rad > M_PI)
        rad -= 2.0 * M_PI;
    while (rad < -M_PI)
        rad += 2.0 * M_PI;
    return rad;
}

Position_rad WheelDeltaToBodyDelta(const double invA[3][3], bool inv_ok, double s1_mm, double s2_mm, double s3_mm) {
    if (!inv_ok) return {0.0, 0.0, 0.0};

    const double s[3] = {s1_mm, s2_mm, s3_mm};

    Position_rad res;
    res.x   = invA[0][0] * s[0] + invA[0][1] * s[1] + invA[0][2] * s[2];
    res.y   = invA[1][0] * s[0] + invA[1][1] * s[1] + invA[1][2] * s[2];
    res.rad = invA[2][0] * s[0] + invA[2][1] * s[1] + invA[2][2] * s[2];
    return res;
}

Position_rad CountToBody(const double invA[3][3], bool inv_ok, long dc1, long dc2, long dc3) {
    const double s1 = static_cast<double>(dc1) / COUNTS_PER_MM;
    const double s2 = static_cast<double>(dc2) / COUNTS_PER_MM;
    const double s3 = static_cast<double>(dc3) / COUNTS_PER_MM;
    return WheelDeltaToBodyDelta(invA, inv_ok, s1, s2, s3);
}

bool Odometry::Invert3x3(const double A[3][3], double invA[3][3]) {
    const double det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                       A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

    if (fabs(det) < 1e-12) return false;

    const double invDet = 1.0 / det;

    invA[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * invDet;
    invA[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * invDet;
    invA[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * invDet;

    invA[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * invDet;
    invA[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * invDet;
    invA[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) * invDet;

    invA[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * invDet;
    invA[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * invDet;
    invA[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * invDet;

    return true;
}

Odometry::Odometry(nnct::interfaces::IncrementalEncoder& encoder1, nnct::interfaces::IncrementalEncoder& encoder2,
                   nnct::interfaces::IncrementalEncoder& encoder3)
    : encoder_1_(encoder1), encoder_2_(encoder2), encoder_3_(encoder3) {}

void Odometry::begin() {
    position_ = {0.0, 0.0, 0.0};
    velocity_ = {0.0, 0.0, 0.0};

    encoder_1_.clear();
    encoder_2_.clear();
    encoder_3_.clear();

    prev_count1_ = encoder_1_.getCount();
    prev_count2_ = encoder_2_.getCount();
    prev_count3_ = encoder_3_.getCount();

    _buildInverse();
}

Position_rad Odometry::get_position_rad() const {
    return position_;
}
Position_deg Odometry::get_position_deg() const {
    return {position_.x, position_.y, position_.rad * 180.0 / M_PI};
}

Position_rad Odometry::get_velocity_rad() const {
    return velocity_;
}

Position_deg Odometry::get_velocity_deg() const {
    return {velocity_.x, velocity_.y, velocity_.rad * 180.0 / M_PI};
}

void Odometry::_buildInverse() {
    const double a1 = 215.0 * M_PI / 180.0;
    const double a2 = 0.0 * M_PI / 180.0;
    const double a3 = 135.0 * M_PI / 180.0;

    const double c1 = cos(a1), s1 = sin(a1);
    const double c2 = cos(a2), s2 = sin(a2);
    const double c3 = cos(a3), s3 = sin(a3);

    const double k = +ROBOT_TO_ODO_RADIUS;

    const double A[3][3] = {
        {c1, s1, k},
        {c2, s2, k},
        {c3, s3, k},
    };

    inv_ok_ = Invert3x3(A, invA_);
}

void Odometry::update(double dt) {
    if (dt <= 0.0) {
        velocity_ = {0.0, 0.0, 0.0};
        return;
    }

    if (!inv_ok_) _buildInverse();
    if (!inv_ok_) return;

    const long c1 = encoder_1_.getCount();
    const long c2 = encoder_2_.getCount();
    const long c3 = encoder_3_.getCount();

    long dc1 = (c1 - prev_count1_) * ENCODER_SIGN_1;
    long dc2 = (c2 - prev_count2_) * ENCODER_SIGN_2;
    long dc3 = (c3 - prev_count3_) * ENCODER_SIGN_3;

    if (labs(dc1) > 200 || labs(dc2) > 200 || labs(dc3) > 200) {

        if (labs(dc1) > 200) dc1 = last_dc1_;
        if (labs(dc2) > 200) dc2 = last_dc2_;
        if (labs(dc3) > 200) dc3 = last_dc3_;
    }

    last_dc1_ = dc1;
    last_dc2_ = dc2;
    last_dc3_ = dc3;

    prev_count1_ = c1;
    prev_count2_ = c2;
    prev_count3_ = c3;

    const Position_rad dpos = CountToBody(invA_, inv_ok_, dc1, dc2, dc3);

    // ロボット座標系での速度
    const double body_vx   = dpos.x / dt;
    const double body_vy   = dpos.y / dt;
    const double angular_v = dpos.rad / dt;

    // 中間角度
    const double th_mid = position_.rad + 0.5 * dpos.rad;

    const double ct_mid = cos(th_mid);
    const double st_mid = sin(th_mid);

    // ワールド座標系速度
    velocity_.x = ct_mid * body_vx - st_mid * body_vy;

    velocity_.y = st_mid * body_vx + ct_mid * body_vy;

    velocity_.rad = angular_v;

    // 位置更新
    position_.x += ct_mid * dpos.x - st_mid * dpos.y;

    position_.y += st_mid * dpos.x + ct_mid * dpos.y;

    position_.rad = wrapPi(position_.rad + dpos.rad);
}