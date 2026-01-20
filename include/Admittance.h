#pragma once

#include <array>
#include <utility>

// 4-DOF 导纳控制器 (X, Y, Z, RotationZ)
// 适用于 PRPR 结构或 SCARA 类机器人
class Admittance4 {
public:
    // 0-2: x, y, z (m)
    // 3:   theta (rad)
    using Vec4 = std::array<double, 4>;

    // 构造函数：传入 4 维的 M, D, K 参数
    Admittance4(const Vec4& M, const Vec4& D, const Vec4& K, double dt);

    // 设置当前状态和期望状态
    void set_state(const Vec4& x, const Vec4& xdesir);

    // 计算误差：x_e = x_desire - x
    void caculate_error(const Vec4& x);

    // 核心更新函数
    // F_ext: [Fx, Fy, Fz, Mz]
    // 返回: (新位置 x, 新速度 xdot)
    std::pair<Vec4, Vec4> update(const Vec4& F_ext);

    // Getters
    const Vec4& x() const { return x_; }
    const Vec4& xdot() const { return xdot_; }
    const Vec4& xdesir() const { return xdesir_; }
    const Vec4& x_e() const { return x_e_; }

private:
    // 参数（4维对角）
    Vec4 M_;
    Vec4 D_;
    Vec4 K_;
    double dt_{0.0};

    // 状态
    Vec4 x_;       // 当前位置 [x, y, z, theta]
    Vec4 xdot_;    // 当前速度
    Vec4 xdesir_;  // 期望位置

    // 误差
    Vec4 x_e_;
};