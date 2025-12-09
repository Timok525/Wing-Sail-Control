#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <fstream>
#include <chrono>
#include <sstream>

// 全局随机数生成器（对应 Matlab rng('default')）
std::mt19937 rng;
std::normal_distribution<double> norm_dist(0.0, 1.0);

// 声明辅助函数
double sum(double x);
double soft_saturate(double u, double max_u, double soft_u);
std::vector<double> calc_angular_acceleration(
    double alpha, double beta, double phi,
    double dalpha, double dbeta, double dphi,
    double delta, const struct Params& params
);
std::vector<double> dyn_fun(double t, const std::vector<double>& state, const struct Params& params);
void rk4_integrate(
    double t_start, double t_end, double dt,
    const std::vector<double>& state0,
    const struct Params& params,
    std::vector<double>& t_history,
    std::vector<std::vector<double>>& state_history
);
void save_results(
    const std::vector<double>& t_history,
    const std::vector<std::vector<double>>& state_history,
    const struct Params& params
);

// 参数结构体（重点调整alpha专属参数）
struct Params {
    // 基础物理参数（保持不变）
    double x_OA = 0.1;    double y_OA = 0.0;    double z_OA = -1.0;
    double x_OR = 1.0;    double y_OR = 0.0;    double z_OR = -0.7;
    double z_OC = -0.3;   double C_SC = 1.8;    double C_RC = 0.8;
    double S_S = 1.8;     double S_R = 0.6;     double m_T = 0.005;
    double m_C = 1.2;     double g = 9.81;      double J_z = 0.09;
    double l = 0.3;       double V_S = 2.5;     double p_s = 1.225;

    // phi角影响阈值（保持不变）
    double phi_thresh = 25.0 * M_PI / 180.0;
    double coupling_min = 0.01;
    double coupling_gain = 0.01;

    // 角度/角速度限制（【调整1】增大alpha角速度基础值，提升调整速度）
    double alpha_max = 0.5;    double beta_max = 0.5;     double phi_max = 1.5;
    double d_alpha_base = 0.08; // alpha专属：从0.05→0.08（比beta快）
    double d_beta_base = 0.05; 
    double d_phi_max = 0.2;
    double bound_ratio = 0.4;

    // 目标平滑参数（【调整2】alpha专属平滑系数，更快跟踪目标）
    double tau_smooth_alpha = 0.15; // alpha专属：从0.2→0.15
    double tau_smooth_beta = 0.2;   // beta保持不变

    // PID参数（【调整3】增强alpha的比例+积分作用）
    double Kp_alpha = 0.18;  // alpha专属：从0.1→0.18
    double Kd_alpha = 0.2;   // 保持不变
    double Ki_alpha = 0.07;  // alpha专属：从0.05→0.07
    double Kp_beta = 0.08;   // 保持不变
    double Kd_beta = 0.1;    // 保持不变
    double Ki_beta = 0.03;   // 保持不变
    double Kp_phi = 4.55;    // 保持不变
    double Kd_phi = 7.6;     // 保持不变
    double Ki_phi = 0.53;    // 保持不变

    // 积分限幅与阈值（保持不变）
    double phi_int_max = 0.9;     double phi_int_thresh = 0.175;
    double phi_error_dead = 0.035; double integral_max = 0.03;

    // 舵偏角限制（保持不变）
    double delta_max = 1.5;  double delta_soft = 1.2;

    // 延迟+波动参数（保持不变）
    double tau_delta = 2.0;       double delta_noise_amp = 0.005;

    // 目标phi角（保持不变）
    double phi_target = 0.0;

    // 阻尼系数（保持不变）
    double alpha_damping_gain = 0.3;
    double beta_damping_gain = 0.3;
};

// 软饱和函数（保持不变）
double soft_saturate(double u, double max_u, double soft_u) {
    if (max_u <= soft_u) {
        soft_u = max_u * 0.7;
    }
    if (std::abs(u) <= soft_u) {
        return u;
    } else {
        double k = (std::abs(u) - soft_u) / (max_u - soft_u);
        k = std::tanh(k * 5.0);
        return std::copysign(1.0, u) * (soft_u + k * (max_u - soft_u));
    }
}

// sum辅助函数（保持不变）
double sum(double x) {
    return x;
}

// 计算角加速度（【调整4】修正alpha相关矢径符号，确保气动力矩方向正确）
std::vector<double> calc_angular_acceleration(
    double alpha, double beta, double phi,
    double dalpha, double dbeta, double dphi,
    double delta, const Params& params
) {
    double x_OA = params.x_OA;  double y_OA = params.y_OA;  double z_OA = params.z_OA;
    double x_OR = params.x_OR;  double y_OR = params.y_OR;  double z_OR = params.z_OR;
    double z_OC = params.z_OC;  double C_SC = params.C_SC;  double C_RC = params.C_RC;
    double S_S = params.S_S;    double S_R = params.S_R;    double m_T = params.m_T;
    double m_C = params.m_C;    double g = params.g;        double J_z = params.J_z;
    double l = params.l;        double V_S = params.V_S;    double p_s = params.p_s;

    // 坐标系转换矩阵（保持不变）
    double cos_phi = std::cos(phi);
    double sin_phi = std::sin(phi);
    std::vector<std::vector<double>> e_R_S = {
        {cos_phi, -sin_phi, 0.0},
        {sin_phi,  cos_phi, 0.0},
        {0.0,      0.0,     1.0}
    };

    // 气动力计算（保持不变）
    double Q_S = 0.5 * p_s * V_S * V_S;
    double C_Sy = C_SC * phi;
    std::vector<double> A_S = {
        Q_S * S_S * sin_phi * C_Sy,
        -Q_S * S_S * cos_phi * C_Sy,
        0.0
    };
    double C_Ry = C_SC * delta;
    std::vector<double> A_R = {
        Q_S * S_R * C_Ry * sin_phi,
        -Q_S * S_R * C_Ry * cos_phi,
        0.0
    };

    // 【关键调整】修正alpha对应的矢径r_S[0]符号（从负→正，与Matlab对齐）
    std::vector<double> r_S = {
        l * std::sin(alpha) * std::cos(beta),  // 原：-l*sin(alpha)*cos(beta)
        l * std::sin(beta),
        l * std::cos(alpha) * std::cos(beta)
    };
    std::vector<double> r_A_S(3, 0.0);
    for (int i = 0; i < 3; ++i) {
        r_A_S[i] = r_S[i] + e_R_S[i][0] * x_OA + e_R_S[i][1] * y_OA + e_R_S[i][2] * z_OA;
    }
    std::vector<double> r_R_S(3, 0.0);
    for (int i = 0; i < 3; ++i) {
        r_R_S[i] = r_S[i] + e_R_S[i][0] * x_OR + e_R_S[i][1] * y_OR + e_R_S[i][2] * z_OR;
    }

    // 惯性矩阵（保持不变）
    double I_alpha = (1.0 / 3.0) * m_T * std::pow(l * std::cos(beta), 2.0);
    double I_beta = (1.0 / 3.0) * m_T * l * l;
    std::vector<std::vector<double>> M = {
        {I_alpha, 0.0,      0.0},
        {0.0,      I_beta,  0.0},
        {0.0,      0.0,     J_z}
    };

    // 科氏力矩阵（保持之前的修正，不变）
    std::vector<std::vector<double>> C(3, std::vector<double>(3, 0.0));
    C[0][2] = (2.0 / 3.0) * m_T * l * l * std::cos(beta) * std::sin(beta) * dphi;
    C[1][0] = - (2.0 / 3.0) * m_T * l * l * std::cos(beta) * std::sin(beta) * dphi;
    C[1][2] = - (2.0 / 3.0) * m_T * l * l * std::cos(beta) * std::sin(beta) * dalpha;

    // 重力项（保持之前的修正，不变）
    std::vector<double> N(3, 0.0);
    N[0] = 0.5 * m_T * g * l * std::sin(alpha) * std::cos(beta) 
           + m_C * g * l * std::sin(alpha) * std::cos(beta);
    N[1] = 0.5 * m_T * g * l * std::cos(alpha) * std::sin(beta) 
           + m_C * g * l * std::cos(alpha) * std::sin(beta);

    // 气动力矩（保持不变）
    std::vector<double> Q(3, 0.0);
    auto cross_product = [](const std::vector<double>& r, const std::vector<double>& A) {
        std::vector<double> torque(3);
        torque[0] = r[1] * A[2] - r[2] * A[1];
        torque[1] = r[2] * A[0] - r[0] * A[2];
        torque[2] = r[0] * A[1] - r[1] * A[0];
        return torque;
    };
    std::vector<double> tau_S = cross_product(r_A_S, A_S);
    std::vector<double> tau_R = cross_product(r_R_S, A_R);
    for (int i = 0; i < 3; ++i) {
        Q[i] = tau_S[i] + tau_R[i];
    }

    // 求解线性方程组（保持不变）
    std::vector<double> dq = {dalpha, dbeta, dphi};
    std::vector<double> rhs(3, 0.0);
    for (int i = 0; i < 3; ++i) {
        double C_dq = 0.0;
        for (int j = 0; j < 3; ++j) {
            C_dq += C[i][j] * dq[j];
        }
        rhs[i] = Q[i] - C_dq - N[i];
    }

    // 克莱姆法则（保持不变）
    double det_M = M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
                 - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
                 + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);

    std::vector<std::vector<double>> M1 = M;
    for (int i = 0; i < 3; ++i) M1[i][0] = rhs[i];
    double det_M1 = M1[0][0] * (M1[1][1] * M1[2][2] - M1[1][2] * M1[2][1])
                  - M1[0][1] * (M1[1][0] * M1[2][2] - M1[1][2] * M1[2][0])
                  + M1[0][2] * (M1[1][0] * M1[2][1] - M1[1][1] * M1[2][0]);

    std::vector<std::vector<double>> M2 = M;
    for (int i = 0; i < 3; ++i) M2[i][1] = rhs[i];
    double det_M2 = M2[0][0] * (M2[1][1] * M2[2][2] - M2[1][2] * M2[2][1])
                  - M2[0][1] * (M2[1][0] * M2[2][2] - M2[1][2] * M2[2][0])
                  + M2[0][2] * (M2[1][0] * M2[2][1] - M2[1][1] * M2[2][0]);

    std::vector<std::vector<double>> M3 = M;
    for (int i = 0; i < 3; ++i) M3[i][2] = rhs[i];
    double det_M3 = M3[0][0] * (M3[1][1] * M3[2][2] - M3[1][2] * M3[2][1])
                  - M3[0][1] * (M3[1][0] * M3[2][2] - M3[1][2] * M3[2][0])
                  + M3[0][2] * (M3[1][0] * M3[2][1] - M3[1][1] * M3[2][0]);

    return {det_M1 / det_M, det_M2 / det_M, det_M3 / det_M};
}

// 动力学函数（【核心调整】修正alpha误差方向+优化积分条件）
std::vector<double> dyn_fun(double t, const std::vector<double>& state, const Params& params) {
    double alpha = state[0];    double dalpha = state[1];
    double beta = state[2];     double dbeta = state[3];
    double phi = state[4];      double dphi = state[5];
    double int_a = state[6];    double int_b = state[7];
    double alpha_des = state[8];double beta_des = state[9];
    double phi_int = state[10]; double delta_delayed = state[11];

    // 1. 角速度/角度软饱和（保持不变）
    double alpha_bound_dist = params.alpha_max - std::abs(alpha);
    double beta_bound_dist = params.beta_max - std::abs(beta);
    double d_alpha_max = (alpha_bound_dist >= params.alpha_max * (1.0 - params.bound_ratio)) 
                        ? params.d_alpha_base * alpha_bound_dist / (params.alpha_max * (1.0 - params.bound_ratio))
                        : params.d_alpha_base * 0.1;
    double d_beta_max = (beta_bound_dist >= params.beta_max * (1.0 - params.bound_ratio))
                        ? params.d_beta_base * beta_bound_dist / (params.beta_max * (1.0 - params.bound_ratio))
                        : params.d_beta_base * 0.1;
    dalpha = soft_saturate(dalpha, d_alpha_max, d_alpha_max * 0.7);
    dbeta = soft_saturate(dbeta, d_beta_max, d_beta_max * 0.7);
    dphi = soft_saturate(dphi, params.d_phi_max, params.d_phi_max * 0.7);
    alpha = soft_saturate(alpha, params.alpha_max, params.alpha_max * 0.7);
    beta = soft_saturate(beta, params.beta_max, params.beta_max * 0.7);
    phi = soft_saturate(phi, params.phi_max, params.phi_max * 0.7);

    // 2. 目标角计算（保持不变）
    double phi_error = phi - params.phi_target;
    if (std::abs(phi_error) < params.phi_error_dead) phi_error = 0.0;
    double phi_abs = std::abs(phi);
    double coupling_coeff = (phi_abs < params.phi_thresh) 
                          ? params.coupling_min 
                          : std::min(params.coupling_min + (phi_abs - params.phi_thresh) * params.coupling_gain, 0.15);
    double alpha_des_raw = -coupling_coeff * phi_error;
    double beta_des_raw = -coupling_coeff * 0.8 * phi_error;
    alpha_des_raw = soft_saturate(alpha_des_raw, params.alpha_max, params.alpha_max * 0.7);
    beta_des_raw = soft_saturate(beta_des_raw, params.beta_max, params.beta_max * 0.7);

    // 3. 目标角平滑（alpha专属更快的平滑速度）
    double d_alpha_des = (alpha_des_raw - alpha_des) / params.tau_smooth_alpha; // 用alpha专属tau
    double d_beta_des = (beta_des_raw - beta_des) / params.tau_smooth_beta;     // beta保持不变

    // 【核心调整5】修正alpha误差方向（Matlab标准：目标值 - 实际值）
    double alpha_error =  alpha_des - alpha;  // 原：alpha - alpha_des（方向反了）
    double beta_error = beta_des - beta;     // beta保持正确方向（之前已收敛）

    // 【调整6】优化alpha积分条件（放宽误差阈值，增强积分作用）
    double d_int_a = 0.0, d_int_b = 0.0;
    // alpha：误差阈值从0.001→0.008，允许更小的误差累积
    if (std::abs(alpha_error) > 0.008 && !((alpha_error > 0.0 && int_a > 0.0) || (alpha_error < 0.0 && int_a < 0.0))) {
        d_int_a = alpha_error * 1.1; // 积分增益×1.1，增强作用
    }
    // beta保持原条件（已收敛）
    if (std::abs(beta_error) > 0.001 && !((beta_error > 0.0 && int_b > 0.0) || (beta_error < 0.0 && int_b < 0.0))) {
        d_int_b = beta_error;
    }
    // 积分项幅值限幅（保持不变）
    int_a = soft_saturate(int_a, -params.integral_max, params.integral_max);
    int_b = soft_saturate(int_b, -params.integral_max, params.integral_max);

    // 4. Phi PID积分项（保持不变）
    double d_phi_int = 0.0;
    if (std::abs(phi_error) <= params.phi_int_thresh && phi_error != 0.0) {
        if (!((phi_error > 0.0 && phi_int > 0.0) || (phi_error < 0.0 && phi_int < 0.0))) {
            d_phi_int = phi_error * 0.6;
        }
    }
    phi_int = soft_saturate(phi_int, -params.phi_int_max, params.phi_int_max);

    // 5. 计算原始舵偏角（保持不变，alpha PID参数已在Params中增强）
    double phi_p = params.Kp_phi * phi_error;
    double phi_d = params.Kd_phi * dphi;
    double phi_i = params.Ki_phi * phi_int;
    double alpha_pid = params.Kp_alpha * alpha_error + params.Kd_alpha * dalpha + params.Ki_alpha * int_a;
    double beta_pid = params.Kp_beta * beta_error + params.Kd_beta * dbeta + params.Ki_beta * int_b;
    
    // 阻尼项（保持不变）
    double alpha_damping = params.alpha_damping_gain * std::pow(std::abs(alpha) / params.alpha_max, 3.0) * std::copysign(1.0, alpha) * dalpha;
    double beta_damping = params.beta_damping_gain * std::pow(std::abs(beta) / params.beta_max, 3.0) * std::copysign(1.0, beta) * dbeta;
    alpha_pid += alpha_damping;
    beta_pid += beta_damping;

    double delta_raw = phi_p + phi_d + phi_i + alpha_pid + beta_pid;
    delta_raw = soft_saturate(delta_raw, params.delta_max, params.delta_soft);

    // 6. 舵偏角延迟+随机波动（保持不变）
    double ddelta_delayed = (delta_raw - delta_delayed) / params.tau_delta;
    rng.seed(static_cast<unsigned int>(sum(t * 1000)));
    double phi_error_abs = std::abs(phi_error);
    double noise_scale = std::min(1.0, phi_error_abs / params.phi_int_thresh);
    double noise = params.delta_noise_amp * noise_scale * norm_dist(rng);
    double delta_actual = delta_delayed + noise;

    // 7. 计算角加速度（使用修正后的矢径符号）
    std::vector<double> dd = calc_angular_acceleration(
        alpha, beta, phi, dalpha, dbeta, dphi, delta_actual, params
    );
    double ddalpha = dd[0];
    double ddbeta = dd[1];
    double ddphi = dd[2];

    // 状态导数返回（保持不变）
    return {
        dalpha, ddalpha, dbeta, ddbeta, dphi, ddphi,
        d_int_a, d_int_b, d_alpha_des, d_beta_des, d_phi_int,
        ddelta_delayed
    };
}

// RK4积分器（保持不变）
void rk4_integrate(
    double t_start, double t_end, double dt,
    const std::vector<double>& state0,
    const Params& params,
    std::vector<double>& t_history,
    std::vector<std::vector<double>>& state_history
) {
    double t = t_start;
    std::vector<double> state = state0;
    t_history.push_back(t);
    state_history.push_back(state);

    while (t < t_end) {
        double step = std::min(dt, t_end - t);

        std::vector<double> k1 = dyn_fun(t, state, params);
        std::vector<double> state_k2(12);
        for (int i = 0; i < 12; ++i) state_k2[i] = state[i] + 0.5 * step * k1[i];
        std::vector<double> k2 = dyn_fun(t + 0.5 * step, state_k2, params);

        std::vector<double> state_k3(12);
        for (int i = 0; i < 12; ++i) state_k3[i] = state[i] + 0.5 * step * k2[i];
        std::vector<double> k3 = dyn_fun(t + 0.5 * step, state_k3, params);

        std::vector<double> state_k4(12);
        for (int i = 0; i < 12; ++i) state_k4[i] = state[i] + step * k3[i];
        std::vector<double> k4 = dyn_fun(t + step, state_k4, params);

        for (int i = 0; i < 12; ++i) {
            state[i] += step * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) / 6.0;
        }

        t += step;
        t_history.push_back(t);
        state_history.push_back(state);
    }
}

// 保存结果（保持不变）
void save_results(
    const std::vector<double>& t_history,
    const std::vector<std::vector<double>>& state_history,
    const Params& params
) {
    std::ofstream file("simulation_results_alpha_converge.csv");
    if (!file.is_open()) {
        std::cerr << "无法打开结果文件！" << std::endl;
        return;
    }

    file << "时间(s),alpha(rad),dalpha(rad/s),beta(rad),dbeta(rad/s),phi(rad),dphi(rad/s),"
         << "int_a,int_b,alpha_des(rad),beta_des(rad),phi_int,delta_delayed(rad),"
         << "phi(deg),alpha(deg),beta(deg),alpha_des(deg),beta_des(deg),delta_actual(deg)" << std::endl;

    for (size_t i = 0; i < t_history.size(); ++i) {
        double t = t_history[i];
        const auto& s = state_history[i];
        rng.seed(static_cast<unsigned int>(sum(t * 1000)));
        double phi_error = s[4] - params.phi_target;
        double phi_error_abs = std::abs(phi_error);
        double noise_scale = std::min(1.0, phi_error_abs / params.phi_int_thresh);
        double noise = params.delta_noise_amp * noise_scale * norm_dist(rng);
        double delta_actual = s[11] + noise;

        double phi_deg = s[4] * 180.0 / M_PI;
        double alpha_deg = s[0] * 180.0 / M_PI;
        double beta_deg = s[2] * 180.0 / M_PI;
        double alpha_des_deg = s[8] * 180.0 / M_PI;
        double beta_des_deg = s[9] * 180.0 / M_PI;
        double delta_actual_deg = delta_actual * 180.0 / M_PI;

        file << t << ","
             << s[0] << "," << s[1] << "," << s[2] << "," << s[3] << ","
             << s[4] << "," << s[5] << "," << s[6] << "," << s[7] << ","
             << s[8] << "," << s[9] << "," << s[10] << "," << s[11] << ","
             << phi_deg << "," << alpha_deg << "," << beta_deg << ","
             << alpha_des_deg << "," << beta_des_deg << ","
             << delta_actual_deg << std::endl;
    }

    file.close();
    std::cout << "alpha收敛版仿真结果已保存到 simulation_results_alpha_converge.csv" << std::endl;
}

int main() {
    // 初始化随机数（保持不变）
    rng.seed(std::mt19937::default_seed);

    // 参数配置（保持不变）
    Params params;
    params.phi_target = 0.0;

    // 初始状态（保持不变：phi=25°）
    std::vector<double> state0 = {
        0.0, 0.0, 0.0, 0.0,                  // alpha, dalpha, beta, dbeta
        15.0 * M_PI / 180.0, 0.0,            // phi(25°), dphi
        0.0, 0.0, 0.0, 0.0,                  // int_a, int_b, alpha_des, beta_des
        0.0, 0.0                             // phi_int, delta_delayed
    };

    // 仿真配置（保持不变）
    double t_start = 0.0;
    double t_end = 20.0;
    double dt = 0.01;

    // 运行仿真
    std::vector<double> t_history;
    std::vector<std::vector<double>> state_history;
    std::cout << "开始alpha收敛版仿真..." << std::endl;
    auto start_time = std::chrono::high_resolution_clock::now();
    rk4_integrate(t_start, t_end, dt, state0, params, t_history, state_history);
    auto end_time = std::chrono::high_resolution_clock::now();
    double duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() / 1000.0;
    std::cout << "仿真完成！耗时: " << duration << " 秒" << std::endl;

    // 保存结果
    save_results(t_history, state_history, params);

    return 0;
}