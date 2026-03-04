#include "Planning/Planning.hpp"

// VehicleState ego_state;
// Method1State m1_state;
// Method2State m2_state;

double TRFEProcess() {
    double method_1_mu = Method_1();
    double method_2_mu = Method_2();
    double mu_final = max(method_1_mu, method_2_mu);
    // 터미널 출력: a_t_comp가 곧 연속적인 마찰계수(Continuous Mu)를 의미합니다.
    cout << fixed << setprecision(3);
    cout << "[TRFE] M1_Continuous(a_t_comp): " << m1_state.a_t_comp 
         << " | M1_Discrete: " << method_1_mu 
         << " | M2_Prob: " << method_2_mu 
         << " | Final Mu: " << mu_final << endl;
    return mu_final;
}

double Method_1() {
// 1. 전륜/후륜 횡가속도 계산 [cite: 139, 140]
    double a_y_front = ego_state.ay + Lf * ego_state.yaw_accel;
    double a_y_rear = ego_state.ay - Lr * ego_state.yaw_accel;

    // 2. 총 가속도 계산 (g 단위 변환) [cite: 143, 144]
    m1_state.a_t_front = sqrt(pow(a_y_front, 2) + pow(ego_state.ax, 2)) / g;
    m1_state.a_t_rear  = sqrt(pow(a_y_rear, 2) + pow(ego_state.ax, 2)) / g;
    m1_state.a_t_cg    = sqrt(pow(ego_state.ay, 2) + pow(ego_state.ax, 2)) / g;

    // 3. 최댓값 산출 [cite: 152]
    m1_state.a_t_max = std::max({m1_state.a_t_front, m1_state.a_t_rear, m1_state.a_t_cg});

    // 4. Rate Limiter 적용 (내려가는 기울기만 제한) [cite: 123, 161, 163, 164]
    double delta = 1.0; // 1 g/sec 제한 [cite: 167]
    static double prev_a_t_comp = 0.0;

    if(m1_state.a_t_max > prev_a_t_comp) {
        m1_state.a_t_comp = m1_state.a_t_max;
    } else {
        m1_state.a_t_comp = prev_a_t_comp - std::min(prev_a_t_comp - m1_state.a_t_max, delta * trfe_dt);
    }
    prev_a_t_comp = m1_state.a_t_comp;

    // 5. 마찰계수 추정 [cite: 170]
    static double mu_m1 = 1.0; // 초기값 (고마찰)
    if (m1_state.a_t_comp >= 0.7) {
        mu_m1 = 0.756;
    } else if (m1_state.a_t_comp <= 0.3) {
        mu_m1 = 0.135;
    }
    // 그 사이값(0.5 ~ 0.7)일 경우 이전 마찰계수를 유지
    
    return mu_m1;
}

double Method_2() {
    // [수정 1] 극저속 상황 예외 처리 (속도가 3 m/s 이하일 때는 기본값 0.85 반환)
    if (ego_state.vx < 3.0) {
        return 0.85; 
    }

    // 1. 횡가속도 기반 요레이트 [cite: 342]
    double gamma_ay = ego_state.ay / ego_state.vx;

    // 2. 정상상태 요레이트 연산 (Bicycle model) [cite: 64]
    double understeer_gradient = (m * (Lf * Cf - Lr * Cr)) / (2 * Cf * Cr * pow(Lf + Lr, 2));
    double gamma_ss = (ego_state.vx / ((Lf + Lr) * (1 + understeer_gradient * pow(ego_state.vx, 2)))) * ego_state.steer_angle;

    // 3. 마찰계수 별 기준 요레이트 [cite: 346, 351]
    double sign_gamma = (gamma_ss >= 0) ? 1.0 : -1.0;
    double gamma_high = sign_gamma * std::min(std::abs(gamma_ss), (0.85 * g) / ego_state.vx);
    double gamma_low  = sign_gamma * std::min(std::abs(gamma_ss), (0.40 * g) / ego_state.vx);

    // 4. Bayesian 확률 연산 
    double S = 0.15; // Error covariance [cite: 365]
    double p_high = exp(-0.5 * pow(gamma_ay - gamma_high, 2) / S) / sqrt(2 * M_PI * S);
    double p_low  = exp(-0.5 * pow(gamma_ay - gamma_low, 2) / S) / sqrt(2 * M_PI * S);

    // [수정 2] 가중치 소실(Degeneracy) 방지를 위해 최소 확률(0.01) 보장
    double current_w_high = std::max(m2_state.w_high, 0.01);
    double current_w_low  = std::max(m2_state.w_low,  0.01);

    double w_high_unnorm = p_high * current_w_high;
    double w_low_unnorm  = p_low * current_w_low;
    double sum_w = w_high_unnorm + w_low_unnorm;

    if (sum_w > 0) {
        m2_state.w_high = w_high_unnorm / sum_w;
        m2_state.w_low  = w_low_unnorm / sum_w;
    }

    // 5. 가중치 합산 최종 추정 [cite: 353]
    double mu_m2 = 0.756 * m2_state.w_high + 0.133 * m2_state.w_low;
    return mu_m2;
}