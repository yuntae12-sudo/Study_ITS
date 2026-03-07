#include "Planning/Planning.hpp"

// VehicleState ego_state;
// Method1State m1_state;
// Method2State m2_state; // 내부에 w_dry, w_wet, w_icy가 0.333으로 초기화되어 있어야 함

double TRFEProcess() {
    double method_1_mu = Method_1();
    double method_2_mu = Method_2();
    double mu_final = max(method_1_mu, method_2_mu);
    
    // 터미널 출력
    cout << fixed << setprecision(3);
    cout << "[TRFE] 보정 총 가속도: " << m1_state.a_t_comp 
         << " | Method 1 결과: " << method_1_mu 
         << " | Method 2 결과: " << method_2_mu 
         << " | Final mu: " << mu_final << endl;
         
    return mu_final;
}

double Method_1() {
    if (ego_state.vx < 3.0) {
        return 0.765; // 정지 및 극저속 시 기본값 (고마찰 노면) 
    }

    // 1. 전륜/후륜 횡가속도 계산 (논문 식 4 준수) [cite: 139, 140]
    double a_y_front = ego_state.ay + Lf * ego_state.yaw_accel;
    double a_y_rear = ego_state.ay - Lr * ego_state.yaw_accel;

    // 2. 총 가속도 계산 (g 단위 변환) (논문 식 5 준수) [cite: 143, 144]
    m1_state.a_t_front = sqrt(pow(a_y_front, 2) + pow(ego_state.ax, 2)) / g;
    m1_state.a_t_rear  = sqrt(pow(a_y_rear, 2) + pow(ego_state.ax, 2)) / g;
    m1_state.a_t_cg    = sqrt(pow(ego_state.ay, 2) + pow(ego_state.ax, 2)) / g;

    // 3. 최댓값 산출 (논문 식 6 준수) [cite: 152]
    m1_state.a_t_max = max({m1_state.a_t_front, m1_state.a_t_rear, m1_state.a_t_cg});

    // 4. Rate Limiter 적용 (내려가는 기울기만 제한) (논문 식 7 준수) [cite: 159, 161, 162, 164]
    double delta = 1.0; // 1 g/sec 제한 [cite: 167]
    static double prev_a_t_comp = 0.0;

    if(m1_state.a_t_max > prev_a_t_comp) {
        m1_state.a_t_comp = m1_state.a_t_max;
    } else {
        m1_state.a_t_comp = prev_a_t_comp - min(prev_a_t_comp - m1_state.a_t_max, delta * trfe_dt);
    }
    prev_a_t_comp = m1_state.a_t_comp;

    // 5. 마찰계수 추정 (2-State: 0.85 or 0.4) [cite: 97, 170]
    static double mu_m1 = 0.765; 
    if (m1_state.a_t_comp >= 0.45) { // High Mu Threshold [cite: 170, 175]
        mu_m1 = 0.765;
    } else if (m1_state.a_t_comp <= 0.3) { // Low Mu Threshold [cite: 170, 176]
        mu_m1 = 0.3;
    }
    
    return mu_m1;
}

double Method_2() {
    if (ego_state.vx < 3.0) {
        return 0.765; 
    }

    double gamma_ay = ego_state.ay / ego_state.vx; // (논문 식 9) [cite: 342]

    double understeer_gradient = (m * (Lf * Cf - Lr * Cr)) / (2 * Cf * Cr * pow(Lf + Lr, 2));
    double gamma_ss = (ego_state.vx / ((Lf + Lr) * (1 + understeer_gradient * pow(ego_state.vx, 2)))) * ego_state.steer_angle;

    // 3. 마찰계수 별 기준 요레이트 (2-State: 0.85, 0.4) (논문 식 10) [cite: 346, 351]
    double sign_gamma = (gamma_ss >= 0) ? 1.0 : -1.0;
    double gamma_high = sign_gamma * min(fabs(gamma_ss), (0.85 * g) / ego_state.vx);
    double gamma_low  = sign_gamma * min(fabs(gamma_ss), (0.4 * g) / ego_state.vx);

    // 4. Bayesian 확률 연산 (2-State 가중치 업데이트)
    if(fabs(gamma_ss) > 0.1) {
        double S = 0.15; // Error covariance [cite: 365]
        // 가우시안 확률 밀도 함수 (논문 식 15) [cite: 360]
        double p_high = exp(-0.5 * pow(gamma_ay - gamma_high, 2) / S) / sqrt(2 * M_PI * S);
        double p_low  = exp(-0.5 * pow(gamma_ay - gamma_low, 2) / S) / sqrt(2 * M_PI * S);

        // 베이지안 가중치 업데이트 (논문 식 14) [cite: 372]
        double current_w_high = max(m2_state.w_high, 0.01);
        double current_w_low  = max(m2_state.w_low, 0.01);

        double w_high_unnorm = p_high * current_w_high;
        double w_low_unnorm  = p_low * current_w_low;
        double sum_w = w_high_unnorm + w_low_unnorm;

        if (sum_w > 0) {
            m2_state.w_high = w_high_unnorm / sum_w;
            m2_state.w_low  = w_low_unnorm / sum_w;
        }
    } else {
        // 직진 구간에서 고마찰 가중치 강제 회복 (Recovery)
        m2_state.w_high = min(m2_state.w_high + 0.1, 1.0);
        m2_state.w_low  = 1.0 - m2_state.w_high;
    }

    // 5. 가중치 합산 최종 추정 (논문 식 11) [cite: 353]
    double mu_m2 = (0.765 * m2_state.w_high) + (0.3 * m2_state.w_low);
    return mu_m2;
}