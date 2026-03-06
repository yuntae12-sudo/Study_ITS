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
    cout << "[TRFE] M1_Continuous(a_t_comp): " << m1_state.a_t_comp 
         << " | M1_Discrete: " << method_1_mu 
         << " | M2_Prob: " << method_2_mu 
         << " | Final Mu: " << mu_final << endl;
         
    return mu_final;
}

double Method_1() {
    // [수정 1] 초기화 안전장치: 극저속 및 정지 상태에서 Icy로 오인하는 것 방지
    if (ego_state.vx < 3.0) {
        return 0.756; // 기본값을 Dry 노면으로 설정
    }

    // 1. 전륜/후륜 횡가속도 계산
    double a_y_front = ego_state.ay + Lf * ego_state.yaw_accel;
    double a_y_rear = ego_state.ay - Lr * ego_state.yaw_accel;

    // 2. 총 가속도 계산 (g 단위 변환)
    m1_state.a_t_front = sqrt(pow(a_y_front, 2) + pow(ego_state.ax, 2)) / g;
    m1_state.a_t_rear  = sqrt(pow(a_y_rear, 2) + pow(ego_state.ax, 2)) / g;
    m1_state.a_t_cg    = sqrt(pow(ego_state.ay, 2) + pow(ego_state.ax, 2)) / g;

    // 3. 최댓값 산출
    m1_state.a_t_max = max({m1_state.a_t_front, m1_state.a_t_rear, m1_state.a_t_cg});

    // 4. Rate Limiter 적용 (내려가는 기울기만 제한)
    double delta = 1.0; // 1 g/sec 제한
    static double prev_a_t_comp = 0.0;

    if(m1_state.a_t_max > prev_a_t_comp) {
        m1_state.a_t_comp = m1_state.a_t_max;
    } else {
        m1_state.a_t_comp = prev_a_t_comp - min(prev_a_t_comp - m1_state.a_t_max, delta * trfe_dt);
    }
    prev_a_t_comp = m1_state.a_t_comp;

    // 5. 마찰계수 추정
    static double mu_m1 = 0.756; // [수정 2] 초기값을 1.0에서 시나리오 기준인 0.756으로 변경
    if (m1_state.a_t_comp >= 0.7) {
        mu_m1 = 0.756;
    } else if (m1_state.a_t_comp >= 0.4 && m1_state.a_t_comp <= 0.6) {
        mu_m1 = 0.615;
    } else if (m1_state.a_t_comp <= 0.3) {
        mu_m1 = 0.133; // [수정 3] 시나리오 값(0.133)과 완벽히 일치시킴
    }
    // 그 사이값일 경우 이전 마찰계수를 유지
    
    return mu_m1;
}

double Method_2() {
    // 극저속 상황 예외 처리 (속도가 3 m/s 이하일 때는 기본값 0.756 반환)
    if (ego_state.vx < 3.0) {
        return 0.756; // [수정 4] 0.85에서 시나리오 기준값인 0.756으로 통일
    }

    // 1. 횡가속도 기반 요레이트
    double gamma_ay = ego_state.ay / ego_state.vx;

    // 2. 정상상태 요레이트 연산 (Bicycle model)
    double understeer_gradient = (m * (Lf * Cf - Lr * Cr)) / (2 * Cf * Cr * pow(Lf + Lr, 2));
    double gamma_ss = (ego_state.vx / ((Lf + Lr) * (1 + understeer_gradient * pow(ego_state.vx, 2)))) * ego_state.steer_angle;

    // 3. 마찰계수 별 기준 요레이트
    // [수정 5] 3-State 모델로 확장 (Dry, Wet, Icy)
    double sign_gamma = (gamma_ss >= 0) ? 1.0 : -1.0;
    double gamma_dry = sign_gamma * min(fabs(gamma_ss), (0.756 * g) / ego_state.vx);
    double gamma_wet = sign_gamma * min(fabs(gamma_ss), (0.615 * g) / ego_state.vx);
    double gamma_icy = sign_gamma * min(fabs(gamma_ss), (0.133 * g) / ego_state.vx);

    // 4. Bayesian 확률 연산 
    double S = 0.15; // Error covariance
    double p_dry = exp(-0.5 * pow(gamma_ay - gamma_dry, 2) / S) / sqrt(2 * M_PI * S);
    double p_wet = exp(-0.5 * pow(gamma_ay - gamma_wet, 2) / S) / sqrt(2 * M_PI * S);
    double p_icy = exp(-0.5 * pow(gamma_ay - gamma_icy, 2) / S) / sqrt(2 * M_PI * S);

    // 가중치 소실(Degeneracy) 방지를 위해 최소 확률(0.01) 보장
    double current_w_dry = max(m2_state.w_high, 0.01);
    double current_w_wet = max(m2_state.w_mid, 0.01);
    double current_w_icy = max(m2_state.w_low, 0.01);

    double w_dry_unnorm = p_dry * current_w_dry;
    double w_wet_unnorm = p_wet * current_w_wet;
    double w_icy_unnorm = p_icy * current_w_icy;
    double sum_w = w_dry_unnorm + w_wet_unnorm + w_icy_unnorm;

    if (sum_w > 0) {
        m2_state.w_high = w_dry_unnorm / sum_w;
        m2_state.w_mid = w_wet_unnorm / sum_w;
        m2_state.w_low = w_icy_unnorm / sum_w;
    }

    // 5. 가중치 합산 최종 추정
    // [수정 6] 3가지 상태의 가중치 합산으로 최종 마찰 계수 도출
    double mu_m2 = (0.756 * m2_state.w_high) + (0.615 * m2_state.w_mid) + (0.133 * m2_state.w_low);
    return mu_m2;
}