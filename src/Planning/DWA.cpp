#include "Planning.hpp"
//코너 파라미터
bool not_first=false;
double mu=0.0;
double mu_prev=0.5;
double filtered_mu = 0.765;     
double corner_mu_memory = 0.5;  
int low_mu_counter = 0;         
int high_mu_counter = 0;         


void DwaProcess (vector<Candidate>& cand_vec, const morai_msgs::EgoVehicleStatus::ConstPtr& ego, const EgoSpec& spec, const EgoPose& ego_pose, const vector<GlobalPath>& global_path_vec, double mu_final) {
    EgoStatus ego_status;
    MinMax minmax;
    VectorSpace space = GetVelocityVectorSpace(ego, spec, ego_status, mu_final);
    GenerateLocalPath(cand_vec, ego, spec, ego_pose, global_path_vec, mu_final);
    EvaluateLocalPath(cand_vec, ego_pose, global_path_vec, minmax, spec, mu_final,ego);
    Candidate best_cand = SelectBestLocalPath(cand_vec);
    // if (best_cand.total_score > -900.0) {
    //     cout << "======================================================" << endl;
    //     cout << "[DWA Result] Final Selection" << endl;
    //     cout << " - Speed      : " << best_cand.v * 3.6 << " km/h" << endl;
    //     cout << " - Steer Angle: " << Rad2Deg(best_cand.steer_angle) << " deg" << endl;
    //     cout << " - Total Score: " << best_cand.total_score << endl;
    //     cout << " - Tp: "<<best_cand.tp<<endl;
    //     cout << "------------------------------------------------------" << endl;
    //     cout << " [Raw Scores]" << endl;
    //     cout << " - Heading    : " << best_cand.score_heading << endl;
    //     cout << " - Distance   : " << best_cand.score_dist << endl;
    //     cout << " - Velocity   : " << best_cand.score_velocity << endl;
    //     cout << "======================================================" << endl << endl;
    // }
}

VectorSpace GetVelocityVectorSpace (const morai_msgs::EgoVehicleStatus::ConstPtr& ego, const EgoSpec& spec, EgoStatus& ego_status, double mu_final) {
    // 1. 상태 업테이트 (ROS 메세지 -> EgoStatus)
    ego_status.v = ego->velocity.x;

    // 2. 각속도 계산 (Bicycle Model)
    double steer_angle_rad = Deg2Rad(ego->wheel_angle);
    if(fabs(ego_status.v) < 0.01) {
        ego_status.w = 0.0;
    } else {
        ego_status.w = (ego_status.v / wheel_base) * tan(steer_angle_rad);
    }
    double mu_ratio=mu_final/0.765;
    if (mu_ratio > 1.0) mu_ratio = 1.0;
    if (mu_ratio < 0.1) mu_ratio = 0.1;
    double k_safe_accel = 0.6; 
    double k_safe_decel = 0.9;
    double dynamic_accel_lin = spec.accel_lin * mu_ratio * k_safe_accel;
    double dynamic_decel_lin = spec.decel_lin * mu_ratio * k_safe_decel;

    // Step 1: Vc (Dynamic Limits - 현재 가감속 능력)
    double min_vd = ego_status.v - dynamic_decel_lin * spec.dt;
    double max_vd = ego_status.v + dynamic_accel_lin * spec.dt;

    double min_wd = ego_status.w - spec.decel_ang * spec.dt;
    double max_wd = ego_status.w + spec.accel_ang * spec.dt;

    VectorSpace result_space;

    result_space.min_v = max(0.0, min_vd);
    result_space.max_v = min(spec.max_speed, max_vd);
    result_space.min_w = max(-spec.max_yaw_rate, min_wd);
    result_space.max_w = min(spec.max_yaw_rate, max_wd);

    return result_space;
}

void GenerateLocalPath (vector<Candidate>& cand_vec, const morai_msgs::EgoVehicleStatus::ConstPtr& ego, const EgoSpec& spec, const EgoPose& ego_pose, const vector<GlobalPath>& global_path_vec, double mu_final) {
    cand_vec.clear();
    EgoStatus curr_status;
    curr_status.v = ego->velocity.x; //[수정] 주석 살림

    VectorSpace space = GetVelocityVectorSpace(ego, spec, curr_status, mu_final);

    int num_steer_samples = 21;
    double max_steer_angle = 0.698; // rad
    double steer_step = (num_steer_samples > 1) ? (max_steer_angle * 2) / (num_steer_samples - 1) : 0;

    // 예측 시간 설정
    double tp_min = 0.5, tp_max = 1.5, tp_step = 0.5; 
    double Ld = max(3.0, curr_status.v * 1.2);  // [수정] ld 키움&통일
    double base_steer = GetStanleyAngle(global_path_vec, ego_pose, Ld); // [수정] for문 밖으로 꺼냄

    // Loop 1: 속도
    for(double v = space.min_v; v <= space.max_v; v += 0.2) {

        // Loop 2: 예측 시간 -> 논문 방식 인용하여 짧은 거리부터 먼 거리까지 다양한 길이의 경로 생성
        for(double tp = tp_min; tp <= tp_max; tp += tp_step) {
  
            // Loop 3: 조향 오프셋
            for(int i = 0; i < num_steer_samples; i++) {
                Candidate cand;
                cand.total_score = 0.0;

                double steer_offset = -max_steer_angle + (i * steer_step);
                double candidate_steer = base_steer + steer_offset;
                if(candidate_steer > 0.698) candidate_steer = 0.698;
                if(candidate_steer < -0.698) candidate_steer = -0.698;

                double candidate_w = (v / wheel_base) * tan(candidate_steer);

                // [수정] ?

                cand.v = v;
                cand.tp = tp;
                cand.steer_angle = candidate_steer;
                cand.angular_vel = candidate_w;

                cand_vec.push_back(cand);
            }
        }
    }
}

void EvaluateLocalPath (vector<Candidate>& cand_vec, const EgoPose& ego_pose, const vector<GlobalPath>& global_path_vec, MinMax& minmax, const EgoSpec& spec, double mu_final, const morai_msgs::EgoVehicleStatus::ConstPtr& ego) {
    minmax = MinMax{}; // 초기화

    for(Candidate& cand : cand_vec) {
        // 1. 시뮬레이터 된 궤적 생성
        // [수정] 미래 위치를 직선이 아닌 궤적 형태로 예측해냄
        double sim_dt = 0.1; // 시뮬레이션 시간 해상도 (Delta t)
        double temp_e = ego_pose.curr_e;
        double temp_n = ego_pose.curr_n;
        double temp_yaw = ego_pose.curr_yaw;

        for (double t = 0.0; t < cand.tp; t += sim_dt) {
            temp_e += cand.v * cos(temp_yaw) * sim_dt;
            temp_n += cand.v * sin(temp_yaw) * sim_dt;
            temp_yaw += cand.angular_vel * sim_dt; 
        }

        // tp 시간 이후의 최종 시뮬레이션 궤적 끝점
        cand.future_e = temp_e;
        cand.future_n = temp_n;
        cand.future_yaw = NormalizeAngle(temp_yaw);

        // 평가함수의 세부 항목 평가하는 함수
        cand.score_heading = GetHeadingScore(global_path_vec, ego_pose, cand, spec);
        cand.score_dist = GetDistScore(global_path_vec, ego_pose, cand);
        cand.score_velocity = GetVelocityScore(cand, ego_pose, global_path_vec, mu_final,ego); // [수정] 관련함수 입력 파라미터 변경
        
        // MinMax 갱신
        minmax.max_heading_score = max(minmax.max_heading_score, cand.score_heading);
        minmax.min_heading_score = min(minmax.min_heading_score, cand.score_heading);
        minmax.max_dist_score = max(minmax.max_dist_score, cand.score_dist);
        minmax.min_dist_score = min(minmax.min_dist_score, cand.score_dist);
        minmax.max_velocity_score = max(minmax.max_velocity_score, cand.score_velocity);
        minmax.min_velocity_score = min(minmax.min_velocity_score, cand.score_velocity);
    }

    // Pass 2: 정규화 및 총점 계산
    GetCandTotalScore(cand_vec, minmax);
}

void CalcObjectScore (vector<Candidate>& cand_vec, const EgoPose& ego_pose, const vector<GlobalPath>& global_path_vec, MinMax& m, const EgoSpec& spec) {
    // Min, Max 구조체 초기화
    // [수정] 이 함수는 main을 포함해 사용되지 않음, evaluatelocapath가 관련 기능 대체, 확인 필요
    m = MinMax{};

    for(Candidate& cand : cand_vec) {
        double future_yaw = cand.future_yaw;

        //cand.score_heading = GetHeadingScore(global_path_vec, ego_pose, cand, spec,ld);
        cand.score_dist = GetDistScore(global_path_vec, ego_pose, cand);
        //cand.score_velocity = GetVelocityScore(cand);

        // 정규화를 위한 최대 최소값 업데이트
        m.max_heading_score = max(m.max_heading_score, cand.score_heading);
        m.min_heading_score = min(m.min_heading_score, cand.score_heading);

        m.max_dist_score = max(m.max_dist_score, cand.score_dist);
        m.min_dist_score = min(m.min_dist_score, cand.score_dist);

        m.max_velocity_score = max(m.max_velocity_score, cand.score_velocity);
        m.min_velocity_score = min(m.min_velocity_score, cand.score_velocity);
    }
}

void GetCandTotalScore (vector<Candidate>& cand_vec, const MinMax& m){
    double epsilon = 1e-6; // 0으로 나누는거 방지하기 위한 값

    // 범위 설정하기
    double range_dist = m.max_dist_score - m.min_dist_score;
    double range_heading = m.max_heading_score - m.min_heading_score;
    double range_velocity = m.max_velocity_score - m.min_velocity_score;

    if(range_dist < epsilon) range_dist = 1.0;
    if(range_heading < epsilon) range_heading = 1.0;
    if(range_velocity < epsilon) range_velocity = 1.0;

    for(Candidate& cand : cand_vec) {
        if(cand.total_score <= -900.0) continue; // 유효하지 않은 후보는 건너뛰기
        // 1. 거리 점수 정규화
        double norm_dist_score = (cand.score_dist - m.min_dist_score) / range_dist;

        // 2. 헤딩 점수 정규화
        double norm_heading_score = (cand.score_heading - m.min_heading_score) / range_heading;

        // 3. 속도 점수 정규화
        double norm_velocity_score = (cand.score_velocity - m.min_velocity_score) / range_velocity;

        // 4. 총 점수 계산 (가중치 부여 가능)
        cand.total_score = (W_dist * norm_dist_score) + (W_heading * norm_heading_score) + (W_velocity * norm_velocity_score);
    }
}

Candidate SelectBestLocalPath (const vector<Candidate>& cand_vec) {
    Candidate best_cand;

    // 초기값 설정 (일단 가장 낮은 점수로 초기화)
    best_cand.total_score = -1e9;
    best_cand.v = 0.0;
    best_cand.steer_angle = 0.0;
    bool found_valid = false;

    for(const Candidate& cand : cand_vec) {
        if(cand.total_score <= -900.0) continue; // 유효하지 않은 후보는 건너뛰기
        if(cand.total_score > best_cand.total_score) {
            best_cand = cand;
            found_valid = true;
        }
    }

    if(!found_valid) {
        // 유효한 후보가 없는 경우, 안전한 기본 행동으로 초기화
        best_cand.v = 0.0;
        best_cand.steer_angle = 0.0;
        best_cand.total_score = -999.0; // 여전히 유효하지 않음을 나타냄

        // 디버깅용 로그 잘 되면 지워도 됨
        cout << "유효한 후보가 없습니다. 차량을 정지시킵니다..." << endl;
    }
    return best_cand;
}

double GetHeadingScore (const vector<GlobalPath>& global_path_vec, const EgoPose& ego_pose, Candidate& cand, const EgoSpec& spec) {
    // [수정] 정확도를 위해 헤딩에러를 현재 위치가 아닌 미래 예상 위치로 평가
    // 1. 후보 궤적의 미래 위치를 담을 임시 EgoPose 생성
    EgoPose future_pose;
    future_pose.curr_e = cand.future_e;
    future_pose.curr_n = cand.future_n;
    future_pose.curr_yaw = cand.future_yaw; 

    // 2. '미래 위치'에서 가장 가까운 경로점 인덱스 찾기
    int target_idx = FindClosestIdx(global_path_vec, future_pose);
    //int target_idx = FindWaypointIdx(global_path_vec, future_pose,Ld);
    
    // 인덱스 초과 방지
    if(target_idx >= (int)global_path_vec.size() - 1) {
        target_idx = max(0, (int)global_path_vec.size() - 2);
    }

    // 3. 미래 위치 발밑에 있는 도로의 진짜 방향(path_yaw) 계산
    double de = global_path_vec[target_idx + 1].e - global_path_vec[target_idx].e;
    double dn = global_path_vec[target_idx + 1].n - global_path_vec[target_idx].n;
    double path_yaw = atan2(dn, de);

    // 4. 미래의 내 차체 방향 vs 미래의 도로 방향 비교
    double theta = fabs(NormalizeAngle(path_yaw - cand.future_yaw));
    return pi - theta;
}

double GetDistScore (const vector<GlobalPath>& global_path_vec, const EgoPose& ego_pose, const Candidate& cand) {
    // [수정] 헤딩에러와 같은 이유로 미래 위치를 기준으로 계산
    EgoPose future_pose;
    future_pose.curr_e = cand.future_e;
    future_pose.curr_n = cand.future_n;
    future_pose.curr_yaw = cand.future_yaw; // DWA 시뮬레이션 끝점

    // 2. 미래 위치에서의 '횡오차(선까지의 거리)' 계산!
    // (직전에 고친 완벽한 공식을 그대로 재사용)
    double future_cte = fabs(GetDistErr(global_path_vec, future_pose));
    return -future_cte;
}

double GetVelocityScore (const Candidate& cand, const EgoPose& ego_pose, const vector<GlobalPath>& global_path_vec, double mu_final, const morai_msgs::EgoVehicleStatus::ConstPtr& ego) {
    //코너/직선에 따라 목표 속도 설정
    double max_vel = KPH2MPS(40.0); 
    double min_vel = KPH2MPS(17.0); 
    if(!not_first){
        mu=0.3;
        if(IsCorner(ego_pose,global_path_vec)){
            not_first=true;
        }
    }
    else {
        mu=mu_final;
    }
    double mu_ratio=mu/0.765;
    if (mu_ratio > 1.0) mu_ratio = 1.0;
    if (mu_ratio < 0.1) mu_ratio = 0.1;
    double mu_prev=FilterAndRememberMu(mu_final, ego_pose, ego);
    cout<<"mu_prev: "<<mu_prev<<endl;
    if(IsCorner(ego_pose,global_path_vec)){
        max_vel = KPH2MPS(40.0)*mu_prev; 
        min_vel = KPH2MPS(17.0)*mu_prev; 
    }
    else {
        max_vel = KPH2MPS(40.0)*mu_ratio; 
        min_vel = KPH2MPS(17.0)*mu_ratio; 
    }

    double base_steer = GetStanleyAngle(global_path_vec, ego_pose, 15.0); 
    

    // 도로의 꺾임 정도를 바탕으로 감속 비율 계산
    double steer_ratio = fabs(base_steer) / 0.698; 
    if(steer_ratio > 1.0) steer_ratio = 1.0; 

    double dynamic_target_vel = max_vel - steer_ratio * (max_vel - min_vel);

    return -fabs(cand.v - dynamic_target_vel);
}

bool IsCorner(const EgoPose& ego_pose, const vector<GlobalPath>& global_path_vec){
    int sight_idx = 50;
    int target_idx = FindWaypointIdx (global_path_vec, ego_pose, sight_idx);
    if(target_idx == -1) return 0.0;

    int next_idx = target_idx + 1;
    if(next_idx >= global_path_vec.size()) {
        next_idx = global_path_vec.size() - 1;
        target_idx = max(0, next_idx - 1);  
    }

    double de = global_path_vec[next_idx].e - global_path_vec[target_idx].e;
    double dn = global_path_vec[next_idx].n - global_path_vec[target_idx].n;

    if(GetDist(de, dn) < 1e-6) return 0.0;

    double path_yaw = atan2(dn, de);
    double yaw_err = fabs(NormalizeAngle(path_yaw - ego_pose.curr_yaw));
    return yaw_err>1.3;
}

double FilterAndRememberMu(double mu_final, const EgoPose& ego_pose, const morai_msgs::EgoVehicleStatus::ConstPtr& ego) {
    // 1. EMA (Exponential Moving Average) 필터 적용 (튀는 값 깎아내기)
    // alpha 값이 작을수록 이전 값을 강하게 유지
    double alpha = 0.05;
    filtered_mu = alpha * mu_final + (1.0 - alpha) * filtered_mu;
    //cout<<filtered_mu<<endl;

    // 2. 현재 차량이 코너를 돌고 있는지 확인 (횡력이 발생하는 구간)
    // 조향각이 약 5도(0.087 rad) 이상일 때를 코너로 간주
    bool in_corner = (fabs(ego->wheel_angle) > 17.0); // degree 기준일 경우

    // 3. 코너 구간에서만 마찰계수를 평가하고 메모리를 업데이트!
    if (in_corner) {
        // [저마찰 판단] 필터링된 값이 0.45 이하로 떨어지면 카운트 시작
        if (filtered_mu < 0.45) {
            low_mu_counter++;
            high_mu_counter = 0; // 반대 카운터는 초기화
           
            // 진짜 저마찰이 15틱(예: 50Hz 기준 0.3초) 이상 지속되면 메모리에 확정 저장!
            if (low_mu_counter > 100) {
                corner_mu_memory = filtered_mu; // 다음 코너를 위해 0.3으로 콱 박아둠
            }
        }
        // [고마찰 판단] 필터링된 값이 0.65 이상으로 올라가면 카운트 시작
        else if (filtered_mu > 0.55) {
            high_mu_counter++;
            low_mu_counter = 0;
           
            // 진짜 고마찰이 15틱 이상 지속되면 메모리 확정
            if (high_mu_counter > 200) {
                corner_mu_memory = filtered_mu;
            }
        }
        //cout<<"high: "<< high_mu_counter<<" low: "<<low_mu_counter<<endl;
    }
    else {
        // [직진 구간]
        // 코너가 끝나고 직진에 들어서면 카운터를 모두 초기화하고,
        // 이전에 확정 지어둔 corner_mu_memory 값을 절대 건드리지 않고 그대로 유지
        low_mu_counter = 0;
        high_mu_counter = 0;
    }


    return corner_mu_memory; // DWA의 GetVelocityScore와 VectorSpace로 넘어갈 최종 값
}

// ================================== Stanley 조향각 계산 ================================== //

double GetDistErr (const vector<GlobalPath>& global_path_vec, const EgoPose& ego_pose) {
    int target_idx = FindClosestIdx(global_path_vec, ego_pose);
    if(target_idx == -1) return 0.0;

    int p1_idx, p2_idx;

    if(target_idx >= (int)global_path_vec.size() - 1) {
        p1_idx = max(0, target_idx - 1);
        p2_idx = target_idx;
    } else {
        p1_idx = target_idx;
        p2_idx = target_idx + 1;
    }

    GlobalPath p_start = global_path_vec[p1_idx];
    GlobalPath p_end = global_path_vec[p2_idx];

    double vecA_e = p_end.e - p_start.e;
    double vecA_n = p_end.n - p_start.n;

    double vecB_e = ego_pose.curr_e - p_start.e;
    double vecB_n = ego_pose.curr_n - p_start.n;

    double cross_product = vecA_e * vecB_n - vecA_n * vecB_e;
    int sign = (cross_product > 0) ? -1 : 1; //부호 반대

    double de = global_path_vec[target_idx].e - ego_pose.curr_e;
    double dn = global_path_vec[target_idx].n - ego_pose.curr_n;
    double path_len = GetDist(vecA_e, vecA_n); // [수정] magnitude 계산 방식 변화
    if (path_len < 1e-6) return 0.0; // 0으로 나누기 방지

    // 2. 수직 거리(Magnitude) = |외적| / 선분 길이
    double magnitude = fabs(cross_product) / path_len;
    return sign * magnitude;
}

double GetYawErr (const vector<GlobalPath>& global_path_vec, const EgoPose& ego_pose) {
    int target_idx = FindClosestIdx(global_path_vec, ego_pose);
    if(target_idx == -1) return 0.0;

    int next_idx = target_idx + 1;
    if(next_idx >= global_path_vec.size()) {
        next_idx = global_path_vec.size() - 1;
        target_idx = max(0, next_idx - 1);  
    }

    double de = global_path_vec[next_idx].e - global_path_vec[target_idx].e;
    double dn = global_path_vec[next_idx].n - global_path_vec[target_idx].n;

    if(GetDist(de, dn) < 1e-6) return 0.0;

    double path_yaw = atan2(dn, de);
    double yaw_err = NormalizeAngle(path_yaw - ego_pose.curr_yaw);
    return yaw_err;
}

double GetStanleyAngle (const vector<GlobalPath>& global_path_vec, const EgoPose& ego_pose, const double Ld) {
    // 1. 횡오차 계산
    double dist_err = GetDistErr(global_path_vec, ego_pose);

    // 2. 헤딩 오차 계산
    double yaw_err = GetYawErr(global_path_vec, ego_pose);
    double k = 3.0; // [수정] k값 도입

    double steer_angle = yaw_err + atan2(k*dist_err, Ld+2.0);
    return steer_angle;
}