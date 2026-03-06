#include "Planning/Planning.hpp"

void DwaProcess (vector<Candidate>& cand_vec, const morai_msgs::EgoVehicleStatus::ConstPtr& ego, const EgoSpec& spec, const EgoPose& ego_pose, const vector<GlobalPath>& global_path_vec) {
    EgoStatus ego_status;
    MinMax minmax;
    VectorSpace space = GetVelocityVectorSpace(ego, spec, ego_status);
    GenerateLocalPath(cand_vec, ego, spec, ego_pose, global_path_vec);
    EvaluateLocalPath(cand_vec, ego_pose, global_path_vec, minmax, spec);
    Candidate best_cand = SelectBestLocalPath(cand_vec);
    if (best_cand.total_score > -900.0) {
        cout << "======================================================" << endl;
        cout << "[DWA Result] Final Selection" << endl;
        cout << " - Speed      : " << best_cand.v * 3.6 << " km/h" << endl;
        cout << " - Steer Angle: " << Rad2Deg(best_cand.steer_angle) << " deg" << endl;
        cout << " - Total Score: " << best_cand.total_score << endl;
        cout << "------------------------------------------------------" << endl;
        cout << " [Raw Scores]" << endl;
        cout << " - Heading    : " << best_cand.score_heading << endl;
        cout << " - Distance   : " << best_cand.score_dist << endl;
        cout << " - Velocity   : " << best_cand.score_velocity << endl;
        cout << "======================================================" << endl << endl;
    }
}

VectorSpace GetVelocityVectorSpace (const morai_msgs::EgoVehicleStatus::ConstPtr& ego, const EgoSpec& spec, EgoStatus& ego_status) {
    // 1. 상태 업테이트 (ROS 메세지 -> EgoStatus)
    ego_status.v = ego->velocity.x;

    // 2. 각속도 계산 (Bicycle Model)
    double steer_angle_rad = Deg2Rad(ego->wheel_angle);
    if(fabs(ego_status.v) < 0.01) {
        ego_status.w = 0.0;
    } else {
        ego_status.w = (ego_status.v / wheel_base) * tan(steer_angle_rad);
    }

    // Step 1: Vc (Dynamic Limits - 현재 가감속 능력)
    double min_vd = ego_status.v - spec.decel_lin * spec.dt;
    double max_vd = ego_status.v + spec.accel_lin * spec.dt;

    double min_wd = ego_status.w - spec.decel_ang * spec.dt;
    double max_wd = ego_status.w + spec.accel_ang * spec.dt;

    VectorSpace result_space;

    result_space.min_v = max(0.0, min_vd);
    result_space.max_v = min(spec.max_speed, max_vd);
    result_space.min_w = max(-spec.max_yaw_rate, min_wd);
    result_space.max_w = min(spec.max_yaw_rate, max_wd);

    return result_space;
}

void GenerateLocalPath (vector<Candidate>& cand_vec, const morai_msgs::EgoVehicleStatus::ConstPtr& ego, const EgoSpec& spec, const EgoPose& ego_pose, const vector<GlobalPath>& global_path_vec) {
    cand_vec.clear();
    EgoStatus curr_status;
    // curr_status.v = ego->velocity.x;

    VectorSpace space = GetVelocityVectorSpace(ego, spec, curr_status);

    int num_steer_samples = 21;
    double max_steer_angle = 0.698; // rad
    double steer_step = (num_steer_samples > 1) ? (max_steer_angle * 2) / (num_steer_samples - 1) : 0;

    // 예측 시간 설정
    double tp_min = 0.5, tp_max = 2.0, tp_step = 0.5;
    // double Ld = max(2.0, curr_status.v * 0.5); 
    // double base_steer = GetStanleyAngle(global_path_vec, ego_pose, Ld);

    // Loop 1: 속도
    for(double v = space.min_v; v <= space.max_v; v += 0.2) {

        double Ld = max(2.0, curr_status.v * 0.5); 
        double base_steer = GetStanleyAngle(global_path_vec, ego_pose, Ld);
        
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

                if(v < space.min_v || v > space.max_v || candidate_w < space.min_w || candidate_w > space.max_w) continue;

                cand.v = v;
                cand.tp = tp;
                cand.steer_angle = candidate_steer;
                cand.angular_vel = candidate_w;

                cand_vec.push_back(cand);
            }
        }
    }
}

void EvaluateLocalPath (vector<Candidate>& cand_vec, const EgoPose& ego_pose, const vector<GlobalPath>& global_path_vec, MinMax& minmax, const EgoSpec& spec) {
    minmax = MinMax{}; // 초기화

    for(Candidate& cand : cand_vec) {
        // 1. 시뮬레이터 된 궤적 생성
        // 단순 키네마틱 모델을 이용한 미래 위치 예측 (평균 헤딩 사용)
        double avg_yaw = NormalizeAngle(ego_pose.curr_yaw + (cand.angular_vel * cand.tp) * 0.5);
        cand.future_yaw = NormalizeAngle(ego_pose.curr_yaw + cand.angular_vel * cand.tp);
        cand.future_e = ego_pose.curr_e + cand.v * cos(avg_yaw) * cand.tp;
        cand.future_n = ego_pose.curr_n + cand.v * sin(avg_yaw) * cand.tp;

        // 평가함수의 세부 항목 평가하는 함수
        cand.score_heading = GetHeadingScore(global_path_vec, ego_pose, cand, spec);
        cand.score_dist = GetDistScore(global_path_vec, ego_pose, cand);
        cand.score_velocity = GetVelocityScore(cand);
        
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
    m = MinMax{};

    for(Candidate& cand : cand_vec) {
        double future_yaw = cand.future_yaw;

        cand.score_heading = GetHeadingScore(global_path_vec, ego_pose, cand, spec);
        cand.score_dist = GetDistScore(global_path_vec, ego_pose, cand);
        cand.score_velocity = GetVelocityScore(cand);

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
    double Ld = max(1.0, min(cand.v * spec.dt, 10.0)); // 1.0 이상 10.0 이하로 Ld 설정 (속도 기반으로 조정)
    int target_idx = FindWaypointIdx(global_path_vec, ego_pose, Ld);
    double de = global_path_vec[target_idx + 1].e - global_path_vec[target_idx].e;
    double dn = global_path_vec[target_idx + 1].n - global_path_vec[target_idx].n;

    double path_yaw = atan2(dn, de);
    double theta = fabs(NormalizeAngle(path_yaw - cand.future_yaw));
    return pi - theta;
}

double GetDistScore (const vector<GlobalPath>& global_path_vec, const EgoPose& ego_pose, const Candidate& cand) {
    double min_dist = 1e9;
    int closest_idx = FindClosestIdx(global_path_vec, ego_pose);
    
    int start_idx = closest_idx;
    int end_idx = min((int)global_path_vec.size(), start_idx + 100);

    for(int i = start_idx; i < end_idx; i++) {
        double dist = GetDist(cand.future_e - global_path_vec[i].e, cand.future_n - global_path_vec[i].n);
        if(dist < min_dist) min_dist = dist;
    }
    return -min_dist; // 거리가 가까울수록 점수가 높도록 음수로 반환
}

double GetVelocityScore (const Candidate& cand) {
    double target_vel = KPH2MPS(40.0);
    return -fabs(cand.v - target_vel);
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
    int sign = (cross_product > 0) ? 1 : -1;

    double de = global_path_vec[target_idx].e - ego_pose.curr_e;
    double dn = global_path_vec[target_idx].n - ego_pose.curr_n;
    double magnitude = GetDist(de, dn);

    return sign * magnitude;
}

double GetYawErr (const vector<GlobalPath>& global_path_vec, const EgoPose& ego_pose, const double Ld) {
    int target_idx = FindWaypointIdx(global_path_vec, ego_pose, Ld);
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
    double yaw_err = GetYawErr(global_path_vec, ego_pose, Ld);

    double steer_angle = yaw_err + atan2(dist_err, Ld);

    return steer_angle;
}