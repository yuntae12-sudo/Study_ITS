#include "Global/Global.hpp"
#include "Control/Control.hpp"
#include "Planning/Planning.hpp"

using namespace std;

// 전역 변수 선언
EgoSpec spec;
vector<Candidate> cand_vec;
EgoStatus ego_status;
EgoPose ego_pose;
VehicleState ego_state;
Method1State m1_state;
Method2State m2_state;

morai_msgs::EgoVehicleStatus::ConstPtr ego_msg;

ros::Subscriber sub_ego;
ros::Subscriber sub_gps;
ros::Subscriber sub_imu;

ros::Publisher control_pub;
ros::Publisher waypoints_pub;
ros::Publisher candidate_paths_pub;
ros::Publisher best_path_pub;
ros::Publisher dwa_visual_pub;

// Callback 함수 구현
void EgoCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& msg) {
    ego_msg = msg;
    ego_status.v = msg->velocity.x;
    // 필요하다면 w도 업데이트 (IMU나 차량 모델 기반)

    ego_state.vx = max(fabs(msg->velocity.x), 0.001);
    ego_state.ax = msg->acceleration.x;
    ego_state.ay = msg->acceleration.y;
    ego_state.steer_angle = Deg2Rad(msg->wheel_angle);

    static double prev_yaw_rate = 0.0;
    ego_state.yaw_rate = ego_state.vx * tan(ego_state.steer_angle) / (Lf + Lr);
    // ego_state.yaw_accel = (ego_state.yaw_rate - prev_yaw_rate) / trfe_dt;

    double raw_yaw_accel = (ego_state.yaw_rate - prev_yaw_rate) / trfe_dt;
    // 이전 yaw_accel 값을 80% 유지하고, 새로운 변화량을 20%만 반영 (가중치는 튜닝 가능)
    ego_state.yaw_accel = (ego_state.yaw_accel * 0.8) + (raw_yaw_accel * 0.2);
    
    prev_yaw_rate = ego_state.yaw_rate;
}

void GPSCallback(const morai_msgs::GPSMessage::ConstPtr& msg) {
    gps2Enu(msg, ego_pose);
}

void IMUCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    yawTf(msg, ego_pose);
}

bool PathCheck() {
    if(!LoadPath()) {
        cout << "경로 불러오기 실패 ..." << endl;
        return false;
    }
    return true;
}

void InitPubSub (ros::NodeHandle& n) {
    // Subscriber
    sub_ego = n.subscribe("/Ego_topic", 1, EgoCallback);
    sub_gps = n.subscribe("/gps", 1, GPSCallback);
    sub_imu = n.subscribe("/imu", 1, IMUCallback);

    control_pub = n.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 1);
    waypoints_pub = n.advertise<visualization_msgs::Marker>("/waypoints", 1);
    candidate_paths_pub = n.advertise<visualization_msgs::MarkerArray>("/cand_path", 1);
    best_path_pub = n.advertise<visualization_msgs::Marker>("/best_path", 1);
    dwa_visual_pub = n.advertise<visualization_msgs::MarkerArray>("/dwa_visual", 1);
}

void MainProcess (vector<Candidate>& cand_vec, const EgoStatus& ego_status, const EgoSpec& spec, morai_msgs::CtrlCmd& cmd_msg, const vector<GlobalPath>& global_path_vec, const EgoPose& ego_pose) {
    ControlInput ctrl_input;
    if (!ego_msg) return; // 데이터 수신 대기
    DwaProcess(cand_vec, ego_msg, spec, ego_pose, global_path_vec);
    ControlProcess(cand_vec, ego_status, ctrl_input, cmd_msg);
    TRFEProcess();

    // 3. 시각화 함수들 호출
    if (!cand_vec.empty()) {
        publishGlobalPath(global_path_vec);    // Global Path 그리기
        publishCandidatePaths(cand_vec);        // 모든 후보 경로 그리기

        // 점수가 가장 높은 Best Candidate 찾기
        auto best_it = std::max_element(cand_vec.begin(), cand_vec.end(), 
            [](const Candidate& a, const Candidate& b) {
                return a.total_score < b.total_score;
            });
        
        if (best_it != cand_vec.end()) {
            publishBestPath(*best_it);          // 최적 경로 초록색으로 그리기
        }

        // 동적 창 시뮬레이션 영역 그리기 (예시 범위: 0 ~ max_v)
        publishDynamicWindow(0.0, 10.0, -0.5, 0.5, 3.0); 
    }
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "main");
    ros::NodeHandle n;

    if(!PathCheck()) { return -1;}

    InitPubSub(n);

    ros::Rate loop_rate(50);

    morai_msgs::CtrlCmd cmd_msg; // command that will be filled each iteration

    while(ros::ok()) {
        ros::spinOnce();

        MainProcess(cand_vec, ego_status, spec, cmd_msg, global_path_vec, ego_pose);

        loop_rate.sleep();
    }
    return 0;
}