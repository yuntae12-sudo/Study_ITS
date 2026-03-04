#include "Visualizer/Visualizer.hpp"
#include <cmath>
#include <vector>

using namespace std;

extern EgoPose ego_pose; // main 등에서 정의된 현재 차량 포즈
extern ros::Publisher waypoints_pub;
extern ros::Publisher candidate_paths_pub;
extern ros::Publisher best_path_pub;
extern ros::Publisher dwa_visual_pub;

void generateTrajectoryPoints(const Candidate& cand, vector<geometry_msgs::Point>& pts) {
    double x = 0.0, y = 0.0, yaw = 0.0; 
    double dt = 0.1; 
    double total_time = (cand.tp > 0) ? cand.tp : 2.0; 

    for (double t = 0; t <= total_time; t += dt) {
        // Unicycle Model 기반 로컬 좌표 갱신
        x += cand.v * cos(yaw) * dt;
        y += cand.v * sin(yaw) * dt;
        yaw += cand.angular_vel * dt;

        geometry_msgs::Point p;
        p.x = x; p.y = y; p.z = 0.05;
        pts.push_back(p);
    }
}

// 1. Global Path 시각화 (base_link 기준으로 좌표 변환)
void publishGlobalPath(const vector<GlobalPath>& global_path) {
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "base_link"; // 변경
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "global_path";
    line_strip.id = 0;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.scale.x = 0.15;
    line_strip.color.r = 1.0; line_strip.color.g = 1.0; line_strip.color.b = 1.0; line_strip.color.a = 0.4;

    double yaw = ego_pose.curr_yaw;

    for (const auto& wp : global_path) {
        // Global(E, N) -> Local(base_link) 변환
        double dx = wp.e - ego_pose.curr_e;
        double dy = wp.n - ego_pose.curr_n;
        
        geometry_msgs::Point p;
        p.x = dx * cos(yaw) + dy * sin(yaw);
        p.y = -dx * sin(yaw) + dy * cos(yaw);
        p.z = 0.0;
        
        // 너무 먼 경로는 그리지 않음 (선택 사항)
        if (hypot(p.x, p.y) < 50.0) {
            line_strip.points.push_back(p);
        }
    }
    waypoints_pub.publish(line_strip);
}

// 2. Candidate Paths 시각화
void publishCandidatePaths(const vector<Candidate>& cand_vec) {
    visualization_msgs::MarkerArray marker_array;
    
    visualization_msgs::Marker del;
    del.header.frame_id = "base_link"; // 변경
    del.ns = "candidates";
    del.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(del);
    candidate_paths_pub.publish(marker_array);
    marker_array.markers.clear();

    for (size_t i = 0; i < cand_vec.size(); ++i) {
        visualization_msgs::Marker m;
        m.header.frame_id = "base_link"; // 변경
        m.header.stamp = ros::Time::now();
        m.ns = "candidates";
        m.id = (int)i;
        m.type = visualization_msgs::Marker::LINE_STRIP;
        m.action = visualization_msgs::Marker::ADD;
        m.scale.x = 0.03;
        m.color.r = 0.0; m.color.g = 0.5; m.color.b = 1.0; m.color.a = 0.3;

        generateTrajectoryPoints(cand_vec[i], m.points);
        if(m.points.size() >= 2) marker_array.markers.push_back(m);
    }
    candidate_paths_pub.publish(marker_array);
}

// 3. Best Path 시각화
void publishBestPath(const Candidate& best_cand) {
    visualization_msgs::Marker m;
    m.header.frame_id = "base_link"; // 변경
    m.header.stamp = ros::Time::now();
    m.ns = "best_path";
    m.id = 0;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = 0.2;
    m.color.r = 1.0; m.color.g = 0.2; m.color.b = 0.0; m.color.a = 1.0; // 눈에 띄게 주황/빨강 계열

    generateTrajectoryPoints(best_cand, m.points);
    for(auto& p : m.points) p.z = 0.1; // 겹침 방지를 위해 살짝 위로

    if(m.points.size() >= 2) best_path_pub.publish(m);
}

// 4. Dynamic Window 경계 시각화 (부채꼴/사다리꼴 형태)
void publishDynamicWindow(double v_min, double v_max, double w_min, double w_max, double predict_time) {
    visualization_msgs::MarkerArray dwa_array;
    
    // 이전 마커 삭제
    visualization_msgs::Marker del;
    del.header.frame_id = "base_link";
    del.ns = "dynamic_window";
    del.action = visualization_msgs::Marker::DELETEALL;
    dwa_array.markers.push_back(del);
    dwa_visual_pub.publish(dwa_array);
    dwa_array.markers.clear();

    visualization_msgs::Marker m;
    m.header.frame_id = "base_link";
    m.header.stamp = ros::Time::now();
    m.ns = "dynamic_window";
    m.id = 1;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = 0.15; // 선 두께
    m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.8; // 노란색

    // 1. 물리적 도달 거리 계산 (X축: 전방)
    double d_near = v_min * predict_time;
    double d_far  = v_max * predict_time;

    // 2. 각속도에 의한 회전각 계산 (Yaw: 좌우 벌어짐)
    // predict_time 동안 w의 속도로 회전했을 때 도달하는 각도
    double yaw_L = w_max * predict_time;
    double yaw_R = w_min * predict_time;

    // 3. 네 꼭짓점 정의 (삼각함수를 이용한 실제 위치 투영)
    geometry_msgs::Point p1, p2, p3, p4;
    
    // 근거리 좌(p1), 원거리 좌(p2), 원거리 우(p3), 근거리 우(p4)
    p1.x = d_near * cos(yaw_L); p1.y = d_near * sin(yaw_L); p1.z = 0.0;
    p2.x = d_far  * cos(yaw_L); p2.y = d_far  * sin(yaw_L); p2.z = 0.0;
    p3.x = d_far  * cos(yaw_R); p3.y = d_far  * sin(yaw_R); p3.z = 0.0;
    p4.x = d_near * cos(yaw_R); p4.y = d_near * sin(yaw_R); p4.z = 0.0;

    // 사각형 그리기
    m.points.push_back(p1);
    m.points.push_back(p2);
    m.points.push_back(p3);
    m.points.push_back(p4);
    m.points.push_back(p1); // 닫기

    dwa_array.markers.push_back(m);
    dwa_visual_pub.publish(dwa_array);
}