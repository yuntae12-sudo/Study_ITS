#include "Visualizer/Visualizer.hpp"
#include <cmath>
#include <vector>

using namespace std;

extern EgoPose ego_pose; // main 등에서 정의된 현재 차량 포즈
extern ros::Publisher waypoints_pub;
extern ros::Publisher candidate_paths_pub;
extern ros::Publisher best_path_pub;
extern ros::Publisher dwa_visual_pub;

/**
 * @brief 예측 궤적 생성 보조 함수 (Candidate 구조체에 path_points가 없을 때 사용)
 */
void generateTrajectoryPoints(const Candidate& cand, vector<geometry_msgs::Point>& pts) {
    double cos_yaw = cos(ego_pose.curr_yaw);
    double sin_yaw = sin(ego_pose.curr_yaw);

    double x = 0.0, y = 0.0, yaw = 0.0; // 로컬 좌표계 시작
    double dt = 0.1; 
    double total_time = cand.tp > 0 ? cand.tp : 3.0; // 예측 시간만큼 시뮬레이션

    for (double t = 0; t <= total_time; t += dt) {
        // 차량 모델에 따른 위치 갱신 (간단한 Unicycle 모델 기준)
        x += cand.v * cos(yaw) * dt;
        y += cand.v * sin(yaw) * dt;
        yaw += cand.angular_vel * dt;

        geometry_msgs::Point p;
        p.x = x; p.y = y; p.z = 0.05;
        pts.push_back(p);
    }
}

// 1. Global Path 시각화 (기존과 동일)
void publishGlobalPath(const vector<GlobalPath>& global_path) {
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "map";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "global_path";
    line_strip.id = 0;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.scale.x = 0.15;
    line_strip.color.r = 1.0; line_strip.color.g = 1.0; line_strip.color.b = 1.0; line_strip.color.a = 0.6;

    double cos_yaw = cos(ego_pose.curr_yaw);
    double sin_yaw = sin(ego_pose.curr_yaw);

    for (const auto& wp : global_path) {
        double dx = wp.e - ego_pose.curr_e;
        double dy = wp.n - ego_pose.curr_n;
        geometry_msgs::Point p;
        p.x = dx * cos_yaw + dy * sin_yaw;
        p.y = -dx * sin_yaw + dy * cos_yaw;
        p.z = 1.0;
        line_strip.points.push_back(p);
    }
    waypoints_pub.publish(line_strip);
}

// 2. Candidate Paths 시각화 (내부에서 궤적 생성)
void publishCandidatePaths(const vector<Candidate>& cand_vec) {
    visualization_msgs::MarkerArray marker_array;
    
    visualization_msgs::Marker del;
    del.header.frame_id = "map";
    del.ns = "candidates";
    del.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(del);
    candidate_paths_pub.publish(marker_array);
    marker_array.markers.clear();

    for (size_t i = 0; i < cand_vec.size(); ++i) {
        visualization_msgs::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = ros::Time::now();
        m.ns = "candidates";
        m.id = (int)i;
        m.type = visualization_msgs::Marker::LINE_STRIP;
        m.action = visualization_msgs::Marker::ADD;
        m.scale.x = 0.02;
        m.color.r = 0.0; m.color.g = 0.8; m.color.b = 1.0; m.color.a = 0.2;

        generateTrajectoryPoints(cand_vec[i], m.points);
        if(m.points.size() >= 2) marker_array.markers.push_back(m);
    }
    candidate_paths_pub.publish(marker_array);
}

// 3. Best Path 시각화 (내부에서 궤적 생성)
void publishBestPath(const Candidate& best_cand) {
    visualization_msgs::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = ros::Time::now();
    m.ns = "best_path";
    m.id = 0;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = 0.2;
    m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0;

    generateTrajectoryPoints(best_cand, m.points);
    // Best Path는 조금 더 높게 표시
    for(auto& p : m.points) p.z = 0.1;

    if(m.points.size() >= 2) best_path_pub.publish(m);
}

// 4. Dynamic Window 경계 시각화
void publishDynamicWindow(double v_min, double v_max, double w_min, double w_max, double predict_time) {
    visualization_msgs::MarkerArray dwa_array;
    
    visualization_msgs::Marker del;
    del.header.frame_id = "map";
    del.ns = "dynamic_window";
    del.action = visualization_msgs::Marker::DELETEALL;
    dwa_array.markers.push_back(del);
    dwa_visual_pub.publish(dwa_array);
    dwa_array.markers.clear();

    // 동적 창의 외곽 사각형을 그리는 함수
    auto create_rect_line = [&](int id) {
        visualization_msgs::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = ros::Time::now();
        m.ns = "dynamic_window";
        m.id = id;
        m.type = visualization_msgs::Marker::LINE_STRIP;
        m.action = visualization_msgs::Marker::ADD;
        m.scale.x = 0.1; // 선 두께
        m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 0.8;

        // 사각형의 네 꼭짓점 정의 (차량 전방 기준 직사각형 영역)
        // X축은 속도(v), Y축은 각속도(w)의 범위를 공간적으로 투영
        double x_front = v_max * predict_time;
        double x_back  = v_min * predict_time;
        double y_left  = w_max * 2.0; // 가독성을 위해 상수를 곱해 폭 조정
        double y_right = w_min * 2.0;

        geometry_msgs::Point p1, p2, p3, p4;
        p1.x = x_back;  p1.y = y_left;  p1.z = -0.1;
        p2.x = x_front; p2.y = y_left;  p2.z = -0.1;
        p3.x = x_front; p3.y = y_right; p3.z = -0.1;
        p4.x = x_back;  p4.y = y_right; p4.z = -0.1;

        m.points.push_back(p1);
        m.points.push_back(p2);
        m.points.push_back(p3);
        m.points.push_back(p4);
        m.points.push_back(p1); // 사각형 닫기

        return m;
    };

    dwa_array.markers.push_back(create_rect_line(1));
    dwa_visual_pub.publish(dwa_array);
}