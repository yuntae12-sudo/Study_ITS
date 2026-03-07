#ifndef GLOBAL_HPP
#define GLOBAL_HPP

// ========================= 기본 헤더파일 ========================= //
#include <iostream>
#include <string>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <vector>
#include <map>
#include <stdio.h>

// ROS 관련 헤더
#include <ros/ros.h>
#include <ros/package.h>

// ========================= Message Type ========================= //
#include <morai_msgs/GPSMessage.h>
#include <sensor_msgs/Imu.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/CtrlCmd.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>



#define _USE_MATH_DEFINES

using namespace std;

// ========================= 전역변수 정의 ========================= //
constexpr double pi = M_PI;
const double a = 6378137.0;           // WGS-84 타원체 장축 반경 (단위: m)
const double e = 0.006694379991;      // WGS-84 타원체 이심률 제곱

// Reference Point
const double ref_lat = 37.238838359501933;
const double ref_lon = 126.772902206454901;
const double ref_alt = 0.0;

// 차량 제원
const double wheel_base = 3.0;

// 가중치 (거리, 헤딩, 속도)
const double W_dist = 1.0;
const double W_heading = 0.3;
const double W_velocity = 0.3;


// TRFE 파라미터
const double m = 1905.2;
const double Lf = 1.3;
const double Lr = 1.7;
const double g = 9.81;
const double Cf = 127092.0;
const double Cr = 97188.0;
const double trfe_dt = 0.02;


// ========================= 구조체 정의 ========================= //

struct GlobalPath { double e, n, u; };
struct EgoPose { double curr_e, curr_n, curr_u, curr_yaw; };

struct VectorSpace {
    double min_v;
    double max_v;
    double min_w;
    double max_w;
};

struct EgoStatus {
    double v, w;
};

struct Candidate {
    double score_heading;
    double score_dist;
    double score_velocity;
    double total_score;

    double v;
    double steer_angle;
    double angular_vel;

    double tp; // 예측 시간

    double future_e;
    double future_n;
    double future_yaw;
};

struct EgoSpec {
    double max_speed = 27.78;
    double max_yaw_rate = 4.78;

    double accel_lin = 3.75;
    double decel_lin = 8.82;
    double accel_ang = 0.64;
    double decel_ang = 1.50226;

    double dt = 1.0;
};

struct ControlInput {
    double accel_input;
    double brake_input;
    double steer_input;
};

struct MinMax {
    double max_dist_score = -1e9, min_dist_score = 1e9;
    double max_heading_score = -1e9, min_heading_score = 1e9;
    double max_velocity_score = -1e9, min_velocity_score = 1e9;
};

struct VehicleState {
    double vx = 0.001; 
    double ax = 0.0;
    double ay = 0.0;
    double yaw_rate = 0.0;
    double yaw_accel = 0.0;
    double steer_angle = 0.0;
};

struct Method1State {
    double a_t_front = 0.0;
    double a_t_rear = 0.0;
    double a_t_cg = 0.0;
    double a_t_max = 0.0;
    double a_t_comp = 0.0;
};

struct Method2State {
    double w_high = 1.0;
    double w_mid = 0.0;
    double w_low = 0.0;
};

// ========================= 구조체 선언 ========================= //
extern vector<GlobalPath> global_path_vec;
extern VectorSpace Space;
extern vector<EgoPose> ego_pose_vec; 
extern VehicleState ego_state;
extern Method1State m1_state;
extern Method2State m2_state;


// ========================= 함수 정의 ========================= //
double Deg2Rad (double deg);
double Rad2Deg (double rad);
double NormalizeAngle (double angle);
double GetDist (double a, double b);
double KPH2MPS (double KPH);
double MPS2KPH (double MPS);
double ClampPID (double PID);
bool LoadPath();

int FindClosestIdx (const vector<GlobalPath>& global_path_vec, const EgoPose& ego_pose);
int FindWaypointIdx (const vector<GlobalPath>& global_path_vec, const EgoPose ego_pose, const double Ld);

void gps2Enu (const morai_msgs::GPSMessage::ConstPtr& gps_msg, EgoPose& egoPose);
void yawTf (const sensor_msgs::Imu::ConstPtr& imu_msg, EgoPose& egoPose);

#endif