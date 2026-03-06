#include "Global/Global.hpp"

vector<GlobalPath> global_path_vec;

// ========================= Utils ========================= //
double Deg2Rad (double deg) {
    double rad = deg * (pi / 180.);
    return rad;
}

double Rad2Deg (double rad) {
    double deg = rad * (180. / pi);
    return deg;
}

double NormalizeAngle (double angle) {
    while (angle > pi) angle -= 2.0 * pi;
    while (angle < -pi) angle += 2.0 * pi;
    return angle;
}

double GetDist (double a, double b) {
    return sqrt(a * a + b * b);
}

double KPH2MPS (double KPH) {
    return KPH / 3.6;
}

double MPS2KPH (double MPS) {
    return MPS * 3.6;;
}

double ClampPID (double PID) {
    return max(-1.0, min(PID, 1.0));
}

// ========================= Utils ========================= //

// ========================= Load Path ========================= //
bool LoadPath() {
    if(!global_path_vec.empty()) { return true; }

    string file_path = "/home/autonav/Study_ITS/src/Map/path.txt";

    ifstream inputFile;
    inputFile.open(file_path);

    if(!inputFile.is_open()) {
        cout << "파일 열기에 실패함.." << endl;
        return false;
    }

    double e, n, u;
    while(inputFile >> e >> n >> u) {
        global_path_vec.push_back(GlobalPath{e, n, u});
    }

    if(global_path_vec.empty()) {
        cout << "GlobalPathVec가 비어있습니다..." << endl;
        return false;
    }
    return true;
}

// ========================================== Find Idx ========================================== //
int FindClosestIdx (const vector<GlobalPath>& global_path_vec, const EgoPose& ego_pose) {
    int closest_idx = 0;
    double min_d = -1.0;

    for(int i = 0; i < global_path_vec.size(); i++) {
        double de = global_path_vec[i].e - ego_pose.curr_e;
        double dn = global_path_vec[i].n - ego_pose.curr_n;
        double d = GetDist(de, dn);

        if(min_d == -1.0 || d < min_d) {
            min_d = d;
            closest_idx = i;
        }
    }
    return closest_idx;
}

int FindWaypointIdx (const vector<GlobalPath>& global_path_vec, const EgoPose ego_pose, const double Ld) {
    if(global_path_vec.empty()) {
        cout << "경로를 찾지 못했습니다..." << endl;
        return -1;
    }

    int closest_idx = FindClosestIdx(global_path_vec, ego_pose);
    for(int i = closest_idx; i < global_path_vec.size(); ++i) {
        double de = global_path_vec[i].e - ego_pose.curr_e;
        double dn = global_path_vec[i].n - ego_pose.curr_n;
        double d = GetDist(de, dn);

        if(d > Ld) { return i; }
    }
    return global_path_vec.size() -1;
}

// ================================== 좌표변환 함수 ===================================//
void gps2Enu (const morai_msgs::GPSMessage::ConstPtr& gps_msg, EgoPose& ego_pose) {
    // GPSMessage에서 받아올 WGS(GPS)값과 그 값에 대한 변수 설정
    double wgs_lat = gps_msg->latitude;
    double wgs_lon = gps_msg->longitude;
    double wgs_alt = gps_msg->altitude;
    // WGS값 radian으로 변경(계산 목적)
    double rad_lat = wgs_lat * pi / 180;
    double rad_lon = wgs_lon * pi / 180;
    double rad_alt = wgs_alt * pi / 180;
    double h = wgs_alt;
    double k = a / sqrt(1-e*pow(sin(rad_lat), 2));

    // WGS->ECEF
    double ecef_x = (k + h) * cos(rad_lat) * cos(rad_lon);
    double ecef_y = (k + h) * cos(rad_lat) * sin(rad_lon);
    double ecef_z = (k * (1-e) + h) * sin(rad_lat);

    double ref_rad_lat = ref_lat * pi / 180;
    double ref_rad_lon = ref_lon * pi / 180;
    double ref_rad_alt = ref_alt * pi / 180;

    double ref_h = ref_alt;
    double ref_k = a / sqrt(1 - e * pow(sin(ref_rad_lat), 2));
    double ref_ecef_x = (ref_k + ref_h) * cos(ref_rad_lat) * cos(ref_rad_lon);
    double ref_ecef_y = (ref_k + ref_h) * cos(ref_rad_lat) * sin(ref_rad_lon);
    double ref_ecef_z = (ref_k * (1 - e) + ref_h) * sin(ref_rad_lat);


    // ecef 좌표 enu 좌표로 변환
    // ref_rad_lon, ref_rad_lat (기준점 변수)를 사용해야 함
    ego_pose.curr_e = (-sin(ref_rad_lon) * (ecef_x - ref_ecef_x) + cos(ref_rad_lon) * (ecef_y - ref_ecef_y));

    ego_pose.curr_n = (-sin(ref_rad_lat) * cos(ref_rad_lon) * (ecef_x - ref_ecef_x) 
                         - sin(ref_rad_lat) * sin(ref_rad_lon) * (ecef_y - ref_ecef_y) 
                         + cos(ref_rad_lat) * (ecef_z - ref_ecef_z));
    ego_pose.curr_u = (cos(rad_lat)*cos(rad_lon)*(ecef_x - ref_ecef_x) + cos(rad_lat)*sin(rad_lon)*(ecef_y-ref_ecef_y) + sin(rad_lat)*(ecef_z-ref_ecef_z));
}

void yawTf (const sensor_msgs::Imu::ConstPtr& imu_msg, EgoPose& ego_pose) {
    double imu_x = imu_msg->orientation.x;
    double imu_y = imu_msg->orientation.y;
    double imu_z = imu_msg->orientation.z;
    double imu_w = imu_msg->orientation.w;
    // imu 쿼터니안 -> 오일러 변환 (yaw 각 범위 -180 < yaw < 180)
    ego_pose.curr_yaw = atan2(2 * (imu_w * imu_z + imu_x * imu_y), 1-2 * (pow(imu_y, 2) + pow(imu_z, 2)));
}