#include "Planning/Planning.hpp"

double TRFEProcess() {
    double method_1_mu = Method_1();
    double method_2_mu = Method_2();
    double mu_final = max(method_1_mu, method_2_mu);
    return mu_final;
}

double Method_1() {
    way_1.a_t_max = max(way_1.a_t_front, way_1.a_t_rear, way_1.a_t_cg);

    if(way_1.a_t_max(k) > a_t_comp(k-1)) {
        a_t_comp(k) = a_t_max(k);
    } else {
        a_t_comp(k) = a_t_max(k) - min(a_t_comp(k-1)-a_t_max(k), delta);
    }

    double method_1_mu = ;
    return method_1_mu;
}

double Method_2() {

}

void GetLateralAcc(Way_1& way, const morai_msgs::EgoVehicleStatus::ConstPtr& ego) {
    double ay = ego->acceleration.y;

    way.a_front = ay + Lf * yaw_rate;
    way.a_rear = ay - Lr * yaw_rate; 
}

void GetTotalAcc(Way_1& way, const morai_msgs::EgoVehicleStatus::ConstPtr& ego) {
    double g = 9.81;
    double ax = ego->acceleration.x;
    double ay = ego->acceleration.y;
    way_1.a_t_front = sqrt(way_1.a_front * way_1.a_front + ax * ax) / g;
    way_1.a_t_rear = sqrt(way_1.a_rear * way_1.a_rear + ax * ax) / g;
    way_1.a_t_cg = sqrt(ay * ay + ax * ax) / g;
}
