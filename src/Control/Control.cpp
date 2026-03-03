# include "Control/Control.hpp"

void ControlProcess(const vector<Candidate>& cand_vec, const EgoStatus& ego_status, ControlInput& ctrl_msg, morai_msgs::CtrlCmd& cmd_msg) {
    Candidate best_cand = SelectBestLocalPath(cand_vec);
    LongitudinalControl(cand_vec, ego_status, ctrl_msg);
    ctrl_msg.steer_input = LateralControl(cand_vec);
    PublishControl(ctrl_msg, cmd_msg);
}

void LongitudinalControl(const vector<Candidate>& cand_vec, const EgoStatus& ego_status, ControlInput& ctrl_msg) {
    double kp = 0.1;
    double ki = 0.0007;
    double kd = 0.007;

    double delta_time = 0.02;
   
    Candidate best_cand= SelectBestLocalPath(cand_vec);

    double target_vel = best_cand.v;
    double current_vel = ego_status.v;

    //ego_status.v = target_vel;

    static double I_control_sum = 0.0;
    static double last_error = 0.0;

    double error = target_vel - current_vel;
    I_control_sum = I_control_sum + (error * delta_time);

    double P_control = kp * error;
    double I_control = ki * I_control_sum;
    double D_control = kd * (error - last_error) / delta_time;

    last_error = error;

    double control_signal = ClampPID(P_control + I_control + D_control);

    if(control_signal >= 0) {
        ctrl_msg.accel_input = control_signal;
        ctrl_msg.brake_input = 0.0;
    } else {
        ctrl_msg.accel_input = 0.0;
        ctrl_msg.brake_input = fabs(control_signal);
    }
} 

double LateralControl(const vector<Candidate>& cand_vec) {
    if(cand_vec.empty()) {
        cout << "후보 경로가 없습니다..." << endl;
        return 0.0;
    }

    Candidate best_cand = SelectBestLocalPath(cand_vec);

    if(best_cand.total_score <= -500.0) {
        cout << "유효한 후보 경로가 없습니다..." << endl;
        best_cand.steer_angle = 0.0;
    }
    return best_cand.steer_angle;
}

void PublishControl(const ControlInput& ctrl_msg, morai_msgs::CtrlCmd& cmd_msg) {
    cmd_msg.longlCmdType = 1;
    cmd_msg.accel = ctrl_msg.accel_input;
    cmd_msg.brake = ctrl_msg.brake_input;
    cmd_msg.steering = ctrl_msg.steer_input;
    control_pub.publish(cmd_msg);
}
