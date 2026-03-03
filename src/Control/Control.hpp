#ifndef CONTROL_HPP
#define CONTROL_HPP

#include "Global/Global.hpp"
#include "Planning/Planning.hpp"

extern ros::Publisher control_pub;

// add cmd_msg reference so caller can receive the command message as well
void ControlProcess(const vector<Candidate>& cand_vec,
                    const EgoStatus& ego_status,
                    ControlInput& ctrl_msg,
                    morai_msgs::CtrlCmd& cmd_msg);

void LongitudinalControl(const vector<Candidate>& cand_vec, const EgoStatus& ego_status, ControlInput& ctrl_msg);
double LateralControl(const vector<Candidate>& cand_vec);

// publish control now requires both the ControlInput and the command message
void PublishControl(const ControlInput& ctrl_msg, morai_msgs::CtrlCmd& cmd_msg);

#endif