#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include "Global/Global.hpp"
#include "Planning/Planning.hpp"

void generateTrajectoryPoints(const Candidate& cand, vector<geometry_msgs::Point>& pts);
void publishGlobalPath(const vector<GlobalPath>& global_path);
void publishCandidatePaths(const vector<Candidate>& cand_vec);
void publishBestPath(const Candidate& best_cand);
void publishDynamicWindow(double v_min, double v_max, double w_min, double w_max, double predict_time);


#endif 