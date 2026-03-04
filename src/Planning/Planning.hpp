#ifndef PLANNING_HPP
#define PLANNING_HPP

#include "Global/Global.hpp"
#include "Visualizer/Visualizer.hpp"

// =================================== DWA 관련 함수 =================================== //
void DwaProcess (vector<Candidate>& cand_vec, const morai_msgs::EgoVehicleStatus::ConstPtr& ego, const EgoSpec& spec, const EgoPose& ego_pose, const vector<GlobalPath>& global_path_vec);
VectorSpace GetVelocityVectorSpace (const morai_msgs::EgoVehicleStatus::ConstPtr& ego, const EgoSpec& spec, EgoStatus& ego_status);
void GenerateLocalPath (vector<Candidate>& cand_vec, const morai_msgs::EgoVehicleStatus::ConstPtr& ego, const EgoSpec& spec, const EgoPose& ego_pose, const vector<GlobalPath>& global_path_vec);
void EvaluateLocalPath (vector<Candidate>& cand_vec, const EgoPose& ego_pose, const vector<GlobalPath>& global_path_vec, MinMax& minmax, const EgoSpec& spec); 
void CalcObjectScore (vector<Candidate>& cand_vec, const EgoPose& ego_pose, const vector<GlobalPath>& global_path_vec, MinMax& m);
void GetCandTotalScore (vector<Candidate>& cand_vec, const MinMax& m);
Candidate SelectBestLocalPath (const vector<Candidate>& cand_vec);
double GetHeadingScore (const vector<GlobalPath>& global_path_vec, const EgoPose& ego_pose, Candidate& cand, const EgoSpec& spec); 
double GetDistScore (const vector<GlobalPath>& global_path_vec, const EgoPose& ego_pose, const Candidate& cand);
double GetVelocityScore (const Candidate& cand);
double GetStanleyAngle (const vector<GlobalPath>& global_path_vec, const EgoPose& ego_pose, const double Ld);




// =================================== TRFE 관련 함수 =================================== //
double TRFEProcess();
double Method_1();
double Method_2();


#endif