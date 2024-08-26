#include "trajectory_planner.h"
#include "2R_kinematics.h"
#include "pi3hat/pi3hat_interface.h"
#include <chrono>
#include <iostream>
#include <fstream>
#include <thread>

using namespace std::chrono;

const std::map<int, int> servo_bus_map ={
  {1,1},
  {2,1}
};

int main(void)
{ 

  double theta1, theta1m, theta1_feed, theta1m_feed;
  double theta2, theta2m, theta2_feed, theta2m_feed;
  double theta1_ini, theta1_fin, theta2_ini, theta2_fin;
  double path_length = 0;
  double total_time;
  double s = 0, time = 0;

  Vector<double, 2> P;

  //Pi3hat initialize
  Pi3HatInterface pi3_interface;

  pi3_interface.initialize(servo_bus_map);

  std::vector<MoteusCommand> cmds;
  std::vector<MoteusResponse> resp;
  
  // Setting up motor commands
  cmds.resize(2);
  cmds[0].id = 1;
  cmds[0].mode = 10; // Position mode
  cmds[0].position = 0.0;
  cmds[0].velocity = 0.0;
  cmds[0].feedforward_torque = 0;
  cmds[0].kp_scale = 7;
  cmds[0].kd_scale = 4;
  cmds[0].watchdog_timeout = 0;

  cmds[1].id = 2;
  cmds[1].mode = 10; // Position mode
  cmds[1].position = 0.0;
  cmds[1].velocity = 0.0;
  cmds[1].feedforward_torque = 0;
  cmds[1].kp_scale = 7;
  cmds[1].kd_scale = 4;
  cmds[1].watchdog_timeout = 0;

  // Dimensions and parameters of the 2R serial chain leg
  double l1 = 0.0894, l2 = 0.0894;
  double theta1_offset = -0.65675;
  double theta2_offset = -0.04075;
  int m1_sign = 1;
  int m2_sign = -1;
  int ik_branch = 1;
  
  cout << "2R dim: l1: " << l1 << ", l2: " << l2 << endl;

  TwoRKinematics twor_kinematics(l1, l2, theta1_offset, theta2_offset, m1_sign, m2_sign);

  TrajectoryPlanner traj;

  // Parameters of the generalized trapezoidal trajectory
  int n_cycle = 5;
  double sF = 2;
  double cycle_freq = 0.5;
  double p2p_time = 2;

  // Parameters of the BPC path
  traj.bpc_params.P1 << -0.04278215165229538, -0.168;
  traj.bpc_params.P2 << 0.04278215165229538, -0.168;
  traj.bpc_params.Po1 << -0.04278215165229538, -0.163;
  traj.bpc_params.Po2 << 0.04278215165229538, -0.163;
  traj.bpc_params.Po << 0., -0.19875781250000002;
  traj.bpc_params.Pc2 << 0.04661857831020053, -0.15979347064592966;
  traj.bpc_params.Pc1 << -0.04661857831020053, -0.15979347064592966;

  traj.bpc_params.r1 = 0.005;
  traj.bpc_params.r2 = 0.005;
  traj.bpc_params.R = 0.060757812500000036;
  traj.bpc_params.theta1 = -2.2669953208315845;
  traj.bpc_params.theta2 = 2.2669953208315845;
  traj.bpc_params.thetaC = 1.749194665516418;

  path_length = traj.pathBPCinitialize();

  double traj_time = traj.initializeTTBPC(sF, cycle_freq);

  // Setting home position
  theta1_ini = -theta1_offset;
  theta2_ini = -theta2_offset;
  twor_kinematics.inverseKinematics(traj.bpc_params.P1, ik_branch, theta1_fin, theta2_fin);

  // Setting up log file
  ofstream log("gait_2R/gait_2R_23_01_02.csv");
  
  log << "2R planar serial chain leg gait frequency test" << endl;
  log << "Leg parameters" << endl;
  log << "l1," << l1 << endl;
  log << "l2," << l2 << endl;
  log << "IK_branch," << ik_branch << endl;
  log << endl;
  
  log << "Gait trajectory: BPC path with generalized trapezoidal trajectory with parameters below" << endl;
  log << "Initial point p1," << traj.bpc_params.P1(0) << "," << traj.bpc_params.P1(1) << endl;
  log << "Final point p2," << traj.bpc_params.P2(0) << "," << traj.bpc_params.P2(1) << endl;
  log << "po1," << traj.bpc_params.Po1(0) << "," << traj.bpc_params.Po1(1) << endl;
  log << "po2," << traj.bpc_params.Po2(0) << "," << traj.bpc_params.Po2(1) << endl;
  log << "po," << traj.bpc_params.Po(0) << "," << traj.bpc_params.Po(1) << endl;
  log << "pc1," << traj.bpc_params.Pc1(0) << "," << traj.bpc_params.Pc1(1) << endl;
  log << "pc2," << traj.bpc_params.Pc2(0) << "," << traj.bpc_params.Pc2(1) << endl;
  log << "r1," << traj.bpc_params.r1 << endl;
  log << "r2," << traj.bpc_params.r2 << endl;
  log << "R," << traj.bpc_params.R << endl;
  log << "theta1," << traj.bpc_params.theta1 << endl;
  log << "theta2," << traj.bpc_params.theta2 << endl;
  log << "thetaC," << traj.bpc_params.thetaC << endl;
  log << "swing-stance_velocity_factor," << sF << endl;
  log << "gait_frequency," << cycle_freq << endl;
  log << "gait_cycles," << n_cycle << endl;
  log << endl;
  
  log << "Actuator gain scales:" << endl;
  log << "Kp," << cmds[1].kp_scale << endl;
  log << "Kd," << cmds[1].kd_scale << endl;
  log << endl;
  
  log << "time,theta1_cmd,theta2_cmd,theta1_feed,theta2_feed,x_cmd,y_cmd" << endl;

  // Start all motors in stopped mode to clear all faults
  pi3_interface.stop();
  // Wait for the stop command to be sent 
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  total_time = 2;
    
  auto ti = high_resolution_clock::now();
  auto timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);

  // From the home configuration to the start configuration for walking trajectory
  while(time <= total_time)
  {
    timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);
    time = timeD.count()*1e-6;

    // Feedback
    resp = pi3_interface.read();
    
    twor_kinematics.getRelativeAngles(resp[0].position, resp[1].position, theta1_feed, theta2_feed);

    theta1 = theta1_ini + (time*(theta1_fin - theta1_ini)/total_time);
    theta2 = theta2_ini + (time*(theta2_fin - theta2_ini)/total_time);
    
    twor_kinematics.forwardKinematics(theta1, theta2, P);
    
    // Command
    twor_kinematics.getMotorAngles(theta1, theta2, theta1m, theta2m);

    cmds[0].position = theta1m;
    cmds[1].position = theta2m;
    pi3_interface.write(cmds);
    
    //pi3_interface.stop();    

    log << time << "," << theta1 << "," << theta2
	    << "," << theta1_feed << "," << theta2_feed
      << "," << P(0) << "," << P(1) << endl;

    /*cout << time << "," << theta1 << "," << theta2
      << "," << theta1m << "," << theta2m
	    << "," << P(0) << "," << P(1) << endl; */

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  
  total_time = n_cycle*traj_time;
  
  ti = high_resolution_clock::now();
  time = 0;

  // Main walking trajectory
  while(time <= total_time)
  {
    timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);
    time = timeD.count()*1e-6;

    // Feedback
    resp = pi3_interface.read();
 
    theta1m_feed = resp[0].position;
    theta2m_feed = resp[1].position;
    
    twor_kinematics.getRelativeAngles(theta1m_feed, theta2m_feed, 
        theta1_feed, theta2_feed);

    traj.trapezoidalTrajectoryBPC(time, s);

    traj.pathBPC(s, P);

    twor_kinematics.inverseKinematics(P, ik_branch, theta1, theta2);

    // Command
    twor_kinematics.getMotorAngles(theta1, theta2, theta1m, theta2m);

    cmds[0].position = theta1m;
    cmds[1].position = theta2m;
    pi3_interface.write(cmds);
    
    //pi3_interface.stop();
 
    log << time << "," << theta1 << "," << theta2
	    << "," << theta1_feed << "," << theta2_feed
      << "," << P(0) << "," << P(1) << endl;

    /*cout << time << "," << theta1 << "," << theta2
      << "," << theta1m << "," << theta2m
	    << "," << P(0) << "," << P(1) << endl; */

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  
  pi3_interface.stop();
  
  return 0;
}
