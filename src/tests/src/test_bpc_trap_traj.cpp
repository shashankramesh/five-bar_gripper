#include <iostream>
#include "trajectory_planner.h"
#include "kinematics.h"
#include "pi3hat/pi3hat_interface.h"
#include <chrono>
#include <fstream>
#include <thread>

using namespace std::chrono;

const std::map<int, int> servo_bus_map ={
  {1,1},
  {2,1}
};

int main(void)
{
  //Pi3hat initialize
  Pi3HatInterface pi3_interface;

  pi3_interface.initialize(servo_bus_map);

  std::vector<MoteusCommand> cmds;
  std::vector<MoteusResponse> resp;
  
  // Dimensions of the five-bar mechanism
  Matrix<double, 2, 1> A0, B0, C0, D0, F0, P0;
  Matrix<double, 2, 6> dim5bar;
  Vector<double, 6> conf_feed;
  Vector<double, 6> conf_cmd;
  Matrix<double, 2, 1> P;

  double phi_cmd, phim, phi_feed, phim_feed;
  double psi_cmd, psim, psi_feed, psim_feed;
  double phi_offset = -61*M_PI/36;
  double psi_offset = 13*M_PI/36;

  A0 << 0.12, 0;
  B0 << 0, 0;
  C0 << 0.09216732905642444, -0.039749173530376616;
  D0 << -0.011227449650384648, -0.02407734347492251;
  F0 << 0.027234061360222564, -0.06273044035186855;
  P0 << 0.10818369270155971, -0.17033426671218643;

  // Setting up five-bar kinematics
  dim5bar << A0(0), B0(0), C0(0), D0(0), F0(0), P0(0),
          A0(1), B0(1), C0(1), D0(1), F0(1), P0(1),

  cout << "Five bar dim: " << endl << dim5bar << endl;

  FiveBarKinematics five_bar_kinematics(dim5bar, phi_offset, psi_offset);

  // Initializing trajectory planning
  TrajectoryPlanner traj;

  double path_length = 0;
  double total_time = 2;
  double max_velocity = 0.11;
  double s = 0, time = 0, rel_time = 0;
  int nloops = 10, loop_i = 0;

  ofstream log("tp_BPC_traj_log4.csv");

  log << "time,phi_cmd,psi_cmd,rho_cmd,theta_cmd,x_cmd,y_cmd,phi_feed,psi_feed,rho_feed,theta_feed,x_feed,y_feed" << endl;

  traj.bpc_params.P1 << 0.1183, -0.173;
  traj.bpc_params.P2 << 0.1653, -0.1015;
  traj.bpc_params.Po1 << 0.11412185577170687, -0.17025352757021292;
  traj.bpc_params.Po2 << 0.16112185577170687, -0.0987535275702129;
  traj.bpc_params.Po << 0.1675021153543596, -0.1541450968063623;
  traj.bpc_params.Pc2 << 0.1605497153579619, -0.09378636989736217;
  traj.bpc_params.Pc1 << 0.10933505934002723, -0.17169802745656074;

  traj.bpc_params.r1 = 0.005;
  traj.bpc_params.r2 = 0.005;
  traj.bpc_params.R = 0.06075781250000004;
  traj.bpc_params.theta1 = -2.2669953208315845;
  traj.bpc_params.theta2 = 2.2669953208315845;
  traj.bpc_params.thetaC = 1.7491946655164166;

  path_length = traj.pathBPCinitialize();

  bool status = traj.initialize(total_time, max_velocity, path_length);

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

  // Start all motors in stopped mode to clear all faults
  pi3_interface.stop();
  // Wait for the stop command to be sent 
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  
  auto ti = high_resolution_clock::now();
  auto timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);

  while(time <= nloops*total_time)
  {
    timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);
    time = timeD.count()*1e-6;

    // Feedback
    resp = pi3_interface.read();

    phim_feed = resp[0].position;
    psim_feed = resp[1].position;

    five_bar_kinematics.getRelativeAngles(phim_feed, psim_feed, phi_feed, psi_feed);

    five_bar_kinematics.forwardKinematics(phi_feed, psi_feed, -1, conf_feed);
    
    rel_time = time - loop_i*total_time;

    if(rel_time >= total_time)
      loop_i++;
      
    traj.trapezoidalTrajectory(rel_time, s);

    traj.pathBPC(s, P);

    five_bar_kinematics.inverseKinematics(P, -1, -1, conf_cmd);
    
    // Command
    five_bar_kinematics.getMotorAngles(conf_cmd(0), conf_cmd(1), phim, psim);

    cmds[0].position = phim;
    cmds[1].position = psim;
    pi3_interface.write(cmds);
    
    //pi3_interface.stop();

    log << time << "," << conf_cmd(0) << "," << conf_cmd(1) 
	    << "," << conf_cmd(2) << "," << conf_cmd(3) 
	    << "," << conf_cmd(4) << "," << conf_cmd(5)
      << "," << conf_feed(0) << "," << conf_feed(1) 
	    << "," << conf_feed(2) << "," << conf_feed(3) 
	    << "," << conf_feed(4) << "," << conf_feed(5) << endl;

    std::cout << "Point: " << s << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  cout << status << endl;
  
  pi3_interface.stop();

  return 0;
}
