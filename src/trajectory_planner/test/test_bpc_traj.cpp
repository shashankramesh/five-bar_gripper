#include "trajectory_planner.h"
#include "kinematics.h"
#include <chrono>
#include <iostream>
#include <fstream>
#include <thread>

using namespace std::chrono;

int main(void)
{
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

  TrajectoryPlanner traj;

  double path_length = 0;
  double total_time = 2;
  double max_velocity = 0.2;
  double s = 0, time = 0;

  ofstream log("tp_BPC_traj_log.txt");

  log << "time,phi,psi,rho,theta,x,y" << endl;

  traj.bpc_params.P1 << 0.1205, -0.1645;
  traj.bpc_params.P2 << 0.1609, -0.1015;
  traj.bpc_params.Po1 << 0.11629107134206083, -0.16180094098760728;
  traj.bpc_params.Po2 << 0.15669107134206084, -0.09880094098760728;
  traj.bpc_params.Po << 0.19121934978837793, -0.16539653541984872;
  traj.bpc_params.Pc2 << 0.15438963105742806, -0.09436209286942736;
  traj.bpc_params.Pc1 << 0.11129681833455447, -0.16156128102638367;

  traj.bpc_params.r1 = 0.005;
  traj.bpc_params.r2 = 0.005;
  traj.bpc_params.R = 0.08001450000000002;
  traj.bpc_params.theta1 = -2.6193294946211703;
  traj.bpc_params.theta2 = 2.6193294946211703;
  traj.bpc_params.thetaC = 1.0445263179372468;

  path_length = traj.pathBPCinitialize();

  bool status = traj.initialize(total_time, max_velocity, path_length);

  auto ti = high_resolution_clock::now();
  auto timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);

  while(time <= total_time)
  {
    timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);
    time = timeD.count()*1e-6;

    traj.trapezoidalTrajectory(time, s);

    traj.pathBPC(s, P);

    five_bar_kinematics.inverseKinematics(P, -1, -1, conf_cmd);

    log << time << "," << conf_cmd(0) << "," << conf_cmd(1) 
	    << "," << conf_cmd(2) << "," << conf_cmd(3) 
	    << "," << conf_cmd(4) << "," << conf_cmd(5) << endl;

    std::cout << "Point: " << P << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  cout << status << endl;

  return 0;
}
