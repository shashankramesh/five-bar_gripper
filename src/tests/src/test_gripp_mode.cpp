/*
 * file: test_gripp_mode.cpp
 *
 * Created: 30 Aug, 2024
 * Author: Shashank Ramesh
 */
 
#include "trajectory_planner.h"
#include "kinematics.h"
#include "pi3hat/pi3hat_interface.h"
#include <chrono>
#include <iostream>
#include <fstream>
#include <thread>

using namespace std::chrono;

ofstream log_file("../verify_codes/sense_mode_gripp_UTM_horizontal gripper.csv");

double phi_cmd_l, phi_feed_l;
double psi_cmd_l, psi_feed_l;
double phi_cmd_r, phi_feed_r;
double psi_cmd_r, psi_feed_r;

double pm_r[2] = {1, -1};
double pm_l[2] = {-1, 1};

const std::map<int, int> servo_bus_map ={
  {1,1},
  {2,1},
  {3,1},
  {4,1}
};

void orderResponse(std::vector<MoteusResponse> resp_in, std::vector<MoteusResponse>& resp_out)
{
  for(int jj = 0; jj < 4; jj++)
  {
    resp_out[int(resp_in[jj].id) - 1] = resp_in[jj];
  }
}

void hold(Vector<double, 2> torque_r, Vector<double, 2> torque_l, double hold_time,
        FiveBarKinematics& right_finger_kinematics, FiveBarKinematics& left_finger_kinematics, Pi3HatInterface& pi3_interface, 
        std::vector<MoteusCommand>& cmds, std::vector<MoteusResponse>& resp)
{
  double phim_cmd_l, phim_feed_l;
  double psim_cmd_l, psim_feed_l;
  double phim_cmd_r, phim_feed_r;
  double psim_cmd_r, psim_feed_r;
  Vector<double, 6> conf_cmd_r, conf_cmd_l;
  Vector<double, 6> conf_feed_r, conf_feed_l;

  auto ti = high_resolution_clock::now();
  auto timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);

  double time = 0;

  while(time <= hold_time)
  {
    timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);
    time = timeD.count()*1e-6;

    // Feedback
    orderResponse(pi3_interface.read(), resp);
    
    phim_feed_r = resp[1].position;
    psim_feed_r = resp[0].position;
    phim_feed_l = resp[3].position;
    psim_feed_l = resp[2].position;
    
    right_finger_kinematics.getRelativeAngles(phim_feed_r, psim_feed_r, phi_feed_r, psi_feed_r);
    left_finger_kinematics.getRelativeAngles(phim_feed_l, psim_feed_l, phi_feed_l, psi_feed_l);

    right_finger_kinematics.forwardKinematics(phi_feed_r, psi_feed_r, 1, conf_feed_r);
    left_finger_kinematics.forwardKinematics(phi_feed_l, psi_feed_l, -1, conf_feed_l);

    // Command
    right_finger_kinematics.getMotorAngles(phi_cmd_r, psi_cmd_r, phim_cmd_r, psim_cmd_r);
    left_finger_kinematics.getMotorAngles(phi_cmd_l, psi_cmd_l, phim_cmd_l, psim_cmd_l);

    cmds[1].position = phim_cmd_r;
    cmds[0].position = psim_cmd_r;
    cmds[3].position = phim_cmd_l;
    cmds[2].position = psim_cmd_l;
    
    right_finger_kinematics.getMotorTorquesCommand(torque_r(0), torque_r(1), cmds[1].feedforward_torque, cmds[0].feedforward_torque);
    left_finger_kinematics.getMotorTorquesCommand(torque_l(0), torque_l(1), cmds[3].feedforward_torque, cmds[2].feedforward_torque);
    
    /*cmds[1].feedforward_torque = torque_r(0);
    cmds[0].feedforward_torque = torque_r(1);
    cmds[3].feedforward_torque = torque_l(0);
    cmds[2].feedforward_torque = torque_l(1);*/
    
    // Uncommet to run
    pi3_interface.write(cmds);

    //pi3_interface.stop();

    right_finger_kinematics.forwardKinematics(phi_cmd_r, psi_cmd_r, 1, conf_cmd_r);
    left_finger_kinematics.forwardKinematics(phi_cmd_l, psi_cmd_l, -1, conf_cmd_l);

    log_file << time << "," << conf_cmd_r(0) << "," << conf_cmd_r(1) 
      << "," << conf_cmd_r(2) << "," << conf_cmd_r(3) 
      << "," << conf_cmd_r(4) << "," << conf_cmd_r(5)
        << "," << conf_feed_r(0) << "," << conf_feed_r(1) 
      << "," << conf_feed_r(2) << "," << conf_feed_r(3) 
      << "," << conf_feed_r(4) << "," << conf_feed_r(5)
      << "," << conf_cmd_l(0) << "," << conf_cmd_l(1) 
      << "," << conf_cmd_l(2) << "," << conf_cmd_l(3) 
      << "," << conf_cmd_l(4) << "," << conf_cmd_l(5)
        << "," << conf_feed_l(0) << "," << conf_feed_l(1) 
      << "," << conf_feed_l(2) << "," << conf_feed_l(3) 
      << "," << conf_feed_l(4) << "," << conf_feed_l(5) 
      << "," << cmds[1].feedforward_torque
      << "," << cmds[0].feedforward_torque
      << "," << cmds[3].feedforward_torque
      << "," << cmds[2].feedforward_torque
      << "," << resp[1].torque
      << "," << resp[0].torque
      << "," << resp[3].torque
      << "," << resp[2].torque
      << "," << resp[1].q_current
      << "," << resp[0].q_current
      << "," << resp[3].q_current
      << "," << resp[2].q_current
      << "," << resp[1].d_current
      << "," << resp[0].d_current
      << "," << resp[3].d_current
      << "," << resp[2].d_current
      << endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void release(double release_time, FiveBarKinematics& right_finger_kinematics, FiveBarKinematics& left_finger_kinematics, Pi3HatInterface& pi3_interface, 
        std::vector<MoteusCommand>& cmds, std::vector<MoteusResponse>& resp)
{
  double phim_cmd_l, phim_feed_l;
  double psim_cmd_l, psim_feed_l;
  double phim_cmd_r, phim_feed_r;
  double psim_cmd_r, psim_feed_r;
  Vector<double, 6> conf_cmd_r, conf_cmd_l;
  Vector<double, 6> conf_feed_r, conf_feed_l;

  auto ti = high_resolution_clock::now();
  auto timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);

  double time = 0;

  while(time <= release_time)
  {
    timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);
    time = timeD.count()*1e-6;

    // Feedback
    orderResponse(pi3_interface.read(), resp);
    
    phim_feed_r = resp[1].position;
    psim_feed_r = resp[0].position;
    phim_feed_l = resp[3].position;
    psim_feed_l = resp[2].position;
    
    right_finger_kinematics.getRelativeAngles(phim_feed_r, psim_feed_r, phi_feed_r, psi_feed_r);
    left_finger_kinematics.getRelativeAngles(phim_feed_l, psim_feed_l, phi_feed_l, psi_feed_l);

    right_finger_kinematics.forwardKinematics(phi_feed_r, psi_feed_r, 1, conf_feed_r);
    left_finger_kinematics.forwardKinematics(phi_feed_l, psi_feed_l, -1, conf_feed_l);

    // Command
    right_finger_kinematics.getMotorAngles(phi_cmd_r, psi_cmd_r, phim_cmd_r, psim_cmd_r);
    left_finger_kinematics.getMotorAngles(phi_cmd_l, psi_cmd_l, phim_cmd_l, psim_cmd_l);

    cmds[1].position = phim_cmd_r;
    cmds[0].position = psim_cmd_r;
    cmds[3].position = phim_cmd_l;
    cmds[2].position = psim_cmd_l;
    
    
    cmds[1].feedforward_torque = 0;
    cmds[0].feedforward_torque = 0;
    cmds[3].feedforward_torque = 0;
    cmds[2].feedforward_torque = 0;
    
    // Uncommet to run
    pi3_interface.write(cmds);

    //pi3_interface.stop();

    right_finger_kinematics.forwardKinematics(phi_cmd_r, psi_cmd_r, 1, conf_cmd_r);
    left_finger_kinematics.forwardKinematics(phi_cmd_l, psi_cmd_l, -1, conf_cmd_l);

    log_file << time << "," << conf_cmd_r(0) << "," << conf_cmd_r(1) 
      << "," << conf_cmd_r(2) << "," << conf_cmd_r(3) 
      << "," << conf_cmd_r(4) << "," << conf_cmd_r(5)
        << "," << conf_feed_r(0) << "," << conf_feed_r(1) 
      << "," << conf_feed_r(2) << "," << conf_feed_r(3) 
      << "," << conf_feed_r(4) << "," << conf_feed_r(5)
      << "," << conf_cmd_l(0) << "," << conf_cmd_l(1) 
      << "," << conf_cmd_l(2) << "," << conf_cmd_l(3) 
      << "," << conf_cmd_l(4) << "," << conf_cmd_l(5)
        << "," << conf_feed_l(0) << "," << conf_feed_l(1) 
      << "," << conf_feed_l(2) << "," << conf_feed_l(3) 
      << "," << conf_feed_l(4) << "," << conf_feed_l(5) 
      << "," << cmds[1].feedforward_torque
      << "," << cmds[0].feedforward_torque
      << "," << cmds[3].feedforward_torque
      << "," << cmds[2].feedforward_torque
      << "," << resp[1].torque
      << "," << resp[0].torque
      << "," << resp[3].torque
      << "," << resp[2].torque
      << "," << resp[1].q_current
      << "," << resp[0].q_current
      << "," << resp[3].q_current
      << "," << resp[2].q_current
      << "," << resp[1].d_current
      << "," << resp[0].d_current
      << "," << resp[3].d_current
      << "," << resp[2].d_current
      << endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void p2p(double joint_ini_right_finger[2], double joint_fin_right_finger[2], double joint_ini_left_finger[2], double joint_fin_left_finger[2], double p2p_time,
        FiveBarKinematics& right_finger_kinematics, FiveBarKinematics& left_finger_kinematics, Pi3HatInterface& pi3_interface, 
        std::vector<MoteusCommand>& cmds, std::vector<MoteusResponse>& resp)
{
  double phim_cmd_l, phim_feed_l;
  double psim_cmd_l, psim_feed_l;
  double phim_cmd_r, phim_feed_r;
  double psim_cmd_r, psim_feed_r;
  Vector<double, 6> conf_cmd_r, conf_cmd_l;
  Vector<double, 6> conf_feed_r, conf_feed_l;

  auto ti = high_resolution_clock::now();
  auto timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);

  double time = 0;

  while(time <= p2p_time)
  {
    timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);
    time = timeD.count()*1e-6;

    // Feedback
    orderResponse(pi3_interface.read(), resp);
    
    phim_feed_r = resp[1].position;
    psim_feed_r = resp[0].position;
    phim_feed_l = resp[3].position;
    psim_feed_l = resp[2].position;
    
    right_finger_kinematics.getRelativeAngles(phim_feed_r, psim_feed_r, phi_feed_r, psi_feed_r);
    left_finger_kinematics.getRelativeAngles(phim_feed_l, psim_feed_l, phi_feed_l, psi_feed_l);

    right_finger_kinematics.forwardKinematics(phi_feed_r, psi_feed_r, 1, conf_feed_r);
    left_finger_kinematics.forwardKinematics(phi_feed_l, psi_feed_l, -1, conf_feed_l);

    phi_cmd_r = joint_ini_right_finger[0] + (time*(joint_fin_right_finger[0] - joint_ini_right_finger[0])/p2p_time);
    psi_cmd_r = joint_ini_right_finger[1] + (time*(joint_fin_right_finger[1] - joint_ini_right_finger[1])/p2p_time);
    phi_cmd_l = joint_ini_left_finger[0] + (time*(joint_fin_left_finger[0] - joint_ini_left_finger[0])/p2p_time);
    psi_cmd_l = joint_ini_left_finger[1] + (time*(joint_fin_left_finger[1] - joint_ini_left_finger[1])/p2p_time);

    // Command
    right_finger_kinematics.getMotorAngles(phi_cmd_r, psi_cmd_r, phim_cmd_r, psim_cmd_r);
    left_finger_kinematics.getMotorAngles(phi_cmd_l, psi_cmd_l, phim_cmd_l, psim_cmd_l);

    cmds[1].position = phim_cmd_r;
    cmds[0].position = psim_cmd_r;
    cmds[3].position = phim_cmd_l;
    cmds[2].position = psim_cmd_l; //TODO: check these

    // Uncommet to run
    pi3_interface.write(cmds);

    //pi3_interface.stop();

    right_finger_kinematics.forwardKinematics(phi_cmd_r, psi_cmd_r, 1, conf_cmd_r);
    left_finger_kinematics.forwardKinematics(phi_cmd_l, psi_cmd_l, -1, conf_cmd_l);

    log_file << time << "," << conf_cmd_r(0) << "," << conf_cmd_r(1) 
	    << "," << conf_cmd_r(2) << "," << conf_cmd_r(3) 
	    << "," << conf_cmd_r(4) << "," << conf_cmd_r(5)
      	<< "," << conf_feed_r(0) << "," << conf_feed_r(1) 
	    << "," << conf_feed_r(2) << "," << conf_feed_r(3) 
	    << "," << conf_feed_r(4) << "," << conf_feed_r(5)
	    << "," << conf_cmd_l(0) << "," << conf_cmd_l(1) 
	    << "," << conf_cmd_l(2) << "," << conf_cmd_l(3) 
	    << "," << conf_cmd_l(4) << "," << conf_cmd_l(5)
      	<< "," << conf_feed_l(0) << "," << conf_feed_l(1) 
	    << "," << conf_feed_l(2) << "," << conf_feed_l(3) 
	    << "," << conf_feed_l(4) << "," << conf_feed_l(5)
      << "," << cmds[1].feedforward_torque
      << "," << cmds[0].feedforward_torque
      << "," << cmds[3].feedforward_torque
      << "," << cmds[2].feedforward_torque
      << "," << resp[1].torque
      << "," << resp[0].torque
      << "," << resp[3].torque
      << "," << resp[2].torque
      << "," << resp[1].q_current
      << "," << resp[0].q_current
      << "," << resp[3].q_current
      << "," << resp[2].q_current
      << "," << resp[1].d_current
      << "," << resp[0].d_current
      << "," << resp[3].d_current
      << "," << resp[2].d_current
      << endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void linear_motion(Vector<double, 2>& pI_r, Vector<double, 2>& pF_r, double pm_r[2], double& path_time_r, double& n_r, 
					Vector<double, 2>& pI_l, Vector<double, 2>& pF_l, double pm_l[2], double& path_time_l, double& n_l, bool move_until_gripp,
          FiveBarKinematics& right_finger_kinematics, FiveBarKinematics& left_finger_kinematics, Pi3HatInterface& pi3_interface, 
          std::vector<MoteusCommand>& cmds, std::vector<MoteusResponse>& resp)
{
  Vector<double, 2> pd_r, P_r, pd_l, P_l;
  pd_r = pF_r - pI_r;
  pd_l = pF_l - pI_l;

  double phim_cmd_l, phim_feed_l;
  double psim_cmd_l, psim_feed_l;
  double phim_cmd_r, phim_feed_r;
  double psim_cmd_r, psim_feed_r;
  Vector<double, 6> conf_cmd_r, conf_cmd_l, conf_feed_r, conf_feed_l;
  bool first_iter = true;
  double expFiltalpha = 0.1;
  
  double phi_torque_l, psi_torque_l, phi_torque_r, psi_torque_r;

  double path_length_r = sqrt(pd_r(0)*pd_r(0) + pd_r(1)*pd_r(1));
  double path_length_l = sqrt(pd_l(0)*pd_l(0) + pd_l(1)*pd_l(1));
  double total_time_r = n_r*path_time_r;
  double total_time_l = n_l*path_time_l;
  double total_time  = std::max(total_time_r, total_time_l);
  double max_velocity_r = 0.35;
  double max_velocity_l = 0.35;
  double s_r = 0, s_l = 0, time = 0;
  int ni_r = 0, ni_l = 0;

  TrajectoryPlanner traj_r;
  TrajectoryPlanner traj_l;

  bool status_r = traj_r.initialize(path_time_r, max_velocity_r, path_length_r);
  bool status_l = traj_l.initialize(path_time_l, max_velocity_l, path_length_l);

  auto ti = high_resolution_clock::now();
  auto timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);

  while(time <= total_time)
  {
    timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);
    time = timeD.count()*1e-6;

    // Feedback
    orderResponse(pi3_interface.read(), resp);

    phim_feed_r = resp[1].position;
    psim_feed_r = resp[0].position;
    phim_feed_l = resp[3].position;
    psim_feed_l = resp[2].position;
    
    right_finger_kinematics.getRelativeAngles(phim_feed_r, psim_feed_r, phi_feed_r, psi_feed_r);
    left_finger_kinematics.getRelativeAngles(phim_feed_l, psim_feed_l, phi_feed_l, psi_feed_l);
        
    if(first_iter)
    {
      first_iter = false;
      phi_torque_r = resp[1].torque;
      psi_torque_r = resp[0].torque;
      phi_torque_l = resp[3].torque;
      psi_torque_l = resp[2].torque;
    }
    else
    {
      phi_torque_r = expFiltalpha*resp[1].torque + (1-expFiltalpha)*phi_torque_r;
      psi_torque_r = expFiltalpha*resp[0].torque + (1-expFiltalpha)*psi_torque_r;
      phi_torque_l = expFiltalpha*resp[3].torque + (1-expFiltalpha)*phi_torque_l;
      psi_torque_l = expFiltalpha*resp[2].torque + (1-expFiltalpha)*psi_torque_l;
    }
    
    cout << phi_torque_r << ", " << psi_torque_r << ", " << phi_torque_l << ", " << psi_torque_l << endl;
    
    if(move_until_gripp)
    {
      if (abs(phi_torque_r) <= 0.15 && abs(psi_torque_r) <= 0.15 && abs(phi_torque_l) <= 0.15 && abs(psi_torque_l) <= 0.15)
      {
        if(time <= total_time_r)
        {
          ni_r = int(floor(time/path_time_r));
          traj_r.trapezoidalTrajectory(time - ni_r*path_time_r, s_r);
        }
        if(time <= total_time_l)
        {
          ni_l = int(floor(time/path_time_l));
          traj_l.trapezoidalTrajectory(time - ni_l*path_time_l, s_l);
        }
      }
    }
    else
    {
      if(time <= total_time_r)
      {
        ni_r = int(floor(time/path_time_r));
        traj_r.trapezoidalTrajectory(time - ni_r*path_time_r, s_r);
      }
      if(time <= total_time_l)
      {
        ni_l = int(floor(time/path_time_l));
        traj_l.trapezoidalTrajectory(time - ni_l*path_time_l, s_l);
      }
    }    
    
    cout << "s_l: " << s_l << ", s_r: " << s_r << endl; 
        
    if(ni_r % 2 == 0)
      traj_r.straightLinePath(pI_r, pF_r, s_r, P_r);
    else
      traj_r.straightLinePath(pF_r, pI_r, s_r, P_r);
    if(ni_l % 2 == 0)
      traj_l.straightLinePath(pI_l, pF_l, s_l, P_l);
    else
      traj_l.straightLinePath(pF_l, pI_l, s_l, P_l);

    // for right finger mode-2: sense (flat ve) {1, -1}, mode-3: gripp (vertical ve) {-1, 1}
    right_finger_kinematics.inverseKinematics(P_r, pm_r[0], pm_r[1], conf_cmd_r);
    // for left finger mode-2: grip (vertical ve) {1, -1}, mode-3: sense (flat ve) {-1, 1}
    left_finger_kinematics.inverseKinematics(P_l, pm_l[0], pm_l[1], conf_cmd_l);

    // Command
    right_finger_kinematics.getMotorAngles(conf_cmd_r(0), conf_cmd_r(1), phim_cmd_r, psim_cmd_r);
    left_finger_kinematics.getMotorAngles(conf_cmd_l(0), conf_cmd_l(1), phim_cmd_l, psim_cmd_l);

    phi_cmd_r = conf_cmd_r(0);
    psi_cmd_r = conf_cmd_r(1);
    phi_cmd_l = conf_cmd_l(0);
    psi_cmd_l = conf_cmd_l(1);

    cmds[1].position = phim_cmd_r;
    cmds[0].position = psim_cmd_r;
    cmds[3].position = phim_cmd_l;
    cmds[2].position = psim_cmd_l; //TODO: check these

    // Uncommet to run
    pi3_interface.write(cmds);

    //pi3_interface.stop();

    right_finger_kinematics.forwardKinematics(phi_feed_r, psi_feed_r, 1, conf_feed_r);
    left_finger_kinematics.forwardKinematics(phi_feed_l, psi_feed_l, -1, conf_feed_l);

    log_file << time << "," << conf_cmd_r(0) << "," << conf_cmd_r(1) 
	    << "," << conf_cmd_r(2) << "," << conf_cmd_r(3) 
	    << "," << conf_cmd_r(4) << "," << conf_cmd_r(5)
      	<< "," << conf_feed_r(0) << "," << conf_feed_r(1) 
	    << "," << conf_feed_r(2) << "," << conf_feed_r(3) 
	    << "," << conf_feed_r(4) << "," << conf_feed_r(5)
	    << "," << conf_cmd_l(0) << "," << conf_cmd_l(1) 
	    << "," << conf_cmd_l(2) << "," << conf_cmd_l(3) 
	    << "," << conf_cmd_l(4) << "," << conf_cmd_l(5)
      	<< "," << conf_feed_l(0) << "," << conf_feed_l(1) 
	    << "," << conf_feed_l(2) << "," << conf_feed_l(3) 
	    << "," << conf_feed_l(4) << "," << conf_feed_l(5) 
      << "," << cmds[1].feedforward_torque
      << "," << cmds[0].feedforward_torque
      << "," << cmds[3].feedforward_torque
      << "," << cmds[2].feedforward_torque
      << "," << resp[1].torque
      << "," << resp[0].torque
      << "," << resp[3].torque
      << "," << resp[2].torque
      << "," << resp[1].q_current
      << "," << resp[0].q_current
      << "," << resp[3].q_current
      << "," << resp[2].q_current
      << "," << resp[1].d_current
      << "," << resp[0].d_current
      << "," << resp[3].d_current
      << "," << resp[2].d_current
      << endl;
      
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
} 

void updateCurrentPos(FiveBarKinematics& left_finger_kinematics, FiveBarKinematics& right_finger_kinematics, Pi3HatInterface& pi3_interface, 
std::vector<MoteusCommand>& cmds, std::vector<MoteusResponse>& resp)
{
  double phim_feed_l, psim_feed_l, phim_feed_r, psim_feed_r;

  auto ti = high_resolution_clock::now();
  auto timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);

  double time = 0;

  while(time <= 0.5)  
  {
    timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);
    time = timeD.count()*1e-6;

    orderResponse(pi3_interface.read(), resp);

    phim_feed_r = resp[1].position;
    psim_feed_r = resp[0].position;
    phim_feed_l = resp[3].position;
    psim_feed_l = resp[2].position;
    
    right_finger_kinematics.getRelativeAngles(phim_feed_r, psim_feed_r, phi_feed_r, psi_feed_r);
    left_finger_kinematics.getRelativeAngles(phim_feed_l, psim_feed_l, phi_feed_l, psi_feed_l);
        
    phi_cmd_r = phi_feed_r;
    psi_cmd_r = psi_feed_r;
    phi_cmd_l = phi_feed_l;
    psi_cmd_l = psi_feed_l;
    
    pi3_interface.stop();

    std::this_thread::sleep_for(std::chrono::milliseconds(10));  
  }
}

void output_mode_switch(const char* op_mode, FiveBarKinematics& right_finger_kinematics, FiveBarKinematics& left_finger_kinematics, Pi3HatInterface& pi3_interface, 
          std::vector<MoteusCommand>& cmds, std::vector<MoteusResponse>& resp)
{

  double joint_ini_right_finger[2];
  double joint_fin_right_finger[2];
  double joint_ini_left_finger[2];
  double joint_fin_left_finger[2];
  double p2p_time = 3;
  
  string in_an, phi_r, psi_r, phi_l, psi_l;

  fstream angles;

  if(op_mode == "gripp")
    angles.open("../verify_codes/sense_to_gripp.csv", ios::in);
  else
    angles.open("../verify_codes/gripp_to_sense.csv", ios::in);

  //getCurrentPos(phi_feed_r, psi_feed_r, phi_feed_l, psi_feed_l, left_finger_kinematics, right_finger_kinematics, pi3_interface, cmds, resp);

  joint_ini_right_finger[0] = phi_cmd_r;
  joint_ini_right_finger[1] = psi_cmd_r;
  joint_ini_left_finger[0] = phi_cmd_l;
  joint_ini_left_finger[1] = psi_cmd_l;

  while(angles >> in_an)
  {  
    stringstream input_angles(in_an);
  
    std::getline(input_angles, phi_r, ',');
    std::getline(input_angles, psi_r, ',');
    std::getline(input_angles, phi_l, ',');
    std::getline(input_angles, psi_l, ',');
    
    joint_fin_right_finger[0] = stod(phi_r);
    joint_fin_right_finger[1] = stod(psi_r);
    joint_fin_left_finger[0] = stod(phi_l);
    joint_fin_left_finger[1] = stod(psi_l);
    
    cout << "phi_r: " << joint_fin_right_finger[0] << ", psi_r: " << joint_fin_right_finger[1]
         << ", phi_l: " << joint_fin_left_finger[0] << ", psi_l: " << joint_fin_right_finger[1] << endl;
  
    p2p(joint_ini_right_finger, joint_fin_right_finger, joint_ini_left_finger, joint_fin_left_finger, p2p_time, right_finger_kinematics, left_finger_kinematics, pi3_interface, cmds, resp);

    joint_ini_right_finger[0] = joint_fin_right_finger[0];
    joint_ini_right_finger[1] = joint_fin_right_finger[1];
    joint_ini_left_finger[0] = joint_fin_left_finger[0];
    joint_ini_left_finger[1] = joint_fin_left_finger[1];
    p2p_time = 0.08;    
  }

  // TODO: check these
  if(op_mode == "gripp")
  {
    pm_r[0] = -1;
    pm_r[1] = 1;  
    pm_l[0] = 1;
    pm_l[1] = -1;
    right_finger_kinematics.addIKAngleOffsets(2*M_PI, 0);
    left_finger_kinematics.addIKAngleOffsets(-2*M_PI, 0);
  }
  else
  {
    pm_r[0] = 1;
    pm_r[1] = -1;  
    pm_l[0] = -1;
    pm_l[1] = 1;
    right_finger_kinematics.addIKAngleOffsets(-2*M_PI, 0);
    left_finger_kinematics.addIKAngleOffsets(2*M_PI, 0);
  }

}

void gripp(Vector<double, 2> P_open_r, Vector<double, 2> P_open_l, Vector<double, 2> P_close_r, Vector<double, 2> P_close_l, Vector<double, 2> F_r, Vector<double, 2> F_l, double hold_timeout, FiveBarKinematics& right_finger_kinematics, FiveBarKinematics& left_finger_kinematics, Pi3HatInterface& pi3_interface, 
          std::vector<MoteusCommand>& cmds, std::vector<MoteusResponse>& resp)
{
  Vector<double, 6> conf_cmd_r, conf_cmd_l;

  // go to specified pose
  double joint_ini_right_finger[2];
  double joint_fin_right_finger[2];
  double joint_ini_left_finger[2];
  double joint_fin_left_finger[2];
  double p2p_time = 3;

  right_finger_kinematics.inverseKinematics(P_open_r, pm_r[0], pm_r[1], conf_cmd_r);
  left_finger_kinematics.inverseKinematics(P_open_l, pm_l[0], pm_l[1], conf_cmd_l);

  joint_ini_right_finger[0] = phi_cmd_r;
  joint_ini_right_finger[1] = psi_cmd_r;
  joint_ini_left_finger[0] = phi_cmd_l;
  joint_ini_left_finger[1] = psi_cmd_l;
  joint_fin_right_finger[0] = conf_cmd_r(0);
  joint_fin_right_finger[1] = conf_cmd_r(1);
  joint_fin_left_finger[0] = conf_cmd_l(0);
  joint_fin_left_finger[1] = conf_cmd_l(1);

  p2p(joint_ini_right_finger, joint_fin_right_finger, joint_ini_left_finger, joint_fin_left_finger, p2p_time, right_finger_kinematics, left_finger_kinematics, pi3_interface, cmds, resp);

  // apply specified force
  Matrix<double, 2, 2> J_l, J_r;
  Vector<double, 2> torque_l, torque_r;

  right_finger_kinematics.jacobian(conf_cmd_r, J_r);
  left_finger_kinematics.jacobian(conf_cmd_l, J_l);

  torque_r = J_r.transpose()*F_r;
  torque_l = J_l.transpose()*F_l;
  
  cout << torque_r << endl;
  cout << torque_l << endl;

  double path_time_r = 3, path_time_l = 3, n_r = 1, n_l = 1;
  
  linear_motion(P_open_r, P_close_r, pm_r, path_time_r, n_r, P_open_l, P_close_l, pm_l, path_time_l, n_l, true, right_finger_kinematics, left_finger_kinematics, pi3_interface, cmds, resp);

  hold(torque_r, torque_l, hold_timeout, right_finger_kinematics, left_finger_kinematics, pi3_interface, cmds, resp);

  joint_ini_right_finger[0] = phi_cmd_r;
  joint_ini_right_finger[1] = psi_cmd_r;
  joint_ini_left_finger[0] = phi_cmd_l;
  joint_ini_left_finger[1] = psi_cmd_l;
  joint_fin_right_finger[0] = conf_cmd_r(0);
  joint_fin_right_finger[1] = conf_cmd_r(1);
  joint_fin_left_finger[0] = conf_cmd_l(0);
  joint_fin_left_finger[1] = conf_cmd_l(1);

  p2p(joint_ini_right_finger, joint_fin_right_finger, joint_ini_left_finger, joint_fin_left_finger, p2p_time, right_finger_kinematics, left_finger_kinematics, pi3_interface, cmds, resp);

  //release(1, right_finger_kinematics, left_finger_kinematics, pi3_interface, cmds, resp);

}

int main(void)
{
  //Pi3hat initialize
  Pi3HatInterface pi3_interface;

  pi3_interface.initialize(servo_bus_map);

  std::vector<MoteusCommand> cmds;
  std::vector<MoteusResponse> resp;
    
  // Dimensions of right finger
  Matrix<double, 2, 1> A0r, B0r, C0r, D0r, F0r, P0r;
  Matrix<double, 2, 6> dim_right_finger;
  Vector<double, 6> conf_feed_r, conf_cmd_r;

  double phi_offset_r = 2.02255;
  double psi_offset_r = 0.34228;
  int phi_sign_r = -1;
  int psi_sign_r = 1;

  A0r << 0.015, -0.045;
  B0r << 0.040919949196745915, 0;
  C0r << -0.03282476284977254, -0.041606786610356335;
  D0r << 0.04304300866986588, 0.011646970752102255;
  F0r << 0.026687870426209, -0.03397484373490171;
  P0r << 0.0101739395037219, 0.0520899255487229;

  // Setting up five-bar kinematics
  dim_right_finger << A0r(0), B0r(0), C0r(0), D0r(0), F0r(0), P0r(0),
          A0r(1), B0r(1), C0r(1), D0r(1), F0r(1), P0r(1),

  cout << "Right finger dim: " << endl << dim_right_finger << endl;

  FiveBarKinematics right_finger_kinematics(dim_right_finger, phi_offset_r, psi_offset_r, phi_sign_r, psi_sign_r);

  // Dimensions of left finger
  Matrix<double, 2, 1> A0l, B0l, C0l, D0l, F0l, P0l;
  Matrix<double, 2, 6> dim_left_finger;
  Vector<double, 6> conf_feed_l, conf_cmd_l;

  double phi_offset_l = -2.07074;//-2.02255;
  double psi_offset_l = -0.34228;
  int phi_sign_l = 1;
  int psi_sign_l = -1;

  A0l << -0.015, -0.045;
  B0l << -0.040919949196745915, 0;
  C0l << 0.03282476284977254, -0.041606786610356335;
  D0l << -0.04304300866986588, 0.011646970752102255;
  F0l << -0.026687870426209, -0.03397484373490171;
  P0l << -0.0101739395037219, 0.0520899255487229;

  // Setting up five-bar kinematics
  dim_left_finger << A0l(0), B0l(0), C0l(0), D0l(0), F0l(0), P0l(0),
          A0l(1), B0l(1), C0l(1), D0l(1), F0l(1), P0l(1),

  cout << "Left finger dim: " << endl << dim_left_finger << endl;

  FiveBarKinematics left_finger_kinematics(dim_left_finger, phi_offset_l, psi_offset_l, phi_sign_l, psi_sign_l);

  // Setting up motor commands
  cmds.resize(4);
  cmds[0].id = 1;
  cmds[0].mode = 10; // Position mode
  cmds[0].position = 0.0;
  cmds[0].velocity = 0.0;
  cmds[0].feedforward_torque = 0;
  cmds[0].kp_scale = 4;
  cmds[0].kd_scale = 2;
  cmds[0].watchdog_timeout = 0;

  cmds[1].id = 2;
  cmds[1].mode = 10; // Position mode
  cmds[1].position = 0.0;
  cmds[1].velocity = 0.0;
  cmds[1].feedforward_torque = 0;
  cmds[1].kp_scale = 4;
  cmds[1].kd_scale = 2;
  cmds[1].watchdog_timeout = 0;

  cmds[2].id = 3;
  cmds[2].mode = 10; // Position mode
  cmds[2].position = 0.0;
  cmds[2].velocity = 0.0;
  cmds[2].feedforward_torque = 0;
  cmds[2].kp_scale = 4;
  cmds[2].kd_scale = 2;
  cmds[2].watchdog_timeout = 0;

  cmds[3].id = 4;
  cmds[3].mode = 10; // Position mode
  cmds[3].position = 0.0;
  cmds[3].velocity = 0.0;
  cmds[3].feedforward_torque = 0;
  cmds[3].kp_scale = 4;
  cmds[3].kd_scale = 2;
  cmds[3].watchdog_timeout = 0;
  
  Vector<double, 2> pI_r, pF_r, pI_l, pF_l;

  pI_r << 0.01, 0.04;
  pF_r << 0.04, 0.04;
  pI_l << -0.01, 0.04;
  pF_l << -0.04, 0.04;

  double path_time_r = 2;
  double path_time_l = 2;
  double n_r = 4;
  double n_l = 4;
  
  Vector<double, 2> torque_r, F_r;
  Vector<double, 2> torque_l, F_l;
  double hold_time = 2.5;

  // Start all motors in stopped mode to clear all faults
  pi3_interface.stop();
  // Wait for the stop command to be sent 
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  resp.resize(4);

  updateCurrentPos(left_finger_kinematics, right_finger_kinematics, pi3_interface, cmds, resp);

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  
  right_finger_kinematics.inverseKinematics(pI_r, pm_r[0], pm_r[1], conf_cmd_r);
  left_finger_kinematics.inverseKinematics(pI_l, pm_l[0], pm_l[1], conf_cmd_l);

  double joint_ini_right_finger[2] = {phi_feed_r, psi_feed_r};
  double joint_fin_right_finger[2] = {conf_cmd_r(0), conf_cmd_r(1)};
  double joint_ini_left_finger[2] = {phi_feed_l, psi_feed_l};
  double joint_fin_left_finger[2] = {conf_cmd_l(0), conf_cmd_l(1)};
  double p2p_time = 3;

  //std::cout << "p2p" << std::endl;

  //p2p(joint_ini_right_finger, joint_fin_right_finger, joint_ini_left_finger, joint_fin_left_finger, p2p_time, right_finger_kinematics, left_finger_kinematics, pi3_interface, cmds, resp);

  //std::cout << "linear" << std::endl;

  //linear_motion(pI_r, pF_r, pm_r, path_time_r, n_r, pI_l, pF_l, pm_l, path_time_l, n_l, right_finger_kinematics, left_finger_kinematics, pi3_interface, cmds, resp);

  //std::cout << "output mode switch" << std::endl;

  //output_mode_switch("gripp", right_finger_kinematics, left_finger_kinematics, pi3_interface, cmds, resp);

  pI_r << 0.02, 0.04;
  pI_l << -0.02, 0.04;
  pF_r << -0.005, 0.04;
  pF_l << 0.005, 0.04;

  path_time_r = 2;
  path_time_l = 2;
  n_r = 4;
  n_l = 4;
  
  right_finger_kinematics.inverseKinematics(pI_r, pm_r[0], pm_r[1], conf_cmd_r);
  left_finger_kinematics.inverseKinematics(pI_l, pm_l[0], pm_l[1], conf_cmd_l);

  joint_ini_right_finger[0] = phi_cmd_r;
  joint_ini_right_finger[1] = psi_cmd_r;
  joint_fin_right_finger[0] = conf_cmd_r(0);
  joint_fin_right_finger[1] = conf_cmd_r(1);
  joint_ini_left_finger[0] = phi_cmd_l;
  joint_ini_left_finger[1] = psi_cmd_l;
  joint_fin_left_finger[0] = conf_cmd_l(0);
  joint_fin_left_finger[1] = conf_cmd_l(1);
  p2p_time = 3;

  std::cout << "p2p" << std::endl;

  p2p(joint_ini_right_finger, joint_fin_right_finger, joint_ini_left_finger, joint_fin_left_finger, p2p_time, right_finger_kinematics, left_finger_kinematics, pi3_interface, cmds, resp);

  std::cout << "hold" << endl;

  torque_r << 0, 0;
  torque_l << 0, 0;

  hold(torque_r, torque_l, hold_time, right_finger_kinematics, left_finger_kinematics, pi3_interface, cmds, resp);

  std::cout << "gripp" << endl;

  F_r << -0, 0;
  F_l << 0, 0;
  hold_time = 33;

  gripp(pI_r, pI_l, pF_r, pF_l, F_r, F_l, hold_time, right_finger_kinematics, left_finger_kinematics, pi3_interface, cmds, resp);

  //std::cout << "output mode switch" << std::endl;

  //output_mode_switch("sense", right_finger_kinematics, left_finger_kinematics, pi3_interface, cmds, resp);

  updateCurrentPos(left_finger_kinematics, right_finger_kinematics, pi3_interface, cmds, resp);
  
  return 0;
}
