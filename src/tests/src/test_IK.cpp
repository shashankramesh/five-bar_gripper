/*
 * file: test_IKcpp
 *
 * Created: 27 Aug, 2024
 * Author: Shashank Ramesh
 */

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <future>
#include <vector>
#include <cmath>
#include <chrono>

#include "pi3hat/pi3hat_interface.h"
#include "kinematics.h"

using namespace std::chrono;

const std::map<int, int> servo_bus_map ={
  {1,1},
  {2,1},
  {3,1},
  {4,1}
}; // TODO: check ids and bus number, map to correct finger

int main(int argc, char** argv)
{
  Pi3HatInterface pi3_interface;

  pi3_interface.initialize(servo_bus_map);

  std::vector<MoteusCommand> cmds;
  std::vector<MoteusResponse> resp;

  std::vector<uint8_t> resp_buffer;
  resp_buffer.reserve(1024);

  double time;
  double total_time;
  double phi_cmd_l, phim_l, phi_feed_l, phim_feed_l;
  double psi_cmd_l, psim_l, psi_feed_l, psim_feed_l;
  double phi_cmd_r, phim_r, phi_feed_r, phim_feed_r;
  double psi_cmd_r, psim_r, psi_feed_r, psim_feed_r;

  // Dimensions of right finger
  Matrix<double, 2, 1> A0r, B0r, C0r, D0r, F0r, P0r;
  Matrix<double, 2, 6> dim_right_finger;
  Vector<double, 6> conf_feed_r, conf_cmd_r;

  double phi_offset_r = -61*M_PI/36; //TODO: set these
  double psi_offset_r = 13*M_PI/36; //TODO: set these
  int phi_sign_r = -1; // TODO: set these
  int psi_sign_r = 1; // TODO: set these

  A0r << 0.015, -0.045;
  B0r << -0.040919949196745915, 0;
  C0r << 0.026909920860331303, -0.021713086177873948;
  D0r << -0.038931473927022224, 0.011670701578338173;
  F0r << -0.03131683587683813, -0.03619220463744048;
  P0r << -0.0471521851343047, 0.04999999999992947;

  // Setting up five-bar kinematics
  dim_right_finger << A0r(0), B0r(0), C0r(0), D0r(0), F0r(0), P0r(0),
          A0r(1), B0r(1), C0r(1), D0r(1), F0r(1), P0r(1),

  cout << "Right finger dim: " << endl << dim_left_finger << endl;

  FiveBarKinematics right_finger_kinematics(dim_right_finger, phi_offset_r, psi_offset_r, phi_sign_r, psi_sign_r);

  // Dimensions of left finger
  Matrix<double, 2, 1> A0l, B0l, C0l, D0l, F0l, P0l;
  Matrix<double, 2, 6> dim_left_finger;
  Vector<double, 6> conf_feed_l, conf_cmd_l;

  double phi_offset_l = -61*M_PI/36; //TODO: set these
  double psi_offset_l = 13*M_PI/36; //TODO: set these
  int phi_sign_l = -1; // TODO: set these
  int psi_sign_l = 1; // TODO: set these

  A0l << 0.015, -0.045;
  B0l << 0.04091994919674591, 0;
  C0l << -0.02690992086033131, -0.02171308617787395;
  D0l << 0.038931473927022213, 0.011670701578338172;
  F0l << 0.03131683587683813, -0.03619220463744048;
  P0l << 0.0471521851343047, 0.04999999999992947;

  // Setting up five-bar kinematics
  dim_left_finger << A0l(0), B0l(0), C0l(0), D0l(0), F0l(0), P0l(0),
          A0l(1), B0l(1), C0l(1), D0l(1), F0l(1), P0l(1),

  cout << "Left finger dim: " << endl << dim_left_finger << endl;

  FiveBarKinematics left_finger_kinematics(dim_left_finger, phi_offset_l, psi_offset_l, phi_sign_l, psi_sign_l);

  // Setting up motor commands
  cmds.resize(4);
  cmds[0].id = 1; // TODO: check id
  cmds[0].mode = 10; // Position mode
  cmds[0].position = 0.0;
  cmds[0].velocity = 0.0;
  cmds[0].feedforward_torque = 0;
  cmds[0].kp_scale = 1;
  cmds[0].kd_scale = 1;
  cmds[0].watchdog_timeout = 0;

  cmds[1].id = 2; // TODO: check id
  cmds[1].mode = 10; // Position mode
  cmds[1].position = 0.0;
  cmds[1].velocity = 0.0;
  cmds[1].feedforward_torque = 0;
  cmds[1].kp_scale = 1;
  cmds[1].kd_scale = 1;
  cmds[1].watchdog_timeout = 0;

  cmds[2].id = 3; // TODO: check id
  cmds[2].mode = 10; // Position mode
  cmds[2].position = 0.0;
  cmds[2].velocity = 0.0;
  cmds[2].feedforward_torque = 0;
  cmds[2].kp_scale = 1;
  cmds[2].kd_scale = 1;
  cmds[2].watchdog_timeout = 0;

  cmds[3].id = 4; // TODO: check id
  cmds[3].mode = 10; // Position mode
  cmds[3].position = 0.0;
  cmds[3].velocity = 0.0;
  cmds[3].feedforward_torque = 0;
  cmds[3].kp_scale = 1;
  cmds[3].kd_scale = 1;
  cmds[3].watchdog_timeout = 0;

  // Start all motors in stopped mode to clear all faults
  pi3_interface.stop();
  // Wait for the stop command to be sent 
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  total_time = 2;

  auto ti = high_resolution_clock::now();
  auto timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);

// time <= total_time
  while(true)
  {

    timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);
    time = timeD.count()*1e-6;

    // Feedback
    resp = pi3_interface.read();
    
    phim_feed_r = resp[1].position; //TODO: check mappings!
    psim_feed_r = resp[0].position; //TODO: check mappings!
    phim_feed_l = resp[3].position; //TODO: check mappings!
    psim_feed_l = resp[4].position; //TODO: check mappings!
    
    right_finger_kinematics.getRelativeAngles(phim_feed_r, psim_feed_r, phi_feed_r, psi_feed_r);
    left_finger_kinematics.getRelativeAngles(phim_feed_l, psim_feed_l, phi_feed_l, psi_feed_l);

    // left finger: 1, right finger: -1
    right_finger_kinematics.forwardKinematics(phi_feed_r, psi_feed_r, -1, conf_feed_r);
    left_finger_kinematics.forwardKinematics(phi_feed_l, psi_feed_l, 1, conf_feed_l);
    
    P_r << conf_feed_r(4), conf_feed_r(5);
    P_l << conf_feed_l(4), conf_feed_l(5);
    
    // for right finger mode-2: sense (flat ve) {1, -1}, mode-3: gripp (vertical ve) {-1, 1}
    right_finger_kinematics.inverseKinematics(P_r, 1, -1, conf_cmd_r);
    // for left finger mode-2: grip (vertical ve) {1, -1}, mode-3: sense (flat ve) {-1, 1}
    left_finger_kinematics.inverseKinematics(P_l, -1, 1, conf_cmd_l);
    
    // Command
    pi3_interface.stop();
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    std::cout << "Left finger feedback configuration: " << conf_feed_l << endl
      << "Left finger command configuration: " << conf_cmd_l << endl
      << "Left phi feed: " << phi_feed_l << endl
      << "Left psi feed: " << psi_feed_l << endl
      << std::endl;

    std::cout << "Right finger feedback configuration: " << conf_feed_r << endl
      << "Right finger command configuration: " << conf_cmd_r << endl
      << "Right phi feed: " << phi_feed_r << endl
      << "Right psi feed: " << psi_feed_r << endl
      << std::endl;
 
  }
  pi3_interface.stop();
  return 0; 
} 
