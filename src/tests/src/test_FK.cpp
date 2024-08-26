/*
 * file: test_position_signs.cpp
 *
 * Created: 29 Nov, 2022
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
  {2,1}
};

int main(int argc, char** argv)
{
  Pi3HatInterface pi3_interface;

  pi3_interface.initialize(servo_bus_map);

  std::vector<MoteusCommand> cmds;
  std::vector<MoteusResponse> resp;

  std::vector<uint8_t> resp_buffer;
  resp_buffer.reserve(1024);

  // Dimensions of the five-bar mechanism
  Matrix<double, 2, 1> A0, B0, C0, D0, F0, P0;
  Matrix<double, 2, 6> dim5bar;
  Vector<double, 6> conf_feed;

  double time;
  double total_time;
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

  // Setting up motor commands
  cmds.resize(2);
  cmds[0].id = 1;
  cmds[0].mode = 10; // Position mode
  cmds[0].position = 0.0;
  cmds[0].velocity = 0.0;
  cmds[0].feedforward_torque = 0;
  cmds[0].kp_scale = 1;
  cmds[0].kd_scale = 1;
  cmds[0].watchdog_timeout = 0;

  cmds[1].id = 2;
  cmds[1].mode = 10; // Position mode
  cmds[1].position = 0.0;
  cmds[1].velocity = 0.0;
  cmds[1].feedforward_torque = 0;
  cmds[1].kp_scale = 1;
  cmds[1].kd_scale = 1;
  cmds[1].watchdog_timeout = 0;

  // Start all motors in stopped mode to clear all faults
  pi3_interface.stop();
  // Wait for the stop command to be sent 
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  total_time = 5;

  auto ti = high_resolution_clock::now();
  auto timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);

// time <= total_time
  while(true)
  {

    timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);
    time = timeD.count()*1e-6;

    // Feedback
    resp = pi3_interface.read();
    
    phim_feed = resp[0].position;
    psim_feed = resp[1].position;
    
    five_bar_kinematics.getRelativeAngles(phim_feed, psim_feed, phi_feed, psi_feed);

    five_bar_kinematics.forwardKinematics(phi_feed, psi_feed, -1, conf_feed);
    
    // Command
    pi3_interface.stop();
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    std::cout << " Configuration: " << conf_feed
      << std::endl;
 
  }
  pi3_interface.stop();
  return 0; 
} 
