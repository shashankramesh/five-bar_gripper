/*
 * file: test_2R_pow_con.cpp
 *
 * Created: 21 Jan, 2023
 * Author: Shashank Ramesh
 */

#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>
#include <fstream>
#include <future>
#include <vector>
#include <cmath>
#include <chrono>
#include <mutex>

#include "pi3hat/pi3hat_interface.h"
#include "2R_kinematics.h"

using namespace std::chrono;

bool get_flag = false;
double current = 0;
std::mutex thread_mutex;

const std::map<int, int> servo_bus_map ={
  {1,1},
  {2,1}
};

inline void updateMaximum(double x, double& x_max)
{
  if(x > x_max)
    x_max = x;
}

inline void updateMinimum(double x, double& x_min)
{
  if(x < x_min)
    x_min = x;
}

void getCurrent(void)
{
  double curr;
  printf("Enter current flow (A): ");  
  cin >> curr;

  {
    std::lock_guard<std::mutex> guard(thread_mutex);
    current = curr;
    get_flag = true;
  }
  
  return;
}

int main(int argc, char** argv)
{
  Matrix<double, 2, 1> A0, B0, C0, D0, F0, P0;
  Matrix<double, 2, 6> dim5bar;
  Vector<double, 6> conf_feed;
  Vector<double, 6> conf_cmd;
  Vector<double, 6> conf_ini, conf_fin;
  Matrix<double, 2, 1> P;

  double time;
  double u;
  double phi, psi;
  double phi_ini, phi_fin, psi_ini, psi_fin;
  double phi_cmd, phim, phi_feed, phim_feed;
  double psi_cmd, psim, psi_feed, psim_feed;
  double torque1, torque2;
  double curr = 0, flag = false;
  int i = 1, j = 1, k = 1;
  double freq;
  Vector<double, 2> P, P_feed, pC, pU;

  Pi3HatInterface pi3_interface;

  pi3_interface.initialize(servo_bus_map);

  std::vector<MoteusCommand> cmds;
  std::vector<MoteusResponse> resp;

  // Dimensions of the five-bar mechanism
  A0 << 0.12, 0;
  B0 << 0, 0;
  C0 << 0.09216732905642444, -0.039749173530376616;
  D0 << -0.011227449650384648, -0.02407734347492251;
  F0 << 0.027234061360222564, -0.06273044035186855;
  P0 << 0.10818369270155971, -0.17033426671218643;

  double phi_offset = -61*M_PI/36;
  double psi_offset = 13*M_PI/36;

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

  theta1_ini = -theta1_offset;
  theta2_ini = -theta2_offset;

  fstream points;
  
  points.open("workspace_2R_points/workspace_2R_points_1cm2.csv", ios::in);

  ofstream log("max_tracking_freq_2R/pow_con_23_01_21_200g_v2.csv");

  log << "time,theta1,theta2,theta1_feed,theta2_feed,x,y,torque1,torque2,current" << endl;

  auto ti = high_resolution_clock::now();
  auto timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);
  
  time = 0;

  string pts, xcoord, ycoord;
  
  while(points >> pts)
  {  
    stringstream xy(pts);
  
    std::getline(xy, xcoord, ',');
    
    std::getline(xy, ycoord, ',');
    
    pC(0) = stod(xcoord);
    pC(1) = stod(ycoord);
    
    cout << "Point: " << j << endl;
    cout << "X: " << pC(0) << ", Y: " << pC(1) << endl;
    j++;

    twor_kinematics.inverseKinematics(pC, ik_branch, theta1_fin, theta2_fin);

    ti = high_resolution_clock::now();
    time = 0;

    while(time <= p2p_time)
    {
      timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);
      time = timeD.count()*1e-6;

      // Feedback
      resp = pi3_interface.read();
      
      twor_kinematics.getRelativeAngles(resp[0].position, resp[1].position, theta1_feed, theta2_feed);

      theta1 = theta1_ini + (time*(theta1_fin - theta1_ini)/p2p_time);
      theta2 = theta2_ini + (time*(theta2_fin - theta2_ini)/p2p_time);
      
      twor_kinematics.forwardKinematics(theta1, theta2, P);
      
      // Command
      twor_kinematics.getMotorAngles(theta1, theta2, theta1m, theta2m);

      cmds[0].position = theta1m; //TODO: beware of the correspondance
      cmds[1].position = theta2m;
      pi3_interface.write(cmds); //TODO: commented for safety
      
      //pi3_interface.stop();    

      /*log << time << "," << theta1 << "," << theta2
        << "," << theta1_feed << "," << theta2_feed
        << "," << P(0) << "," << P(1) << endl;*/

      /*cout << time << "," << theta1 << "," << theta2
        << "," << theta1m << "," << theta2m
        << "," << P(0) << "," << P(1) << endl; */

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    {
      std::lock_guard<std::mutex> guard(thread_mutex);
      get_flag = false;
    }

    flag = false;
    std::thread current_thread(getCurrent);
    
    ti = high_resolution_clock::now();
    time = 0;
    
    while(!flag)
    {

      timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);
      time = timeD.count()*1e-6;

      // Feedback
      resp = pi3_interface.read();
      
      theta1m_feed = resp[0].position;
      theta2m_feed = resp[1].position;
      
      torque1 = resp[0].torque;
      torque2 = resp[1].torque;
      
      twor_kinematics.getRelativeAngles(theta1m_feed, theta2m_feed, 
          theta1_feed, theta2_feed);

      //twor_kinematics.forwardKinematics(theta1_feed, theta2_feed, P_feed);
      
      {
        std::lock_guard<std::mutex> guard(thread_mutex);
        flag = get_flag;
        curr = current;
      }
            
      if(time >= test_start_time)
      {          
        
        log << time << "," << theta1 << "," << theta2
        << "," << theta1_feed << "," << theta2_feed
        << "," << pC(0) << "," << pC(1) << "," 
        << torque1 << "," << torque2
        << "," << curr << endl;
        
        k++;
      }
      
      cmds[0].position = theta1m; //TODO: beware of the correspondance
      cmds[1].position = theta2m;
      pi3_interface.write(cmds);
      
      //pi3_interface.stop();
      
      std::this_thread::sleep_for(std::chrono::milliseconds(1));

/*      cout << time << "," << theta1 << "," << theta2
        << "," << theta1m << "," << theta2m
        << "," << P(0) << "," << P(1) << endl; */
   
    }
    
    current_thread.join();
    cout << "The read value of current is: " << current << endl;
  
    theta1_ini = theta1;
    theta2_ini = theta2;
    
    log << endl;
  
  }

  pi3_interface.stop();

  return 0; 
} 
