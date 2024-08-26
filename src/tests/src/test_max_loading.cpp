/*
 * file: test_pow_con.cpp
 *
 * Created: 25 Jan, 2023
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

#include "kinematics.h"
#include "pi3hat/pi3hat_interface.h"
#include "power_monitor.h"

using namespace std::chrono;

rpi_power_monitor::RpiPowerMonitor pm_phi(1, 0x40);  
rpi_power_monitor::RpiPowerMonitor pm_psi(1, 0x41);  

float voltage_phi = 0, voltage_psi = 0;
float current_phi = 0, current_psi = 0;
float power_phi = 0, power_psi = 0;
std::mutex thread_mutex;

const std::map<int, int> servo_bus_map ={
  {1,1},
  {2,1}
};

void logPowerData()
{
  while(true)
  {
    std::lock_guard<std::mutex> guard(thread_mutex);
    pm_phi.getIP(current_phi, power_phi);
    pm_psi.getIP(current_psi, power_psi);
  }
}

inline void exponentialFilterTemperature(double input, double& output, double alpha)
{
  output = alpha*input + (1-alpha)*output;
}

int main(int argc, char** argv)
{
  Matrix<double, 2, 1> A0, B0, C0, D0, F0, P0;
  Matrix<double, 2, 6> dim5bar;
  Vector<double, 6> conf_feed;
  Vector<double, 6> conf_cmd;
  Vector<double, 6> conf_ini, conf_fin;
  Matrix<double, 2, 1> P;

  double time, print_time;
  double u;
  double phi, psi;
  double phi_ini, phi_fin, psi_ini, psi_fin;
  double phi_cmd, phim, phi_feed, phim_feed;
  double psi_cmd, psim, psi_feed, psim_feed;
  double phi_offset = -61*M_PI/36;
  double psi_offset = 13*M_PI/36;
  double torque1, torque2;
  double temp1, temp2;
  int i = 1, j = 1, k = 1;
  double curr = 0, flag = false;
  double freq;
  Vector<double, 2> P_feed, pC, pU;

  Pi3HatInterface pi3_interface;

  pi3_interface.initialize(servo_bus_map);

  std::vector<MoteusCommand> cmds;
  std::vector<MoteusResponse> resp;

  std::vector<uint8_t> resp_buffer;
  resp_buffer.reserve(1024);

  // Dimensions of the five-bar mechanism
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

  double p2p_time = 5;
  double print_period = 1;
  double total_time = 60*5;
  double dock_conf_phi = -0.36, dock_conf_psi = -1.85;
  int ikb1 = -1, ikb2 = -1;

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

  phi_ini = dock_conf_phi;
  psi_ini = dock_conf_psi;

  fstream points;
  
  points.open("workspace_points_5bar/workspace_points_max_loading.csv", ios::in);

  ofstream log("max_loading_5R/max_loading_23_06_29_800g_v1.csv");

  log << "time,phi_cmd,psi_cmd,rho_cmd,theta_cmd,x_cmd,y_cmd,phi_feed,psi_feed,rho_feed,theta_feed,x_feed,y_feed,torque1,torque2,temp1,temp2,current_phi,current_psi" << endl;

  auto ti = high_resolution_clock::now();
  auto timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);
  
  time = 0;

  string pts, xcoord, ycoord;

  std::thread current_thread(logPowerData); 

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

    five_bar_kinematics.inverseKinematics(pC, ikb1, ikb2, conf_fin);
    
    phi_fin = conf_fin(0);
    psi_fin = conf_fin(1);

    ti = high_resolution_clock::now();
    time = 0;

    while(time <= p2p_time)
    {
      timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);
      time = timeD.count()*1e-6;

      // Feedback
      resp = pi3_interface.read();

      phim_feed = resp[0].position;
      psim_feed = resp[1].position;
      
      five_bar_kinematics.getRelativeAngles(phim_feed, psim_feed, phi_feed, psi_feed);

      five_bar_kinematics.forwardKinematics(phi_feed, psi_feed, -1, conf_feed);

      phi_cmd = phi_ini + (time*(phi_fin - phi_ini)/p2p_time);
      psi_cmd = psi_ini + (time*(psi_fin - psi_ini)/p2p_time);

      five_bar_kinematics.forwardKinematics(phi_cmd, psi_cmd, -1, conf_cmd);
            
      // Command
      five_bar_kinematics.getMotorAngles(phi_cmd, psi_cmd, phim, psim);

      cmds[0].position = phim;
      cmds[1].position = psim;
      pi3_interface.write(cmds);
      
      //pi3_interface.stop();    

      /*log << time << "," << conf_cmd(0) << "," << conf_cmd(1) 
        << "," << conf_cmd(2) << "," << conf_cmd(3) 
        << "," << conf_cmd(4) << "," << conf_cmd(5)
        << "," << conf_feed(0) << "," << conf_feed(1) 
        << "," << conf_feed(2) << "," << conf_feed(3) 
        << "," << conf_feed(4) << "," << conf_feed(5) << endl;*/

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    ti = high_resolution_clock::now();
    time = 0;
    print_time = 0;   
    
    while(time <= total_time)
    {

      timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);
      time = timeD.count()*1e-6;

      // Feedback
      resp = pi3_interface.read();
      
      phim_feed = resp[0].position;
      psim_feed = resp[1].position;
      
      torque1 = resp[0].torque;
      torque2 = resp[1].torque;
      
      five_bar_kinematics.getRelativeAngles(phim_feed, psim_feed, phi_feed, psi_feed);

      five_bar_kinematics.forwardKinematics(phi_feed, psi_feed, -1, conf_feed);
      
      temp1 = resp[0].temperature;
      temp2 = resp[1].temperature;
        
      log << time << "," << conf_cmd(0) << "," << conf_cmd(1) 
        << "," << conf_cmd(2) << "," << conf_cmd(3) 
        << "," << conf_cmd(4) << "," << conf_cmd(5)
        << "," << conf_feed(0) << "," << conf_feed(1) 
        << "," << conf_feed(2) << "," << conf_feed(3) 
        << "," << conf_feed(4) << "," << conf_feed(5)
        << "," << torque1 << "," << torque2 << "," 
        << temp1 << "," << temp2 << ","
        << current_phi << "," << current_psi << endl;
              
      cmds[0].position = phim;
      cmds[1].position = psim;
      pi3_interface.write(cmds);
      
      //pi3_interface.stop();
      
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
   
      if(time - print_time >= print_period)
      {
        cout << "Time: " << time << ", Temp1: " << temp1
             << ", Temp2: " << temp2 << endl;
        print_time = time;
      }

    }

  
    phi_ini = phi_cmd;
    psi_ini = psi_cmd;
    
    log << endl;
  
  }

  pi3_interface.stop();

  current_thread.join();

  return 0; 
} 
