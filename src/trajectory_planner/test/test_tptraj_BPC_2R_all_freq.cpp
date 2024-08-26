#include "trajectory_planner.h"
#include "2R_kinematics.h"
#include "pi3hat/pi3hat_interface.h"
#include <chrono>
#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>

#include "power_monitor.h"

using namespace std::chrono;

rpi_power_monitor::RpiPowerMonitor pm_theta1(1, 0x40);  
rpi_power_monitor::RpiPowerMonitor pm_theta2(1, 0x41);  

float voltage_theta1 = 24, voltage_theta2 = 24;
float current_theta1 = 0, current_theta2 = 0;
float power_theta1 = 0, power_theta2 = 0;
bool logStop = true;
std::mutex thread_mutex;

void logPowerData()
{
  while(logStop)
  {
    std::lock_guard<std::mutex> guard(thread_mutex);
    pm_theta1.getCurrent_mA(current_theta1);
    current_theta1 = 3*current_theta1; // Only for a shunt resistance of 0.01/3 ohm
    pm_theta2.getCurrent_mA(current_theta2);
  }
}

const std::map<int, int> servo_bus_map ={
  {3,1},
  {4,1}
};

inline void exponentialFilterTemperature(double input, double& output, double alpha)
{
  output = alpha*input + (1-alpha)*output;
}

int main(void)
{ 

  double theta1, theta1m, theta1_feed, theta1m_feed;
  double theta2, theta2m, theta2_feed, theta2m_feed;
  double theta1_ini, theta1_fin, theta2_ini, theta2_fin;
  double path_length = 0;
  double s = 0, time = 0;
  double cycle_freq, traj_time;
  int n_cycle;
  double total_time;
  int i = 0;
  double temp1, temp2;

  Vector<double, 2> P;

  //Pi3hat initialize
  Pi3HatInterface pi3_interface;

  pi3_interface.initialize(servo_bus_map);

  std::vector<MoteusCommand> cmds;
  std::vector<MoteusResponse> resp;
  
  // Setting up motor commands
  cmds.resize(2);
  cmds[0].id = 3;
  cmds[0].mode = 10; // Position mode
  cmds[0].position = 0.0;
  cmds[0].velocity = 0.0;
  cmds[0].feedforward_torque = 0;
  cmds[0].kp_scale = 7;
  cmds[0].kd_scale = 4;
  cmds[0].watchdog_timeout = 0;

  cmds[1].id = 4;
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
  double cycle_time = 20;
  double sF = 2;
  double min_freq = 0.5;
  double max_freq = 2;
  double freq_step = 0.1;
  double p2p_time = 5;
  double delay_time = 1;
  double dock_conf_theta1 = 0.178, dock_conf_theta2 = -2.19;

  bool temperature_wait = true;
  double alpha = 0.01;
  double temp_thresh = 23, temp_thresh_low = 23;
    
  int n_freq = floor((max_freq-min_freq)/freq_step);

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

  // Setting home position
  theta1_ini = dock_conf_theta1;//-theta1_offset;
  theta2_ini = dock_conf_theta2;//-theta2_offset;
  twor_kinematics.inverseKinematics(traj.bpc_params.P1, ik_branch, theta1_fin, theta2_fin);

  // Setting up log file
  ofstream log("gait_2R/gait_2R_23_04_03_500g_v1.csv");
  
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
  log << "gait_frequency_minimum," << min_freq << endl;
  log << "gait_frequency_maximum," << max_freq << endl;
  log << "gait_frequency_step," << freq_step << endl;
  log << "gait_cycles," << n_cycle << endl;
  log << endl;
  
  log << "Actuator gain scales:" << endl;
  log << "Kp," << cmds[1].kp_scale << endl;
  log << "Kd," << cmds[1].kd_scale << endl;
  log << endl;
  
  log << "time,arc_length_param,theta1_cmd,theta2_cmd,theta1_feed,theta2_feed,x_cmd,y_cmd,theta1_vel,theta2_vel,torque1,torque2,temp1,temp2,temp1_filt,temp2_filt,current_theta1,current_theta2,power_theta1,power_theta2" << endl;

  // Start all motors in stopped mode to clear all faults
  pi3_interface.stop();
  // Wait for the stop command to be sent 
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
  auto ti = high_resolution_clock::now();
  auto timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);

  // From the home configuration to the start configuration for walking trajectory
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

    cmds[0].position = theta1m;
    cmds[1].position = theta2m;
    pi3_interface.write(cmds);
    
    //pi3_interface.stop();    
    
    temp1 = resp[0].temperature;
    temp2 = resp[1].temperature;

    /*log << time << "," << theta1 << "," << theta2
	    << "," << theta1_feed << "," << theta2_feed
      << "," << P(0) << "," << P(1) << endl;*/

    /*cout << time << "," << theta1 << "," << theta2
      << "," << theta1m << "," << theta2m
	    << "," << P(0) << "," << P(1) << endl;*/

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  
  std::thread current_thread(logPowerData); 
  
  for(i=0; i<=n_freq; i++)
  {
    cycle_freq = freq_step*i + min_freq;
    traj_time = traj.initializeTTBPC(sF, cycle_freq);
    n_cycle = cycle_time*cycle_freq;
    
    log << "Gait frequency," << cycle_freq << endl;
    
    cout << "Gait frequency: " << cycle_freq << endl;
    cout << "Number of cycles: " << n_cycle << endl;
  
    //std::this_thread::sleep_for(std::chrono::milliseconds(1000));    
    ti = high_resolution_clock::now();
    time = 0;
    total_time = n_cycle*traj_time;

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
      
      exponentialFilterTemperature(resp[0].temperature, temp1, alpha);
      exponentialFilterTemperature(resp[1].temperature, temp2, alpha);
   
      log << time << "," << s 
        << "," << theta1 << "," << theta2
        << "," << theta1_feed << "," << theta2_feed
        << "," << P(0) << "," << P(1)
        << "," << resp[0].velocity << "," << resp[1].velocity
        << "," << resp[0].torque << "," << resp[1].torque
        << "," << resp[0].temperature << "," << resp[1].temperature
        << "," << temp1 << "," << temp2
        << "," << current_theta1 << "," << current_theta2
        << "," << (voltage_theta1*current_theta1/1000.) << "," << (voltage_theta2*current_theta2/1000.)
        << endl;
        
      /*cout << time << "," << theta1 << "," << theta2
        << "," << theta1m << "," << theta2m
        << "," << P(0) << "," << P(1) << endl;*/

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    ti = high_resolution_clock::now();
    time = 0;
    
    while(time <= delay_time)
    {
      timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);
      time = timeD.count()*1e-6;
      resp = pi3_interface.read();
      pi3_interface.write(cmds);
      std::cout << "Temparatures: " << temp1 << "," << temp2 << endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    if(temperature_wait && ((temp1 > temp_thresh)||(temp2 > temp_thresh)))
    {
      ti = high_resolution_clock::now();
      time = 0;
      
      theta1_ini = theta1;
      theta2_ini = theta2;
      theta1_fin = dock_conf_theta1;
      theta2_fin = dock_conf_theta2;
      
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

        cmds[0].position = theta1m;
        cmds[1].position = theta2m;
        pi3_interface.write(cmds);
        
        //pi3_interface.stop();    
        
        temp1 = resp[0].temperature;
        temp2 = resp[1].temperature;

        /*log << time << "," << theta1 << "," << theta2
          << "," << theta1_feed << "," << theta2_feed
          << "," << P(0) << "," << P(1) << endl;*/

        /*cout << time << "," << theta1 << "," << theta2
          << "," << theta1m << "," << theta2m
          << "," << P(0) << "," << P(1) << endl;*/

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      
      while((temp1 > temp_thresh_low)||(temp2 > temp_thresh_low))
      {
        resp = pi3_interface.read();
        exponentialFilterTemperature(resp[0].temperature, temp1, alpha);
        exponentialFilterTemperature(resp[1].temperature, temp2, alpha);

        pi3_interface.stop();
        std::cout << "Temparatures: " << temp1 << "," << temp2 << endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      
      ti = high_resolution_clock::now();
      time = 0;
      
      theta1_ini = dock_conf_theta1;
      theta2_ini = dock_conf_theta2;
      twor_kinematics.inverseKinematics(traj.bpc_params.P1, ik_branch, theta1_fin, theta2_fin);
      
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

        cmds[0].position = theta1m;
        cmds[1].position = theta2m;
        pi3_interface.write(cmds);
        
        //pi3_interface.stop();    
        
        temp1 = resp[0].temperature;
        temp2 = resp[1].temperature;

        /*log << time << "," << theta1 << "," << theta2
          << "," << theta1_feed << "," << theta2_feed
          << "," << P(0) << "," << P(1) << endl;*/

        /*cout << time << "," << theta1 << "," << theta2
          << "," << theta1m << "," << theta2m
          << "," << P(0) << "," << P(1) << endl;*/

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    
    //std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  }
  
  ti = high_resolution_clock::now();
  time = 0;
  
  theta1_ini = theta1;
  theta2_ini = theta2;
  theta1_fin = dock_conf_theta1;
  theta2_fin = dock_conf_theta2;
  
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

    cmds[0].position = theta1m;
    cmds[1].position = theta2m;
    pi3_interface.write(cmds);
    
    //pi3_interface.stop();    
    
    temp1 = resp[0].temperature;
    temp2 = resp[1].temperature;

    /*log << time << "," << theta1 << "," << theta2
      << "," << theta1_feed << "," << theta2_feed
      << "," << P(0) << "," << P(1) << endl;*/

    /*cout << time << "," << theta1 << "," << theta2
      << "," << theta1m << "," << theta2m
      << "," << P(0) << "," << P(1) << endl;*/

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  
  pi3_interface.stop();
  logStop = false;

  current_thread.join();  
    
  return 0;
}
