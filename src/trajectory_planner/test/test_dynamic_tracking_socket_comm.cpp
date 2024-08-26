#include "trajectory_planner.h"
#include "kinematics.h"
#include "pi3hat/pi3hat_interface.h"
#include <chrono>
#include <iostream>
#include <fstream>
#include <thread>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <arpa/inet.h>
#include <queue>
#include <cstring>

#include "power_monitor.h"

using namespace std::chrono;

const std::map<int, int> servo_bus_map ={
  {1,1},
  {2,1}
};

rpi_power_monitor::RpiPowerMonitor pm_phi(1, 0x40);  
rpi_power_monitor::RpiPowerMonitor pm_psi(1, 0x41);  

float voltage_phi = 24, voltage_psi = 24;
float current_phi = 0, current_psi = 0;
float power_phi = 0, power_psi = 0;
int mode = 0;
bool logStop = true;
std::mutex thread_mutex;
std::queue<std::string> dataBuffer;
std::mutex bufferMutex;

// Function to send data to Python server
void send_data_to_python(const char* serverAddress, int serverPort) {

    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd == -1) {
        std::cerr << "Error creating socket." << std::endl;
        return;
    }

    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(serverPort);
    inet_pton(AF_INET, serverAddress, &serverAddr.sin_addr);

    if (connect(sockfd, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == -1) {
        std::cerr << "Error connecting to the server." << std::endl;
        close(sockfd);
        return;
    }

    std::string dataToSend = "0,0";

    while (true) {

        if(!dataBuffer.empty())
        {
          dataToSend = dataBuffer.front();//std::to_string((float)(rand()) / (float)(RAND_MAX) * 5) + ',' + std::to_string(rand() % 5);
          dataBuffer.pop();
        }

        send(sockfd, dataToSend.c_str(), dataToSend.size(), 0);

        char buffer[1024];
        int bytesRead = recv(sockfd, buffer, sizeof(buffer), 0);
        if (bytesRead > 0) {
            buffer[bytesRead] = '\0';
            mode = int(buffer[0])-48;
            std::cout << "Received response from Python server: " << buffer << ", Int: " << mode << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    close(sockfd);
}

void logPowerData()
{
  while(logStop)
  {
    std::lock_guard<std::mutex> guard(thread_mutex);
    pm_phi.getCurrent_mA(current_phi);
    current_phi = 3*current_phi; // Only for a shunt resistance of 0.01/3 ohm
    pm_psi.getCurrent_mA(current_psi);
  }
}

inline void exponentialFilterTemperature(double input, double& output, double alpha)
{
  output = alpha*input + (1-alpha)*output;
}

int main(void)
{

  const char* serverAddress = "10.42.0.1"; // Server's IP address
  const int serverPort = 12345; // Server's port number

  // Run the client function in a separate thread
  std::thread clientThread(send_data_to_python, serverAddress, serverPort);

  Matrix<double, 2, 1> A0, B0, C0, D0, F0, P0;
  Matrix<double, 2, 6> dim5bar;
  Vector<double, 6> conf_feed;
  Vector<double, 6> conf_cmd;
  Vector<double, 6> conf_ini, conf_fin;
  Matrix<double, 2, 1> P;

  double phi_ini, phi_fin, psi_ini, psi_fin;
  double phi_cmd, phim, phi_feed, phim_feed;
  double psi_cmd, psim, psi_feed, psim_feed;
  double cycle_freq;
  double path_length = 0;
  double total_time;
  double temp1 = 0, temp2 = 0;
  int i=0, pub_iter = 0;
  int ikb1 = -1, ikb2 = -1;
  double p2p_time = 2;

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

  // Dimensions of the five-bar mechanism
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

  // Generalized trapezoidal BPC trajectory parameters
  double cycle_time = 10;
  int n_cycle = 5;
  double sF = 2;
  double min_freq = 1.9;
  double max_freq = 2;
  double freq_step = 0.1;
  bool temperature_wait = true;
  double s = 0, time = 0, tpause = 0;
  double delay_time = 1;
  int n_freq = floor((max_freq-min_freq)/freq_step);
  
  double dock_conf_phi = -1.62, dock_conf_psi = -1.55;
  double alpha = 0.01;
  double temp_thresh = 23, temp_thresh_low = 23;

  TrajectoryPlanner traj;

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

  double traj_time = traj.initializeTTBPC(sF, cycle_freq);

  // Setting up the log file
  ofstream log("gait_5bar/gait_5bar_23_06_19_ikb4_500g_v2.csv");
//  ofstream log("test.csv");

  log << "5-bar planar leg gait frequency test" << endl;
  log << "IK_branch,-1,-1" << endl;
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
  
  log << "time,phi_cmd,psi_cmd,rho_cmd,theta_cmd,x_cmd,y_cmd,phi_feed,psi_feed,rho_feed,theta_feed,x_feed,y_feed,phi_torque,psi_torque,phi_velocity,psi_velocity,temp_phi,temp_psi,temp_phi_filt,temp_psi_filt,current_phi,current_psi,power_phi,power_psi" << endl;

  total_time = n_cycle*traj_time;

  // Start all motors in stopped mode to clear all faults
  pi3_interface.stop();
  // Wait for the stop command to be sent 
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  
  time = 0;

  phi_ini = dock_conf_phi;
  psi_ini = dock_conf_psi;

  five_bar_kinematics.inverseKinematics(traj.bpc_params.P1, ikb1, ikb2, conf_fin);
  
  phi_fin = conf_fin(0);
  psi_fin = conf_fin(1);

  auto ti = high_resolution_clock::now();
  auto timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);

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

/*    {
      std::lock_guard<std::mutex> lock(bufferMutex);
      dataBuffer.push(std::to_string(conf_feed(4)) + ',' + std::to_string(conf_feed(5)));
    }*/
    
    //pi3_interface.stop();    

    /*log << time << "," << conf_cmd(0) << "," << conf_cmd(1) 
      << "," << conf_cmd(2) << "," << conf_cmd(3) 
      << "," << conf_cmd(4) << "," << conf_cmd(5)
      << "," << conf_feed(0) << "," << conf_feed(1) 
      << "," << conf_feed(2) << "," << conf_feed(3) 
      << "," << conf_feed(4) << "," << conf_feed(5) << endl;*/
      
    temp1 = resp[0].temperature;
    temp2 = resp[0].temperature;

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
    tpause = 0;

    while(time <= total_time)
    {
      timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);
      if(mode != 4)
      {
        time = (timeD.count()*1e-6) - tpause;
      }
      else
      {
        tpause = (timeD.count()*1e-6) - time;
      }

      // Feedback
      resp = pi3_interface.read();

      phim_feed = resp[0].position;
      psim_feed = resp[1].position;

      five_bar_kinematics.getRelativeAngles(phim_feed, psim_feed, phi_feed, psi_feed);

      five_bar_kinematics.forwardKinematics(phi_feed, psi_feed, -1, conf_feed);

      exponentialFilterTemperature(resp[0].temperature, temp1, alpha);
      exponentialFilterTemperature(resp[1].temperature, temp2, alpha);
      
      /*log << time << "," << s << ","
        << conf_cmd(0) << "," << conf_cmd(1) 
        << "," << conf_cmd(2) << "," << conf_cmd(3) 
        << "," << conf_cmd(4) << "," << conf_cmd(5)
        << "," << conf_feed(0) << "," << conf_feed(1) 
        << "," << conf_feed(2) << "," << conf_feed(3) 
        << "," << conf_feed(4) << "," << conf_feed(5)
        << "," << resp[0].torque << "," << resp[1].torque
        << "," << resp[0].velocity << "," << resp[1].velocity
        << "," << resp[0].temperature << "," << resp[1].temperature
        << "," << temp1 << "," << temp2
        << "," << current_phi << "," << current_psi
        << "," << (voltage_phi*current_phi/1000.) << "," << (voltage_psi*current_psi/1000.)
        << endl;*/

      traj.trapezoidalTrajectoryBPC(time, s);

      traj.pathBPC(s, P);

      five_bar_kinematics.inverseKinematics(P, ikb1, ikb2, conf_cmd);

      // Command
      five_bar_kinematics.getMotorAngles(conf_cmd(0), conf_cmd(1), phim, psim);

      cmds[0].position = phim;
      cmds[1].position = psim;
      pi3_interface.write(cmds);

      if(pub_iter == 20){
        pub_iter = 0;
        std::lock_guard<std::mutex> lock(bufferMutex);
        dataBuffer.push(std::to_string(conf_feed(4)) + ',' + std::to_string(conf_feed(5)));
      }
      pub_iter++;
      
      //pi3_interface.stop();

      std::cout << "Point: " << P << ", Time: " << time << std::endl;
      
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
      //pi3_interface.stop();
      std::cout << "Temparatures: " << temp1 << "," << temp2 << endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));      
    }    

    if(temperature_wait && ((temp1 > temp_thresh)||(temp2 > temp_thresh)))
    {
      ti = high_resolution_clock::now();
      time = 0;

      phi_ini = conf_cmd(0);
      psi_ini = conf_cmd(1);
      phi_fin = dock_conf_phi;
      psi_fin = dock_conf_psi;

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
          
        temp1 = resp[0].temperature;
        temp2 = resp[0].temperature;

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
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

      phi_ini = dock_conf_phi;
      psi_ini = dock_conf_psi;

      five_bar_kinematics.inverseKinematics(traj.bpc_params.P1, ikb1, ikb2, conf_fin);
      
      phi_fin = conf_fin(0);
      psi_fin = conf_fin(1);

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
          
        temp1 = resp[0].temperature;
        temp2 = resp[0].temperature;

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }

    }    
    
  }
  
  ti = high_resolution_clock::now();
  time = 0;

  phi_ini = conf_cmd(0);
  psi_ini = conf_cmd(1);
  phi_fin = dock_conf_phi;
  psi_fin = dock_conf_psi;

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
      
    temp1 = resp[0].temperature;
    temp2 = resp[0].temperature;

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  pi3_interface.stop();
  logStop = false;

  current_thread.join();  
  clientThread.join();

  return 0;
}
