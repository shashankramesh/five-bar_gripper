#include "trajectory_planner.h"
#include "kinematics.h"
#include "2R_kinematics.h"
#include "pi3hat/pi3hat_interface.h"
#include "power_monitor.h"
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
#include <algorithm>

using namespace std::chrono;

const std::map<int, int> servo_bus_map ={
  {1,1},
  {2,1},
  {3,1},
  {4,1}
};

Pi3HatInterface pi3_interface;

rpi_power_monitor::RpiPowerMonitor pm_phi(1, 0x40);  
rpi_power_monitor::RpiPowerMonitor pm_psi(1, 0x41);  
rpi_power_monitor::RpiPowerMonitor pm_theta1(1, 0x42);  
rpi_power_monitor::RpiPowerMonitor pm_theta2(1, 0x43);  

float current_phi = 0, current_psi = 0;
float current_theta1 = 0, current_theta2 = 0;
int mode = 0, mode1 = 0, prev_mode1 = 0;
bool logStop = true, run = true;
double phi_cmd, phim, phi_feed, phim_feed;
double psi_cmd, psim, psi_feed, psim_feed;
double theta1_cmd, theta2_cmd;
double theta1_feed, theta2_feed;
double theta1m, theta2m;
double joint_ini_five_bar[2], joint_fin_five_bar[2], joint_ini_2R[2], joint_fin_2R[2];
std::mutex thread_mutex;
std::queue<std::string> dataBuffer;
std::mutex bufferMutex;
std::mutex modeMutex;

double phi_home = -1.62, psi_home = -1.55;
int ikb1 = -1, ikb2 = -1;
int ik_branch = 1;

double theta1_home = -M_PI/2, theta2_home = -M_PI/2; //TODO: fill this
double theta1_home_vm = -0.85, theta2_home_vm = -2.407; //TODO: fill this

Vector<double, 6> conf_feed;
Vector<double, 6> conf_cmd;
Vector<double, 2> P_feed;

ofstream log_file("test.csv");

// Current logging
void logPowerData()
{
  while(logStop)
  {
    std::lock_guard<std::mutex> guard(thread_mutex);
    pm_phi.getCurrent_mA(current_phi);
    //current_phi = 3*current_phi; // Only for a shunt resistance of 0.01/3 ohm
    pm_psi.getCurrent_mA(current_psi);
    pm_theta1.getCurrent_mA(current_theta1);
    pm_theta2.getCurrent_mA(current_theta2);
  }
}

// Send data to server via socket
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

  std::string dataToSend = "0,0,0,0,0,0,0,0,0,0,0,0,1";

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
      mode1 = int(buffer[0])-48;
      if(mode1 != prev_mode1)
      {
        mode = mode1;
        prev_mode1 = mode;
      }
      //std::cout << "Received response from Python server: " << buffer << ", Int: " << mode << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  close(sockfd);
}

inline void exponentialFilterTemperature(double input, double& output, double alpha)
{
  output = alpha*input + (1-alpha)*output;
}

void orderResponse(std::vector<MoteusResponse> resp_in, std::vector<MoteusResponse>& resp_out)
{
  for(int jj = 0; jj < 4; jj++)
  {
    resp_out[int(resp_in[jj].id) - 1] = resp_in[jj];
  }
}

void p2p(double joint_ini_five_bar[2], double joint_fin_five_bar[2], double joint_ini_2R[2], double joint_fin_2R[2], double p2p_time,
FiveBarKinematics& five_bar_kinematics, TwoRKinematics& twor_kinematics, Pi3HatInterface& pi3_interface, TrajectoryPlanner& traj, TrajectoryPlanner& traj_2R, 
std::vector<MoteusCommand>& cmds, std::vector<MoteusResponse>& resp)
{
  double phi_cmd = 0, psi_cmd = 0, phim_feed = 0, psim_feed = 0, psi_feed = 0, phi_feed = 0;
  double theta1_cmd = 0, theta2_cmd = 0, theta1m = 0, theta2m = 0;

  auto ti = high_resolution_clock::now();
  auto timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);

  double time = 0;

  while(time <= p2p_time)
  {
    timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);
    time = timeD.count()*1e-6;

    // Feedback
    orderResponse(pi3_interface.read(), resp);

    phim_feed = resp[0].position;
    psim_feed = resp[1].position;

    five_bar_kinematics.getRelativeAngles(phim_feed, psim_feed, phi_feed, psi_feed);
    twor_kinematics.getRelativeAngles(resp[2].position, resp[3].position, theta1_feed, theta2_feed);
    
    five_bar_kinematics.forwardKinematics(phi_feed, psi_feed, -1, conf_feed);
    twor_kinematics.forwardKinematics(theta1_feed, theta2_feed, P_feed);

    phi_cmd = joint_ini_five_bar[0] + (time*(joint_fin_five_bar[0] - joint_ini_five_bar[0])/p2p_time);
    psi_cmd = joint_ini_five_bar[1] + (time*(joint_fin_five_bar[1] - joint_ini_five_bar[1])/p2p_time);

    theta1_cmd = joint_ini_2R[0] + (time*(joint_fin_2R[0] - joint_ini_2R[0])/p2p_time);
    theta2_cmd = joint_ini_2R[1] + (time*(joint_fin_2R[1] - joint_ini_2R[1])/p2p_time);

    // Command
    five_bar_kinematics.getMotorAngles(phi_cmd, psi_cmd, phim, psim);
    twor_kinematics.getMotorAngles(theta1_cmd, theta2_cmd, theta1m, theta2m);
    
    cmds[0].position = phim;
    cmds[1].position = psim;
    cmds[2].position = theta1m;
    cmds[3].position = theta2m;
    pi3_interface.write(cmds);

    /*    {
          std::lock_guard<std::mutex> lock(bufferMutex);
          dataBuffer.push(std::to_string(conf_feed(4)) + ',' + std::to_string(conf_feed(5)));
          }*/

    //pi3_interface.stop();    

    //std::cout << "cmd1: " << cmds[0].position << ", cmd2: " << cmds[1].position << ", cmd3: " << cmds[2].position << ", cmd4: " << cmds[3].position << std::endl;

    //std::cout << resp[0].position << std::endl;

    five_bar_kinematics.forwardKinematics(phi_cmd, psi_cmd, -1, conf_cmd);
    log_file << time << "," << conf_cmd(0) << "," << conf_cmd(1) 
      << "," << conf_cmd(2) << "," << conf_cmd(3) 
      << "," << conf_cmd(4) << "," << conf_cmd(5)
      << "," << conf_feed(0) << "," << conf_feed(1) 
      << "," << conf_feed(2) << "," << conf_feed(3) 
      << "," << conf_feed(4) << "," << conf_feed(5) 
      << "," << theta1_cmd << "," << theta2_cmd
      << "," << theta1_feed << "," << theta2_feed 
      << "," << cmds[0].position << "," << cmds[1].position
      << "," << cmds[2].position << "," << cmds[3].position
      << "," << resp[0].position << "," << resp[1].position
      << "," << resp[2].position << "," << resp[3].position << endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void linear_motion(Vector<double, 2>& pI, Vector<double, 2>& pF, Vector<double, 2>& pI_2R, Vector<double, 2>& pF_2R, double& path_time, double& n, 
FiveBarKinematics& five_bar_kinematics, TwoRKinematics& twor_kinematics, Pi3HatInterface& pi3_interface, TrajectoryPlanner& traj, TrajectoryPlanner& traj_2R, 
std::vector<MoteusCommand>& cmds, std::vector<MoteusResponse>& resp)
{
  Vector<double, 2> pd, pd_2R, P, P_2R;
  pd = pF-pI;
  pd_2R = pF_2R-pI_2R;

  double path_length = sqrt(pd(0)*pd(0) + pd(1)*pd(1));
  double path_length_2R = sqrt(pd_2R(0)*pd_2R(0) + pd_2R(1)*pd_2R(1));
  double total_time = n*path_time;
  double max_velocity = 0.35;
  double max_velocity_2R = 0.35;
  double s = 0, s_2R = 0, time = 0, tpause = 0;
  int pub_iter = 0, ni = 0, ni_2R = 0, iter = 1;
  float pow_2R = 0, pow_five_bar = 0;
  double s_comm = 0, s_comm_2R = 0;

  bool status = traj.initialize(path_time, max_velocity, path_length);
  status = traj_2R.initialize(path_time, max_velocity_2R, path_length_2R);

  auto ti = high_resolution_clock::now();
  auto timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);

  while(time <= total_time && mode != 5)
  {
    timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);
    if(mode != 0)
    {
      time = (timeD.count()*1e-6) - tpause;
    }
    else
    {
      tpause = (timeD.count()*1e-6) - time;
    }

    // Feedback
    orderResponse(pi3_interface.read(), resp);

    phim_feed = resp[0].position;
    psim_feed = resp[1].position;

    five_bar_kinematics.getRelativeAngles(phim_feed, psim_feed, phi_feed, psi_feed);
    twor_kinematics.getRelativeAngles(resp[2].position, resp[3].position, theta1_feed, theta2_feed);

    five_bar_kinematics.forwardKinematics(phi_feed, psi_feed, -1, conf_feed);
    twor_kinematics.forwardKinematics(theta1_feed, theta2_feed, P_feed);

    //exponentialFilterTemperature(resp[0].temperature, temp1, alpha);
    //exponentialFilterTemperature(resp[1].temperature, temp2, alpha);

    ni = int(floor(time/path_time));
    traj.trapezoidalTrajectory(time - ni*path_time, s);
    traj_2R.trapezoidalTrajectory(time - ni*path_time, s_2R);

    if(ni % 2 == 0)
    {
      traj.straightLinePath(pI, pF, s, P);
      traj_2R.straightLinePath(pI_2R, pF_2R, s_2R, P_2R);
      s_comm = s/2.;
      s_comm_2R = s_2R/2.;
    }
    else
    {
      traj.straightLinePath(pF, pI, s, P);
      traj_2R.straightLinePath(pF_2R, pI_2R, s_2R, P_2R);
      s_comm = s/2. + 0.5;
      s_comm_2R = s_2R/2. + 0.5;
    }

    five_bar_kinematics.inverseKinematics(P, ikb1, ikb2, conf_cmd);
    twor_kinematics.inverseKinematics(P_2R, ik_branch, theta1_cmd, theta2_cmd);

    // Command
    five_bar_kinematics.getMotorAngles(conf_cmd(0), conf_cmd(1), phim, psim);
    twor_kinematics.getMotorAngles(theta1_cmd, theta2_cmd, theta1m, theta2m);

    cmds[0].position = phim;
    cmds[1].position = psim;
    cmds[2].position = theta1m;
    cmds[3].position = theta2m;
    pi3_interface.write(cmds);

    if(pub_iter == 10){
      pub_iter = 0;
      pow_2R = 24.*(current_theta1 + current_theta2)*0.001;
      pow_five_bar = 24.*(current_phi + current_psi)*0.001;
      std::lock_guard<std::mutex> lock(bufferMutex);
      dataBuffer.push(std::to_string(P_feed(0)) + ',' + std::to_string(P_feed(1)) + ',' + std::to_string(P_2R(0)) + ',' + std::to_string(P_2R(1)) + ','
                      + std::to_string(conf_feed(4)) + ',' + std::to_string(conf_feed(5)) + ',' + std::to_string(conf_cmd(4)) + ',' + std::to_string(conf_cmd(5)) + ','
                      + std::to_string(pow_five_bar) + ',' + std::to_string(s_comm) + ',' + std::to_string(pow_2R) + ',' + std::to_string(s_comm_2R) 
                      + ',' + std::to_string(iter));
      iter++;
    }
    pub_iter++;

    //std::cout << "cmd1: " << cmds[0].position << ", cmd2: " << cmds[1].position << ", cmd3: " << cmds[2].position << ", cmd4: " << cmds[3].position << std::endl;

    //std::cout << resp[0].position << std::endl;

    log_file << time << "," << conf_cmd(0) << "," << conf_cmd(1) 
      << "," << conf_cmd(2) << "," << conf_cmd(3) 
      << "," << conf_cmd(4) << "," << conf_cmd(5)
      << "," << conf_feed(0) << "," << conf_feed(1) 
      << "," << conf_feed(2) << "," << conf_feed(3) 
      << "," << conf_feed(4) << "," << conf_feed(5)
      << "," << theta1_cmd << "," << theta2_cmd
      << "," << theta1_feed << "," << theta2_feed 
      << "," << cmds[0].position << "," << cmds[1].position
      << "," << cmds[2].position << "," << cmds[3].position
      << "," << resp[0].position << "," << resp[1].position
      << "," << resp[2].position << "," << resp[3].position 
      << "," << current_phi << "," << current_psi
      << "," << current_theta1 << "," << current_theta2
      << "," << s_comm << "," << s_comm_2R << endl;

    //pi3_interface.stop();

    //std::cout << "Point: " << P << ", Time: " << time << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
} 

void velocity_ellipse(FiveBarKinematics& five_bar_kinematics, TwoRKinematics& twor_kinematics, Pi3HatInterface& pi3_interface, TrajectoryPlanner& traj, TrajectoryPlanner& traj_2R, 
std::vector<MoteusCommand>& cmds, std::vector<MoteusResponse>& resp)
{
  double p2p_time = 1, total_time = 5;
  double a1 = 0.035, a2 = 0.035;
  double freq1 = 1, freq2 = 1;

  double time = 0, tpause = 0;
  double joint_fin_2R[2] = {0}, joint_fin_five_bar[2] = {0}, joint_ini_2R[2] = {0}, joint_ini_five_bar[2] = {0};
  double phi_cmd = 0, psi_cmd = 0, phim_feed = 0, psim_feed = 0, psi_feed = 0, phi_feed = 0;
  double theta1_cmd = 0, theta2_cmd = 0, theta1m = 0, theta2m = 0;
  double theta1_fin = 0, theta2_fin = 0;
  
  fstream points_2R;
  fstream points_5R;

  points_5R.open("workspace_points_5bar/workspace_points_5bar.csv", ios::in);
  points_2R.open("workspace_2R_points/workspace_points_2R.csv", ios::in);

  string pts_5R, xcoord_5R, ycoord_5R, pts_2R, xcoord_2R, ycoord_2R;

  Vector<double, 2> pC_2R, pC_5R;
  Vector<double, 6> conf_fin;
  
  int iterator = 0;

  while((points_5R >> pts_5R) && (points_2R >> pts_2R) && (mode != 5))
  {
    iterator++;
    
    stringstream xy_5R(pts_5R);
    stringstream xy_2R(pts_2R);
  
    std::getline(xy_5R, xcoord_5R, ',');
    std::getline(xy_2R, xcoord_2R, ',');
    
    std::getline(xy_5R, ycoord_5R, ',');
    std::getline(xy_2R, ycoord_2R, ',');
    
    pC_5R(0) = stod(xcoord_5R);
    pC_5R(1) = stod(ycoord_5R);

    pC_2R(0) = stod(xcoord_2R);
    pC_2R(1) = stod(ycoord_2R);
    
    five_bar_kinematics.inverseKinematics(pC_5R, ikb1, ikb2, conf_fin);
    twor_kinematics.inverseKinematics(pC_2R, ik_branch, theta1_fin, theta2_fin);

    joint_ini_five_bar[0] = conf_feed(0);
    joint_ini_five_bar[1] = conf_feed(1);
    joint_fin_five_bar[0] = conf_fin(0) + a1;
    joint_fin_five_bar[1] = conf_fin(1);
    joint_ini_2R[0] = theta1_feed;
    joint_ini_2R[1] = theta2_feed;
    joint_fin_2R[0] = theta1_fin + a2;
    joint_fin_2R[1] = theta2_fin;

    p2p(joint_ini_five_bar, joint_fin_five_bar, joint_ini_2R, joint_fin_2R, p2p_time, five_bar_kinematics, twor_kinematics, pi3_interface, traj, traj_2R, cmds, resp);

    auto ti = high_resolution_clock::now();
    auto timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);
    time = 0;

    while(time <= total_time && mode != 5)
    {
      timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);
      if(mode != 0)
      {
        time = (timeD.count()*1e-6) - tpause;
      }
      else
      {
        tpause = (timeD.count()*1e-6) - time;
      }

      // Feedback
      orderResponse(pi3_interface.read(), resp);

      phim_feed = resp[0].position;
      psim_feed = resp[1].position;

      five_bar_kinematics.getRelativeAngles(phim_feed, psim_feed, phi_feed, psi_feed);
      twor_kinematics.getRelativeAngles(resp[2].position, resp[3].position, theta1_feed, theta2_feed);

      five_bar_kinematics.forwardKinematics(phi_feed, psi_feed, -1, conf_feed);
      twor_kinematics.forwardKinematics(theta1_feed, theta2_feed, P_feed);

      //exponentialFilterTemperature(resp[0].temperature, temp1, alpha);
      //exponentialFilterTemperature(resp[1].temperature, temp2, alpha);

      if(iterator >= 3)
      {
        phi_cmd = conf_fin(0) + a1*cos(2*M_PI*freq1*time);
        psi_cmd = conf_fin(1) + a1*sin(2*M_PI*freq1*time);

        theta1_cmd = theta1_fin + a2*cos(2*M_PI*freq2*time);
        theta2_cmd = theta2_fin + a2*sin(2*M_PI*freq2*time);
      }
      else
      {
        phi_cmd = conf_fin(0);
        psi_cmd = conf_fin(1);

        theta1_cmd = theta1_fin;
        theta2_cmd = theta2_fin;
      }

      // Command
      five_bar_kinematics.getMotorAngles(phi_cmd, psi_cmd, phim, psim);
      twor_kinematics.getMotorAngles(theta1_cmd, theta2_cmd, theta1m, theta2m);

      cmds[0].position = phim;
      cmds[1].position = psim;
      cmds[2].position = theta1m;
      cmds[3].position = theta2m;
      pi3_interface.write(cmds);
      //pi3_interface.stop();

      /*if(pub_iter == 20){
        pub_iter = 0;
        std::lock_guard<std::mutex> lock(bufferMutex);
        dataBuffer.push(std::to_string(conf_cmd(4)) + ',' + std::to_string(conf_cmd(5)) + ',' + std::to_string(conf_feed(4)) + ',' + std::to_string(conf_feed(5)) + ",0,0,0,0");
      }
      pub_iter++;*/

      //std::cout << "cmd1: " << cmds[0].position << ", cmd2: " << cmds[1].position << ", cmd3: " << cmds[2].position << ", cmd4: " << cmds[3].position << std::endl;

      //std::cout << resp[0].position << std::endl;

      log_file << time << "," << conf_cmd(0) << "," << conf_cmd(1) 
        << "," << conf_cmd(2) << "," << conf_cmd(3) 
        << "," << conf_cmd(4) << "," << conf_cmd(5)
        << "," << conf_feed(0) << "," << conf_feed(1) 
        << "," << conf_feed(2) << "," << conf_feed(3) 
        << "," << conf_feed(4) << "," << conf_feed(5)
        << "," << theta1_cmd << "," << theta2_cmd
        << "," << theta1_feed << "," << theta2_feed 
        << "," << cmds[0].position << "," << cmds[1].position
        << "," << cmds[2].position << "," << cmds[3].position
        << "," << resp[0].position << "," << resp[1].position
        << "," << resp[2].position << "," << resp[3].position << endl;


      //std::cout << "Point: " << P << ", Time: " << time << std::endl;

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

  }

}

void home(double phi_home, double psi_home, double theta1_home, double theta2_home, double p2p_time,
FiveBarKinematics& five_bar_kinematics, TwoRKinematics& twor_kinematics, Pi3HatInterface& pi3_interface, TrajectoryPlanner& traj, TrajectoryPlanner& traj_2R, 
std::vector<MoteusCommand>& cmds, std::vector<MoteusResponse>& resp)
{
  joint_ini_five_bar[0] = conf_feed[0];
  joint_ini_five_bar[1] = conf_feed[1];
  joint_fin_five_bar[0] = phi_home;
  joint_fin_five_bar[1] = psi_home;
  joint_ini_2R[0] = theta1_feed;
  joint_ini_2R[1] = theta2_feed;
  joint_fin_2R[0] = theta1_home;
  joint_fin_2R[1] = theta2_home;

  p2p(joint_ini_five_bar, joint_fin_five_bar, joint_ini_2R, joint_fin_2R, p2p_time, five_bar_kinematics, twor_kinematics, pi3_interface, traj, traj_2R, cmds, resp);
}

void getCurrentPos(FiveBarKinematics& five_bar_kinematics, TwoRKinematics& twor_kinematics, Pi3HatInterface& pi3_interface, TrajectoryPlanner& traj, TrajectoryPlanner& traj_2R, 
std::vector<MoteusCommand>& cmds, std::vector<MoteusResponse>& resp)
{
  auto ti = high_resolution_clock::now();
  auto timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);

  double time = 0;

  while(time <= 0.5)  
  {
    timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);
    time = timeD.count()*1e-6;

    orderResponse(pi3_interface.read(), resp);

    five_bar_kinematics.getRelativeAngles(resp[0].position, resp[1].position, phi_feed, psi_feed);
    twor_kinematics.getRelativeAngles(resp[2].position, resp[3].position, theta1_feed, theta2_feed);
    
  //        std::cout << "theta1: " << theta1_feed << ", theta2: " << theta2_feed << ", phi: " << phi_feed << ", psi: " << psi_feed << std::endl;
//    std::cout << "phi: " << resp[0].position << ", psi: " << resp[1].position << ", theta1: " << resp[2].position << ", theta2: " << resp[3].position << std::endl;
//    std::cout << "phi: " << std::to_string(resp[0].id) << ", psi: " << std::to_string(resp[1].id) << ", theta1: " << std::to_string(resp[2].id) << ", theta2: " << std::to_string(resp[3].id) << std::endl;
    
    pi3_interface.stop();

    std::this_thread::sleep_for(std::chrono::milliseconds(10));  
  }
}

int main(void)
{

  std::vector<MoteusCommand> cmds;
  std::vector<MoteusResponse> resp;

  Matrix<double, 2, 1> A0, B0, C0, D0, F0, P0;
  Matrix<double, 2, 6> dim5bar;

  // Dimensions of the five-bar mechanism
  double phi_offset = -61*M_PI/36;
  double psi_offset = 13*M_PI/36;

  // Dimensions of the 2R mechanism
  double l1 = 0.0894, l2 = 0.0894;
  double theta1_offset = M_PI/2;//-0.65675;
  double theta2_offset = M_PI/2;//-0.04075;
  double m1_sign = -1;
  double m2_sign = 1;

  // Five-bar dimensions
  A0 << 0.12, 0;
  B0 << 0, 0;
  C0 << 0.09216732905642444, -0.039749173530376616;
  D0 << -0.011227449650384648, -0.02407734347492251;
  F0 << 0.027234061360222564, -0.06273044035186855;
  P0 << 0.10818369270155971, -0.17033426671218643;

  // Setting up five-bar kinematics
  dim5bar << A0(0), B0(0), C0(0), D0(0), F0(0), P0(0),
          A0(1), B0(1), C0(1), D0(1), F0(1), P0(1);
          
  FiveBarKinematics five_bar_kinematics(dim5bar, phi_offset, psi_offset);
  TwoRKinematics twor_kinematics(l1, l2, theta1_offset, theta2_offset, m1_sign, m2_sign);

  TrajectoryPlanner traj;
  TrajectoryPlanner traj_2R;

  const char* serverAddress = "10.42.0.1"; // Server's IP address
  const int serverPort = 12345; // Server's port number

  // Run the client function in a separate thread
  std::thread clientThread(send_data_to_python, serverAddress, serverPort);

  Vector<double, 6> conf_ini, conf_fin;
  Matrix<double, 2, 1> P;

  double phi_ini, phi_fin, psi_ini, psi_fin;
  double cycle_freq;
  double path_length = 0;
  double total_time;
  double temp1 = 0, temp2 = 0;
  int i=0, pub_iter = 0;
  double p2p_time = 2;
  bool print_once = true, print_once_standby = true;

  pi3_interface.initialize(servo_bus_map);

//  FiveBarKinematics five_bar_kin(dim5bar, phi_offset, psi_offset);

//  five_bar_kinematics = five_bar_kin;

  cout << "Five bar dim: " << endl << dim5bar << endl;

  // Setting up motor commands
  cmds.resize(4);
  resp.resize(4);
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

  cmds[2].id = 3;
  cmds[2].mode = 10; // Position mode
  cmds[2].position = 0.0;
  cmds[2].velocity = 0.0;
  cmds[2].feedforward_torque = 0;
  cmds[2].kp_scale = 7;
  cmds[2].kd_scale = 4;
  cmds[2].watchdog_timeout = 0;

  cmds[3].id = 4;
  cmds[3].mode = 10; // Position mode
  cmds[3].position = 0.0;
  cmds[3].velocity = 0.0;
  cmds[3].feedforward_torque = 0;
  cmds[3].kp_scale = 7;
  cmds[3].kd_scale = 4;
  cmds[3].watchdog_timeout = 0;

  double alpha = 0.01;
  double temp_thresh = 23, temp_thresh_low = 23;

  Vector<double, 2> pI_vm, pF_vm, pI_hm, pF_hm;
  Vector<double, 2> pI_2R_vm, pF_2R_vm, pI_2R_hm, pF_2R_hm;
  double path_time_vm = 0.5; //Vertical motion path time
  double n_vm = 30; // VM number of times
  double path_time_hm = 0.25; //Horizontal motion path time
  double n_hm = 30; // HM number of times

  pI_vm << 0.15757, -0.1145;
  pF_vm << 0.1341, -0.101;
  pI_hm << 0.1218, -0.1687;
  pF_hm << 0.1627, -0.098;

  pI_2R_vm << 0,-0.175;
  pF_2R_vm << 0,-0.148;
  pI_2R_hm << -0.0408,-0.168;
  pF_2R_hm << 0.0408,-0.168;

  five_bar_kinematics.forwardKinematics(phi_home, psi_home, -1, conf_feed);
  twor_kinematics.forwardKinematics(theta1_home, theta2_home, P_feed);
  theta1_feed = theta1_home;
  theta2_feed = theta2_home;

  five_bar_kinematics.inverseKinematics(pI_vm, ikb1, ikb2, conf_ini);
  double phi_vm = conf_ini(0);
  double psi_vm = conf_ini(1);

  five_bar_kinematics.inverseKinematics(pI_hm, ikb1, ikb2, conf_ini);
  double phi_hm = conf_ini(0);
  double psi_hm = conf_ini(1);

  double theta1_vm, theta2_vm, theta1_hm, theta2_hm;
  twor_kinematics.inverseKinematics(pI_2R_vm, ik_branch, theta1_vm, theta2_vm);
  twor_kinematics.inverseKinematics(pI_2R_hm, ik_branch, theta1_hm, theta2_hm);

  std::thread current_thread(logPowerData);

  log_file << "time,phi_cmd,psi_cmd,rho_cmd,theta_cmd,x_cmd,y_cmd,phi_feed,psi_feed,rho_feed,theta_feed,x_feed,y_feed,theta1_cmd,theta2_cmd,cmd0,cmd1,cmd2,cmd3,resp0,resp1,resp2,resp3,I_phi,I_psi,I_theta1,I_theta2,s_5bar,s_2R" << endl;

  // Start all motors in stopped mode to clear all faults
  pi3_interface.stop();
  // Wait for the stop command to be sent 
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  while(run)
  {
    //printf("Running\n");

    switch(mode){
      case 0:
        print_once_standby = true;
        if(print_once)
        {
          printf("Pause\n");
          print_once = false;
        }

        getCurrentPos(five_bar_kinematics, twor_kinematics, pi3_interface, traj, traj_2R, cmds, resp);
        five_bar_kinematics.forwardKinematics(phi_feed, psi_feed, -1, conf_feed);
        twor_kinematics.forwardKinematics(theta1_feed, theta2_feed, P_feed);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        break;

      case 1: // Vertical Motion
        print_once = true;

        std::cout << "Vertical Motion" << std::endl;

        home(phi_home, psi_home, theta1_home_vm, theta2_home_vm, p2p_time, five_bar_kinematics, twor_kinematics, pi3_interface, traj, traj_2R, cmds, resp);

        joint_ini_five_bar[0] = conf_feed(0);
        joint_ini_five_bar[1] = conf_feed(1);
        joint_fin_five_bar[0] = phi_vm;
        joint_fin_five_bar[1] = psi_vm;
        joint_ini_2R[0] = theta1_feed;
        joint_ini_2R[1] = theta2_feed;
        joint_fin_2R[0] = theta1_vm;
        joint_fin_2R[1] = theta2_vm;

        p2p(joint_ini_five_bar, joint_fin_five_bar, joint_ini_2R, joint_fin_2R, p2p_time, five_bar_kinematics, twor_kinematics, pi3_interface, traj, traj_2R, cmds, resp);

        linear_motion(pI_vm, pF_vm, pI_2R_vm, pF_2R_vm, path_time_vm, n_vm, five_bar_kinematics, twor_kinematics, pi3_interface, traj, traj_2R, cmds, resp);

        home(phi_home, psi_home, theta1_home_vm, theta2_home_vm, p2p_time, five_bar_kinematics, twor_kinematics, pi3_interface, traj, traj_2R, cmds, resp);

        pi3_interface.stop();

        {
          std::lock_guard<std::mutex> guard(modeMutex);
          mode = 0;
        }

        break;

      case 2: // Horizontal Motion
        print_once = true;

        std::cout << "Horizontal Motion" << std::endl;

        joint_ini_five_bar[0] = conf_feed(0);
        joint_ini_five_bar[1] = conf_feed(1);
        joint_fin_five_bar[0] = phi_hm;
        joint_fin_five_bar[1] = psi_hm;
        joint_ini_2R[0] = theta1_feed;
        joint_ini_2R[1] = theta2_feed;
        joint_fin_2R[0] = theta1_hm;
        joint_fin_2R[1] = theta2_hm;

        p2p(joint_ini_five_bar, joint_fin_five_bar, joint_ini_2R, joint_fin_2R, p2p_time, five_bar_kinematics, twor_kinematics, pi3_interface, traj, traj_2R, cmds, resp);

        linear_motion(pI_hm, pF_hm, pI_2R_hm, pF_2R_hm, path_time_hm, n_hm, five_bar_kinematics, twor_kinematics, pi3_interface, traj, traj_2R, cmds, resp);

        home(phi_home, psi_home, theta1_home, theta2_home, p2p_time, five_bar_kinematics, twor_kinematics, pi3_interface, traj, traj_2R, cmds, resp);
        pi3_interface.stop();

        //        while(mode == 2); // Idle untill mode changes

        {
          std::lock_guard<std::mutex> guard(modeMutex);
          mode = 0;
        }

        break;

      case 3: // Velocity Ellipse
        print_once = true;

        std::cout << "Velocity Ellipse" << std::endl;

        velocity_ellipse(five_bar_kinematics, twor_kinematics, pi3_interface, traj, traj_2R, cmds, resp);

        pi3_interface.stop();

        {
          std::lock_guard<std::mutex> guard(modeMutex);
          mode = 0;
        }

        break;

      case 5: // Standby mode
        print_once = true;

        if(print_once_standby)
        {
          printf("StandBy\n");
          print_once_standby = false;
          std::queue<std::string> empty; // Empty buffer
          std::swap(dataBuffer, empty);
        }

        pi3_interface.stop();

        {
          std::lock_guard<std::mutex> guard(modeMutex);
          mode = 0;
        }

        break;

        //      default:
        //        pi3_interface.stop();
        //        break;

    }
  }

  pi3_interface.stop();
  logStop = false;

  current_thread.join();  
  clientThread.join();

  return 0;
}
