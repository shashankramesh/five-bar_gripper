#include "trajectory_planner.h"
#include <chrono>
#include <iostream>
#include <fstream>
#include <thread>

using namespace std::chrono;

int main(void)
{
  TrajectoryPlanner traj;

  Vector<double, 2> p1, p2, P, pd;

  p1 << 0, 0;
  p2 << 0, 1;

  pd = p2-p1;

  double path_length = sqrt(pd(0)*pd(0) + pd(1)*pd(1));
  double total_time = 4;
  double max_velocity = 0.35;
  double s = 0, time = 0;

  bool status = traj.initialize(total_time, max_velocity, path_length);

  auto ti = high_resolution_clock::now();
  auto timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);

  ofstream log("tp_traj_log.txt");

  log << "time,x,y" << endl;

  while(time <= total_time)
  {
    timeD = duration_cast<microseconds>(high_resolution_clock::now() - ti);
    time = timeD.count()*1e-6;

    traj.trapezoidalTrajectory(time, s);

    traj.straightLinePath(p1, p2, s, P);
    //P = p1 + s*(p2-p1);

    log << time << "," << P(0) << "," << P(1) << endl;

    std::cout << "Point: " << P << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  cout << status << endl;

  return 0;
}
