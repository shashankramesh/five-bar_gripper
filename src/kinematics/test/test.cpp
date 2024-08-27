#include <iostream>
#include <chrono>
#include "kinematics.h"

using namespace std::chrono;

int main(void)
{
  Matrix<double, 2, 1> A0, B0, C0, D0, F0, P0;
  Matrix<double, 2, 6> dim5bar;

  double phi = 15.*M_PI/180.;
  double psi = -20.*M_PI/180.;

  A0 << 0.015, -0.045;
  B0 << 0.04091994919674591, 0;
  C0 << -0.02690992086033131, -0.02171308617787395;
  D0 << 0.038931473927022213, 0.011670701578338172;
  F0 << 0.03131683587683813, -0.03619220463744048;
  P0 << 0.0471521851343047, 0.04999999999992947;

  dim5bar << A0(0), B0(0), C0(0), D0(0), F0(0), P0(0),
          A0(1), B0(1), C0(1), D0(1), F0(1), P0(1),

  cout << "Five bar dim: " << endl << dim5bar << endl;

  FiveBarKinematics five_bar_kinematics(dim5bar, 0, 0, 1, 1);

  Vector<double, 6> result;
  bool res;

  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  for(int i=0; i<=100; i++)
  {
    res = five_bar_kinematics.forwardKinematics(phi, psi, -1, result);
  }
  high_resolution_clock::time_point t2 = high_resolution_clock::now();

  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

  cout << "Result: " << res << endl << result << endl;
  std::cout << "Time Taken: " << time_span.count()*10000 << " microseconds." << endl;

  Matrix<double, 2, 1> P = {result(4), result(5)};

  t1 = high_resolution_clock::now();
  for(int i=0; i<=100; i++)
  {
    five_bar_kinematics.inverseKinematics(P, -1, 1, result);
  }
  t2 = high_resolution_clock::now();

  time_span = duration_cast<duration<double>>(t2 - t1);

  cout << "Result: " << endl << result << endl;
  std::cout << "Time Taken: " << time_span.count()*10000 << " microseconds." << endl;

  Matrix<double, 2, 2> J;

  t1 = high_resolution_clock::now();
  for(int i=0; i<=100; i++)
  {
    five_bar_kinematics.jacobian(result, J);
  }
  t2 = high_resolution_clock::now();

  time_span = duration_cast<duration<double>>(t2 - t1);

  cout << "Jacobian: " << endl << J << endl;
  std::cout << "Time Taken: " << time_span.count()*10000 << " microseconds." << endl;

  double CDF = 0;
  t1 = high_resolution_clock::now();
  for(int i=0; i<=100; i++)
  {
    CDF = five_bar_kinematics.CFDAngle(result);
  }
  t2 = high_resolution_clock::now();

  time_span = duration_cast<duration<double>>(t2 - t1);

  cout << "CDF Angle: " << endl << CDF << endl;
  std::cout << "Time Taken: " << time_span.count()*10000 << " microseconds." << endl;



}
