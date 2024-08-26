#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/LU>
#include <iostream>

using namespace Eigen;
using namespace std;

class TrajectoryPlanner
{
  public:
    TrajectoryPlanner()
    {}

    ~TrajectoryPlanner()
    {}

    bool initialize(double total_time, double max_velocity, double path_length);

    void trapezoidalTrajectory(double time, double& s);

    void straightLinePath(Vector<double, 2> pI, Vector<double, 2> pF, 
        double s, Vector<double, 2>& P);

    void pathBPC(double t, Vector<double, 2>& P);

    double pathBPCinitialize();
    
    double initializeTTBPC(double sF, double freq);
    
    void trapezoidalTrajectoryBPC(double t, double& s);
    
    struct BPCparameters
    {
      Vector<double, 2> P1;
      Vector<double, 2> P2;
      Vector<double, 2> Po1;
      Vector<double, 2> Po2;
      Vector<double, 2> Po;
      Vector<double, 2> Pc1;
      Vector<double, 2> Pc2;

      double r1;
      double r2;
      double R;
      double theta1;
      double theta2;
      double thetaC;

    } bpc_params;

  private:
    double t1, t2, v, a, u, sm, vm, Tm;
    double p1p2, pL1, pL2, pL, pt;
    bool bpc_initialize = false;
    Vector<double, 2> p12;
    Matrix<double, 2, 2> Rtheta1, Rtheta2, RthetaC;
    double theta1, theta2, thetaC;
    double vsw, vst, stlen, swlen, alen, dlen;
    double tst, tacc, tsw, tdec, acc, dcc, time;
    int nTTBPC;

};
