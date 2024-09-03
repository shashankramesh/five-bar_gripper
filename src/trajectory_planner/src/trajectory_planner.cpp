#include "trajectory_planner.h"

// TODO: Acceleration cap
bool TrajectoryPlanner::initialize(double total_time, double max_velocity, double path_length)
{
  bool st = true;
  if (path_length >= 0.0001)
	{
    Tm = total_time;
    vm = max_velocity;
    sm = path_length;

    double avsp = sm/Tm;

    if(vm < avsp)
    {
      v = avsp;
      st = false;
    }
    else
    {
      if(vm > 2*avsp)
      {
        v = 2*avsp;
        st = false;
      }
      else
      {
        v = vm;
        st = true;
      }
    }

    t2 = sm/v;
    t1 = Tm - t2;

    a = v/t1;
    
    zero_path_length = false;

  }
  else
  {
    zero_path_length = true;
  }

  return st;
}

double TrajectoryPlanner::initializeTTBPC(double sF, double freq)
{
  vst = (swlen + (2*alen + 2*dlen + stlen + swlen)*sF + stlen*sF*sF)*freq/(sF*(1+sF));
  vsw = sF*vst;

  tst = stlen/vst;
  tacc = tst + (2*alen/(vst+vsw));
  tsw = tacc + swlen/vsw;
  tdec = tsw + (2*dlen/(vst+vsw));

  acc = (vsw*vsw - vst*vst)/(2*alen);
  dcc = (vsw*vsw - vst*vst)/(2*dlen);

  nTTBPC = 0;

  return tdec;
}

void TrajectoryPlanner::trapezoidalTrajectory(double time, double& s)
{
  if(zero_path_length)
  {
    s = 0;
  }
  else
  {
    if(time <= t1)
    {
      u = a*time*time/2;
    }
    else
    {
      if(time <= t2)
      {
        u = v*time - (v*v/(2*a));
      }
      else
      {
        if(time <= Tm)
        {
          u = (2*a*v*Tm - 2*v*v - a*a*(time-Tm)*(time-Tm))/(2*a);
        }
        else
        {
          u = sm;
        }
      }
    }

    s = u/sm;
  }
}

void TrajectoryPlanner::trapezoidalTrajectoryBPC(double t, double& s)
{
  time = t - nTTBPC*tdec;

  if(time <= tst)
  {
    s = vst*time/pL;
  }
  else
  {
    if(time <= tacc)
    {
      double td = time - tst;
      s = (stlen + vst*td + 0.5*acc*td*td)/pL;
    }
    else
    {
      if(time <= tsw)
      {
        s = (pL1 + vsw*(time - tacc))/pL;
      }
      else
      {
        if(time <= tdec)
        {
          double td = time - tsw;
          s = (pL2 + vsw*td - 0.5*dcc*td*td)/pL;
        }
        else
        {
          nTTBPC++;
        }
      }
    }
  }
}

void TrajectoryPlanner::straightLinePath(Vector<double, 2> pI, 
    Vector<double, 2> pF, double s, Vector<double, 2>& P)
{
  P = pI + s*(pF-pI);
}

double TrajectoryPlanner::pathBPCinitialize()
{
  bpc_initialize = true;

  p12 = bpc_params.P2-bpc_params.P1;
  p1p2 = sqrt(p12(0)*p12(0) + p12(1)*p12(1));

  stlen = p1p2;
  alen = bpc_params.r2*abs(bpc_params.theta2);
  swlen = bpc_params.R*abs(bpc_params.thetaC);
  dlen = bpc_params.r1*abs(bpc_params.theta1);

  pL1 = p1p2 + alen;
  pL2 = pL1 + swlen;
  pL = pL2 + dlen;

  return pL;
}

void TrajectoryPlanner::pathBPC(double t, Vector<double, 2>& P)
{
/*  cout << "bpc params: " << bpc_params.P1 << endl;
  cout << "bpc params: " << bpc_params.P2 << endl;
  cout << "bpc params: " << bpc_params.Po1 << endl;
  cout << "bpc params: " << bpc_params.Po2 << endl;
  cout << "bpc params: " << bpc_params.Po << endl;
  cout << "bpc params: " << bpc_params.Pc1 << endl;
  cout << "bpc params: " << bpc_params.Pc2 << endl;*/

  if(bpc_initialize)
  {
    pt = pL*t;

    if(pt <= p1p2)
    {
      P = bpc_params.P1 + (pt/p1p2)*p12;
    }
    else
    {
      if(pt <= pL1)
      {
        theta2 = (pt - p1p2)/bpc_params.r2;
        Rtheta2 << cos(theta2), -sin(theta2), sin(theta2), cos(theta2);
        P = bpc_params.Po2 + Rtheta2*(bpc_params.P2 - bpc_params.Po2);
      }
      else
      {
        if(pt <= pL2)
        {
          thetaC = (pt - pL1)/bpc_params.R;
          RthetaC << cos(thetaC), -sin(thetaC), sin(thetaC), cos(thetaC);
          P = bpc_params.Po + RthetaC*(bpc_params.Pc2 - bpc_params.Po);
        }
        else
        {
          if(pt <= pL)
          {
            theta1 = (pt - pL2)/bpc_params.r1;
            Rtheta1 << cos(theta1), -sin(theta1), sin(theta1), cos(theta1);
            P = bpc_params.Po1 + Rtheta1*(bpc_params.Pc1 - bpc_params.Po1);
          }
          else
          {
            P = bpc_params.P1;
          }
        }
      }
    }
  }
  else
  {
    cout << "BPC function not initialized!" << endl;
  }

  return;

}

  


