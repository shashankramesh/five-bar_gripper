#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/LU>
//#include "pi3hat/pi3hat_interface.h"

using namespace Eigen;
using namespace std;
 
class FiveBarKinematics
{
  public:
    FiveBarKinematics(Matrix<double, 2, 6> fivebardim, double phi_offset, double psi_offset, double phi_s, double psi_s)
    {
      A0 = fivebardim(all, 0);
      B0 = fivebardim(all, 1);
      C0 = fivebardim(all, 2);
      D0 = fivebardim(all, 3);
      F0 = fivebardim(all, 4);
      P0 = fivebardim(all, 5);

      CA = C0-A0;
      PC = P0-C0;
      FC = F0-C0;
      FD = F0-D0;
      PF = P0-F0;
      PC = P0-C0;
      DB = D0-B0;
      FD = F0-D0;

      I90 << 0, -1, 1, 0;

      FClen = norm(FC);
      FDlen = norm(FD);
      
      phi_ref = phi_offset;
      psi_ref = psi_offset;

      phi_sign = phi_s;
      psi_sign = psi_s;

    }

    ~FiveBarKinematics()
    {
    }

    void setAngleOffsets(double phi_offset, double psi_offset)
    {
      phi_ref = phi_offset;
      psi_ref = psi_offset;
    }

    void addAngleOffsets(double phi_add, double psi_add)
    {
      phi_ref += phi_add;
      psi_ref += psi_add;
    }

    void getRelativeAngles(double phim, double psim, double& phi, double& psi)
    {
      phi = ((double)phi_sign)*phim - phi_ref;
      psi = ((double)psi_sign)*psim - psi_ref;
    }
    
    void getMotorAngles(double phi, double psi, double& phim, double& psim)
    {
      phim = ((double)phi_sign)*(phi + phi_ref);
      psim = ((double)psi_sign)*(psi + psi_ref);
    }

    /*
     * Forward Kinematics of the Five-Bar Mechanism
     *  phi - Angular displacement of AC link
     *  psi - Angular displacement of BD link
     *  pm - FK solution branch, can be -1 or 1
     *  result - Vector of results from FK passed by reference:
     *    result(0) - phi
     *    result(1) - psi
     *    result(2) - rho (Angular displacement of CF)
     *    result(3) - theta (Angular displacement of DF)
     *    result(4) - P(0) (x coordinate of end-effector)
     *    result(5) - P(1) (y coordinate of end-effector)
     *  returns - 1 if FK computation was successful
     *          - 0 if the given phi and psi are outside the input bounds
     * */
    bool forwardKinematics(double phi, double psi, int pm, 
        Vector<double, 6>& result)
    {
      Rotphi << cos(phi), -sin(phi),
      sin(phi), cos(phi);
      Rotpsi << cos(psi), -sin(psi),
             sin(psi), cos(psi);


      delta = A0-B0 + Rotphi*CA - Rotpsi*(D0-B0);

      if(norm(delta) <= FClen + FDlen)
      {
        alpha = 2*dot(delta, FC);
        beta = 2*dot(delta, I90*FC);
        gamma = dot(delta, delta) + dot(FC, FC) - dot(FD, FD);
        rho = atan2(beta, alpha) + 
          ((double)pm)*acos(-gamma/sqrt(alpha*alpha+beta*beta));
        Rotrho << cos(rho), -sin(rho),
               sin(rho), cos(rho);
        matep << FD(0), FD(1), -FD(1), FD(0);
        ep = matep*(delta + Rotrho*FC);
        theta = atan2(ep(1), ep(0));
        P = A0 + Rotphi*(C0-A0) + Rotrho*(P0-C0);

        result(0) = phi;
        result(1) = psi;
        result(2) = rho;
        result(3) = theta;
        result(4) = P(0);
        result(5) = P(1);

        return true;
      }
      else
      {
        return false;
      }
    }

    /*
     * Inverse Kinematics of the Five-Bar Mechanism
     *  P - coordinates of the end-effector (x, y)
     *  pm1, pm2 - IK solution branch, can be -1 or 1
     *  result - Vector of results from FK passed by reference:
     *    result(0) - phi
     *    result(1) - psi
     *    result(2) - rho (Angular displacement of CF)
     *    result(3) - theta (Angular displacement of DF)
     *    result(4) - P(0) (x coordinate of end-effector)
     *    result(5) - P(1) (y coordinate of end-effector)
     * */
    // TODO: Detect output boundaries
    void inverseKinematics(Matrix<double, 2, 1> P, int pm1, int pm2, 
        Vector<double, 6>& result)
    {
      PA = P-A0;
      alpha = 2*dot(PA, CA);
      beta = 2*dot(PA, I90*CA);
      gamma = dot(PC, PC) - dot(PA, PA) - dot(CA, CA);
      phi = atan2(beta, alpha) + ((double)pm1)*acos(-gamma/sqrt(alpha*alpha + beta*beta));
      Rotphi << cos(phi), -sin(phi),
             sin(phi), cos(phi);
      matep << PC(0), PC(1), -PC(1), PC(0);
      ep = matep*(PA-Rotphi*CA);
      rho = atan2(ep(1), ep(0));
      Rotrho << cos(rho), -sin(rho),
             sin(rho), cos(rho);

      delta = P-B0 + Rotrho*(F0-P0);
      alpha = 2*dot(delta, DB);
      beta = 2*dot(delta, I90*DB);
      gamma = -dot(delta, delta) - dot(DB, DB) + dot(FD, FD);
      psi = atan2(beta, alpha) + ((double)pm2)*acos(-gamma/sqrt(alpha*alpha + beta*beta));
      Rotpsi << cos(psi), -sin(psi),
             sin(psi), cos(psi);
      matep << FD(0), FD(1), -FD(1), FD(0);
      ep << matep*(delta - Rotpsi*DB);
      theta = atan2(ep(1), ep(0));

      result(0) = phi;
      result(1) = psi;
      result(2) = rho;
      result(3) = theta;
      result(4) = P(0);
      result(5) = P(1);
    }

    /*
     * Jacobian of the given configuration of the Five-Bar Mechanism
     *  config - Configuration of the mechanism:
     *    config(0) - phi
     *    config(1) - psi
     *    config(2) - rho (Angular displacement of CF)
     *    config(3) - theta (Angular displacement of DF)
     *    config(4) - P(0) (x coordinate of end-effector)
     *    config(5) - P(1) (y coordinate of end-effector)
     * */
    void jacobian(Vector<double, 6> config, Matrix<double, 2, 2>& J)
    {
      phi = config(0);
      psi = config(1);
      rho = config(2);
      theta = config(3);

      Rotphi << cos(phi), -sin(phi),
             sin(phi), cos(phi);
      Rotpsi << cos(psi), -sin(psi),
             sin(psi), cos(psi);
      Rotrho << cos(rho), -sin(rho),
             sin(rho), cos(rho);
      Rottheta << cos(theta), -sin(theta),
               sin(theta), cos(theta);

      vec1 = -Rotrho*FC;
      vec2 = -Rottheta*FD;
      vec3 = Rotrho*PF;
      vec4 = Rotrho*PC;
      vec5 = -Rotphi*CA;
      vec6 = -Rotpsi*DB;

      mat1 << vec1(0), vec1(1), vec2(0), vec2(1);
      mat2 << vec2(0), vec2(1), vec3(0), vec3(1);
      mat3 << vec4(0), vec4(1), vec5(0), vec5(1);
      mat4 << -vec2(0), -vec2(1), vec6(0), vec6(1);
      den = det(mat1);
      col1 = -det(mat2)*I90*Rotphi*CA - det(mat3)*I90*Rottheta*FD;
      col2 = det(mat4)*I90*Rotrho*PC;
      J << col1(0)/den, col2(0)/den, col1(1)/den, col2(1)/den;

    }

    double CFDAngle(Vector<double, 6> config)
    {
      rho = config(2);
      theta = config(3);

      Rotrho << cos(rho), -sin(rho),
             sin(rho), cos(rho);
      Rottheta << cos(theta), -sin(theta),
               sin(theta), cos(theta);

      vec1 = -Rotrho*FC;
      vec2 = -Rottheta*FD;

      return acos(dot(vec1, vec2)/(FClen*FDlen));
    }

    bool checkInputSingularSafe(Vector<double, 6> config, double CFDmin, 
        double CFDmax)
    {
      CFD = CFDAngle(config);

      return (CFD <= CFDmax) && (CFD >= CFDmin);
    }

  private:
    Matrix<double, 2, 1> A0, B0, D0, C0, F0, P0;
    Matrix<double, 2, 2> Rotphi, Rotrho, Rotpsi, Rottheta;
    Matrix<double, 2, 2> I90;
    Matrix<double, 2, 2> matep;
    Matrix<double, 2, 1> FC, FD, ep, P, delta;
    Matrix<double, 2, 1> PA, CA, PC, DB, PF, col1, col2;
    Matrix<double, 2, 1> vec1, vec2, vec3, vec4, vec5, vec6;
    Matrix<double, 2, 2> mat1, mat2, mat3, mat4;

    double alpha, beta, gamma, theta, rho, psi, phi, den, FClen, FDlen;
    double CFD, phi_ref, psi_ref, phi_sign, psi_sign;

    inline double dot(Matrix<double, 2, 1> v1, Matrix<double, 2, 1> v2)
    {
      return v1(0)*v2(0) + v1(1)*v2(1);
    }

    inline double det(Matrix<double, 2, 2> mat)
    {
      return mat(0, 0)*mat(1, 1) - mat(1, 0)*mat(0, 1);
    }

    inline double norm(Matrix<double, 2, 1> vec)
    {
      return sqrt(vec(0)*vec(0) + vec(1)*vec(1));
    }

};
