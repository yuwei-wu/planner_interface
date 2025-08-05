// Initial by 2015 Michael Watterson, University of Pennsylvania
// Revised by 2025 Yuwei Wu, University of Pennsylvania
#ifndef TRAJ_DATcoeffsHPP
#define TRAJ_DATcoeffsHPP

#include <Eigen/Eigen>
#include <cmath>
#include <iostream>
#include <vector>

namespace traj_opt
{

enum PolyType
{
  STANDARD,  // polynomials
  BEZIER,    // B-spline
  BERNSTEIN, // bernstein 
  BOUNDARY   // boundary conditions  also quintic polynomial
};

/**
 * @brief 
 * 
 * - STANDARD: polynomial trajectory
 * 
 * p(t) =  C * [1, t, t^2, t^3, t^4, t^5]^T
 * 
 * polynomials are stored with parameterization s \in [0,1]
 * time dt dt is used to evaluate polynomial p(t/dt) for t \in [0,dt]
 *
 * - BEZIER:
 * - BERNSTEIN: 
 * 
 * p(t) = cpts * A * [1, t, t^2, t^3, t^4, t^5]^T
 * cpts: control points  (N+1)x3 
 * A: transformation matrix (N+1) * (N+1)
 * 
 * then C = cpts * A can also represent with general matrix representations
 * 
 * reference:
 * Kaihuai Qin, "General matrix representations for B-splines," 
 * Proceedings Pacific Graphics '98. Sixth Pacific Conference on Computer Graphics and Applications (Cat. No.98EX208), 
 * 1998, pp. 37-43, doi: 10.1109/PCCGA.1998.731996.
 * 
 * 
 * - BOUNDARY:
 * p(t) = C * [1, t, t^2, t^3, t^4, t^5]^T
 * C = [c0, c1, c2, c3, c4, c5]
 * c0: initial position
 * 
 **/
template <int dim>
class Piece
{
 private:
  int degree = 5;
  float dt = 0.0;
  PolyType basis = STANDARD;
  Eigen::MatrixXd ncoeffs; // dim x (degree + 1) matrix of normalized coefficients

 public:
  Piece() = default;
  ~Piece() = default;

  Piece(PolyType ptype, const Eigen::MatrixXd& matrix, float dur)
      : basis(ptype), dt(dur)
  {
    degree = ncoeffs.cols() - 1;
    if (basis == STANDARD)
    {
      ncoeffs = matrix;
    }
    else
    if (ptype == BOUNDARY)
    {
      boundCond2coeffs(matrix);
    }
  }

  // Constructor for Bezier or Bernstein basis
  Piece(PolyType ptype, const Eigen::MatrixXd& cpts, float dur, int deg)
      : basis(ptype), degree(deg), dt(dur)
  {
    getNcoeffs(cpts);
  
  }
  
  // boundCond (dim x 6): [p0, v0, a0, p1, v1, a1] as columns
  inline boundCond2coeffs(const Eigen::MatrixXd& boundCond)
  {
    double t1 = dt;
    double t2 = t1 * t1;

    // Resize ncoeffs: rows = dimension, cols = degree+1
    ncoeffs.resize(boundCond.rows(), degree + 1);

    ncoeffs.col(0) = 0.5 * (boundCond.col(5) - boundCond.col(2)) * t2 -
                    3.0 * (boundCond.col(1) + boundCond.col(4)) * t1 +
                    6.0 * (boundCond.col(3) - boundCond.col(0));
    ncoeffs.col(1) = (-boundCond.col(5) + 1.5 * boundCond.col(2)) * t2 +
                    (8.0 * boundCond.col(1) + 7.0 * boundCond.col(4)) * t1 +
                    15.0 * (-boundCond.col(3) + boundCond.col(0));
    ncoeffs.col(2) = (0.5 * boundCond.col(5) - 1.5 * boundCond.col(2)) * t2 -
                    (6.0 * boundCond.col(1) + 4.0 * boundCond.col(4)) * t1 +
                    10.0 * (boundCond.col(3) - boundCond.col(0));
    ncoeffs.col(3) = 0.5 * boundCond.col(2) * t2;
    ncoeffs.col(4) = boundCond.col(1) * t1;
    ncoeffs.col(5) = boundCond.col(0);

    // Convert normalized polynomial coefficients to unnormalized (real time)
    for (int i = 0; i <= degree; ++i)
    {
      ncoeffs.col(i) /= std::pow(dt, i);
    }
  }


  inline double getDur() const { return dt; }

  inline Eigen::VectorXd getPos(double t) const
  {
    t /= dt;
    Eigen::VectorXd pos = Eigen::VectorXd::Zero(dim);
    double tn = 1.0;
    for (int i = 0; i <= degree; ++i)
    {
      pos += tn * ncoeffs.col(i);
      tn *= t;
    }
    return pos;
  }

  inline Eigen::VectorXd getVel(double t) const
  {
    t /= dt;
    Eigen::VectorXd vel = Eigen::VectorXd::Zero(dim);
    double tn = 1.0;
    for (int i = 1; i <= degree; ++i)
    {
      vel += i * tn * ncoeffs.col(i);
      tn *= t;
    }
    vel /= dt;
    return vel;
  }

  inline Eigen::VectorXd getAcc(double t) const
  {
    t /= dt;
    Eigen::VectorXd acc = Eigen::VectorXd::Zero(dim);
    double tn = 1.0;
    for (int i = 2; i <= degree; ++i)
    {
      acc += i * (i - 1) * tn * ncoeffs.col(i);
      tn *= t;
    }
    acc /= (dt * dt);
    return acc;
  }

 private:
  void getNcoeffs(const Eigen::MatrixXd& cpts)
  {
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(degree + 1, degree + 1);
    bool valid = true;

    switch (basis)
    {
      case BEZIER:
      {
        switch (degree)
        {
          case 1:
            A << 1, -1,
                 0,  1;
            break;
          case 2:
            A << 1, -2,  1,
                 1,  2, -2,
                 0,  0,  1;
            A /= 2.0;
            break;
          case 3:
            A << 1, -3,  3,  1,
                 4,  0, -6,  3,
                 1,  3,  3, -3,
                 0,  0,  0,  1;
            A /= 6.0;
            break;
          case 4:
            A <<  1, -4,   6,  -4,  1,
                 11,-12,  -6,  12, -4,
                 11, 12,  -6, -12,  6,
                  1,  4,   6,   4, -4,
                  0,  0,   0,   0,  1;
            A /= 24.0;
            break;
          default:
            valid = false;
            break;
        }
        break;
      }
      case BERNSTEIN:
      {
        switch (degree)
        {
          case 1:
            A << 1, -1,
                 0,  1;
            break;
          case 2:
            A << 1, -2,  1,
                 0,  2, -2,
                 0,  0,  1;
            break;
          case 3:
            A << 1, -3,  3,  1,
                 0,  3, -6,  3,
                 0,  0,  3, -3,
                 0,  0,  0,  1;
            break;
          case 4:
            A <<  1, -4,   6,  -4,  1,
                  0,  4, -12, 12, -4,
                  0,  0,   6, -12, 6,
                  0,  0,   0,  4, -4,
                  0,  0,   0,  0,  1;
            break;
          default:
            valid = false;
            break;
        }
        break;
      }
      default:
        valid = false;
        break;
    }

    if (!valid)
    {
      std::cerr << "[ERROR] Unsupported basis or degree: basis=" << basis << ", degree=" << degree << std::endl;
      ncoeffs = cpts.transpose();  // fallback
    }
    else
    {
      ncoeffs = cpts.transpose() * A;
    }
  }
};

template <int dim>
class Trajectory
{
 private:
  std::vector<Piece<dim>> seg_pieces;
  int seg_num = 0;
  double dttotal = 0.0;

 public:
  Trajectory() = default;
  ~Trajectory() = default;

  Trajectory(const std::vector<Piece<dim>>& segs, double dur)
      : seg_pieces(segs), seg_num(segs.size()), dttotal(dur)
  {}

  inline bool isValid() const { return seg_num > 0; }

  inline int locatePieceIdx(double& t) const
  {
    int idx = 0;
    for (; idx < seg_num; ++idx)
    {
      double dur = seg_pieces[idx].getDur();
      if (t <= dur) break;
      t -= dur;
    }
    if (idx >= seg_num)
    {
      idx = seg_num - 1;
      t = seg_pieces[idx].getDur();
    }
    return idx;
  }

  inline Eigen::VectorXd getPos(double t) const
  {
    int pieceIdx = locatePieceIdx(t);
    return seg_pieces[pieceIdx].getPos(t);
  }

  inline Eigen::VectorXd getVel(double t) const
  {
    int pieceIdx = locatePieceIdx(t);
    return seg_pieces[pieceIdx].getVel(t);
  }

  inline Eigen::VectorXd getAcc(double t) const
  {
    int pieceIdx = locatePieceIdx(t);
    return seg_pieces[pieceIdx].getAcc(t);
  }
};

typedef Trajectory<1> Trajectory1D;
typedef Trajectory<2> Trajectory2D;
typedef Trajectory<3> Trajectory3D;
typedef Trajectory<4> Trajectory4D;


// Linear discrete states
// State: [x, y, z, vx, vy, vz, ax, ay, az]^T
class DiscreteStates
{
 private:
  int N = 20;
  double dt = 0.0;
  std::vector<Eigen::VectorXd> states;
  bool is_linear_cut = false;

 public:
  DiscreteStates() = default;

  DiscreteStates(double interval, int num, const std::vector<Eigen::VectorXd>& discrete_states)
      : dt(interval), N(num), states(discrete_states)
  {}

  ~DiscreteStates() = default;

  inline void useLinearCut() { is_linear_cut = true; }

  inline Eigen::VectorXd getState(double t) const
  {
    int index = std::floor(t / dt);
    index = std::min(std::max(index, 0), N - 2);

    Eigen::VectorXd x1 = states[index];
    Eigen::VectorXd x2 = states[index + 1];

    double tau = t - index * dt;

    if (is_linear_cut)
    {
      return x1 + (tau / dt) * (x2 - x1);
    }

    Eigen::MatrixXd psi = Phi(dt - tau).transpose();
    Eigen::MatrixXd lambda = Phi(tau) - psi * Phi(dt);

    return lambda * x1 + psi * x2;
  }

  inline Eigen::VectorXd getPreState(double t) const
  {
    int index = std::floor(t / dt);
    return states[std::min(index, N - 1)];
  }

  inline Eigen::VectorXd getNextState(double t) const
  {
    int index = std::floor(t / dt);
    return states[std::min(index + 1, N - 1)];
  }

  inline Eigen::Vector3d getNextPos(double t) const
  {
    int index = std::floor(t / dt);
    return states[std::min(index + 1, N - 1)].head<3>();
  }

 private:
  static inline Eigen::Matrix<double, 9, 9> Phi(double tau)
  {
    Eigen::Matrix<double, 9, 9> phi = Eigen::Matrix<double, 9, 9>::Identity();
    for (int i = 0; i < 6; ++i) phi(i, i + 3) = tau;
    for (int i = 0; i < 3; ++i) phi(i, i + 6) = 0.5 * tau * tau;
    return phi;
  }
};


// traj data
struct TrajData
{
  /* info of generated traj */
  double traj_dur_ = 0, traj_yaw_dur_ = 0;
  rclcpp::Time start_time_;
  int dim_;

  traj_opt::Trajectory3D traj_3d_;
  traj_opt::Trajectory1D traj_yaw_;
  traj_opt::DiscreteStates traj_discrete_;
};


typedef std::vector<TrajData> MultiTrajData;


}  // namespace traj_opt

#endif
