#include <rclcpp/rclcpp.hpp>

#include <poly_lib/traj_data.hpp>
#include <traj_msgs/msg/single_traj.hpp>


void SingleTraj2TrajData(const traj_msgs::msg::SingleTraj::SharedPtr msg,
                         traj_opt::TrajData& traj)
{
  // # === Identification ===
  // int32 drone_id          # Drone Unique ID
  // int32 traj_id           # Trajectory Unique ID
  
  // # === Timing ===
  // builtin_interfaces/Time start_time
  // float64 duration
  
  // # === Trajectory Type Enum ===
  // int8 TRAJ_POLY = 0      # standard polynomial
  // int8 TRAJ_SPLINE = 1   # Bspline or Bernstein
  // int8 TRAJ_DISCRETE = 2  # discrete method
  // int8 TRAJ_BD_DERIV = 3  # hermite quintic 
  
  // int8 traj_type          # Active trajectory type
  
  // # === Union: Only 1 Active Trajectory ===
  // PiecewisePoly polytraj
  // Spline splinetraj
  // Discrete discretetraj
  // BdDervi bddervitraj
  traj = traj_opt::TrajData();  // Resets all members to default values
  traj.traj_dur_ = msg->duration;

  std::vector<traj_opt::Piece<3>> segs_3d;

  switch (msg->traj_type)
  {
  case traj_msgs::msg::SingleTraj::TRAJ_POLY:
    {
      for(size_t i = 0; i < msg->polytraj.seg_x.size(); ++i)
      {
        Eigen::MatrixXd Coeffs(3, 6);
        float dt = msg->polytraj.seg_x[i].dt;
        traj_opt::Piece<3> seg(traj_opt::STANDARD, Coeffs, dt);
        for(size_t j = 0; j < 6; ++j)
        {
          Coeffs(0, j) = msg->polytraj.seg_x[i].coeffs[j];
          Coeffs(1, j) = msg->polytraj.seg_y[i].coeffs[j];
          Coeffs(2, j) = msg->polytraj.seg_z[i].coeffs[j];
        }
        segs_3d.push_back(seg);
      }
      traj.traj_3d_ = traj_opt::Trajectory3D(segs_3d, traj.traj_dur_);
      break;
    }
  case traj_msgs::msg::SingleTraj::TRAJ_SPLINE:
    {
      std::cout << "Received Spline trajectory with " << msg->splinetraj.pos_pts.size() << " control points." << std::endl;
      size_t N = msg->splinetraj.pos_pts.size() - 1;
      size_t M = msg->splinetraj.knots.size();
      size_t degree = M - N - 1;
      int spline_type = msg->splinetraj.spline_type; //0: B-spline, 1: Bernstein (Bezier)


      Eigen::MatrixXd pos_pts(N + 1, 3);  // N + 1
      Eigen::VectorXd knots(M);              // N + degree + 1
      for(size_t i = 0; i < M; ++i)
      {
        knots(i) = msg->splinetraj.knots[i];
      }
      for(size_t i = 0; i <= N; ++i)
      {
        pos_pts(i, 0) = msg->splinetraj.pos_pts[i].x;
        pos_pts(i, 1) = msg->splinetraj.pos_pts[i].y;
        pos_pts(i, 2) = msg->splinetraj.pos_pts[i].z;
      }

      for(size_t i = 0; i < M - 2 * degree; i++)
      {
        Eigen::MatrixXd cpts(degree + 1, 3);
        
        for(size_t j = 0; j <= degree; j++)
        {
          cpts.row(j) = pos_pts.row(i + j);
        }

        double dt = knots(degree + i + 1) - knots(degree + i);
        traj_opt::Piece<3> seg(
          spline_type == 0 ? traj_opt::BEZIER : traj_opt::BERNSTEIN, cpts, dt, degree);
        segs_3d.push_back(seg);
      }

      std::cout << "Total segments in B-spline trajectory: " << segs_3d.size() << std::endl;
      traj.traj_3d_ = traj_opt::Trajectory3D(segs_3d, traj.traj_dur_);
      break;
    }
  case traj_msgs::msg::SingleTraj::TRAJ_DISCRETE:
    {
      std::vector<Eigen::VectorXd> states;
      auto distraj = msg->discretetraj;
      size_t N = distraj.pos_pts.size();
      double dt = distraj.dt;
      for(size_t j = 0; j < N; ++j)
      {
        Eigen::VectorXd s(9);

        s  << distraj.pos_pts[j].x, distraj.pos_pts[j].y, distraj.pos_pts[j].z,
              distraj.vel_pts[j].x, distraj.vel_pts[j].y, distraj.vel_pts[j].z,
              distraj.acc_pts[j].x, distraj.acc_pts[j].y, distraj.acc_pts[j].z;
        states.push_back(s);
      }
      traj.traj_discrete_ = traj_opt::DiscreteStates(dt, N, states);
      std::cout << "Received discrete trajectory with " << N << " states." << std::endl;
      break;
    }

  case traj_msgs::msg::SingleTraj::TRAJ_BD_DERIV:
    {
      std::cout << "Received boundary derivative trajectory." << std::endl;

      size_t N = msg->bddervitraj.durations.size();

      // Helper lambda to convert a geometry_msgs::Point/Vector3 to Eigen::Vector3d
      auto toEigen = [](const auto& v) {
        return Eigen::Vector3d(v.x, v.y, v.z);
      };
      // Pre-extract references for brevity
      const auto& start_pos = msg->bddervitraj.start_pos;
      const auto& start_vel = msg->bddervitraj.start_vel;
      const auto& start_acc = msg->bddervitraj.start_acc;
      const auto& end_pos   = msg->bddervitraj.end_pos;
      const auto& end_vel   = msg->bddervitraj.end_vel;
      const auto& end_acc   = msg->bddervitraj.end_acc;

      const auto& inner_pos = msg->bddervitraj.inner_pos;
      const auto& inner_vel = msg->bddervitraj.inner_vel;
      const auto& inner_acc = msg->bddervitraj.inner_acc;

      for (size_t i = 0; i < N; ++i)
      {
        Eigen::MatrixXd boundCond(3, 6);

        // Starting boundary conditions
        if (i == 0) {
          boundCond.col(0) = toEigen(start_pos);
          boundCond.col(1) = toEigen(start_vel);
          boundCond.col(2) = toEigen(start_acc);
        } else {
          boundCond.col(0) = toEigen(inner_pos[i - 1]);
          boundCond.col(1) = toEigen(inner_vel[i - 1]);
          boundCond.col(2) = toEigen(inner_acc[i - 1]);
        }

        // Ending boundary conditions
        if (i == N - 1) {
          boundCond.col(3) = toEigen(end_pos);
          boundCond.col(4) = toEigen(end_vel);
          boundCond.col(5) = toEigen(end_acc);
        } else {
          boundCond.col(3) = toEigen(inner_pos[i]);
          boundCond.col(4) = toEigen(inner_vel[i]);
          boundCond.col(5) = toEigen(inner_acc[i]);
        }

        double dt = msg->bddervitraj.durations[i];
        traj_opt::Piece<3> seg(traj_opt::BOUNDARY, boundCond, dt);
        segs_3d.push_back(seg);
      }
      traj.traj_3d_ = traj_opt::Trajectory3D(segs_3d, traj.traj_dur_);
      break;
    }
  default:
    {
      RCLCPP_ERROR(rclcpp::get_logger("traj_server"), "Unknown trajectory type: %d", msg->traj_type);
      return;
    }
  }

}
