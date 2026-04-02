// #include <fstream>
#include <ego_planner/planner_manager.h>
#include <thread>
#include "visualization_msgs/msg/marker.hpp" // zx-todo

namespace ego_planner
{

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() {}

  void EGOPlannerManager::initPlanModules(rclcpp::Node::SharedPtr &node, PlanningVisualization::Ptr vis)
  {
    node_ = node;

    node_->declare_parameter("manager/max_vel", -1.0);
    node_->declare_parameter("manager/max_acc", -1.0);
    node_->declare_parameter("manager/max_jerk", -1.0);
    node_->declare_parameter("manager/feasibility_tolerance", 0.0);
    node_->declare_parameter("manager/control_points_distance", -1.0);
    node_->declare_parameter("manager/planning_horizon", 5.0);
    node_->declare_parameter("manager/use_distinctive_trajs", false);
    node_->declare_parameter("manager/drone_id", -1);
    node_->declare_parameter("manager/use_snake_yaw", false);

    node_->get_parameter("manager/max_vel", pp_.max_vel_);
    node_->get_parameter("manager/max_acc", pp_.max_acc_);
    node_->get_parameter("manager/max_jerk", pp_.max_jerk_);
    node_->get_parameter("manager/feasibility_tolerance", pp_.feasibility_tolerance_);
    node_->get_parameter("manager/control_points_distance", pp_.ctrl_pt_dist);
    node_->get_parameter("manager/planning_horizon", pp_.planning_horizen_);
    node_->get_parameter("manager/use_distinctive_trajs", pp_.use_distinctive_trajs);
    node_->get_parameter("manager/drone_id", pp_.drone_id);
    node_->get_parameter("manager/use_snake_yaw", pp_.use_snake_yaw);

    local_data_.traj_id_ = 0;
    grid_map_.reset(new GridMap);
    // grid_map_->initMap(nh);
    grid_map_->initMap(node);
    bspline_optimizer_.reset(new BsplineOptimizer);
    // bspline_optimizer_->setParam(nh);
    bspline_optimizer_->setParam(node);
    bspline_optimizer_->setEnvironment(grid_map_, obj_predictor_);
    bspline_optimizer_->a_star_.reset(new AStar);
    bspline_optimizer_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

    visualization_ = vis;

    snake_yaw_sub_ = node_->create_subscription<traj_utils::msg::SnakeYaw>(
    "planning/snake_yaw",
    10,
    std::bind(&EGOPlannerManager::snakeyawCallback, this, std::placeholders::_1));

  }
  
  void EGOPlannerManager::setMaxVelAcc(float max_vel, float max_acc)
  {
    bspline_optimizer_->setBsplineMaxVelAcc(max_vel, max_acc);
    if (max_vel != 0.0)    
    {
      pp_.max_vel_ = max_vel;
    }
    if (max_acc != 0.0)    
    {
      pp_.max_acc_ = max_acc;
    }
  }

  void EGOPlannerManager::snakeyawCallback(const traj_utils::msg::SnakeYaw::SharedPtr msg)
  {
    pp_.use_snake_yaw = msg->use_snake_yaw;
  }


  bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
  {

    static int count = 0;
    printf("\033[47;30m\n[drone %d replan %d]==============================================\033[0m\n", pp_.drone_id, count++);

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
      continous_failures_count_++;
      return false;
    }

    bspline_optimizer_->setLocalTargetPt(local_target_pt);

    if (pp_.use_snake_yaw)
    {
      if (tryStraightLinePlan(start_pt, start_vel, start_acc, local_target_pt, local_target_vel))
      {
        std::cout << "[EGOPlannerManager] straight-line plan succeeded.\n";
        return true;
      }
      std::cout << "[EGOPlannerManager] straight-line blocked/infeasible, fallback to rebound.\n";
    }


    rclcpp::Time t_start = rclcpp::Clock().now();
    rclcpp::Duration t_init(0, 0), t_opt(0, 0), t_refine(0, 0);

    /*** STEP 1: INIT
    Calculate the first time step ts based on the distance between the start and target points; if the vector magnitude is greater than 0.1 use 1.5×, otherwise 5×.
    ***/
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.5 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5.0; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    // std::cout << "Initial ts: " << ts << "and dist:" << (start_pt - local_target_pt).norm() << std::endl;
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;
    do
    {
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      
      // If we enter the if-branch normally (usually on the first run), the do-block executes only once and just clears the point set.
      // If we enter the else-branch, abnormal situations may set flag_regenerate to true, causing the do-block to run again.
      if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
      {
        //std::cout << "Initial path generated from polynomial trajectory." << flag_first_call << flag_polyInit << flag_force_polynomial << std::endl;
        flag_first_call = false;
        flag_force_polynomial = false;
        // Used to store the generated trajectory
        PolynomialTraj gl_traj;

        double dist = (start_pt - local_target_pt).norm();
        // Check whether (velocity^2 / acceleration) is greater than dist and decide how to compute the time
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;
        // double time_straight_line = dist / pp_.max_vel_;
        // // cout << "Initial polynomial traj generation: dist=" << dist << ", time=" << time << endl;

        // time *= 2.0;
        // cout << "AFTER: Initial polynomial traj generation: dist=" << dist << ", time=" << time << endl;
        // cout << "Local target point" << local_target_pt.transpose() << endl;
        // cout << "Local target vel: " << local_target_vel.transpose() << endl;

        if (!flag_randomPolyTraj)
        // false → generate a single polynomial segment, true → generate a trajectory with random inserted points
        {
          gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
        else
        {
          Eigen::Vector3d mid_point = 0.5 * (start_pt + local_target_pt);
          Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
          Eigen::Vector3d random_inserted_pt = mid_point;
          // only start to add randomness when the straight line connection fails, randomness should help escape from local minima
          if (continous_failures_count_ > 0){
                random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);
          }
          Eigen::MatrixXd pos(3, 3);
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        // Given the initial trajectory, sample points along it that will be used as control points for the B-spline optimization.
        double t;
        bool flag_too_far;
        ts *= 1.5; // ts will be divided by 1.5 in the next
        do
        {
          ts /= 1.5;
          point_set.clear();
          flag_too_far = false;
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);
          for (t = 0; t < time; t += ts)
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5) // If distance between consecutive points is too large, trajectory is too agressive, regenerate with smaller sampling time step.
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
          
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.

        // Enure that last point is close enough to the target point 
        if ((point_set.back() - local_target_pt).norm() > 1e-4)
        {
          cout << "Warning: last point of initial trajectory is not close to the target, add target as the last point. dist=" << (point_set.back() - local_target_pt).norm() << endl;
          point_set.push_back(local_target_pt);
        }

        // Use the true end time for derivatives
        start_end_derivatives.push_back(start_vel);
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(start_acc);
        start_end_derivatives.push_back(gl_traj.evaluateAcc(time));
      }
      else // Initial path generated from previous trajectory.
      {
        double t_cur = (node_->get_clock()->now() - local_data_.start_time_).seconds();
        t_cur = std::max(0.0, std::min(t_cur, local_data_.duration_));

        point_set.clear();

        bool used_prev_traj = false;
        int outer_iter = 0;

        const double ts_nominal = ts;

        do
        {
          if (++outer_iter > 30)
          {
            std::cout << "Time-resampling loop did not converge, break.\n";
            point_set.clear();
            break;
          }

          point_set.clear();

          const double old_remain_time = std::max(0.0, local_data_.duration_ - t_cur);

          const Eigen::Vector3d old_end_pt =
              local_data_.position_traj_.evaluateDeBoorT(local_data_.duration_);
          const Eigen::Vector3d old_end_vel =
              local_data_.velocity_traj_.evaluateDeBoorT(local_data_.duration_);
          const Eigen::Vector3d old_end_acc =
              local_data_.acceleration_traj_.evaluateDeBoorT(local_data_.duration_);


          double tail_dist = (old_end_pt - local_target_pt).norm();
          double tail_time = 0.0;
          bool use_tail = false;
          PolynomialTraj tail_traj;

          if (tail_dist > 1e-4)
          {
            // compute the trail time based on the average of the end velocity of the previous traj and the local target velocity, 
            // with a lower bound to ensure it's not 0 (protect against division by 0).
            double const_vel = std::max(std::max(local_target_vel.norm(), old_end_vel.norm()), pp_.max_vel_ / 2.0);
            tail_time = tail_dist / const_vel;

            if (std::isfinite(tail_time) && tail_time > 1e-6)
            {
              // cout << "Local target velocity" << local_target_vel << endl;

              tail_traj = PolynomialTraj::one_segment_traj_gen(old_end_pt,
                                                               old_end_vel,
                                                               old_end_acc,
                                                               local_target_pt,
                                                               local_target_vel,
                                                               Eigen::Vector3d::Zero(),
                                                               tail_time);
              use_tail = true;
              
            }
          }


          const double total_warmstart_time = old_remain_time + tail_time;

          if (!std::isfinite(total_warmstart_time) || total_warmstart_time < 1e-6)
          {
            point_set.clear();
            point_set.push_back(local_target_pt);
            break;
          }

          // Keep at least 6 intervals => at least 7 points when possible.
          // On repeated attempts, increase interval count to get finer sampling.
          int N = std::max(6, static_cast<int>(std::round(total_warmstart_time / ts_nominal)));
          N = std::max(N, 6 * outer_iter);
          // cout << "Warm-start sampling: total_warmstart_time=" << total_warmstart_time << ", N=" << N << ", ts=" << total_warmstart_time / N << endl;
          ts = total_warmstart_time / static_cast<double>(N);

          if (!std::isfinite(ts) || ts <= 1e-6)
          {
            std::cout << "Invalid ts in warm-start resampling.\n";
            point_set.clear();
            break;
          }

          point_set.reserve(N + 1);

          for (int k = 0; k <= N; ++k)
          {
            double tau = std::min(k * ts, total_warmstart_time);
            Eigen::Vector3d pt;

            if (tau <= old_remain_time || !use_tail)
            {
              // Sample old trajectory
              double t_sample = std::min(t_cur + tau, local_data_.duration_);
              pt = local_data_.position_traj_.evaluateDeBoorT(t_sample);
            }
            else
            {
              // Sample tail trajectory
              double tail_tau = tau - old_remain_time;
              tail_tau = std::min(tail_tau, tail_time);
              pt = tail_traj.evaluate(tail_tau);
            }

            // Avoid duplicate consecutive points caused by phase boundary / clamping
            if (point_set.empty() || (pt - point_set.back()).norm() > 1e-8)
            {
              point_set.push_back(pt);
            }
          }

          // Ensure exact target is included as final point
          if (point_set.empty() || (point_set.back() - local_target_pt).norm() > 1e-4)
          {
            point_set.push_back(local_target_pt);
          }

          used_prev_traj = !point_set.empty();

        } while (point_set.size() < 7); // ensure enough samples for parameterization

        // Fallback: straight-line initial path if previous trajectory is unusable
        if (!used_prev_traj)
        {
          std::cout << "[B-spline init] using fallback straight-line path.\n";

          if (!tryStraightLinePlan(start_pt, start_vel, start_acc, local_target_pt, local_target_vel))
          {
            std::cout << "[B-spline init] fallback straight-line path blocked/infeasible.\n";
            continous_failures_count_++;
            return false;
          }
          else
          {
            // If straight-line plan succeeded, the trajectory has already been updated and visualized in tryStraightLinePlan, return early.
            return true;
          }
        }

        // Final safety check before parameterization
        if (point_set.size() < 4)
        {
          std::cout << "[B-spline] point_set too small (" << point_set.size()
                    << "), aborting.\n";
          return false;
        }

        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        if (point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3) // The initial path is unnormally too long!
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);

    // Convert the trajectory into a B-spline trajectory
    Eigen::MatrixXd ctrl_pts, ctrl_pts_temp;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    vector<std::pair<int, int>> segments;
    segments = bspline_optimizer_->initControlPoints(ctrl_pts, true);
    // Compute the time difference and update the time
    auto now = rclcpp::Clock().now();
    t_init = now - t_start;
    t_start = now;

    /*** STEP 2: OPTIMIZE ***/
    bool flag_step_1_success = false;
    vector<vector<Eigen::Vector3d>> vis_trajs;

    if (pp_.use_distinctive_trajs)
    {
      std::vector<ControlPoints> trajs = bspline_optimizer_->distinctiveTrajs(segments);
      cout << "\033[1;33m"
           << "multi-trajs=" << trajs.size() << "\033[1;0m" << endl;

      double final_cost, min_cost = 999999.0;
      for (int i = trajs.size() - 1; i >= 0; i--)
      {
        if (bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts_temp, final_cost, trajs[i], ts))
        {

          cout << "traj " << trajs.size() - i << " success." << endl;

          flag_step_1_success = true;
          if (final_cost < min_cost)
          {
            min_cost = final_cost;
            ctrl_pts = ctrl_pts_temp;
          }

          // visualization
          point_set.clear();
          for (int j = 0; j < ctrl_pts_temp.cols(); j++)
          {
            point_set.push_back(ctrl_pts_temp.col(j));
          }
          vis_trajs.push_back(point_set);
        }
        else
        {
          cout << "traj " << trajs.size() - i << " failed." << endl;
        }
      }

      t_opt = rclcpp::Clock().now() - t_start;

      visualization_->displayMultiInitPathList(vis_trajs, 0.2);
    }
    else
    {
      flag_step_1_success = bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
      t_opt = rclcpp::Clock().now() - t_start;
      // static int vis_id = 0;
      visualization_->displayInitPathList(point_set, 0.2, 0);
    }

    cout << "plan_success=" << flag_step_1_success << endl;
    if (!flag_step_1_success)
    {
      visualization_->displayOptimalList(ctrl_pts, 0);
      continous_failures_count_++;
      return false;
    }

    t_start = rclcpp::Clock().now();

    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    // Note: Only adjust time in single drone mode. But we still allow ego to adjust its time profile.
    if (pp_.drone_id <= 0)
    {

      double ratio;
      bool flag_step_2_success = true;
      if (!pos.checkFeasibility(ratio, false))
      {
        cout << "Need to reallocate time." << endl;

        Eigen::MatrixXd optimal_control_points;
        flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
        if (flag_step_2_success)
          pos = UniformBspline(optimal_control_points, 3, ts);
      }

      if (!flag_step_2_success)
      {
        printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appeares occasionally. But if continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
        continous_failures_count_++;
        return false;
      }
    }
    else
    {
      static bool print_once = true;
      if (print_once)
      {
        print_once = false;
        RCLCPP_ERROR(rclcpp::get_logger("ego_planner"), "IN SWARM MODE, REFINE DISABLED!");
      }
    }

    // t_refine = ros::Time::now() - t_start;
    t_refine = rclcpp::Clock().now() - t_start;
    
    // save planned results
    updateTrajInfo(pos, node_->get_clock()->now());

    static double sum_time = 0;
    static int count_success = 0;

    sum_time += (t_init + t_opt + t_refine).seconds();

    count_success++;
    
    // cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << ",avg_time=" << sum_time / count_success << endl;
    cout << "total time:\033[42m" << (t_init + t_opt + t_refine).seconds() << "\033[0m,optimize:" << (t_init + t_opt).seconds() << ",refine:" << t_refine.seconds() << ",avg_time=" << sum_time / count_success << endl;

    // success. YoY
    continous_failures_count_ = 0;
    return true;
  }

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), node_->get_clock()->now());

    return true;
  }


  bool EGOPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    total_len += (start_pos - waypoints[0]).norm();
    for (size_t i = 0; i < waypoints.size() - 1; i++)
    {
      total_len += (waypoints[i + 1] - waypoints[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = node_->get_clock()->now();

    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    /*Read each point and compute distances to determine whether interpolation points are needed,
       then compute inserted points and write them into the matrix,
       and finally generate the global trajectory based on the number of inserted points.
       The return value indicates whether planning succeeded. */
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = node_->get_clock()->now();

    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  void EGOPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }

  void EGOPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }
  bool EGOPlannerManager::isStraightLineFree(const Eigen::Vector3d& p0,
                                            const Eigen::Vector3d& p1,
                                            double step) const
  {
    const double dist = (p1 - p0).norm();
    if (dist < 1e-6) return true;

    const Eigen::Vector3d dir = (p1 - p0) / dist;
    const int n = std::max(2, int(std::ceil(dist / step)));

    for (int i = 0; i <= n; ++i)
    {
      const double s = dist * (double(i) / double(n));
      const Eigen::Vector3d p = p0 + s * dir;

      if (grid_map_->getInflateOccupancy(p))
        return false;
    }
    return true;
  }

  bool EGOPlannerManager::tryStraightLinePlan(const Eigen::Vector3d& start_pt,
                                              const Eigen::Vector3d& start_vel,
                                              const Eigen::Vector3d& start_acc,
                                              const Eigen::Vector3d& target_pt,
                                              const Eigen::Vector3d& target_vel)
  {
    // 1) segment collision check (fast reject)
    const double seg_step = std::max(0.5 * grid_map_->getResolution(), 0.05);
    if (!isStraightLineFree(start_pt, target_pt, seg_step))
      {cout << "[EGOPlannerManager] straight-line path blocked by obstacle.\n" << endl;
      return false;}

    // 2) create enough points on the line for Bspline parameterization
    const double dist = (target_pt - start_pt).norm();
    const int num_pts = std::max(7, int(std::ceil(dist / pp_.ctrl_pt_dist)) + 1);

    std::vector<Eigen::Vector3d> point_set;
    point_set.reserve(num_pts);
    for (int i = 0; i < num_pts; ++i)
    {
      const double a = double(i) / double(num_pts - 1);
      point_set.push_back(start_pt + a * (target_pt - start_pt));
    }

    // 3) choose ts identical to existing EGO Planner heuristic
    double avg_ds = (target_pt - start_pt).norm() / std::max(1, num_pts - 1);
    double ts = avg_ds / std::max(1e-3, pp_.max_vel_);

    // add safety margin (slower is safer)
    ts *= 1.5; 

    // 4) derivatives
    std::vector<Eigen::Vector3d> start_end_derivatives;
    start_end_derivatives.reserve(4);
    start_end_derivatives.push_back(start_vel);
    start_end_derivatives.push_back(target_vel);
    start_end_derivatives.push_back(start_acc);
    start_end_derivatives.push_back(Eigen::Vector3d::Zero());

    // 5) parameterize to Bspline
    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    UniformBspline pos(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    // 6) feasibility check
    double ratio = 1.0;
    if (!pos.checkFeasibility(ratio, false)) {
      // scale ts and re-parameterize
      ts *= ratio;
      UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
      pos = UniformBspline(ctrl_pts, 3, ts);
      pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

      ratio = 1.0;
      if (!pos.checkFeasibility(ratio, false)) {
        std::cout << "[EGOPlannerManager] straight-line Bspline infeasible even after ts scaling.\n";
        return false;
      }
    }


    // 7) extra safety: sample the produced Bspline for collision
    const double T = pos.getTimeSum();
    const double dt = std::max(0.02, 0.25 * T / (ctrl_pts.cols() - 3));

    for (double t = 0.0; t <= T; t += dt)
    {
      if (grid_map_->getInflateOccupancy(pos.evaluateDeBoorT(t)))
      {
        cout << "[EGOPlannerManager] straight-line Bspline hits obstacle.\n";      
        return false;
      }
    }

    // 8) commit & visualize
    updateTrajInfo(pos, node_->get_clock()->now());
    continous_failures_count_ = 0;
    visualization_->displayInitPathList(point_set, 0.2, 0);

    return true;
  }

} // namespace ego_planner
