#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/msg/odometry.hpp"
#include "traj_utils/msg/bspline.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "std_msgs/msg/empty.hpp"
#include "traj_utils/msg/snake_yaw.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <mutex>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <algorithm>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <rclcpp/parameter_event_handler.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include "traj_utils/msg/trajectory_sample.hpp"
#include "traj_utils/msg/optimized_trajectory.hpp"
#include "std_srvs/srv/trigger.hpp"

using std::vector;
using std::min;
using std::cout;
using std::endl;


enum class TrajType { NONE, BSPLINE, SAMPLED };

TrajType type_of_active_traj_{TrajType::NONE};

Eigen::Vector3d Kp_g(0.4, 0.4, 0.4);
Eigen::Vector3d Kd_g(0.0, 0.0, 0.0);
Eigen::Vector3d Ki_g(0.1, 0.1, 0.1);

Eigen::Vector3d Kp_yaw_g(0.4, 0.4, 0.4);
Eigen::Vector3d Kd_yaw_g(0.0, 0.0, 0.0);
Eigen::Vector3d Ki_yaw_g(0.1, 0.1, 0.1);

Eigen::Vector3d v_max_(0.9, 0.9, 0.9);// TODO tune, or make tunable
Eigen::Vector3d v_min_ = -v_max_;

Eigen::Vector3d integrator_max(1.0, 1.0, 1.0);
Eigen::Vector3d integrator_min = -integrator_max;


rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr gains_cb_handle;
static std::shared_ptr<rclcpp::ParameterEventHandler> g_param_handler;
static rclcpp::ParameterEventCallbackHandle::SharedPtr g_param_event_handle;



rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr body_vel_pub;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr new_goal_pub_;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub;
rclcpp::Subscription<traj_utils::msg::Bspline>::SharedPtr bspline_sub;
rclcpp::Subscription<traj_utils::msg::OptimizedTrajectory>::SharedPtr optimized_trajectory_sub;
rclcpp::Subscription<traj_utils::msg::SnakeYaw>::SharedPtr snake_yaw_sub;
rclcpp::Subscription<std_msgs::msg::String>::SharedPtr controller_state_sub;
rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr rviz_clicked_sub_;
rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_traj_server_service_;


bool derivative_ready_ = false;
bool second_ready_ = false;
Eigen::Vector3d last_p_error_;
Eigen::Vector3d last_last_p_error_;
Eigen::Vector3d p_error_deriv_approx_;
Eigen::Vector3d integrated_error_;
double last_yaw_error_;
double last_last_yaw_error_;
double integrated_yaw_error_;
double yaw_error_deriv_approx_;
std::shared_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;
geometry_msgs::msg::PoseStamped new_goal_;
std::atomic<bool> have_odom{false};

quadrotor_msgs::msg::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};
constexpr double PI = 3.1415926;
using ego_planner::UniformBspline;

bool receive_traj_ = false;
bool vel_mode_ = false;
bool received_goal_ = false;
bool active_traj_ = false;
vector<UniformBspline> traj_;
double traj_duration_;
rclcpp::Time start_time_;
int traj_id_;

// yaw control
double last_yaw_;          // wrapped yaw, mainly for compatibility/debug
double last_yaw_dot_;      // commanded yaw rate
double last_yaw_unwrapped_; // continuous internal yaw state
bool yaw_initialized_ = false;

double time_forward_;
bool use_snake_yaw_ = false;
double snake_yaw_ = 0.0;
rclcpp::Node::SharedPtr node_;

Eigen::Vector3d odom_pos_, odom_vel_, odom_ang_vel_;
Eigen::Quaterniond odom_orient_;

static inline double wrapToPi(double a)
{
  a = std::fmod(a + PI, 2.0 * PI);
  if (a < 0.0) a += 2.0 * PI;
  return a - PI;
}

static inline double angleDiff(double target, double current)
{
  return wrapToPi(target - current);
}

static inline double clampd(double x, double lo, double hi)
{
  return std::max(lo, std::min(hi, x));
}

// Returns an angle equivalent to angle_wrapped, but chosen to be closest to reference_unwrapped.
static inline double unwrapNear(double angle_wrapped, double reference_unwrapped)
{
  return reference_unwrapped + angleDiff(angle_wrapped, wrapToPi(reference_unwrapped));
}

std::mutex gains_mtx;
// Protect shared variables
/*
  Kp_g
  Kd_g
  Ki_g
  Kp_yaw_g
  Kd_yaw_g
  Ki_yaw_g
*/
std::mutex traj_mtx;
// Protect shared variables
/*
  receive_traj_
  type_of_active_traj_
  start_time_
  traj_id_
  traj_
  traj_duration_
  optimized_trajectory_
  use_snake_yaw_
  snake_yaw_
*/
std::mutex controller_state_mtx;
// Protect shared variables
/*
  last_yaw_
  last_yaw_dot_
  last_p_error_
  last_last_p_error_
  p_error_deriv_approx_
  integrated_error_
  last_yaw_error_
  last_last_yaw_error_
  integrated_yaw_error_
  yaw_error_deriv_approx_
  derivative_ready_
  second_ready_
*/
std::mutex odometry_mtx;
// Protect shared variables
/*
  odom_vel_
  odom_pos_
  odom_orient_
  odom_ang_vel_
*/

static inline Eigen::Vector3d rotate_target_source(
    const geometry_msgs::msg::TransformStamped& T_target_source,
    const Eigen::Vector3d& v_source)
{
  const auto& q = T_target_source.transform.rotation;

  tf2::Quaternion q_T_target_source(q.x, q.y, q.z, q.w);

  tf2::Matrix3x3 R_T_target_source(q_T_target_source);

  tf2::Vector3 vsource(v_source.x(), v_source.y(), v_source.z());
  tf2::Vector3 vtarget = R_T_target_source * vsource;

  return Eigen::Vector3d(vtarget.x(), vtarget.y(), vtarget.z());
}

struct TrajectorySample
{
  Eigen::Vector3d pos{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acc{Eigen::Vector3d::Zero()};
  double yaw{0.0};
  double yaw_dot{0.0};
  bool valid{false};
};

Eigen::Vector3d transform_body_velocity_into_world_frame(
    double yaw,
    Eigen::Vector3d body_velocity)
{
  // Rotate the desired body veloctiy into world frame

  Eigen::Matrix3d R;
  R << std::cos(yaw), -std::sin(yaw), 0,
       std::sin(yaw),  std::cos(yaw), 0,
                   0,              0, 1;

  return R * body_velocity;
};


class SampledOptimizedTrajectory {
public:
  void setTrajectory(const std::vector<double>& t,
                     const std::vector<Eigen::Vector3d>& p,
                     const std::vector<Eigen::Vector3d>& v,
                     const std::vector<double>& yaw,
                     const std::vector<double>& yaw_dot)
  {
    if (t.empty() || p.empty() || v.empty() || yaw.empty() || yaw_dot.empty() ||
        t.size() != p.size() || t.size() != v.size() ||
        t.size() != yaw.size() || t.size() != yaw_dot.size()) {
      timestamps_.clear();
      position_.clear();
      body_velocity_.clear();
      yaw_.clear();
      yaw_dot_.clear();
      return;
    }

    timestamps_ = t;
    position_ = p;
    body_velocity_ = v;
    yaw_ = yaw;
    yaw_dot_ = yaw_dot;
  }

  void resetTrajectory()
  {
    timestamps_.clear();
    position_.clear();
    body_velocity_.clear();
    yaw_.clear();
    yaw_dot_.clear();
  }

  TrajectorySample sample(double time_since_start) const
  {
    TrajectorySample out;
    Eigen::Vector3d body_v;
    if (timestamps_.empty()) {
      return out;
    }
    if (time_since_start <= timestamps_.front()) {
      out.pos = position_.front();
      body_v = body_velocity_.front();
      out.acc.setZero();
      out.yaw = yaw_.front();
      out.yaw_dot = yaw_dot_.front();
      out.vel = transform_body_velocity_into_world_frame(out.yaw, body_v);
      out.valid = true;
      return out;
    }

    if (time_since_start >= timestamps_.back()) {
      out.pos = position_.back();
      out.vel.setZero();
      out.acc.setZero();
      out.yaw = yaw_.back();
      out.yaw_dot = 0.0;
      out.valid = true;
      // TODO add actual logic when trajectory is finished
      // if (time_since_start >= timestamps_.back() + 1.0){
      //   type_of_active_traj_ = TrajType::NONE;
      //   }
      return out;
    }

    auto it = std::upper_bound(timestamps_.begin(), timestamps_.end(), time_since_start);
    size_t i1 = std::distance(timestamps_.begin(), it);
    size_t i0 = i1 - 1;

    double alpha = (time_since_start - timestamps_[i0]) /
                   (timestamps_[i1] - timestamps_[i0]);

    out.pos = (1.0 - alpha) * position_[i0] + alpha * position_[i1];
    body_v = (1.0 - alpha) * body_velocity_[i0] + alpha * body_velocity_[i1];
    out.acc.setZero();
    out.yaw = wrapToPi(yaw_[i0] + alpha * angleDiff(yaw_[i1], yaw_[i0]));
    out.yaw_dot = (1.0 - alpha) * yaw_dot_[i0] + alpha * yaw_dot_[i1];
    out.vel = transform_body_velocity_into_world_frame(out.yaw, body_v);
    out.valid = true;
    return out;
  }

private:
  std::vector<double> timestamps_, yaw_, yaw_dot_;
  std::vector<Eigen::Vector3d> position_, body_velocity_;
};

SampledOptimizedTrajectory optimized_trajectory_; 


static void load_gains_from_params(const rclcpp::Node::SharedPtr& node)
{

  std::lock_guard<std::mutex> lkg(gains_mtx);

  Kp_g.x() = node->get_parameter("gains.kp.x").as_double();
  Kp_g.y() = node->get_parameter("gains.kp.y").as_double();
  Kp_g.z() = node->get_parameter("gains.kp.z").as_double();

  Kd_g.x() = node->get_parameter("gains.kd.x").as_double();
  Kd_g.y() = node->get_parameter("gains.kd.y").as_double();
  Kd_g.z() = node->get_parameter("gains.kd.z").as_double();

  Ki_g.x() = node->get_parameter("gains.ki.x").as_double();
  Ki_g.y() = node->get_parameter("gains.ki.y").as_double();
  Ki_g.z() = node->get_parameter("gains.ki.z").as_double();

  Kp_yaw_g.x() = node->get_parameter("gains.kp_yaw.x").as_double();
  Kp_yaw_g.y() = node->get_parameter("gains.kp_yaw.y").as_double();
  Kp_yaw_g.z() = node->get_parameter("gains.kp_yaw.z").as_double();

  Kd_yaw_g.x() = node->get_parameter("gains.kd_yaw.x").as_double();
  Kd_yaw_g.y() = node->get_parameter("gains.kd_yaw.y").as_double();
  Kd_yaw_g.z() = node->get_parameter("gains.kd_yaw.z").as_double();

  Ki_yaw_g.x() = node->get_parameter("gains.ki_yaw.x").as_double();
  Ki_yaw_g.y() = node->get_parameter("gains.ki_yaw.y").as_double();
  Ki_yaw_g.z() = node->get_parameter("gains.ki_yaw.z").as_double();

}


// reset the controller
void resetTrajectoryTrackingController()
{
  derivative_ready_ = false;
  second_ready_ = false;

  double yaw0 = 0.0;
  {
    std::lock_guard<std::mutex> lko(odometry_mtx);

    tf2::Quaternion q(
      odom_orient_.x(),
      odom_orient_.y(),
      odom_orient_.z(),
      odom_orient_.w());

    double roll = 0.0, pitch = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw0);
  }

  last_yaw_ = wrapToPi(yaw0);
  last_yaw_unwrapped_ = yaw0;
  last_yaw_dot_ = 0.0;
  yaw_initialized_ = have_odom.load();

  last_p_error_.setZero();
  last_last_p_error_.setZero();
  p_error_deriv_approx_.setZero();
  integrated_error_.setZero();
  last_yaw_error_ = 0.0;
  last_last_yaw_error_ = 0.0;
  integrated_yaw_error_ = 0.0;
  yaw_error_deriv_approx_ = 0.0;
}

void resetTrajectoryServer()
{
  // This function can be called to reset the trajectory server, used in repeated testing to reset the state of the trajectory tracking controller without having to restart the entire node.

  std::lock_guard<std::mutex> lkc(controller_state_mtx);
  resetTrajectoryTrackingController();
  std::lock_guard<std::mutex> lkt(traj_mtx);
  receive_traj_ = false;
  type_of_active_traj_ = TrajType::NONE;
  traj_.clear();
  optimized_trajectory_.resetTrajectory();
  received_goal_ = false;
  active_traj_ = false;
  }

void pointClickedCallback(const std::shared_ptr<const geometry_msgs::msg::PointStamped> &msg)
  {
    received_goal_ = true;
    new_goal_.header.stamp = node_->get_clock()->now();
    new_goal_.header.frame_id = msg->header.frame_id;
    new_goal_.pose.position.x = msg->point.x;
    new_goal_.pose.position.y = msg->point.y;
    new_goal_.pose.position.z = node_->get_parameter("fsm.point_clicked_z_up").as_double(); // msg->pose.pose.position.z; //TODO once 3D point selection works, remove this
    cout << "New Point Clicked, sent Goal Point!" << endl;
    
    {
      std::lock_guard<std::mutex> lkc(controller_state_mtx);
      resetTrajectoryTrackingController();
      new_goal_pub_->publish(new_goal_);
    }
  }

void controllerStateCallback(
  const std::shared_ptr<const std_msgs::msg::String> msg)
{
  
  vel_mode_ = (msg->data == "auv_controller");
  // If we go back to auv_controller mode, resend the last clicked goal if we have one
  // But if we already have an active trajectory, don't resend
  if(vel_mode_ && received_goal_ && !active_traj_){
    active_traj_ = true;
    cout << "Resending last clicked goal point & setting active_traj_ to True!" << endl;
    {
      std::lock_guard<std::mutex> lkc(controller_state_mtx);
      resetTrajectoryTrackingController();
      new_goal_pub_->publish(new_goal_);
    }
  }
  if(!vel_mode_ && active_traj_){
    cout << "No longer in auv_controller mode, setting active_traj_ to False & Stopping the robot!" << endl;
    active_traj_ = false;
      {
      std::lock_guard<std::mutex> lkc(controller_state_mtx);
      resetTrajectoryTrackingController();
      body_vel_pub->publish(geometry_msgs::msg::Twist()); // send zero vel command to stop the robot
    }
  }
}

void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {

    static const std::string velocity_frame = "base_link";      
    static const std::string world_frame  = "odom";
    geometry_msgs::msg::TransformStamped T_wb;
    try {
      // check if transform is available
      if (!tf_buffer->canTransform(world_frame, velocity_frame, tf2::TimePointZero,
                              tf2::durationFromSec(0.002))) {
        RCLCPP_WARN(node_->get_logger(), "TF lookup failed");
        return;
      }
      // use latest transform
      T_wb = tf_buffer->lookupTransform(world_frame, velocity_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(node_->get_logger(), "TF lookup failed: %s", ex.what());
      return;
    }

    std::lock_guard<std::mutex> lko(odometry_mtx);
    Eigen::Vector3d odom_vel_base_link;
    // Child frame (base_link)
    odom_vel_base_link(0) = msg->twist.twist.linear.x;
    odom_vel_base_link(1) = msg->twist.twist.linear.y;
    odom_vel_base_link(2) = msg->twist.twist.linear.z;

    odom_vel_ = rotate_target_source(T_wb, odom_vel_base_link);

    // map frame
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;


    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    Eigen::Vector3d odom_ang_vel_base_link;
    // Child frame (base_link)
    odom_ang_vel_base_link(0) = msg->twist.twist.angular.x;
    odom_ang_vel_base_link(1) = msg->twist.twist.angular.y;
    odom_ang_vel_base_link(2) = msg->twist.twist.angular.z;

    odom_ang_vel_ = rotate_target_source(T_wb, odom_ang_vel_base_link);

    have_odom.store(true);

  }



void bsplineCallback(const traj_utils::msg::Bspline::SharedPtr msg)
{
  // parse pos traj
  
  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // parse yaw traj

  // Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  // for (int i = 0; i < msg->yaw_pts.size(); ++i) {
  //   yaw_pts(i, 0) = msg->yaw_pts[i];
  // }

  // UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);
  
  std::lock_guard<std::mutex> lkc(controller_state_mtx);
  std::lock_guard<std::mutex> lkt(traj_mtx);
  if (type_of_active_traj_ == TrajType::SAMPLED)
  {
    return;
  }

  resetTrajectoryTrackingController();
  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());
  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;
  type_of_active_traj_ = TrajType::BSPLINE;
}

std::pair<double, double> calculate_yaw(
    double t_cur,
    const Eigen::Vector3d& pos,
    const Eigen::Vector3d& vel,
    const Eigen::Vector3d& acc,
    double yaw_meas,
    double dt)
{
  if (use_snake_yaw_)
  {
    double snake_yaw_unwrapped;
    if (!yaw_initialized_) {
      snake_yaw_unwrapped = snake_yaw_;
      yaw_initialized_ = true;
    } else {
      snake_yaw_unwrapped = unwrapNear(snake_yaw_, last_yaw_unwrapped_);
    }

    last_yaw_unwrapped_ = snake_yaw_unwrapped;
    last_yaw_ = wrapToPi(last_yaw_unwrapped_);
    last_yaw_dot_ = 0.0;
    return {last_yaw_, 0.0};
  }

  constexpr double VEL_EPS_LOW  = 0.15;  // below this, use lookahead direction
  constexpr double VEL_EPS_HIGH = 0.30;  // above this, use tangent direction
  constexpr double LOOKAHEAD_DT = 0.5;  // short lookahead for low-speed heading
  constexpr double POS_EPS      = 0.05;  // if lookahead vector too small, hold yaw
  constexpr double YAW_DOT_MAX  = 1.0;   // rad/s
  constexpr double TAU_YAW      = 0.15;  // s

  if (dt <= 1e-6) {
    return {last_yaw_, last_yaw_dot_};
  }

  const double vx = vel.x();
  const double vy = vel.y();
  const double ax = acc.x();
  const double ay = acc.y();
  const double speed = std::hypot(vx, vy);
  const double v2 = vx * vx + vy * vy;

  double yaw_geom_wrapped = yaw_meas;
  double yawdot_ff = 0.0;

  if (speed >= VEL_EPS_HIGH)
  {
    // Use trajectory tangent
    yaw_geom_wrapped = std::atan2(vy, vx);
    yawdot_ff = (vx * ay - vy * ax) / std::max(v2, 1e-6);
  }
  else
  {
    // Use short lookahead direction
    const double t_look = std::min(t_cur + LOOKAHEAD_DT, traj_duration_);
    Eigen::Vector3d p_look = traj_[0].evaluateDeBoorT(t_look);
    Eigen::Vector3d dir = p_look - pos;
    const double dir_norm = dir.head<2>().norm();

    if (dir_norm > POS_EPS) {
      yaw_geom_wrapped = std::atan2(dir.y(), dir.x());
    } else if (speed > 1e-3) {
      yaw_geom_wrapped = std::atan2(vy, vx);
    } else if (yaw_initialized_) {
      yaw_geom_wrapped = wrapToPi(last_yaw_unwrapped_);
    } else {
      yaw_geom_wrapped = yaw_meas;
    }

    // No reliable feedforward yaw acceleration in low-speed lookahead mode
    yawdot_ff = 0.0;
  }

  // Optional blend in the transition region to avoid mode switching discontinuity
  if (speed > VEL_EPS_LOW && speed < VEL_EPS_HIGH)
  {
    const double t_look = std::min(t_cur + LOOKAHEAD_DT, traj_duration_);
    Eigen::Vector3d p_look = traj_[0].evaluateDeBoorT(t_look);
    Eigen::Vector3d dir = p_look - pos;

    double yaw_look_wrapped = yaw_geom_wrapped;
    if (dir.head<2>().norm() > POS_EPS) {
      yaw_look_wrapped = std::atan2(dir.y(), dir.x());
    }

    const double yaw_tangent_wrapped =
        (v2 > 1e-6) ? std::atan2(vy, vx) : yaw_look_wrapped;

    const double beta =
        (speed - VEL_EPS_LOW) / (VEL_EPS_HIGH - VEL_EPS_LOW); // 0..1

    double yaw_ref_base = yaw_initialized_ ? last_yaw_unwrapped_ : yaw_meas;
    double yaw_look_unwrapped = unwrapNear(yaw_look_wrapped, yaw_ref_base);
    double yaw_tangent_unwrapped = unwrapNear(yaw_tangent_wrapped, yaw_ref_base);

    double yaw_blend_unwrapped =
        (1.0 - beta) * yaw_look_unwrapped + beta * yaw_tangent_unwrapped;

    yaw_geom_wrapped = wrapToPi(yaw_blend_unwrapped);

    double yawdot_tangent = (vx * ay - vy * ax) / std::max(v2, 1e-6);
    yawdot_ff = beta * yawdot_tangent;
  }

  yawdot_ff = clampd(yawdot_ff, -YAW_DOT_MAX, YAW_DOT_MAX);

  if (!yaw_initialized_) {
    last_yaw_unwrapped_ = unwrapNear(yaw_geom_wrapped, yaw_meas);
    yaw_initialized_ = true;
  }

  const double yaw_des_unwrapped = unwrapNear(yaw_geom_wrapped, last_yaw_unwrapped_);
  const double yaw_err = yaw_des_unwrapped - last_yaw_unwrapped_;

  double yawdot_fb = yaw_err / TAU_YAW;
  double yawdot_cmd = yawdot_ff + yawdot_fb;
  yawdot_cmd = clampd(yawdot_cmd, -YAW_DOT_MAX, YAW_DOT_MAX);

  last_yaw_unwrapped_ += yawdot_cmd * dt;
  last_yaw_ = wrapToPi(last_yaw_unwrapped_);
  last_yaw_dot_ = yawdot_cmd;

  return {last_yaw_, last_yaw_dot_};
}

void snakeyawCallback(const traj_utils::msg::SnakeYaw::SharedPtr msg)
{
  std::lock_guard<std::mutex> lkt(traj_mtx);
  type_of_active_traj_ = TrajType::BSPLINE;
  use_snake_yaw_ = msg->use_snake_yaw;
  snake_yaw_ = msg->snake_yaw;
}

void optimizedTrajCallback(const traj_utils::msg::OptimizedTrajectory::SharedPtr msg)
{
  cout << "New optimized Trajectory Received" << endl;
  std::vector<Eigen::Vector3d> pos_samples, vel_samples;
  std::vector<double> t_samples, yaw_samples, yawdot_samples;

  t_samples.reserve(msg->points.size());
  pos_samples.reserve(msg->points.size());
  vel_samples.reserve(msg->points.size());
  yaw_samples.reserve(msg->points.size());
  yawdot_samples.reserve(msg->points.size());

  for (const auto& p : msg->points)
  {
    t_samples.emplace_back(p.t);
    pos_samples.emplace_back(p.position.x, p.position.y, p.position.z);
    vel_samples.emplace_back(p.body_velocity.x, p.body_velocity.y, p.body_velocity.z);
    yaw_samples.push_back(p.yaw);
    yawdot_samples.push_back(p.yaw_dot);
  }

  
  std::lock_guard<std::mutex> lkc(controller_state_mtx);
  resetTrajectoryTrackingController();
  std::lock_guard<std::mutex> lkt(traj_mtx);
  
  optimized_trajectory_.setTrajectory(
      t_samples, pos_samples, vel_samples, yaw_samples, yawdot_samples);

  receive_traj_ = true;
  type_of_active_traj_ = TrajType::SAMPLED;
  start_time_ = node_->get_clock()->now();
  static int next_opt_traj_id = 1000;
  traj_id_ = next_opt_traj_id++;
}

void cmdCallback()
{
  if (!have_odom.load())
    return;

  if (!vel_mode_) {
    return;
  }

  const rclcpp::Time time_now = node_->get_clock()->now();

  static rclcpp::Time time_last = time_now;
  double dt = (time_now - time_last).seconds();
  if (dt <= 1e-5) {
    dt = 1e-5;
  }

  // Variables that must remain accessible after the mutex block
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d pos_f = Eigen::Vector3d::Zero();

  Eigen::Vector3d p_des = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_des = Eigen::Vector3d::Zero();
  Eigen::Vector3d euler_des = Eigen::Vector3d::Zero();
  Eigen::Vector3d w_des = Eigen::Vector3d::Zero();

  Eigen::Vector3d p_meas = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_meas = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q_meas = Eigen::Quaterniond::Identity();
  Eigen::Vector3d w_meas = Eigen::Vector3d::Zero();

  Eigen::Vector3d Kp = Eigen::Vector3d::Zero();
  Eigen::Vector3d Kd = Eigen::Vector3d::Zero();
  Eigen::Vector3d Ki = Eigen::Vector3d::Zero();
  Eigen::Vector3d Kp_yaw = Eigen::Vector3d::Zero();
  Eigen::Vector3d Kd_yaw = Eigen::Vector3d::Zero();
  Eigen::Vector3d Ki_yaw = Eigen::Vector3d::Zero();

  Eigen::Vector3d p_error = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_cmd_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d w_cmd_world = Eigen::Vector3d::Zero();

  std::pair<double, double> yaw_yawdot{0.0, 0.0};

  double t_cur = 0.0;
  double yaw_des = 0.0;
  double yaw_meas = 0.0;
  double yaw_err = 0.0;
  int local_traj_id = -1;

  {
    std::lock_guard<std::mutex> lkc(controller_state_mtx);

    {
      std::lock_guard<std::mutex> lkt(traj_mtx);

      if (!receive_traj_)
        return;

      local_traj_id = traj_id_;
      t_cur = (time_now - start_time_).seconds();

      if (type_of_active_traj_ == TrajType::BSPLINE) {
        if (t_cur < traj_duration_ && t_cur >= 0.0) {
          pos = traj_[0].evaluateDeBoorT(t_cur);
          vel = traj_[1].evaluateDeBoorT(t_cur);
          acc = traj_[2].evaluateDeBoorT(t_cur);

          double yaw_meas_for_ref = 0.0;
        {
          std::lock_guard<std::mutex> lko(odometry_mtx);
          tf2::Quaternion q(
            odom_orient_.x(),
            odom_orient_.y(),
            odom_orient_.z(),
            odom_orient_.w());
          double roll_tmp = 0.0, pitch_tmp = 0.0;
          tf2::Matrix3x3(q).getRPY(roll_tmp, pitch_tmp, yaw_meas_for_ref);
        }

        yaw_yawdot = calculate_yaw(t_cur, pos, vel, acc, yaw_meas_for_ref, dt);

          const double tf = min(traj_duration_, t_cur + 2.0);
          pos_f = traj_[0].evaluateDeBoorT(tf);
        } else if (t_cur >= traj_duration_) {
          pos = traj_[0].evaluateDeBoorT(traj_duration_);
          vel.setZero();
          acc.setZero();

          yaw_yawdot.first = last_yaw_;
          yaw_yawdot.second = 0.0;

          pos_f = pos;
        } else {
          cout << "[Traj server]: invalid time." << endl;
          return;
        }
      } else if (type_of_active_traj_ == TrajType::SAMPLED) {
        TrajectorySample trajectory_element = optimized_trajectory_.sample(t_cur);
        if (!trajectory_element.valid) {
          return;
        }

        pos = trajectory_element.pos;
        vel = trajectory_element.vel;
        acc = trajectory_element.acc;
        yaw_yawdot.first = trajectory_element.yaw;
        yaw_yawdot.second = trajectory_element.yaw_dot;
      } else {
        return;
      }
    } // traj_mtx

    time_last = time_now;

    // Publish debug position command
    cmd.header.stamp = time_now;
    cmd.header.frame_id = "odom";
    cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
    cmd.trajectory_id = local_traj_id;

    cmd.position.x = pos.x();
    cmd.position.y = pos.y();
    cmd.position.z = pos.z();

    cmd.velocity.x = vel.x();
    cmd.velocity.y = vel.y();
    cmd.velocity.z = vel.z();

    cmd.acceleration.x = acc.x();
    cmd.acceleration.y = acc.y();
    cmd.acceleration.z = acc.z();

    cmd.yaw = yaw_yawdot.first;
    cmd.yaw_dot = yaw_yawdot.second;

    pos_cmd_pub->publish(cmd);

    p_des = pos;
    v_des = vel;
    euler_des.setZero();
    w_des.setZero();
    euler_des(2) = cmd.yaw;
    w_des(2) = cmd.yaw_dot;

    {
      std::lock_guard<std::mutex> lko(odometry_mtx);
      p_meas = odom_pos_;
      v_meas = odom_vel_;
      q_meas = odom_orient_;
      w_meas = odom_ang_vel_;
    } // odometry_mtx

    tf2::Quaternion q(
      q_meas.x(),
      q_meas.y(),
      q_meas.z(),
      q_meas.w());

    double roll_meas = 0.0;
    double pitch_meas = 0.0;
    tf2::Matrix3x3(q).getRPY(roll_meas, pitch_meas, yaw_meas);

    {
      std::lock_guard<std::mutex> lkg(gains_mtx);
      Kp = Kp_g;
      Kd = Kd_g;
      Ki = Ki_g;
      Kp_yaw = Kp_yaw_g;
      Kd_yaw = Kd_yaw_g;
      Ki_yaw = Ki_yaw_g;
    }  // gains_mtx

    p_error = p_des - p_meas;
    yaw_des = euler_des(2);
    yaw_err = angleDiff(yaw_des, yaw_meas);

    if (second_ready_) {
      p_error_deriv_approx_ = (p_error - last_p_error_) / dt;
      yaw_error_deriv_approx_ = angleDiff(yaw_err, last_yaw_error_) / dt;
    }

    integrated_error_ += p_error * dt;
    integrated_yaw_error_ += yaw_err * dt;

    integrated_error_ = integrated_error_.cwiseMax(integrator_min).cwiseMin(integrator_max);
    integrated_yaw_error_ = std::clamp(integrated_yaw_error_, integrator_min(2), integrator_max(2));

    v_cmd_world = v_des
                + Kp.cwiseProduct(p_error)
                - Kd.cwiseProduct(v_meas)
                + Ki.cwiseProduct(integrated_error_);

    w_cmd_world = w_des;
    w_cmd_world(2) = w_des(2)
                   + Kp_yaw(2) * yaw_err
                   - Kd_yaw(2) * w_meas(2)
                   + Ki_yaw(2) * integrated_yaw_error_;

    last_last_p_error_ = last_p_error_;
    last_p_error_ = p_error;

    last_last_yaw_error_ = last_yaw_error_;
    last_yaw_error_ = yaw_err;

    if (!derivative_ready_) {
      derivative_ready_ = true;
    } else if (!second_ready_) {
      second_ready_ = true;
    }
  } // controller_state_mtx

  const std::string world_frame = "odom";
  const std::string body_frame = "base_link_fsd";

  geometry_msgs::msg::TransformStamped T_bw;
  try {
    if (!tf_buffer->canTransform(body_frame, world_frame, tf2::TimePointZero,
                                 tf2::durationFromSec(0.002))) {
      RCLCPP_WARN(node_->get_logger(), "TF lookup failed");
      return;
    }

    T_bw = tf_buffer->lookupTransform(body_frame, world_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(node_->get_logger(), "TF lookup failed: %s", ex.what());
    return;
  }

  Eigen::Vector3d v_cmd_base = rotate_target_source(T_bw, v_cmd_world);
  v_cmd_base = v_cmd_base.cwiseMax(v_min_).cwiseMin(v_max_);

  Eigen::Vector3d w_cmd_base = rotate_target_source(T_bw, w_cmd_world);

  const double yaw_dot_sat = std::clamp(w_cmd_base.z(), -1.0, 1.0);

  geometry_msgs::msg::Twist body_cmd;
  body_cmd.linear.x = v_cmd_base.x();
  body_cmd.linear.y = v_cmd_base.y();
  body_cmd.linear.z = v_cmd_base.z();
  body_cmd.angular.x = 0.0;
  body_cmd.angular.y = 0.0;
  body_cmd.angular.z = yaw_dot_sat;

  if (std::abs(body_cmd.linear.x) > 10.0) {
    cout << "Extremely high linear velocity! " << body_cmd.linear.x << endl;
  }

  body_vel_pub->publish(body_cmd);
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("traj_server");
  node_ = node;

  node->declare_parameter("gains.kp.x", 0.6);
  node->declare_parameter("gains.kp.y", 0.6);
  node->declare_parameter("gains.kp.z", 0.6);

  node->declare_parameter("gains.kd.x", -0.2);
  node->declare_parameter("gains.kd.y", -0.2);
  node->declare_parameter("gains.kd.z", -0.2);

  node->declare_parameter("gains.ki.x", 0.1);
  node->declare_parameter("gains.ki.y", 0.1);
  node->declare_parameter("gains.ki.z", 0.1);

  node->declare_parameter("gains.kp_yaw.x", 0.8);
  node->declare_parameter("gains.kp_yaw.y", 0.8);
  node->declare_parameter("gains.kp_yaw.z", 0.8);

  node->declare_parameter("gains.kd_yaw.x", 0.1);
  node->declare_parameter("gains.kd_yaw.y", 0.1);
  node->declare_parameter("gains.kd_yaw.z", -0.2);

  node->declare_parameter("gains.ki_yaw.x", 0.1);
  node->declare_parameter("gains.ki_yaw.y", 0.1);
  node->declare_parameter("gains.ki_yaw.z", 0.1);

  node->declare_parameter("fsm.point_clicked_z_up", -1.0);

  load_gains_from_params(node);

  g_param_handler = std::make_shared<rclcpp::ParameterEventHandler>(node);
  const std::string my_fqn = node->get_fully_qualified_name();

  g_param_event_handle = g_param_handler->add_parameter_event_callback(
    [node, my_fqn](const rcl_interfaces::msg::ParameterEvent & event)
    {
      // Only react to this node's events (optional but recommended)
      if (event.node != my_fqn) return;

      auto is_gain = [](const std::string & name) {
        return name.rfind("gains.", 0) == 0;  // starts with "gains."
      };

      for (const auto & p : event.changed_parameters) {
        if (is_gain(p.name)) { load_gains_from_params(node); return; }
      }
      for (const auto & p : event.new_parameters) {
        if (is_gain(p.name)) { load_gains_from_params(node); return; }
      }
    });


  tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  body_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel_body", 50);


  odometry_sub = node_->create_subscription<nav_msgs::msg::Odometry>(
        "odometry/filtered_enu",
        10,
        odometryCallback
        );
        
  controller_state_sub = node->create_subscription<std_msgs::msg::String>(
    "controller_state",
    10,
    controllerStateCallback);
  new_goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/ego_planner/move_base_simple/goal",
        10);
  rviz_clicked_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
    "/ego_planner/clicked_point",
    10,
    pointClickedCallback);


  bspline_sub = node->create_subscription<traj_utils::msg::Bspline>(
      "planning/bspline",
      10,
      bsplineCallback);

  snake_yaw_sub = node->create_subscription<traj_utils::msg::SnakeYaw>(
      "planning/snake_yaw",
      10,
      snakeyawCallback);

  optimized_trajectory_sub = node->create_subscription<traj_utils::msg::OptimizedTrajectory>(
      "planning/optimized_trajectory",
      10,
      optimizedTrajCallback);

  pos_cmd_pub = node->create_publisher<quadrotor_msgs::msg::PositionCommand>(
      "/position_cmd",
      50);
    
  // Service for resetting the trajectory server 
  reset_traj_server_service_ = node->create_service<std_srvs::srv::Trigger>(
    "/ego_traj_server/reset_traj_server",
    [](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
       std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      (void)request; // unused
      resetTrajectoryServer();
      response->success = true;
      response->message = "Trajectory server reset successfully.";
      return true;
    });

  auto cmd_timer = node->create_timer(
      std::chrono::milliseconds(10),
      cmdCallback);
  
  /* control parameter */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  node->declare_parameter("traj_server/time_forward", 1.0);
  node->get_parameter("traj_server/time_forward", time_forward_);

  resetTrajectoryTrackingController();

  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_WARN(node->get_logger(), "[Traj server]: ready.");

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();

  return 0;
}