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

using std::vector;
using std::min;
using std::cout;
using std::endl;

// Gain storage (shared between timer + param callback)
std::mutex gains_mtx;
Eigen::Vector3d Kp_g(0.4, 0.4, 0.4);
Eigen::Vector3d Kd_g(0.0, 0.0, 0.0);
Eigen::Vector3d Ki_g(0.1, 0.1, 0.1);

Eigen::Vector3d Kp_yaw_g(0.4, 0.4, 0.4);
Eigen::Vector3d Kd_yaw_g(0.0, 0.0, 0.0);
Eigen::Vector3d Ki_yaw_g(0.1, 0.1, 0.1);

Eigen::Vector3d v_max(0.9, 0.9, 0.9);// TODO tune, or make tunable
Eigen::Vector3d v_min = -v_max;

Eigen::Vector3d integrator_max(1.0, 1.0, 1.0);
Eigen::Vector3d integrator_min = -integrator_max;


rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr gains_cb_handle;

rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr body_vel_pub;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr new_goal_pub_;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub;
rclcpp::Subscription<traj_utils::msg::Bspline>::SharedPtr bspline_sub;
rclcpp::Subscription<traj_utils::msg::SnakeYaw>::SharedPtr snake_yaw_sub;
rclcpp::Subscription<std_msgs::msg::String>::SharedPtr controller_state_sub;
rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr rviz_clicked_sub_;


bool derivative_ready = false;
bool second_ready = false;
Eigen::Vector3d last_p_error;
Eigen::Vector3d last_last_p_error;
Eigen::Vector3d p_error_deriv_approx;
Eigen::Vector3d integrated_error;
double last_yaw_error;
double last_last_yaw_error;
double integrated_yaw_error;
double yaw_error_deriv_approx;
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
double last_yaw_, last_yaw_dot_;
double time_forward_;
bool use_snake_yaw = false;
double snake_yaw = 0.0;
rclcpp::Node::SharedPtr node_;

Eigen::Vector3d odom_pos_, odom_vel_, odom_ang_vel_;
Eigen::Quaterniond odom_orient_;



static inline double wrapToPi(double a)
{
  // returns in [-pi, pi]
  a = std::fmod(a + PI, 2.0 * PI);
  //cout << "WRAP TO PI: a " << a << endl;
  if (a < 0) a += 2.0 * PI;
  //cout << "WRAP TO PI AFTER MOD: a " << a << endl;
  //cout << "WRAP TO PI RESULT: " << a - PI << endl;
  return a - PI;
}

static inline double angleDiff(double target, double current)
{
  // shortest signed difference target-current in [-pi,pi]
  //cout << "ANGLE DIFF: target " << target << " current " << current << endl;
  return wrapToPi(target - current);
}


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

static void load_gains_from_params(const rclcpp::Node::SharedPtr& node)
{
  std::lock_guard<std::mutex> lk(gains_mtx);

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

void pointClickedCallback(const std::shared_ptr<const geometry_msgs::msg::PointStamped> &msg)
  {
    received_goal_ = true;
    new_goal_.header.stamp = node_->get_clock()->now();
    new_goal_.header.frame_id = msg->header.frame_id;
    new_goal_.pose.position.x = msg->point.x;
    new_goal_.pose.position.y = msg->point.y;
    new_goal_.pose.position.z = node_->get_parameter("fsm.point_clicked_z_up").as_double(); // msg->pose.pose.position.z; //TODO once 3D point selection works, remove this
    cout << "New Point Clicked, sent Goal Point!" << endl;
    new_goal_pub_->publish(new_goal_);
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
    new_goal_pub_->publish(new_goal_);
  }
  if(!vel_mode_ && active_traj_){
    cout << "No longer in auv_controller mode, setting active_traj_ to False & Stopping the robot!" << endl;
    active_traj_ = false;
    body_vel_pub->publish(geometry_msgs::msg::Twist()); // send zero vel command to stop the robot
  }
}

void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {

    static const std::string velocity_frame = "base_link";      
    static const std::string world_frame  = "odom";
    geometry_msgs::msg::TransformStamped T_wb;
    try {
      // use latest transform
      T_wb = tf_buffer->lookupTransform(world_frame, velocity_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(node_->get_logger(), "TF lookup failed: %s", ex.what());
      return;
    }

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
  // reset the controller
  derivative_ready = false;
  second_ready = false;
  last_p_error.setZero();
  last_last_p_error.setZero();
  p_error_deriv_approx.setZero();
  integrated_error.setZero();
  last_yaw_error = 0.0;
  last_last_yaw_error = 0.0;
  integrated_yaw_error = 0.0;
  yaw_error_deriv_approx = 0.0;

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

  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());
  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, double dt)
{
  // If the robot is in inspection mode, use the fixed orientation provided by snake_yaw.
  if (use_snake_yaw)
  {
    return std::make_pair(snake_yaw, 0.0);
  }

  
  constexpr double YAW_DOT_MAX_PER_SEC = PI;
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * dt;
  if (yaw_temp - last_yaw_ > PI)
  {
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / dt;
    }
  }
  else if (yaw_temp - last_yaw_ < -PI)
  {
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / dt;
    }
  }
  else
  {
    if (yaw_temp - last_yaw_ < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else if (yaw_temp - last_yaw_ > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / dt;
    }
  }

  if (fabs(yaw - last_yaw_) <= max_yaw_change)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
  yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}

void snakeyawCallback(const traj_utils::msg::SnakeYaw::SharedPtr msg)
{
  use_snake_yaw = msg->use_snake_yaw;
  snake_yaw = msg->snake_yaw;
}

void cmdCallback()
{
  /* no publishing before receive traj_ */
  if (!receive_traj_)
    return;
  if (!have_odom.load())
    return;
  if (!vel_mode_){
    cout << "Cannot publish velocity commands, not using auv_control mode!" << endl;
    return;
  }
  // unified time source
  //rclcpp::Clock clock(RCL_ROS_TIME);  
  rclcpp::Time time_now = node_->get_clock()->now();
  double t_cur = (time_now - start_time_).seconds();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  static rclcpp::Time time_last = node_->get_clock()->now();

  double dt = (time_now - time_last).seconds();
  // Guard dt
  if (dt <= 1e-7) {
    dt = 1e-7;
  }

  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos, dt);
    /*** calculate yaw ***/

    double tf = min(traj_duration_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);
  }
  else if (t_cur >= traj_duration_)
  {
    /* hover when finish traj_ */
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();

    yaw_yawdot.first = last_yaw_;
    yaw_yawdot.second = 0;

    pos_f = pos;
  }
  else
  {
    cout << "[Traj server]: invalid time." << endl;
  }
  time_last = time_now;

  cmd.header.stamp = time_now;
  cmd.header.frame_id = "odom";
  cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = yaw_yawdot.first;
  cmd.yaw_dot = yaw_yawdot.second;
  // Only for debugging: Send position command to see where we want to actually go
  pos_cmd_pub->publish(cmd);

  last_yaw_ = cmd.yaw;

  Eigen::Vector3d p_des = pos;
  Eigen::Vector3d v_des = vel;
  Eigen::Vector3d euler_des;
  Eigen::Vector3d w_des;
  euler_des.setZero();
  w_des.setZero();
  euler_des(2) = cmd.yaw;
  w_des(2) = cmd.yaw_dot;

  // position in odom frame
  Eigen::Vector3d p_meas = odom_pos_;
  // velocity in odom frame
  Eigen::Vector3d v_meas = odom_vel_;
  // orientation in odom frame
  Eigen::Quaterniond q_meas = odom_orient_;
  // angular velocity in odom frame
  Eigen::Vector3d w_meas = odom_ang_vel_;

  tf2::Quaternion q(
    q_meas.x(),
    q_meas.y(),
    q_meas.z(),
    q_meas.w()
  );

  
  double roll_meas, pitch_meas, yaw_meas;
  tf2::Matrix3x3(q).getRPY(roll_meas, pitch_meas, yaw_meas);
  Eigen::Vector3d euler_meas(roll_meas, pitch_meas, yaw_meas);

  Eigen::Vector3d Kp, Kd, Kp_yaw, Kd_yaw, Ki, Ki_yaw;
  {
    std::lock_guard<std::mutex> lk(gains_mtx);
    Kp = Kp_g;
    Kd = Kd_g;
    Ki = Ki_g;
    Kp_yaw = Kp_yaw_g;
    Kd_yaw = Kd_yaw_g;
    Ki_yaw = Ki_yaw_g;
  }

  Eigen::Vector3d p_error = p_des - p_meas;

  double yaw_des  = euler_des(2);
  yaw_meas = euler_meas(2);

  double yaw_err = angleDiff(yaw_des, yaw_meas);

  // Compute derivative approximation and integral of error
  // Second order approx of first derivative 
  if (second_ready)
  {
      p_error_deriv_approx =
          ( 3.0 * p_error
          - 4.0 * last_p_error
          + 1.0 * last_last_p_error ) / (2.0 * 0.01);

      yaw_error_deriv_approx =
          ( 3.0 * yaw_err
          - 4.0 * last_yaw_error
          + 1.0 * last_last_yaw_error ) / (2.0 * 0.01);
  }

  integrated_error += p_error*0.01;
  integrated_yaw_error += yaw_err*0.01;

  integrated_error = integrated_error.cwiseMax(integrator_min).cwiseMin(integrator_max);
  integrated_yaw_error = std::clamp(integrated_yaw_error, integrator_min(2), integrator_max(2));

  Eigen::Vector3d v_cmd_world = v_des
    + Kp.cwiseProduct(p_error)
    - Kd.cwiseProduct(p_error_deriv_approx)
    + Ki.cwiseProduct(integrated_error);

  last_last_p_error = last_p_error;
  last_p_error = p_error;

  // If you want yaw-rate feedback, use measured yaw rate (in the same frame as w_des!)
  double yaw_rate_des  = w_des(2);
  double yaw_rate_meas = w_meas(2); 

  Eigen::Vector3d w_cmd_world = w_des;

  // only yaw control here (roll, pitch should be self stabilizing)
  w_cmd_world(2) = yaw_rate_des
                + Kp_yaw(2) * yaw_err
                - Kd_yaw(2) * yaw_error_deriv_approx
                + Ki_yaw(2) * integrated_yaw_error;
  
  last_last_yaw_error = last_yaw_error;
  last_yaw_error = yaw_err;
  

  if (!derivative_ready)
      derivative_ready = true;
  else{
    if (!second_ready)
      second_ready = true;
  }

  const std::string world_frame = "odom";      
  // The desired final frame of the velocity (to be fed to the low level controller)
  const std::string body_frame  = "base_link_fsd";
  geometry_msgs::msg::TransformStamped T_bw;
  try {
      // use latest transform
      if (!tf_buffer->canTransform(body_frame, world_frame, tf2::TimePointZero,
                              tf2::durationFromSec(0.002))) {
    return;
    }
    T_bw = tf_buffer->lookupTransform(body_frame, world_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(node_->get_logger(), "TF lookup failed: %s", ex.what());
    return;
  }

  
  Eigen::Vector3d v_cmd_base = rotate_target_source(T_bw, v_cmd_world);

  // Limit base frame velocity to v_min and v_max!
  // TODO check if this could / should not be handled elsewhere
  v_cmd_base = v_cmd_base.cwiseMax(v_min).cwiseMin(v_max);
  Eigen::Vector3d w_cmd_base = rotate_target_source(T_bw, w_cmd_world);

  // For debugging
  //cout << "New ITER! P DES" << p_des << "V DES" << v_des << " | P MEAS" << p_meas << " | P ERR" << p_error << " | P DER" << p_error_deriv_approx << " | V WRL" << v_cmd_world << " | V BAS" << v_cmd_base << endl;
  //cout << "New ITER! P DES" << yaw_des << "V DES" << yaw_rate_des << " | P MEAS" << yaw_meas << " | P ERR" << yaw_err << " | P DER" << yaw_error_deriv_approx <<  " | P INT" << integrated_yaw_error << " | V WRL" << w_cmd_world << " | V BAS" << w_cmd_base << endl;
  double yaw_dot_sat = std::clamp(w_cmd_base.z(), -1.0, 1.0);
  geometry_msgs::msg::Twist body_cmd;
  body_cmd.linear.x = v_cmd_base.x();
  body_cmd.linear.y = v_cmd_base.y();
  body_cmd.linear.z = v_cmd_base.z();
  if(body_cmd.linear.x < - 10 || body_cmd.linear.x > 10){
    cout << "STH went wrong!" << v_des << "  " << p_error << "  " << last_p_error << "  " << p_error << endl;
  }
  body_cmd.angular.x = 0.0;//w_cmd_base.x();
  body_cmd.angular.y = 0.0;//w_cmd_base.y();
  body_cmd.angular.z = w_cmd_base.z();

  body_vel_pub->publish(body_cmd);

}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("traj_server");
  node_ = node;

    // Declare gain parameters (defaults match your current hardcoded values)
  node->declare_parameter("gains.kp.x", 0.6);
  node->declare_parameter("gains.kp.y", 0.6);
  node->declare_parameter("gains.kp.z", 0.6);

  node->declare_parameter("gains.kd.x", 0.0);
  node->declare_parameter("gains.kd.y", 0.0);
  node->declare_parameter("gains.kd.z", 0.0);

  node->declare_parameter("gains.ki.x", 0.1);
  node->declare_parameter("gains.ki.y", 0.1);
  node->declare_parameter("gains.ki.z", 0.1);

  node->declare_parameter("gains.kp_yaw.x", 0.8);
  node->declare_parameter("gains.kp_yaw.y", 0.8);
  node->declare_parameter("gains.kp_yaw.z", 0.8);

  node->declare_parameter("gains.kd_yaw.x", 0.0);
  node->declare_parameter("gains.kd_yaw.y", 0.0);
  node->declare_parameter("gains.kd_yaw.z", 0.0);

  node->declare_parameter("gains.ki_yaw.x", 0.1);
  node->declare_parameter("gains.ki_yaw.y", 0.1);
  node->declare_parameter("gains.ki_yaw.z", 0.1);

  node->declare_parameter("fsm.point_clicked_z_up", -1.0);

  // Load initial values
  load_gains_from_params(node);

  // Live update callback
  gains_cb_handle = node->add_on_set_parameters_callback(
    [node](const std::vector<rclcpp::Parameter>& params)
      -> rcl_interfaces::msg::SetParametersResult
    {
      rcl_interfaces::msg::SetParametersResult res;
      res.successful = true;
      res.reason = "ok";

      // Apply update by re-reading
      load_gains_from_params(node);

      return res;
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

  pos_cmd_pub = node->create_publisher<quadrotor_msgs::msg::PositionCommand>(
      "/position_cmd",
      50);

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

  node->declare_parameter("traj_server/time_forward", -1.0);
  node->get_parameter("traj_server/time_forward", time_forward_);

  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;
  last_p_error.setZero();
  last_last_p_error.setZero();
  p_error_deriv_approx.setZero();
  integrated_error.setZero();
  last_yaw_error = 0.0;
  last_last_yaw_error = 0.0;
  integrated_yaw_error = 0.0;
  yaw_error_deriv_approx = 0.0;

  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_WARN(node->get_logger(), "[Traj server]: ready.");

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();

  return 0;
}