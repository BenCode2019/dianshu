#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <cmath>
#include "dianshu_controller/heading_latch.hpp"
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace dianshu_controller {
class SmoothGoalController : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController {
  using Base = nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController;
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    Base::configure(parent, name, tf, costmap);
    auto node = parent.lock();
    nav2_util::declare_parameter_if_not_declared(node, name + ".terminal_max_angular_vel", rclcpp::ParameterValue(0.25));
    nav2_util::declare_parameter_if_not_declared(node, name + ".terminal_angular_accel", rclcpp::ParameterValue(0.4));
    node->get_parameter(name + ".terminal_max_angular_vel", terminal_rate_);
    node->get_parameter(name + ".terminal_angular_accel", terminal_accel_);
    if (!std::isfinite(terminal_rate_) || !std::isfinite(terminal_accel_) || terminal_rate_ <= 0 || terminal_accel_ <= 0) {
      throw nav2_core::PlannerException("Terminal rate and acceleration must be finite and positive");
    }
    resetTerminal();
  }
  void activate() override {Base::activate(); resetTerminal();}
  void deactivate() override {Base::deactivate(); resetTerminal();}
  void setPlan(const nav_msgs::msg::Path &path) override {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (path.poses.empty()) {throw nav2_core::PlannerException("Empty path");}
    auto next = path.poses.back();
    next.header.frame_id = path.header.frame_id;
    const double idle = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_compute_).count();
    const bool changed = !has_goal_ || next.header.frame_id != goal_.header.frame_id ||
      std::hypot(next.pose.position.x - goal_.pose.position.x,
        next.pose.position.y - goal_.pose.position.y) > 1e-4 ||
      std::abs(wrap(tf2::getYaw(next.pose.orientation) - tf2::getYaw(goal_.pose.orientation))) > 1e-3;
    if (changed || idle > 2.0) {resetTerminal();}
    goal_ = next;
    has_goal_ = true;
    Base::setPlan(path);
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose, const geometry_msgs::msg::Twist &velocity,
    nav2_core::GoalChecker *checker) override {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - last_compute_).count();
    const double dt = std::clamp(elapsed, 0.01, 0.2);
    last_compute_ = now;
    if (!has_goal_) {throw nav2_core::PlannerException("Goal is unavailable");}
    geometry_msgs::msg::Pose pose_tolerance;
    geometry_msgs::msg::Twist velocity_tolerance;
    if (!checker || !checker->getTolerances(pose_tolerance, velocity_tolerance)) {
      throw nav2_core::PlannerException("Goal tolerance is unavailable");
    }
    geometry_msgs::msg::PoseStamped target = goal_, local_goal;
    target.header.stamp = pose.header.stamp;
    if (!transformPose(pose.header.frame_id, target, local_goal)) {
      throw nav2_core::PlannerException("Cannot transform final goal");
    }
    const double distance = std::hypot(local_goal.pose.position.x - pose.pose.position.x,
      local_goal.pose.position.y - pose.pose.position.y);
    if (!terminal_ && distance <= pose_tolerance.position.x) {
      terminal_ = true;
      RCLCPP_INFO(logger_, "Terminal alignment latched at %.3f m; stopping translation", distance);
    }
    if (!terminal_) {
      auto command = Base::computeVelocityCommands(pose, velocity, checker);
      command.twist.angular.z = std::clamp(command.twist.angular.z, -0.35, 0.35);
      return command;
    }
    // Do not drive after latching position; an unsafe localization jump is an error.
    if (distance > std::max(0.18, 3.0 * pose_tolerance.position.x)) {
      throw nav2_core::PlannerException("Final position moved outside alignment safety window");
    }
    geometry_msgs::msg::TwistStamped command;
    command.header = pose.header;
    if (!settled_) {
      if (std::abs(velocity.linear.x) < 0.008 && std::abs(velocity.angular.z) < 0.03) {
        if (++stopped_samples_ >= 2) {settled_ = true;}
      } else {stopped_samples_ = 0;}
      return command;
    }
    const double error = wrap(tf2::getYaw(local_goal.pose.orientation) - tf2::getYaw(pose.pose.orientation));
    const bool selecting = !heading_.locked();
    command.twist.angular.z = heading_.step(error, dt, terminal_rate_, terminal_accel_, 0.04);
    if (selecting) {
      RCLCPP_INFO(logger_, "Final turn direction locked, yaw error %.3f rad", error);
    }
    {
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*(costmap_->getMutex()));
      if (use_collision_detection_ && isCollisionImminent(pose, 0.0, command.twist.angular.z, 0.0)) {
        throw nav2_core::PlannerException("Collision detected during final alignment");
      }
    }
    return command;
  }
private:
  void resetTerminal() {terminal_ = settled_ = false; stopped_samples_ = 0; heading_.reset();}
  std::mutex goal_mutex_;
  geometry_msgs::msg::PoseStamped goal_;
  bool has_goal_{false}, terminal_{false}, settled_{false};
  unsigned stopped_samples_{0};
  double terminal_rate_{0.25}, terminal_accel_{0.4};
  HeadingLatch heading_;
  std::chrono::steady_clock::time_point last_compute_{std::chrono::steady_clock::now()};
};
}
PLUGINLIB_EXPORT_CLASS(dianshu_controller::SmoothGoalController, nav2_core::Controller)
