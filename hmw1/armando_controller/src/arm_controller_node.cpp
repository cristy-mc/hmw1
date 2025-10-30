#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <vector>
#include <cmath>

using namespace std::chrono_literals;

class ArmControllerNode : public rclcpp::Node
{
public:
  ArmControllerNode() : Node("arm_controller_node")
  {
    controller_type_ = this->declare_parameter<std::string>("controller_type", "position");

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&ArmControllerNode::jointStateCallback, this, std::placeholders::_1));

    if (controller_type_ == "trajectory")
    {
      trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/joint_trajectory_controller/joint_trajectory", 10);
      use_trajectory_controller_ = true;
      RCLCPP_INFO(this->get_logger(), "Using the Joint Trajectory Controller.");
    }
    else
    {
      position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/position_controller/commands", 10);
      use_trajectory_controller_ = false;
      RCLCPP_INFO(this->get_logger(), "Using the Position Controller.");
    }

    timer_ = this->create_wall_timer(500ms, std::bind(&ArmControllerNode::publishCommand, this));

    RCLCPP_INFO(this->get_logger(), "Arm controller node started!");
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    current_positions_ = msg->position;
  }

  bool isAtTarget(const std::vector<double>& target, double tol = 0.01)
  {
    if (current_positions_.size() != target.size())
      return false;

    for (size_t i = 0; i < target.size(); ++i)
      if (std::fabs(current_positions_[i] - target[i]) > tol)
        return false;

    return true;
  }

  void publishCommand()
  {
    if (step_ < commands_.size())
    {
      if (step_ == 0 || isAtTarget(commands_[step_ - 1]))
      {
        if (use_trajectory_controller_)
        {
          trajectory_msgs::msg::JointTrajectory traj;
          traj.joint_names = {"j0", "j1", "j2", "j3"};

          trajectory_msgs::msg::JointTrajectoryPoint point;
          point.positions = commands_[step_];
          point.time_from_start = rclcpp::Duration::from_seconds(2.0);

          traj.points.push_back(point);
          trajectory_pub_->publish(traj);

          RCLCPP_INFO(this->get_logger(), "Trajectory #%zu sent.", step_ + 1);
        }
        else
        {
          std_msgs::msg::Float64MultiArray msg;
          msg.data = commands_[step_];
          position_pub_->publish(msg);

          RCLCPP_INFO(this->get_logger(), "Position command #%zu sent.", step_ + 1);
        }

        step_++;
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Waiting to reach position #%zu...", step_);
        for (size_t i = 0; i < current_positions_.size(); ++i)
        {
          RCLCPP_INFO(this->get_logger(), "  joint[%zu] = %.3f", i, current_positions_[i]);
        }
      }
    }
    else
    {
      if (isAtTarget(commands_.back()))
      {
        RCLCPP_INFO(this->get_logger(), "All positions reached!");
        timer_->cancel();
      }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<double> current_positions_;
  std::vector<std::vector<double>> commands_ = {
      {0.9,  1.3, -1.1, -1.0},
      {-0.9, 1.0,  0.2,  1.2},
      {-2.0, 0.0,  1.0,  1.0},
      {0.5,  0.4,  1.5, -0.6},
      {0.0,  0.0,  0.0,  0.0}
  };

  std::string controller_type_;
  bool use_trajectory_controller_;
  size_t step_ = 0;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmControllerNode>());
  rclcpp::shutdown();
  return 0;
}
