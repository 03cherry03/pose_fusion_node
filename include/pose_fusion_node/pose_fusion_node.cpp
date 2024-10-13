#ifndef POSE_FUSION_NODE_HPP
#define POSE_FUSION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

class PoseFusionNode : public rclcpp::Node
{
public:
  PoseFusionNode(const std::string & node_name, const rclcpp::NodeOptions & node_options);

private:
  void callbackPoseLidar(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void callbackPoseGnss(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void callbackTwistLidar(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
  void callbackTwistGnss(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
  void timerCallback();

  void fusePose();
  void fuseTwist();

  // ROS2 Subscription 객체
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_with_cov_lidar_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_with_cov_gnss_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_twist_with_cov_lidar_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_twist_with_cov_gnss_;

  // ROS2 Publisher 객체
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_fused_pose_cov_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_fused_twist_cov_;

  // 융합할 데이터 저장 변수
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_lidar_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_gnss_;
  geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr twist_lidar_;
  geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr twist_gnss_;

  // 가중치
  double lidar_weight_;
  double gnss_weight_;

  // 융합 관련 변수들
  bool is_activated_;
  double ekf_dt_;
  int dim_x_;
  
  void initFusion();
  void updatePredictFrequency();
  void publishEstimateResult();
  void publishDiagnostics();
};

#endif  // POSE_FUSION_NODE_HPP
