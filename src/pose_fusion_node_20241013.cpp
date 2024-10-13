// 2024.10.14 1:44
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <deque>

class PoseTwistFusionNode : public rclcpp::Node {
public:
  PoseTwistFusionNode() : Node("pose_twist_fusion_node") {

    // GNSS 및 LiDAR로부터 kinematic_state를 구독
    sub_kinematic_state_lidar_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/localization/pose_twist_fusion_filter/kinematic_state_lidar", 10,
      std::bind(&PoseTwistFusionNode::callbackKinematicStateLidar, this, std::placeholders::_1));
    sub_kinematic_state_gnss_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/localization/pose_twist_fusion_filter/kinematic_state_gnss", 10,
      std::bind(&PoseTwistFusionNode::callbackKinematicStateGnss, this, std::placeholders::_1));
      
    // GNSS 및 LiDAR로부터 biased_pose_with_covariance를 구독
    sub_biased_pose_lidar_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/pose_twist_fusion_filter/biased_pose_with_covariance_lidar", 10,
      std::bind(&PoseTwistFusionNode::callbackBiasedPoseLidar, this, std::placeholders::_1));
    sub_biased_pose_gnss_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/pose_twist_fusion_filter/biased_pose_with_covariance_gnss", 10,
      std::bind(&PoseTwistFusionNode::callbackBiasedPoseGnss, this, std::placeholders::_1));

    // 융합된 Pose, Twist, Kinematic 상태를 발행
    pub_fused_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/pose_with_covariance", 10);
    pub_fused_twist_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/localization/pose_twist_fusion_filter/twist_with_covariance", 10);
    pub_fused_kinematic_state_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/localization/pose_twist_fusion_filter/kinematic_state", 10);
    pub_fused_biased_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/pose_twist_fusion_filter/biased_pose_with_covariance", 10);

    // PoseStamped 발행을 위한 publisher
    pub_fused_pose_stamped_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/localization/pose_twist_fusion_filter/pose", 10);

    tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    max_buffer_size_ = 15;

    // 50Hz 주기로 fusepose와 fusetwist 계산
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),  // 50Hz 주기
      std::bind(&PoseTwistFusionNode::fusePoseAndTwist, this));
  }

private:
  // LiDAR & GNSS 데이터 버퍼
  std::deque<nav_msgs::msg::Odometry::SharedPtr> kinematic_state_lidar_buffer_;
  std::deque<nav_msgs::msg::Odometry::SharedPtr> kinematic_state_gnss_buffer_;
  std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr> biased_pose_lidar_buffer_;
  std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr> biased_pose_gnss_buffer_;

  size_t max_buffer_size_;

  // 타이머 콜백: 50Hz 주기로 pose와 twist 융합
  void fusePoseAndTwist() {
      nav_msgs::msg::Odometry fused_odom;
      fused_odom.header.stamp = this->now();
      fused_odom.header.frame_id = "map";

      double weight_lidar = 0.5;
      double weight_gnss = 0.5;

      // LiDAR 또는 GNSS 데이터가 없을 때 가장 최근 데이터를 사용
      auto latest_lidar_odom = !kinematic_state_lidar_buffer_.empty() ? kinematic_state_lidar_buffer_.back() : nullptr;
      auto latest_gnss_odom = !kinematic_state_gnss_buffer_.empty() ? kinematic_state_gnss_buffer_.back() : nullptr;

      // 둘 다 없는 경우
      if (!latest_lidar_odom && !latest_gnss_odom) {
          RCLCPP_WARN(this->get_logger(), "No LiDAR or GNSS data available for fusion.");
          return; 
      }

      // LiDAR 데이터만 있는 경우
      if (latest_lidar_odom && !latest_gnss_odom) {
          fused_odom.pose.pose = latest_lidar_odom->pose.pose;
          fused_odom.twist.twist = latest_lidar_odom->twist.twist;

          RCLCPP_INFO(this->get_logger(), "Only LiDAR data used for fusion.");

      // GNSS 데이터만 있는 경우
      } else if (!latest_lidar_odom && latest_gnss_odom) {
          fused_odom.pose.pose = latest_gnss_odom->pose.pose;
          fused_odom.twist.twist = latest_gnss_odom->twist.twist;

          RCLCPP_INFO(this->get_logger(), "Only GNSS data used for fusion.");

      // 둘 다 있는 경우
      } else if (latest_lidar_odom && latest_gnss_odom) {
          // 타임스탬프 차이 계산
          double time_diff = std::abs((rclcpp::Time(latest_lidar_odom->header.stamp) - rclcpp::Time(latest_gnss_odom->header.stamp)).seconds());

          // 0.08초 이상 차이나면 오차가 크므로 가장 가까운 값을 사용
          if (time_diff > 0.08) {
              RCLCPP_WARN(this->get_logger(), "Time difference between LiDAR and GNSS is too large: %f seconds", time_diff);

              // 타임스탬프가 더 최근인 데이터를 사용
              if (rclcpp::Time(latest_lidar_odom->header.stamp) > rclcpp::Time(latest_gnss_odom->header.stamp)) {
                  fused_odom.pose.pose = latest_lidar_odom->pose.pose;
                  fused_odom.twist.twist = latest_lidar_odom->twist.twist;

                  RCLCPP_INFO(this->get_logger(), "Large time difference. Using latest LiDAR data.");
              } else {
                  fused_odom.pose.pose = latest_gnss_odom->pose.pose;
                  fused_odom.twist.twist = latest_gnss_odom->twist.twist;

                  RCLCPP_INFO(this->get_logger(), "Large time difference. Using latest GNSS data.");
              }
          } else {
              // 위치 융합 (차이가 0.08초 이하일 때만 융합)
              fused_odom.pose.pose.position.x = latest_lidar_odom->pose.pose.position.x * weight_lidar +
                                                latest_gnss_odom->pose.pose.position.x * weight_gnss;
              fused_odom.pose.pose.position.y = latest_lidar_odom->pose.pose.position.y * weight_lidar +
                                                latest_gnss_odom->pose.pose.position.y * weight_gnss;
              fused_odom.pose.pose.position.z = latest_lidar_odom->pose.pose.position.z * weight_lidar +
                                                latest_gnss_odom->pose.pose.position.z * weight_gnss;

              // orientation 융합 (쿼터니언 융합)
              fused_odom.pose.pose.orientation = fuseOrientation(
                  latest_lidar_odom->pose.pose.orientation, latest_gnss_odom->pose.pose.orientation, weight_lidar, weight_gnss);

              // 속도 융합
              fused_odom.twist.twist.linear.x = latest_lidar_odom->twist.twist.linear.x * weight_lidar +
                                                latest_gnss_odom->twist.twist.linear.x * weight_gnss;
              fused_odom.twist.twist.angular.z = latest_lidar_odom->twist.twist.angular.z * weight_lidar +
                                                latest_gnss_odom->twist.twist.angular.z * weight_gnss;

              RCLCPP_INFO(this->get_logger(), "LiDAR and GNSS data successfully fused.");
          }
      }

      pub_fused_kinematic_state_->publish(fused_odom);
      broadcastTransform(fused_odom.pose.pose);

      // 융합된 pose와 twist 발행
      geometry_msgs::msg::PoseWithCovarianceStamped fused_pose;
      fused_pose.header = fused_odom.header;
      fused_pose.pose = fused_odom.pose;

      geometry_msgs::msg::TwistWithCovarianceStamped fused_twist;
      fused_twist.header = fused_odom.header;
      fused_twist.twist = fused_odom.twist;

      pub_fused_pose_->publish(fused_pose);
      pub_fused_twist_->publish(fused_twist);

      // PoseStamped 메시지 발행
      geometry_msgs::msg::PoseStamped fused_pose_stamped;
      fused_pose_stamped.header = fused_odom.header;
      fused_pose_stamped.pose = fused_odom.pose.pose;
      
      pub_fused_pose_stamped_->publish(fused_pose_stamped);
  }

  // TF 발행
  void broadcastTransform(const geometry_msgs::msg::Pose& pose) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = "map";
    transform_stamped.child_frame_id = "base_link";
    transform_stamped.transform.translation.x = pose.position.x;
    transform_stamped.transform.translation.y = pose.position.y;
    transform_stamped.transform.translation.z = pose.position.z;
    transform_stamped.transform.rotation = pose.orientation;
    tf_br_->sendTransform(transform_stamped);
  }

  void callbackKinematicStateLidar(const nav_msgs::msg::Odometry::SharedPtr msg) {
    kinematic_state_lidar_buffer_.push_back(msg);
    if (kinematic_state_lidar_buffer_.size() > max_buffer_size_) {
      kinematic_state_lidar_buffer_.pop_front();
    }
  }

  void callbackKinematicStateGnss(const nav_msgs::msg::Odometry::SharedPtr msg) {
    kinematic_state_gnss_buffer_.push_back(msg);
    if (kinematic_state_gnss_buffer_.size() > max_buffer_size_) {
      kinematic_state_gnss_buffer_.pop_front();
    }
  }

  void callbackBiasedPoseLidar(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    biased_pose_lidar_buffer_.push_back(msg);
    if (biased_pose_lidar_buffer_.size() > max_buffer_size_) {
      biased_pose_lidar_buffer_.pop_front();
    }
    fuseBiasedPose();
  }

  void callbackBiasedPoseGnss(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    biased_pose_gnss_buffer_.push_back(msg);
    if (biased_pose_gnss_buffer_.size() > max_buffer_size_) {
      biased_pose_gnss_buffer_.pop_front();
    }
    fuseBiasedPose();
  }

  void fuseBiasedPose() {
    geometry_msgs::msg::PoseWithCovarianceStamped fused_biased_pose;
    fused_biased_pose.header.stamp = this->now();
    fused_biased_pose.header.frame_id = "map";

    double weight_lidar = 0.5;
    double weight_gnss = 0.5;

    // LiDAR 값만 들어오는 경우
    if (!biased_pose_lidar_buffer_.empty() && biased_pose_gnss_buffer_.empty()) {
        auto latest_lidar_pose = biased_pose_lidar_buffer_.back();
        fused_biased_pose.pose.pose = latest_lidar_pose->pose.pose;

    // GNSS 값만 들어오는 경우
    } else if (biased_pose_lidar_buffer_.empty() && !biased_pose_gnss_buffer_.empty()) {
        auto latest_gnss_pose = biased_pose_gnss_buffer_.back();
        fused_biased_pose.pose.pose = latest_gnss_pose->pose.pose;

    // 둘 다 들어오는 경우
    } else if (!biased_pose_lidar_buffer_.empty() && !biased_pose_gnss_buffer_.empty()) {
        auto latest_lidar_pose = biased_pose_lidar_buffer_.back();
        auto closest_gnss_pose = findClosestPose(biased_pose_gnss_buffer_, latest_lidar_pose->header.stamp);
        if (!closest_gnss_pose) return;

        // 위치 융합
        fused_biased_pose.pose.pose.position.x = latest_lidar_pose->pose.pose.position.x * weight_lidar +
                                                 closest_gnss_pose->pose.pose.position.x * weight_gnss;
        fused_biased_pose.pose.pose.position.y = latest_lidar_pose->pose.pose.position.y * weight_lidar +
                                                 closest_gnss_pose->pose.pose.position.y * weight_gnss;
        fused_biased_pose.pose.pose.position.z = latest_lidar_pose->pose.pose.position.z * weight_lidar +
                                                 closest_gnss_pose->pose.pose.position.z * weight_gnss;

        // 방향(쿼터니언) 융합
        fused_biased_pose.pose.pose.orientation = fuseOrientation(
            latest_lidar_pose->pose.pose.orientation, closest_gnss_pose->pose.pose.orientation, weight_lidar, weight_gnss);
    }

    pub_fused_biased_pose_->publish(fused_biased_pose);
  }

  geometry_msgs::msg::Quaternion fuseOrientation(const geometry_msgs::msg::Quaternion& q1, const geometry_msgs::msg::Quaternion& q2, double w1, double w2) {
    tf2::Quaternion quat1, quat2;
    tf2::fromMsg(q1, quat1);
    tf2::fromMsg(q2, quat2);
    tf2::Quaternion fused_quat = quat1 * w1 + quat2 * w2;
    fused_quat.normalize();
    return tf2::toMsg(fused_quat);
  }

  std::array<double, 36> fuseCovariance(const std::array<double, 36>& cov1, const std::array<double, 36>& cov2, double w1, double w2) {
    std::array<double, 36> fused_cov;
    for (size_t i = 0; i < 36; ++i) {
      fused_cov[i] = cov1[i] * w1 + cov2[i] * w2;
    }
    return fused_cov;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr findClosestPose(
    const std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr>& buffer,
    const builtin_interfaces::msg::Time& target_time) {
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr closest_msg = nullptr;
    double min_time_diff = std::numeric_limits<double>::max();

    rclcpp::Time target_time_rclcpp(target_time);
    for (const auto& msg : buffer) {
      rclcpp::Time msg_time(msg->header.stamp);
      double time_diff = std::abs((msg_time - target_time_rclcpp).seconds());
      if (time_diff < min_time_diff) {
        min_time_diff = time_diff;
        closest_msg = msg;
      }
    }
    return closest_msg;
  }

  nav_msgs::msg::Odometry::SharedPtr findClosestOdometry(
    const std::deque<nav_msgs::msg::Odometry::SharedPtr>& buffer,
    const builtin_interfaces::msg::Time& target_time) {
    nav_msgs::msg::Odometry::SharedPtr closest_msg = nullptr;
    double min_time_diff = std::numeric_limits<double>::max();

    rclcpp::Time target_time_rclcpp(target_time);
    for (const auto& msg : buffer) {
      rclcpp::Time msg_time(msg->header.stamp);
      double time_diff = std::abs((msg_time - target_time_rclcpp).seconds());
      if (time_diff < min_time_diff) {
        min_time_diff = time_diff;
        closest_msg = msg;
      }
    }
    return closest_msg;
  }

  // 구독 및 발행
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_kinematic_state_lidar_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_kinematic_state_gnss_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_biased_pose_lidar_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_biased_pose_gnss_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_fused_pose_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_fused_twist_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_fused_kinematic_state_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_fused_biased_pose_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_fused_pose_stamped_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;

  // 50Hz 타이머
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseTwistFusionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}