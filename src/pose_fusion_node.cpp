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

    // 매개변수 선언
    this->declare_parameter<std::string>("lidar_pose_topic", "/localization/pose_twist_fusion_filter/kinematic_state_lidar");
    this->declare_parameter<std::string>("gnss_pose_topic", "/localization/pose_twist_fusion_filter/kinematic_state_gnss");
    this->declare_parameter<std::string>("can_topic", "/can_odom");
    this->declare_parameter<double>("velocity_threshold", 0.5);  // 속도 차이 임계값
    this->declare_parameter<double>("angular_threshold", 0.05);  // 회전 속도 차이 임계값
    this->declare_parameter<double>("lidar_pose_weight", 0.5);
    this->declare_parameter<double>("gnss_pose_weight", 0.5);
    this->declare_parameter<double>("lidar_twist_weight", 0.5);
    this->declare_parameter<double>("gnss_twist_weight", 0.5);

    // 매개변수 값 읽기
    this->get_parameter("lidar_pose_topic", lidar_pose_topic_);
    this->get_parameter("gnss_pose_topic", gnss_pose_topic_);
    this->get_parameter("can_topic", can_topic_);
    this->get_parameter("velocity_threshold", velocity_threshold_);
    this->get_parameter("angular_threshold", angular_threshold_);
    this->get_parameter("lidar_pose_weight", lidar_pose_weight_);
    this->get_parameter("gnss_pose_weight", gnss_pose_weight_);
    this->get_parameter("lidar_twist_weight", lidar_twist_weight_);
    this->get_parameter("gnss_twist_weight", gnss_twist_weight_);

    // GNSS 및 LiDAR로부터 kinematic_state를 구독
    sub_kinematic_state_lidar_ = this->create_subscription<nav_msgs::msg::Odometry>(
      lidar_pose_topic_, 10,
      std::bind(&PoseTwistFusionNode::callbackKinematicStateLidar, this, std::placeholders::_1));
    sub_kinematic_state_gnss_ = this->create_subscription<nav_msgs::msg::Odometry>(
      gnss_pose_topic_, 10,
      std::bind(&PoseTwistFusionNode::callbackKinematicStateGnss, this, std::placeholders::_1));

    // CAN data 구독 (angular.z 및 linear.x만 사용)
    sub_can_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      can_topic_, 10, std::bind(&PoseTwistFusionNode::callbackCanOdom, this, std::placeholders::_1));

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

    pub_fused_pose_stamped_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/localization/pose_twist_fusion_filter/pose", 10);

    tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    max_buffer_size_ = 15;

    // 50Hz 주기로 fusePoseAndTwist 계산
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),  // 50Hz 주기
      std::bind(&PoseTwistFusionNode::fusePoseAndTwist, this));

    can_velocity_ = 0.0;
    can_angular_ = 0.0;
  }

private:
  // LiDAR & GNSS 데이터 버퍼
  std::deque<nav_msgs::msg::Odometry::SharedPtr> kinematic_state_lidar_buffer_;
  std::deque<nav_msgs::msg::Odometry::SharedPtr> kinematic_state_gnss_buffer_;
  std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr> biased_pose_lidar_buffer_;
  std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr> biased_pose_gnss_buffer_;

  size_t max_buffer_size_;

  // CAN 데이터 버퍼
  double can_velocity_;
  double can_angular_;

  // 매개변수 변수
  double velocity_threshold_;
  double angular_threshold_;
  double lidar_pose_weight_;
  double gnss_pose_weight_;
  double lidar_twist_weight_;
  double gnss_twist_weight_;
  std::string lidar_pose_topic_;
  std::string gnss_pose_topic_;
  std::string can_topic_;

  void callbackCanOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    can_velocity_ = msg->twist.twist.linear.x;
    can_angular_ = msg->twist.twist.angular.z;  // angular.z 값을 사용
  }

  // 타이머 콜백: 50Hz 주기로 pose와 twist 융합
  void fusePoseAndTwist() {
    nav_msgs::msg::Odometry fused_odom;
    fused_odom.header.stamp = this->now();
    fused_odom.header.frame_id = "map";

    // pose와 twist에 대해 다른 가중치 적용
    double weight_lidar_pose = lidar_pose_weight_;
    double weight_gnss_pose = gnss_pose_weight_;
    double weight_lidar_twist = lidar_twist_weight_;
    double weight_gnss_twist = gnss_twist_weight_;

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
        // CAN 데이터와 비교하여 LiDAR 및 GNSS 속도와 angular.z 차이를 계산
        double lidar_velocity = latest_lidar_odom->twist.twist.linear.x;
        double gnss_velocity = latest_gnss_odom->twist.twist.linear.x;

        if (std::abs(lidar_velocity - can_velocity_) <= velocity_threshold_ &&
            std::abs(gnss_velocity - can_velocity_) <= velocity_threshold_ &&
            std::abs(latest_lidar_odom->twist.twist.angular.z - can_angular_) <= angular_threshold_ &&
            std::abs(latest_gnss_odom->twist.twist.angular.z - can_angular_) <= angular_threshold_) {

            // LiDAR와 GNSS 데이터를 융합 (pose와 twist 각각에 대해 가중치를 다르게 적용)
            weight_lidar_pose = 0.5;
            weight_gnss_pose = 0.5;
            weight_lidar_twist = 0.5;
            weight_gnss_twist = 0.5;

            RCLCPP_INFO(this->get_logger(), "LiDAR and GNSS data successfully fused.");
        } else {
            // LiDAR 또는 GNSS 중 하나가 CAN과 차이가 임계값 이상인 경우
            if (std::abs(lidar_velocity - can_velocity_) > velocity_threshold_ || 
                std::abs(latest_lidar_odom->twist.twist.angular.z - can_angular_) > angular_threshold_) {
                // LiDAR가 CAN과 차이가 크면 GNSS 데이터를 사용
                weight_lidar_pose = 0.0;
                weight_gnss_pose = 1.0;
                weight_lidar_twist = 0.0;
                weight_gnss_twist = 1.0;
                RCLCPP_INFO(this->get_logger(), "LiDAR error is large. Using GNSS data.");
            } else if (std::abs(gnss_velocity - can_velocity_) > velocity_threshold_ || 
                      std::abs(latest_gnss_odom->twist.twist.angular.z - can_angular_) > angular_threshold_) {
                // GNSS가 CAN과 차이가 크면 LiDAR 데이터를 사용
                weight_lidar_pose = 1.0;
                weight_gnss_pose = 0.0;
                weight_lidar_twist = 1.0;
                weight_gnss_twist = 0.0;
                RCLCPP_INFO(this->get_logger(), "GNSS error is large. Using LiDAR data.");
            }
        }

        // pose 융합
        fused_odom.pose.pose.position.x = latest_lidar_odom->pose.pose.position.x * weight_lidar_pose +
                                          latest_gnss_odom->pose.pose.position.x * weight_gnss_pose;
        fused_odom.pose.pose.position.y = latest_lidar_odom->pose.pose.position.y * weight_lidar_pose +
                                          latest_gnss_odom->pose.pose.position.y * weight_gnss_pose;
        fused_odom.pose.pose.position.z = latest_lidar_odom->pose.pose.position.z * weight_lidar_pose +
                                          latest_gnss_odom->pose.pose.position.z * weight_gnss_pose;

        // orientation 융합 (쿼터니언 융합)
        fused_odom.pose.pose.orientation = fuseOrientation(
            latest_lidar_odom->pose.pose.orientation, latest_gnss_odom->pose.pose.orientation, weight_lidar_pose, weight_gnss_pose);

        // twist 융합
        fused_odom.twist.twist.linear.x = latest_lidar_odom->twist.twist.linear.x * weight_lidar_twist +
                                          latest_gnss_odom->twist.twist.linear.x * weight_gnss_twist;
        fused_odom.twist.twist.angular.z = latest_lidar_odom->twist.twist.angular.z * weight_lidar_twist +
                                           latest_gnss_odom->twist.twist.angular.z * weight_gnss_twist;

        RCLCPP_INFO(this->get_logger(), "LiDAR and GNSS data successfully fused.");
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
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_can_odom_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_fused_pose_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_fused_twist_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_fused_kinematic_state_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_fused_biased_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_fused_pose_stamped_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseTwistFusionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}