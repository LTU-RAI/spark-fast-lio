#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace spark_fast_lio {

class Relocalization : public rclcpp::Node {
 public:
  using PointT      = pcl::PointXYZI;
  using PointCloudT = pcl::PointCloud<PointT>;

  explicit Relocalization(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~Relocalization() override;

 private:
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  void correctionLoop();
  void broadcastMapToOdomTf(const rclcpp::Time& stamp);

  bool runGicp(const PointCloudT::Ptr& scan_in_odom,
               const Eigen::Isometry3d& map_T_odom_guess,
               double max_corresp_dist,
               Eigen::Isometry3d& map_T_odom_out,
               double& fitness_out);

  PointCloudT::Ptr voxelDownsample(const PointCloudT::Ptr& in, double leaf) const;
  void publishLocalizedOdom(const rclcpp::Time& stamp);

  // Params
  std::string map_file_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  double prior_map_voxel_size_{0.4};
  double scan_voxel_size_{0.4};
  double max_corresp_dist_init_{5.0};
  double max_corresp_dist_track_{1.0};
  int max_iterations_{50};
  double transformation_epsilon_{1e-4};
  double fitness_threshold_{1.0};
  double correction_rate_hz_{1.0};
  double tf_publish_rate_hz_{50.0};
  bool publish_localized_odom_{true};

  // State (protected by mutex_)
  std::mutex mutex_;
  Eigen::Isometry3d map_T_odom_{Eigen::Isometry3d::Identity()};
  std::atomic<bool> relocalized_{false};

  PointCloudT::Ptr prior_map_ds_;
  PointCloudT::Ptr latest_scan_in_odom_;
  rclcpp::Time latest_scan_stamp_;
  Eigen::Isometry3d latest_odom_T_body_{Eigen::Isometry3d::Identity()};
  rclcpp::Time latest_odom_stamp_;
  bool has_scan_{false};
  bool has_odom_{false};

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initialpose_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_prior_map_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_localized_odom_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_status_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::thread correction_thread_;
  std::atomic<bool> stop_thread_{false};
};

}  // namespace spark_fast_lio
