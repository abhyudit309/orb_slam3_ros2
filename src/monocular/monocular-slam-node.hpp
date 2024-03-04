#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

class MonocularSlamNode : public rclcpp::Node
{
public:
    MonocularSlamNode(ORB_SLAM3::System* pSLAM);

    ~MonocularSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    using PointCloudMsg = sensor_msgs::msg::PointCloud2;
    using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
    using MarkerMsg = visualization_msgs::msg::Marker;

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);

    ORB_SLAM3::System* m_SLAM;

    cv_bridge::CvImagePtr m_cvImPtr;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;

    rclcpp::Time current_frame_time;

    std::string map_frame_id;
    std::string camera_frame_id;
    std::string target_frame_id;

    void Update();

    void PublishRenderedImage(cv::Mat image);

    rclcpp::Publisher<ImageMsg>::SharedPtr rendered_image_publisher;

    void PublishMapPoints(std::vector<ORB_SLAM3::MapPoint*> map_points);

    PointCloudMsg MapPointsToPointCloud(std::vector<ORB_SLAM3::MapPoint*> map_points);

    rclcpp::Publisher<PointCloudMsg>::SharedPtr map_points_publisher;

    Sophus::SE3f current_pose;

    void PublishPositionAsPoseStamped(Sophus::SE3f position);

    tf2::Transform TransformFromMat(Sophus::SE3f position);
    tf2::Transform TransformToTarget(tf2::Transform tf_in, std::string frame_in, std::string frame_target);

    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;

    rclcpp::Publisher<PoseStampedMsg>::SharedPtr pose_publisher;

    void PublishPositionAsTransform(Sophus::SE3f position);

    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;

    rclcpp::Publisher<MarkerMsg>::SharedPtr marker_publisher;
};

#endif
