#include "monocular-slam-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;
    tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer, this);
    tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    map_frame_id = "map";
    camera_frame_id = "camera_link";
    target_frame_id = "base_link";
    rendered_image_publisher = this->create_publisher<ImageMsg>("/testing/image", 10);
    map_points_publisher = this->create_publisher<PointCloudMsg>("/testing/point_cloud", 10);
    pose_publisher = this->create_publisher<PoseStampedMsg>("/testing/pose_stamped", 10);
    marker_publisher = this->create_publisher<MarkerMsg>("/testing/marker", 10);

    m_image_subscriber = this->create_subscription<ImageMsg>(
        "/image",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
    std::cout << "slam changed" << std::endl;
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularSlamNode::Update() {
    PublishRenderedImage(m_SLAM->DrawCurrentFrame());
    PublishMapPoints(m_SLAM->GetAllMapPoints());
    PublishPositionAsPoseStamped(current_pose);
    PublishPositionAsTransform(current_pose);
}

void MonocularSlamNode::PublishRenderedImage(cv::Mat image) {
    std_msgs::msg::Header header;
    header.stamp = current_frame_time;
    header.frame_id = map_frame_id;
    sensor_msgs::msg::Image::SharedPtr rendered_image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    rendered_image_publisher->publish(*rendered_image_msg.get());
}

void MonocularSlamNode::PublishMapPoints(std::vector<ORB_SLAM3::MapPoint*> map_points) {
    PointCloudMsg cloud = MapPointsToPointCloud(map_points);
    map_points_publisher->publish(cloud);
}

sensor_msgs::msg::PointCloud2 MonocularSlamNode::MapPointsToPointCloud(std::vector<ORB_SLAM3::MapPoint*> map_points) {
    if (map_points.size() == 0) {
        std::cout << "Map point vector is empty!" << std::endl;
    }

    PointCloudMsg cloud;

    const int num_channels = 3; // x y z

    cloud.header.stamp = current_frame_time;
    cloud.header.frame_id = map_frame_id;
    cloud.height = 1;
    cloud.width = map_points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = {"x", "y", "z"};
    for (int i = 0; i < num_channels; i++) {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);

    float data_array[num_channels];
    for (unsigned int i = 0; i < cloud.width; i++) {
        if (map_points.at(i)->nObs >= 2) {
            data_array[0] = map_points.at(i)->GetWorldPos()(2); //x. Do the transformation by just reading at the position of z instead of x
            data_array[1] = -1.0 * map_points.at(i)->GetWorldPos()(0); //y. Do the transformation by just reading at the position of x instead of y
            data_array[2] = -1.0 * map_points.at(i)->GetWorldPos()(1); //z. Do the transformation by just reading at the position of y instead of z
            memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, num_channels*sizeof(float));
        }
    }
    return cloud;
}

void MonocularSlamNode::PublishPositionAsPoseStamped(Sophus::SE3f position) {
    tf2::Transform tf_position = TransformFromMat(position);

    // Make transform from camera frame to target frame
    tf2::Transform tf_position_target = TransformToTarget(tf_position, camera_frame_id, target_frame_id);

    // Make message
    tf2::Stamped<tf2::Transform> tf_position_target_stamped;
    tf_position_target_stamped = tf2::Stamped<tf2::Transform>(tf_position_target, tf2_ros::fromMsg(current_frame_time), map_frame_id);
    ////
    PoseStampedMsg pose_msg;
    geometry_msgs::msg::TransformStamped temp_msg = tf2::toMsg(tf_position_target_stamped);
    pose_msg.header = temp_msg.header;
    pose_msg.pose.position.x = temp_msg.transform.translation.x;
    pose_msg.pose.position.y = temp_msg.transform.translation.y;
    pose_msg.pose.position.z = temp_msg.transform.translation.z;
    pose_msg.pose.orientation = temp_msg.transform.rotation;
    // Creating a marker
    MarkerMsg marker_msg;
    marker_msg.header = temp_msg.header;
    marker_msg.id = 0;
    marker_msg.type = 2; // sphere
    marker_msg.pose.position.x = temp_msg.transform.translation.x;
    marker_msg.pose.position.y = temp_msg.transform.translation.y;
    marker_msg.pose.position.z = temp_msg.transform.translation.z;
    marker_msg.pose.orientation.x = 0.0;
    marker_msg.pose.orientation.y = 0.0;
    marker_msg.pose.orientation.z = 0.0;
    marker_msg.pose.orientation.w = 1.0;
    marker_msg.scale.x = 0.1;
    marker_msg.scale.y = 0.1;
    marker_msg.scale.z = 0.1;
    marker_msg.color.a = 1.0;
    marker_msg.color.r = 0.0;
    marker_msg.color.g = 1.0;
    marker_msg.color.b = 0.0;
    ////
    pose_publisher->publish(pose_msg);
    marker_publisher->publish(marker_msg);
}

void MonocularSlamNode::PublishPositionAsTransform(Sophus::SE3f position) {
    // Get transform from map to camera frame
    tf2::Transform tf_transform = TransformFromMat(position);

    // Make transform from camera frame to target frame
    tf2::Transform tf_map2target = TransformToTarget(tf_transform, camera_frame_id, target_frame_id);

    // Make message
    tf2::Stamped<tf2::Transform> tf_map2target_stamped;
    tf_map2target_stamped = tf2::Stamped<tf2::Transform>(tf_map2target, tf2_ros::fromMsg(current_frame_time), map_frame_id);
    geometry_msgs::msg::TransformStamped msg = tf2::toMsg(tf_map2target_stamped);
    msg.child_frame_id = target_frame_id;
    // Broadcast tf
    tfBroadcaster->sendTransform(msg);
}

tf2::Transform MonocularSlamNode::TransformFromMat(Sophus::SE3f position) {
    Eigen::Matrix3f rotation = position.rotationMatrix();
    Eigen::Vector3f translation = position.translation();

    tf2::Matrix3x3 tf_camera_rotation(rotation(0, 0), rotation(0, 1), rotation(0, 2),
                                    rotation(1, 0), rotation(1, 1), rotation(1, 2),
                                    rotation(2, 0), rotation(2, 1), rotation(2, 2));

    tf2::Vector3 tf_camera_translation(translation(0), translation(1), translation(2));

    // Coordinate transformation matrix from orb coordinate system to ros coordinate system
    const tf2::Matrix3x3 tf_orb_to_ros(0, 0, 1, 
                                        -1, 0, 0, 
                                        0, -1, 0);

    // Transform from orb coordinate system to ros coordinate system on camera coordinates
    tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
    tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

    // Inverse matrix
    tf_camera_rotation = tf_camera_rotation.transpose();
    tf_camera_translation = -(tf_camera_rotation * tf_camera_translation);

    // Transform from orb coordinate system to ros coordinate system on map coordinates
    tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
    tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

    return tf2::Transform(tf_camera_rotation, tf_camera_translation);
}

tf2::Transform MonocularSlamNode::TransformToTarget(tf2::Transform tf_in, std::string frame_in, std::string frame_target) {
    // Transform tf_in from frame_in to frame_target
    tf2::Transform tf_map2orig = tf_in;
    tf2::Transform tf_orig2target;
    tf2::Transform tf_map2target;

    tf2::Stamped<tf2::Transform> transformStampedTemp;
    try {
        // Get the transform from camera to target
        geometry_msgs::msg::TransformStamped tf_msg = tfBuffer->lookupTransform(frame_in, frame_target, this->now());
        // Convert to tf2
        tf2::fromMsg(tf_msg, transformStampedTemp);
        tf_orig2target.setBasis(transformStampedTemp.getBasis());
        tf_orig2target.setOrigin(transformStampedTemp.getOrigin());

    } 
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s", ex.what());
        tf_orig2target.setIdentity();
    }

    // Transform from map to target
    tf_map2target = tf_map2orig * tf_orig2target;
    return tf_map2target;
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    std::cout<<"one frame has been sent"<<std::endl;
    current_frame_time = msg->header.stamp;
    current_pose = m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));

    Update();
}
