#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/ColorRGBA.h>
#include <Eigen/Dense>
#include <cmath>

geometry_msgs::Pose tf_local2global(const Eigen::Matrix4d& global_frame, const geometry_msgs::Pose local_pos) {
    Eigen::Vector3d local_translation(local_pos.position.x, local_pos.position.y, local_pos.position.z);
    Eigen::Quaterniond local_orientation(local_pos.orientation.w, local_pos.orientation.x, local_pos.orientation.y, local_pos.orientation.z);  // Ensure w is first

    Eigen::Matrix3d global_rotation = global_frame.block<3,3>(0, 0);
    Eigen::Quaterniond global_orientation(global_rotation);  // API built-in constructor
    Eigen::Vector3d global_translation = global_frame.block<3, 1>(0, 3);

    Eigen::Vector3d transformed_translation = global_translation + global_rotation * local_translation;
    Eigen::Quaterniond transformed_orientation = global_orientation * local_orientation;

    // Fill return-value
    geometry_msgs::Pose transformed_pos;
    transformed_pos.position.x = transformed_translation.x();
    transformed_pos.position.y = transformed_translation.y();
    transformed_pos.position.z = transformed_translation.z();

    transformed_pos.orientation.x = transformed_orientation.x();
    transformed_pos.orientation.y = transformed_orientation.y();
    transformed_pos.orientation.z = transformed_orientation.z();
    transformed_pos.orientation.w = transformed_orientation.w();

    return transformed_pos;
}


void setup_frame_arrow(visualization_msgs::Marker& arrow, const Eigen::Matrix4d& H, char axis_type,
                       float r, float g, float b, float a, const std::string& name){
  // header setup
  arrow.header.frame_id = "world";
  arrow.header.stamp = ros::Time();

  // namespace setup
  arrow.ns = name;

  arrow.type = visualization_msgs::Marker::ARROW;
  arrow.action = visualization_msgs::Marker::ADD;
  arrow.lifetime = ros::Duration();

  // define local arrow positions
  // arrow is always located at (0, 0, 0) in frame coordinates
  geometry_msgs::Pose local_pos;

  local_pos.position.x = 0;
  local_pos.position.y = 0;
  local_pos.position.z = 0;

  // orientation
  switch (axis_type) {
  case 'x':
    arrow.id = 0;
    local_pos.orientation.x = 0.0;
    local_pos.orientation.y = 0.0;
    local_pos.orientation.z = 0.0;
    local_pos.orientation.w = 1.0;
    break;

  case 'y':
    arrow.id = 1;
    local_pos.orientation.x = 0.0;
    local_pos.orientation.y = 0.0;
    local_pos.orientation.z = std::sin(M_PI / 4);
    local_pos.orientation.w = std::cos(M_PI / 4);
    break;

  case 'z':
    arrow.id = 2;
    local_pos.orientation.x = 0.0;
    local_pos.orientation.y = std::sin(-M_PI / 4);
    local_pos.orientation.z = 0.0;
    local_pos.orientation.w = std::cos(M_PI / 4);
    break;
  default:;
  }

  // transform position into frame coordinates
  arrow.pose = tf_local2global(H, local_pos);

  // arrow.pose = local_pos;
  // setup size
  arrow.scale.x = 1;    // length
  arrow.scale.y = 0.1;  // width
  arrow.scale.z = 0.1;  // height

  // setup color
  arrow.color.r = r;
  arrow.color.g = g;
  arrow.color.b = b;
  arrow.color.a = a;

}

void visualizeFrame(ros::Publisher& publisher, const Eigen::Matrix4d& H, const std::string& name) {
  // frame visualization == 3 arrows for xyz axis
  visualization_msgs::Marker x_axis, y_axis, z_axis;

  setup_frame_arrow(x_axis, H, 'x', 1.0, 0.0, 0.0, 1.0, name);
  setup_frame_arrow(y_axis, H, 'y', 0.0, 1.0, 0.0, 1.0, name);
  setup_frame_arrow(z_axis, H, 'z', 0.0, 0.0, 1.0, 1.0, name);

  visualization_msgs::MarkerArray frame_markers;
  frame_markers.markers.push_back(x_axis);
  frame_markers.markers.push_back(y_axis);
  frame_markers.markers.push_back(z_axis);

  publisher.publish(frame_markers);
}

Eigen::Matrix4d createTransformation(double angle, const Eigen::Vector3d& translation, const Eigen::Vector3d& axis) {
    Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
    Eigen::AngleAxisd rotation(angle, axis);
    H.block<3,3>(0,0) = rotation.toRotationMatrix();
    H.block<3,1>(0,3) = translation;
    return H;
}
// =============== Main function =================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "frames_eigen");
  ros::NodeHandle n; //("~"); - if uncomment doesn't work

  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);

  while (ros::ok())
  {
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN("NUM Subscribers: %d", marker_pub.getNumSubscribers());
      ROS_WARN_ONCE("Please create a subscriber to the frame visualizer");
      sleep(1);
    }

    Eigen::Matrix4d T0 = createTransformation(0, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1));// initial transform
    visualizeFrame(marker_pub, T0, "arrows_World");

    Eigen::Matrix4d T1 = createTransformation(M_PI/4, Eigen::Vector3d(2, 0, 0), Eigen::Vector3d(0, 0, 1));
    visualizeFrame(marker_pub, T1, "arrows_T1");

    Eigen::Matrix4d T2 = createTransformation(M_PI/2, Eigen::Vector3d(4, 0, 2), Eigen::Vector3d(0, 1, 0));
    visualizeFrame(marker_pub, T2, "arrows_T2");

    Eigen::Matrix4d T3 = T1 * T2;
    visualizeFrame(marker_pub, T3, "arrows_T3");

    ROS_INFO("Visualizing");
    r.sleep();
  }
}
