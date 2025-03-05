#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include <cmath>
#include <tf2/LinearMath/Transform.h>

void publish_transform(const std::string& parent_frame_id, const std::string& child_frame_id,
                       const tf2::Transform& transform, tf2_ros::TransformBroadcaster& br) {

  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();

  transformStamped.header.frame_id = parent_frame_id;
  transformStamped.child_frame_id = child_frame_id;

  transformStamped.transform.translation.x = transform.getOrigin().x();
  transformStamped.transform.translation.y = transform.getOrigin().y();
  transformStamped.transform.translation.z = transform.getOrigin().z();

  transformStamped.transform.rotation.x = transform.getRotation().x();
  transformStamped.transform.rotation.y = transform.getRotation().y();
  transformStamped.transform.rotation.z = transform.getRotation().z();
  transformStamped.transform.rotation.w = transform.getRotation().w();

  br.sendTransform(transformStamped);
}

// =============== Main function =================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "frames_tf");
  ros::NodeHandle n; //("~");

  // specially for that case it would be better to use static transform broadcaster
  tf2_ros::TransformBroadcaster br;

  ros::Rate rate(10);
  while (ros::ok()) {
    tf2::Transform T1, T2, T3;
    tf2::Quaternion q1, q2;

    // set vals for T1
    q1.setRPY(0, 0, M_PI / 4);
    T1.setOrigin(tf2::Vector3(2, 0, 0));
    T1.setRotation(q1);

    // set vals for T2
    q2.setRPY(0, M_PI / 2, 0);
    T2.setOrigin(tf2::Vector3(4, 0, 2));
    T2.setRotation(q2);

    publish_transform("world", "T1", T1, br);
    publish_transform("world", "T2", T2, br);

    // T3 == T1 * T2 => parent_frame == T1, publish-transform == T2
    publish_transform("T1", "T3", T2, br);

    ros::spinOnce();

    // ROS_INFO("sending");

    rate.sleep();
  }

  return 0;
}
