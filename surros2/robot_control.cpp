// Include ROS packages
#include <ros/ros.h>
#include <pxpincher_lib/pxpincher_lib.h> // this is our robot interface

// Include all your messages, tf, etc. here (according to the tutorials)
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2/LinearMath/Transform.h>

void log_current_joints(pxpincher::PhantomXControl &robot){
  pxpincher::JointVector curr_angles = robot.getJointAngles();
  ROS_INFO("Current Joints: {%lf, %lf, %lf, %lf}", curr_angles(0, 0), curr_angles(1, 0), curr_angles(2, 0), curr_angles(3, 0));
}

void log_current_endeffector_state(pxpincher::PhantomXControl &robot){
  tf::StampedTransform tf_base_T_gripper;
  robot.getEndeffectorState(tf_base_T_gripper);

  double r,p,y;
  tf_base_T_gripper.getBasis().getRPY(r, p, y);

  ROS_INFO("\n\tEndeffector State:\n\t\tTranslation: {%lf, %lf, %lf}\n\t\tQutarenions[w, x, y, z]: {%lf, %lf, %lf, %lf}\n\t\tRPY: {%lf, %lf, %lf}",
           tf_base_T_gripper.getOrigin().x(), tf_base_T_gripper.getOrigin().y(), tf_base_T_gripper.getOrigin().z(),
           tf_base_T_gripper.getRotation().w(), tf_base_T_gripper.getRotation().x(), tf_base_T_gripper.getRotation().y(), tf_base_T_gripper.getRotation().z(),
           r, p, y);

}

void grap_and_move(pxpincher::PhantomXControl &robot){
  ROS_INFO("Sending Inverse Kinematik commands:");

  // go to object
  Eigen::Vector3d desired_xyz(0.1, 0.1, 0.07);
  robot.setEndeffectorPose(desired_xyz, 1.57, ros::Duration(5));

  // move down, grap and move up
  // move down
  desired_xyz(2) = -0.07;
  robot.setEndeffectorPose(desired_xyz, 1.57, ros::Duration(5));

  // grap
  robot.setGripperJoint(50);

  // move up
  desired_xyz(2) = 0.07;
  robot.setEndeffectorPose(desired_xyz, 1.57, ros::Duration(5));

  // rotate to other place
  desired_xyz(1) = -0.1;
  robot.setEndeffectorPose(desired_xyz, 1.57, ros::Duration(5));

  // move down and release
  desired_xyz(2) = -0.07;
  robot.setEndeffectorPose(desired_xyz, 1.57, ros::Duration(5));

  // release
  robot.setGripperJoint(100);

  // move up
  desired_xyz(2) = 0.07;
  robot.setEndeffectorPose(desired_xyz, 1.57, ros::Duration(5));

  // go home
  robot.setJointsDefault();
}

// =============== Main function =================
int main( int argc, char** argv )
{
  ros::init(argc, argv, "surros2");
  ros::NodeHandle n; //("~");
  
  pxpincher::PhantomXControl robot;
  robot.initialize();
  
  robot.setJoints({M_PI/6, M_PI/3, M_PI/3, M_PI/3});
  //log_current_endeffector_state(robot);

  //grap_and_move(robot);

    
  return 0;
}
