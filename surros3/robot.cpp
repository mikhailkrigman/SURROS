#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <pxpincher_lib/pxpincher_lib.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <tuple>
#include <vector>

// small house
// Eigen::Vector3d(0.28,  0.03, 0.06),
// Eigen::Vector3d(0.23,  0.03, 0.06),
// Eigen::Vector3d(0.30,  0.00, 0.06),
// Eigen::Vector3d(0.28, -0.03, 0.06),
// Eigen::Vector3d(0.23, -0.03, 0.06),

//#define SIM_CONFIG
#define REAL_CONFIG

#ifdef SIM_CONFIG

#define TOLERANCE_POS 0.001
#define TOLERANCE_PITCH 0.02
#define SEGMENT_POINTS_AMOUNT 50
#define KP 0.25
#define STEP_DURATION 0.5 // in sec

#endif

#ifdef REAL_CONFIG

#define TOLERANCE_POS 0.0075
#define TOLERANCE_PITCH 0.02
#define SEGMENT_POINTS_AMOUNT 20
#define KP 0.3
#define STEP_DURATION 0.5 // in sec
#endif

class RobotRegulator{
private:
  ros::NodeHandle _n;
  pxpincher::PhantomXControl* _robot; // operator= is missing for PhatomXControl
                                      // use pointer as member attribute

  // publishers
  ros::Publisher _current_path_pub;
  ros::Publisher _desired_path_pub;
  ros::Publisher _position_error_pub;

  nav_msgs::Path _current_path_msg;
  nav_msgs::Path _desired_path_msg;

private:
  Eigen::Matrix4d _K;
  Eigen::Vector4d _goal_approach_velocities;

private:
  Eigen::Vector3d _goal_pos;
  double _goal_pitch; // evtl const cause doesn't change while drawing;
  Eigen::Quaterniond _goal_rot_quat;

  Eigen::Vector3d _current_pos;
  Eigen::Quaterniond _current_rot_quat;

private:
  Eigen::Vector3d _error_pos;
  double _error_rot;
  double _tolerance_pos;
  double _tolerance_rot;

  ros::Time _current_step_start_time;
  ros::Duration _step_duration;

private:
  std::vector<Eigen::Vector3d> _bounding_points;
  std::vector<Eigen::Vector3d> _waypoints;

public:
  RobotRegulator() = default;
  RobotRegulator(pxpincher::PhantomXControl &robot)
    : _n("~"),
      _robot(&robot),
      _current_path_pub(_n.advertise<nav_msgs::Path>("/surros3/current_path", 10)),
      _desired_path_pub(_n.advertise<nav_msgs::Path>("/surros3/desired_path", 10)),
      _position_error_pub(_n.advertise<std_msgs::Float64MultiArray>("/surros3/position_error", 10)),

      _K(Eigen::Matrix4d::Identity(4, 4) * KP),
      _goal_approach_velocities(Eigen::Vector4d(0.0, 0.0, 0.0, 0.0)),
      _goal_pos(Eigen::Vector3d(0.0, 0.0, 0.0)),
      _goal_pitch(1.57),
      _goal_rot_quat(Eigen::Quaterniond(Eigen::AngleAxisd(_goal_pitch, Eigen::Vector3d::UnitY()))),


      _current_pos(Eigen::Vector3d(0.0, 0.0, 0.0)),
      _current_rot_quat(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)),

      _error_pos(Eigen::Vector3d(0.0, 0.0, 0.0)),
      _error_rot(0.0),
      _tolerance_pos(TOLERANCE_POS),
      _tolerance_rot(TOLERANCE_PITCH),

      _current_step_start_time(ros::Time::now()),
      _step_duration(ros::Duration(STEP_DURATION)),

      _bounding_points({Eigen::Vector3d(0.11, -0.05,  -0.015),
                        Eigen::Vector3d(0.11,  0.05,  -0.015),
                        Eigen::Vector3d(0.15,  0.075, -0.015),
                        Eigen::Vector3d(0.19,  0.05,  -0.015),
                        Eigen::Vector3d(0.19, -0.05,  -0.015)})
  {
    _robot->initialize();

    _current_path_msg.header.frame_id = "arm_base_link";
    _desired_path_msg.header.frame_id = "arm_base_link";

    // add start point to desired path msg
    geometry_msgs::PoseStamped pose;

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "arm_base_link";
    pose.pose.position.x = _bounding_points[0](0);
    pose.pose.position.y = _bounding_points[0](1);
    pose.pose.position.z = 0;

    pose.pose.orientation.w = 1;

    this->_desired_path_msg.poses.push_back(pose);


    // fill waypoints and desired path msg
    std::vector<int> sequence = {0, 3, 1, 4, 0, 1, 2, 3, 4};

    for(int i = 0; i < sequence.size() - 1; ++i){
      geometry_msgs::PoseStamped pose;

      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = "arm_base_link";
      pose.pose.position.x = _bounding_points[sequence[i + 1]](0);
      pose.pose.position.y = _bounding_points[sequence[i + 1]](1);
      pose.pose.position.z = 0;

      pose.pose.orientation.w = 1;

      this->_desired_path_msg.poses.push_back(pose);

      std::vector<Eigen::Vector3d> interpolated_points = this->interpolate_points(_bounding_points[sequence[i]], _bounding_points[sequence[i + 1]], SEGMENT_POINTS_AMOUNT);
      _waypoints.insert(_waypoints.end(), interpolated_points.begin(), interpolated_points.end());
    }

    this->_desired_path_pub.publish(_desired_path_msg);
  }

  // "main"
  void draw_house_of_nicolaus(){
    ROS_INFO_STREAM("Driving into start position...");
    this->go_to_start_position();
    this->set_goal_position(this->_waypoints[1], this->_goal_pitch);
    ROS_INFO_STREAM("Ready to draw.");

    ros::Rate r(100);
    ROS_INFO("Start regulation:");
    int current_step = 1;
    while(ros::ok){
      this->get_current_position();
      this->calculate_position_error();
      this->publish_position_error();
      this->calculate_goal_approach_velocities();
      this->update_joint_velocities();
      this->update_current_path_msg();


      if(this->goal_reached(true)){
        //ROS_INFO("Goal #%d reached", i);
        if (current_step == _waypoints.size() - 1) {
          this->stop();
          break;
        }
        this->set_goal_position(this->_waypoints[++current_step], this->_goal_pitch);
        this->_current_step_start_time = ros::Time::now();
      }

      r.sleep();
    }
    ROS_WARN_STREAM("JOB DONE");
  }


  void check_bounding_points(){
    while (ros::ok) {
      _robot->setEndeffectorPose(_bounding_points[0], _goal_pitch, ros::Duration(5));
      _robot->setEndeffectorPose(_bounding_points[1], _goal_pitch, ros::Duration(5));
      _robot->setEndeffectorPose(_bounding_points[2], _goal_pitch, ros::Duration(5));
      _robot->setEndeffectorPose(_bounding_points[3], _goal_pitch, ros::Duration(5));
      _robot->setEndeffectorPose(_bounding_points[4], _goal_pitch, ros::Duration(5));
      _robot->setJointsDefault();
      break;
    }
  }
private:

  void update_current_path_msg(){
    geometry_msgs::PoseStamped pose;

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "arm_base_link";
    pose.pose.position.x = this->_current_pos(0);
    pose.pose.position.y = this->_current_pos(1);
    pose.pose.position.z = 0;

    pose.pose.orientation.w = 1;

    this->_current_path_msg.poses.push_back(pose);
    this->_current_path_msg.header.stamp = ros::Time::now();
    this->_current_path_pub.publish(_current_path_msg);

  };

  void stop(){
    this->_robot->setJointVel({0.0, 0.0, 0.0, 0.0});
  }

  bool goal_reached(bool use_position_error = false){
    if (!use_position_error){
      return (ros::Time::now() - this->_current_step_start_time > this->_step_duration);
    }
    else {
      return ((this->_error_pos.norm() < this->_tolerance_pos)
              && (std::fabs(this->_error_rot) < this->_tolerance_rot))
              || (ros::Time::now() - this->_current_step_start_time > this->_step_duration);
    }
  }


  void go_to_start_position(){
    this->_robot->setEndeffectorPose(_bounding_points[0], _goal_pitch, ros::Duration(5));
  }

  void set_goal_position(Eigen::Vector3d goal_pos, double goal_pitch){
    this->_goal_pos = goal_pos;
    this->_goal_pitch = goal_pitch;
  }

  void get_current_position(){
    Eigen::Affine3d ee_transform;
    this->_robot->getEndeffectorState(ee_transform);

    this->_current_pos = ee_transform.translation();
    this->_current_rot_quat = Eigen::Quaterniond(ee_transform.rotation());
  }

  void calculate_position_error(){
    this->_error_pos = this->_goal_pos - this->_current_pos;

    Eigen::Quaterniond error_quat = this->_goal_rot_quat * this->_current_rot_quat.inverse();
    this->_error_rot = error_quat.y();
  }

  void publish_position_error(){
    std_msgs::Float64MultiArray error_msg;
    error_msg.data = {this->_error_pos(0), this->_error_pos(1), this->_error_pos(2), this->_error_rot};

    this->_position_error_pub.publish(error_msg);
  }

  void update_joint_velocities(){
    Eigen::Vector4d control_velocities, joint_velocities;
    control_velocities << this->_error_pos(0), this->_error_pos(1), this->_error_pos(2), this->_error_rot;

    Eigen::Matrix4d jacobian_reduced;
    this->_robot->getJacobianReduced(jacobian_reduced);
    \
    joint_velocities = jacobian_reduced.inverse() * (this->_goal_approach_velocities + this->_K * control_velocities);

    std::vector<double> joint_velocities_vec;

    for(int i = 0; i < 4; ++i)
      joint_velocities_vec.push_back(joint_velocities(i));

    this->_robot->setJointVel(joint_velocities);
    //ROS_INFO_STREAM(joint_velocities[0] << ", " << joint_velocities[1] << ", "  << joint_velocities[2] << ", "  << joint_velocities[3]);
  }

  void calculate_goal_approach_velocities(){
    if (std::find(_bounding_points.begin(), _bounding_points.end(), this->_goal_pos) != _bounding_points.end()){
      this->_goal_approach_velocities = Eigen::Vector4d(0.0, 0.0, 0.0, 0.0);
    }
    else{
      this->_goal_approach_velocities = Eigen::Vector4d(this->_error_pos(0), this->_error_pos(1), this->_error_pos(2), this->_error_rot);
    }

    // smarter ways:
    // 1) set velocity towards the bounding points (unit_vector * A, with A as predefined const)
    // 2) control acceleration instead of velocity  (depending on relative position to bounding point set it faster or slower)
    // 3) lookup on 2-3 points ahead (should make the movement smoother)
  }

  std::vector<Eigen::Vector3d> interpolate_points(Eigen::Vector3d start, Eigen::Vector3d end, int total_count = 100){
    std::vector<Eigen::Vector3d> interpolated_points = {};

    for (int i = 0; i < total_count; ++i) {
      Eigen::Vector3d new_point;

      double t = static_cast<double>(i) / (total_count - 1);
      new_point(0) = (1 - t) * start(0) + t * end(0);
      new_point(1) = (1 - t) * start(1) + t * end(1);
      new_point(2) = start(2); // start(2) == end(2)

      interpolated_points.push_back(new_point);
    }

    return interpolated_points;
  }

};

  // =============== Main function =================
  int main(int argc, char** argv){
    ros::init(argc, argv, "surros3");
    pxpincher::PhantomXControl robot;
    RobotRegulator robot_regulator(robot);
    //robot_regulator.check_bounding_points();
    robot_regulator.draw_house_of_nicolaus();
    //robot.setEndeffectorPose(Eigen::Vector3d(0.27, -0.07, 0.06), 0.03, ros::Duration(5));
    //robot_regulator.show_current_position();

    return 0;
  }
