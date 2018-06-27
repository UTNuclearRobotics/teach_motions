///////////////////////////////////////////////////////////////////////////////
//      Title     : compliant_replay.h
//      Project   : compliant_replay
//      Created   : 4/2/2018
//      Author    : Andy Zelenak
//      Platforms : Ubuntu 64-bit
//      Copyright : Copyright© The University of Texas at Austin, 2014-2017. All
//      rights reserved.
//
//          All files within this directory are subject to the following, unless
//          an alternative
//          license is explicitly included within the text of each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or
//          documentation,
//          including but not limited to those resulting from defects in
//          software
//          and/or documentation, or loss or inaccuracy of data of any kind.
//
///////////////////////////////////////////////////////////////////////////////

// Demonstrate compliance on a stationary robot. The robot should act like a
// spring
// when pushed.

#ifndef COMPLIANT_REPLAY_H
#define COMPLIANT_REPLAY_H

#include <actionlib/server/simple_action_server.h>
#include <teach_motions/CompliantReplayAction.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <jog_arm/compliant_control.h>
#include <fstream>
#include <iostream>
#include <ros/package.h>
#include <ros/ros.h>
#include <sstream>
#include <teach_motions/get_ros_params.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

namespace compliant_replay
{

// This class holds all data for the trajectory of one arm
class SingleArmData
{
public:

  // Incoming trajectory data is stored in these vectors
  std::vector<double> times_, x_dot_, y_dot_, z_dot_, roll_dot_, pitch_dot_, yaw_dot_;

  // TF frame of incoming jog cmds
  std::string jog_command_frame_ = "";

  // TF frame of force/torque data
  std::string force_torque_frame_ = "";

  // Key equation: compliance_velocity[i] = wrench[i]/stiffness[i]
  std::vector<double> stiffness_{25000, 25000, 25000, 2000, 2000, 2000};

  // Related to the cutoff frequency of the low-pass filter.
  double filter_param_ = 10.;

  // Deadband for force/torque measurements
  std::vector<double> deadband_{10, 10, 10, 10, 10, 10};

  // Stop when any force exceeds X N, or torque exceeds X Nm
  // The robot controller's built-in safety limits are ~90 N, ? Nm
  std::vector<double> end_condition_wrench_{70, 70, 70, 60, 60, 60};

  // Current force/torque data
  geometry_msgs::WrenchStamped ft_data_;

  // Topic from force/torque sensor
  std::string force_torque_data_topic_;

  // Outgoing velocity msg
  std::vector<double> velocity_out_{0, 0, 0, 0, 0, 0};
};


// Send the compliant motion commands with this class
class CompliantReplay
{
public:
  CompliantReplay();

  void setComplianceParams(int armIndex);

private:
  // An action server goal triggers this
  void actionCB(const teach_motions::CompliantReplayGoalConstPtr &goal);

  // CB for halt warnings from the jog_arm nodes
  void haltCB(const std_msgs::Bool::ConstPtr& msg);

  // CBs for force/torque data
  void ftCB0(const geometry_msgs::WrenchStamped::ConstPtr& msg);
  void ftCB1(const geometry_msgs::WrenchStamped::ConstPtr& msg);

  // Transform a wrench to the EE frame
  geometry_msgs::WrenchStamped transformToEEF(const geometry_msgs::WrenchStamped wrench_in,
                                              const std::string desired_ee_frame);

  // Read configuration parameters and setup vectors
  void setup();

  // Read trajectories from csv datafile
  void readTraj();

  int num_arms_ = 1;

  ros::NodeHandle n_;

  ros::Subscriber jog_arm_warning_sub_;

  // Did one of the jog nodes halt motion?
  bool jog_is_halted_ = false;
  // Was a force/torque target achieved?
  bool force_or_torque_limit_ = false;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string input_file_;

  // Store the trajectory data for each arm
  std::vector<SingleArmData> arm_data_objects_;

  // Compliance for each arm
  std::vector<compliant_control::CompliantControl> compliance_objects_;
  std::vector<compliant_control::exitCondition> compliance_status_;

  // Subscribe to force/torque topics
  // Unfortunately this is hard-coded to 2 callback functions because programmatically generating multiple callbacks ain't easy.
  std::vector<ros::Subscriber> force_torque_subs_;

  // Publish velocity cmd(s) to the jog_arm node(s)
  std::vector<ros::Publisher> velocity_pubs_;

  actionlib::SimpleActionServer<teach_motions::CompliantReplayAction> action_server_;
  teach_motions::CompliantReplayResult action_result_;
};
}  // end namespace compliant_replay

#endif