///////////////////////////////////////////////////////////////////////////////
//      Title     : compliant_replay.cpp
//      Project   : compliant_replay
//      Created   : 5/10/2018
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

// Read a datafile of saved velocities. Republish them to the robot(s).
// Modify the velocity to include compliance.

#include "teach_motions/compliant_replay.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "compliant_replay");

  compliant_replay::CompliantReplay replay_it;

  return 0;
}

compliant_replay::CompliantReplay::CompliantReplay() : spinner_(1), tf_listener_(tf_buffer_)
{
  spinner_.start();

  // To publish commands to robots
  vel_pub_ = n_.advertise<geometry_msgs::TwistStamped>("left_arm/jog_arm_server/delta_jog_cmds", 1);

  // Listen to the jog_arm warning topic. Exit if the jogger stops
  jog_arm_warning_sub_ = n_.subscribe("jog_arm_server/halted", 1, &CompliantReplay::haltCB, this);

  // Listen to wrench data from a force/torque sensor
  ft_sub_ = n_.subscribe("left_ur5_wrench", 1, &CompliantReplay::ftCB, this);

  // Sleep to allow the publishers to be created and FT data to stabilize
  ros::Duration(2.).sleep();

  // Prompt user for datafile name
  ROS_INFO_NAMED("compliant_replay", "Enter the datafile name, e.g. 'handle7': ");
  std::cin >> datafile_;

  setup();

  // Read the 6 nominal velocity components from a datafile.
  readTraj();

  // Wait for first force/torque data to arrive for each arm
  for (int arm_index=0; arm_index<num_arms_; arm_index++)
  {
    ROS_INFO_STREAM("Waiting for first force/torque data on topic " << arm_data_objects_.at(arm_index).ft_data_topic_ );
    while (ros::ok() && ft_data_.header.frame_id == "")
      ros::Duration(0.1).sleep();
    ROS_INFO_STREAM("Received initial FT data on topic " << arm_data_objects_.at(arm_index).ft_data_topic_ );
  }

/*
  // The 6 velocity feedback components
  std::vector<double> vel_out(6, 0.0);
  geometry_msgs::TwistStamped jog_cmd;
  // Make sure this command frame matches what the jog_arm node expects
  jog_cmd.header.frame_id = "left_ur5_ee_link";

  // Get initial status
  ft_data_ = transformToEEF(ft_data_, jog_cmd.header.frame_id);
  compliantEnum::exitCondition compliance_condition = comp.getVelocity(vel_nom, ft_data_, vel_out);

  // Loop at X Hz. Specific frequency is not critical
  ros::Rate rate(100.);

  while (ros::ok() && !jog_is_halted_ && (compliance_condition == compliantEnum::CONDITION_NOT_MET))
  {
    ft_data_ = transformToEEF(ft_data_, jog_cmd.header.frame_id);
    compliance_condition = comp.getVelocity(vel_nom, ft_data_, vel_out);

    // Send cmds to the robot(s)
    jog_cmd.header.stamp = ros::Time::now();
    jog_cmd.twist.linear.x = vel_out[0];
    jog_cmd.twist.linear.y = vel_out[1];
    jog_cmd.twist.linear.z = vel_out[2];
    jog_cmd.twist.angular.x = vel_out[3];
    jog_cmd.twist.angular.y = vel_out[4];
    jog_cmd.twist.angular.z = vel_out[5];

    vel_pub_.publish(jog_cmd);

    rate.sleep();
  }

  if (jog_is_halted_)
    ROS_WARN_NAMED("compliant_replay", "Jogging was halted. Singularity, jt "
                                      "limit, or collision?");
*/
}

// CB for halt warnings from the jog_arm nodes
void compliant_replay::CompliantReplay::haltCB(const std_msgs::Bool::ConstPtr& msg)
{
  jog_is_halted_ = msg->data;
}

// CB for force/torque data
void compliant_replay::CompliantReplay::ftCB(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  ft_data_ = *msg;
  ft_data_.header.frame_id = "left_ur5_base";
}

void compliant_replay::CompliantReplay::setup()
{
  num_arms_ = get_ros_params::getIntParam("teach_motions/num_arms", n_);

  // Set up vectors to hold data for each arm
  for (int arm_index=0; arm_index<num_arms_; arm_index++)
  {
    arm_data_objects_.push_back( SingleArmData() );
    arm_data_objects_.at(arm_index).frame_ = get_ros_params::getStringParam("teach_motions/ee" + std::to_string(arm_index) + "/ee_frame_name", n_);

    // Customize the compliance parameters
    arm_data_objects_.at(arm_index).setComplianceParams();

    // Create a vector of compliance objects from the CompliantControl library
    compliance_objects_.push_back( compliant_control::CompliantControl(
      arm_data_objects_.at(arm_index).stiffness_,
      arm_data_objects_.at(arm_index).deadband_,
      arm_data_objects_.at(arm_index).end_condition_wrench_,
      arm_data_objects_.at(arm_index).filter_param_,
      arm_data_objects_.at(arm_index).ft_data_,
      100.,
      50.
      ) );

    // Force/torque data topics to subscribe to
    arm_data_objects_.at(arm_index).ft_data_topic_ = get_ros_params::getStringParam("compliant_replay/ee" + std::to_string(arm_index) + "/ft_topic", n_);
  }
}

void compliant_replay::CompliantReplay::readTraj()
{
  std::string path = ros::package::getPath("teach_motions");

  std::string line;
  std::string value;

  // Read vectors for each arm
  for (int arm_index=0; arm_index<num_arms_; arm_index++)
  {
    std::ifstream file( path + "/data/handle7_arm" + std::to_string(arm_index) + "_processed.csv" );

    // Ignore the first line (headers)
    getline( file, line);

    while ( file.good() )
    {
      // Read a whole line
      getline( file, line);

      // Read each value
      std::stringstream ss(line);

      getline(ss, value, ',');
      if (value != "")  // Check for end of file
      {
        // Push one value in at a time
        arm_data_objects_.at(arm_index).times_.push_back( std::stod(value) );

        getline(ss, value, ',');
        arm_data_objects_.at(arm_index).x_dot_.push_back( std::stod(value) );

        getline(ss, value, ',');
        arm_data_objects_.at(arm_index).y_dot_.push_back( std::stod(value) );

        getline(ss, value, ',');
        arm_data_objects_.at(arm_index).z_dot_.push_back( std::stod(value) );

        getline(ss, value, ',');
        arm_data_objects_.at(arm_index).roll_dot_.push_back( std::stod(value) );

        getline(ss, value, ',');
        arm_data_objects_.at(arm_index).pitch_dot_.push_back( std::stod(value) );

        getline(ss, value, '\n');
        arm_data_objects_.at(arm_index).yaw_dot_.push_back( std::stod(value) );
      }
    }
  }

  ROS_INFO_STREAM_NAMED("compliant_replay", "Done reading datafiles for all arms.");
}

// Transform a wrench to the EE frame
geometry_msgs::WrenchStamped compliant_replay::CompliantReplay::transformToEEF(
    const geometry_msgs::WrenchStamped wrench_in, const std::string desired_ee_frame)
{
  geometry_msgs::TransformStamped prev_frame_to_new;

  prev_frame_to_new =
      tf_buffer_.lookupTransform(desired_ee_frame, wrench_in.header.frame_id, ros::Time(0), ros::Duration(1.0));

  // There is no method to transform a Wrench, so break it into vectors and
  // transform one at a time
  geometry_msgs::Vector3Stamped force_vector;
  force_vector.vector = wrench_in.wrench.force;
  force_vector.header.frame_id = wrench_in.header.frame_id;
  tf2::doTransform(force_vector, force_vector, prev_frame_to_new);

  geometry_msgs::Vector3Stamped torque_vector;
  torque_vector.vector = wrench_in.wrench.torque;
  torque_vector.header.frame_id = wrench_in.header.frame_id;
  tf2::doTransform(torque_vector, torque_vector, prev_frame_to_new);

  // Put these components back into a WrenchStamped
  geometry_msgs::WrenchStamped wrench_out;
  wrench_out.header.stamp = wrench_in.header.stamp;
  wrench_out.header.frame_id = desired_ee_frame;
  wrench_out.wrench.force = force_vector.vector;
  wrench_out.wrench.torque = torque_vector.vector;

  return wrench_out;
}

void compliant_replay::SingleArmData::setComplianceParams()
{
  // TODO: read stiffness from datafile
  // Rotational components
  stiffness_[3] = 200.;
  stiffness_[4] = 200.;
  stiffness_[5] = 200.;
}