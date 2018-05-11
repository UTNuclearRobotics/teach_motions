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

  // Wait for first ft data to arrive
/*
  ROS_INFO_NAMED("compliant_replay", "Waiting for first force/torque data.");
  while (ros::ok() && ft_data_.header.frame_id == "")
    ros::Duration(0.1).sleep();
  ROS_INFO_NAMED("compliant_replay", "Received initial FT data.");
*/

  // Sleep to allow the publishers to be created and FT data to stabilize
  ros::Duration(2.).sleep();

  // Prompt user for datafile name
  ROS_INFO_NAMED("compliant_replay", "Enter the datafile name, e.g. 'handle7': ");
  std::cin >> datafile_;

  getParameters();

  // Key equation: compliance_velocity[i] = wrench[i]/stiffness[i]
  // TODO: read this from datafile
  std::vector<double> stiffness(6, 50.);
  // Rotational components
  stiffness[3] = 200.;
  stiffness[4] = 200.;
  stiffness[5] = 200.;


  double filterParam = 10.;

  // Deadband for force/torque measurements
  std::vector<double> deadband(6, 10.);

  // Stop when any force exceeds X N, or torque exceeds X Nm
  std::vector<double> endConditionWrench(6, 60.0);

  // An object for compliant control
  compliant_control::CompliantControl comp(stiffness, 
    deadband, 
    endConditionWrench, 
    filterParam, 
    ft_data_, 
    100., 
    50.);

  // The 6 nominal velocity components.
  std::vector<double> vel_nom(6, 0.0);
  readTraj();

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

/*
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

void compliant_replay::CompliantReplay::getParameters()
{
  num_arms_ = get_ros_params::getIntParam("teach_motions/num_arms", n_);
}

void compliant_replay::CompliantReplay::readTraj()
{
  std::ifstream file("/home/nrgadmin/vaultbot-nrg/src/components/teach_motions/data/handle7_arm0_processed.csv");
  std::string line;
  std::string value;

  // Temporarily hold a single arm's data
  std::vector<double> time, x_dot, y_dot, z_dot, roll_dot, pitch_dot, yaw_dot;

  // Read vectors for each arm
  for (int i=0; i<num_arms_; i++)
  {
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
        ROS_WARN_STREAM(value);
        time.push_back( std::stod(value) );

        getline(ss, value, ',');
        x_dot.push_back( std::stod(value) );

        getline(ss, value, ',');
        y_dot.push_back( std::stod(value) );

        getline(ss, value, ',');
        z_dot.push_back( std::stod(value) );

        getline(ss, value, ',');
        roll_dot.push_back( std::stod(value) );

        getline(ss, value, ',');
        pitch_dot.push_back( std::stod(value) );

        getline(ss, value, '\n');
        yaw_dot.push_back( std::stod(value) );
      }
    }

    // Push this arm vector into the member variable for all arms.
    // For example, times for arm0 are stored in times[0]
    // First sample time for arm0 is at times[0][0]
    times_.push_back(time);
    x_dot_.push_back(x_dot);
    y_dot_.push_back(y_dot);
    z_dot_.push_back(z_dot);
    roll_dot_.push_back(roll_dot);
    pitch_dot_.push_back(pitch_dot);
    yaw_dot_.push_back(yaw_dot);
  }

  // Debugging:
  ROS_WARN_STREAM(times_[0][0]);
  ROS_WARN_STREAM(times_[1][0]);
  ROS_WARN_STREAM(times_[0].back());
  ROS_WARN_STREAM(times_[1].back());

  ROS_ERROR_STREAM(yaw_dot_[0][0]);
  ROS_ERROR_STREAM(yaw_dot_[1][0]);
  ROS_ERROR_STREAM(yaw_dot_[0].back());
  ROS_ERROR_STREAM(yaw_dot_[1].back());
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