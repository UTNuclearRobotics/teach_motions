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

  // Listen to the jog_arm warning topic. Exit if the jogger stops
  jog_arm_warning_sub_ = n_.subscribe("jog_arm_server/halted", 1, &CompliantReplay::haltCB, this);

  // Prompt user for datafile name
  ROS_INFO_NAMED("compliant_replay", "Enter the datafile name, e.g. 'handle7': ");
  std::cin >> datafile_;

  setup();

  // Read the 6 nominal velocity components from a datafile.
  readTraj();

  double avg_timestep = ( arm_data_objects_[0].times_.back()-arm_data_objects_[0].times_[0] ) / arm_data_objects_[0].times_.size();
  ros::Rate rate(1/avg_timestep);

  // Store the outgoing Twist msg
  geometry_msgs::TwistStamped jog_cmd;

  int datapoint = 0;
  std::vector<double> velocity_nominal{0., 0., 0., 0., 0., 0.}, velocity_out{0., 0., 0., 0., 0., 0.};

  while (ros::ok() && !jog_is_halted_ && !force_or_torque_limit_ && datapoint < arm_data_objects_[0].times_.size() )
  {
    // For each arm
    for (int arm_index=0; arm_index<num_arms_; arm_index++)
    {
      arm_data_objects_.at(arm_index).ft_data_ = transformToEEF(arm_data_objects_.at(arm_index).ft_data_, arm_data_objects_[arm_index].jog_command_frame_);

      // Read the next velocity datapoint
      velocity_nominal[0] = arm_data_objects_.at(arm_index).x_dot_[datapoint];
      velocity_nominal[1] = arm_data_objects_.at(arm_index).y_dot_[datapoint];
      velocity_nominal[2] = arm_data_objects_.at(arm_index).z_dot_[datapoint];
      velocity_nominal[3] = arm_data_objects_.at(arm_index).roll_dot_[datapoint];
      velocity_nominal[4] = arm_data_objects_.at(arm_index).pitch_dot_[datapoint];
      velocity_nominal[5] = arm_data_objects_.at(arm_index).yaw_dot_[datapoint];

      // Add the compliance velocity to nominal velocity, and check for force/torque limits
      compliance_status_[arm_index] = compliance_objects_[arm_index].getVelocity(velocity_nominal, arm_data_objects_[arm_index].ft_data_, velocity_out);

      // Send cmds to the robot(s)
      jog_cmd.header.frame_id = arm_data_objects_[arm_index].jog_command_frame_;
      jog_cmd.header.stamp = ros::Time::now();
      // TODO: this should be velocity_out, not velocity_nominal:
      jog_cmd.twist.linear.x = velocity_nominal[0];
      jog_cmd.twist.linear.y = velocity_nominal[1];
      jog_cmd.twist.linear.z = velocity_nominal[2];
      jog_cmd.twist.angular.x = velocity_nominal[3];
      jog_cmd.twist.angular.y = velocity_nominal[4];
      jog_cmd.twist.angular.z = velocity_nominal[5];

      velocity_pubs_[arm_index].publish(jog_cmd);

      // Set the flag if a force or torque limit is reached
      if ( compliance_status_[arm_index] == compliantEnum::CONDITION_MET )
        force_or_torque_limit_ = true;
    }

    datapoint++;
    rate.sleep();
  }

  if (jog_is_halted_)
    ROS_WARN_NAMED("compliant_replay", "Jogging was halted. Singularity, joint "
                                      "limit, or collision?");
}

// CB for halt warnings from the jog_arm nodes
void compliant_replay::CompliantReplay::haltCB(const std_msgs::Bool::ConstPtr& msg)
{
  jog_is_halted_ = msg->data;
}

// CB for force/torque data of arm0
void compliant_replay::CompliantReplay::ftCB0(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  arm_data_objects_[0].ft_data_ = *msg;
  arm_data_objects_[0].ft_data_.header.frame_id = arm_data_objects_.at(0).force_torque_frame_;
}

// CB for force/torque data of arm1
void compliant_replay::CompliantReplay::ftCB1(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  arm_data_objects_[1].ft_data_ = *msg;
  arm_data_objects_[1].ft_data_.header.frame_id = arm_data_objects_.at(1).force_torque_frame_;
}

void compliant_replay::CompliantReplay::setup()
{
  num_arms_ = get_ros_params::getIntParam("teach_motions/num_arms", n_);

  // Set up vectors to hold data/objects for each arm
  for (int arm_index=0; arm_index<num_arms_; arm_index++)
  {
    arm_data_objects_.push_back( SingleArmData() );
    arm_data_objects_.at(arm_index).jog_command_frame_ = get_ros_params::getStringParam("teach_motions/ee" + std::to_string(arm_index) + "/ee_frame_name", n_);
    arm_data_objects_.at(arm_index).force_torque_frame_ = get_ros_params::getStringParam("compliant_replay/ee" + std::to_string(arm_index) + "/force_torque_frame", n_);

    // Force/torque data topics to subscribe to.
    // Unfortunately this is hard-coded to 2 callback functions because programmatically generating multiple callbacks ain't easy.
    arm_data_objects_.at(arm_index).force_torque_data_topic_ = get_ros_params::getStringParam("compliant_replay/ee" + std::to_string(arm_index) + "/force_torque_topic", n_);

    // Listen to wrench data from a force/torque sensor.
    // Unfortunately this is hard-coded to 2 callback functions because programmatically generating multiple callbacks ain't easy.
    if (arm_index == 0)
    {
      ros::Subscriber sub0 = n_.subscribe(
        arm_data_objects_.at(arm_index).force_torque_data_topic_,
        1,
        &CompliantReplay::ftCB0,
        this);

      force_torque_subs_.push_back(sub0);
    }

    if (arm_index == 1)
    {
      ros::Subscriber sub1 = n_.subscribe(
        arm_data_objects_.at(arm_index).force_torque_data_topic_,
        1,
        &CompliantReplay::ftCB1,
        this);

      force_torque_subs_.push_back(sub1);
    }

    // To publish commands to robots
    ros::Publisher pub = n_.advertise<geometry_msgs::TwistStamped>(
      get_ros_params::getStringParam("teach_motions/ee" + std::to_string(arm_index) + "/jog_cmd_topic", n_),
      1);
    velocity_pubs_.push_back( pub );

    // Initially, have not achieved the desired force/torque, so enable compliant motion.
    compliance_status_.push_back( compliantEnum::CONDITION_NOT_MET );

    // Customize the compliance parameters
    arm_data_objects_.at(arm_index).setComplianceParams();

    // Wait for first force/torque data to arrive for this arm
    ROS_INFO_STREAM("Waiting for first force/torque data on topic " << arm_data_objects_.at(arm_index).force_torque_data_topic_ );
    while (ros::ok() && arm_data_objects_.at(arm_index).ft_data_.header.frame_id == "")
      ros::Duration(0.1).sleep();
    ROS_INFO_STREAM("Received initial FT data on topic " << arm_data_objects_.at(arm_index).force_torque_data_topic_ );

    // Create a vector of compliance objects from the CompliantControl library
    // The initial force/torque reading is taken as the bias.
    arm_data_objects_.at(arm_index).ft_data_ = transformToEEF(arm_data_objects_.at(arm_index).ft_data_, arm_data_objects_.at(arm_index).jog_command_frame_);

    compliance_objects_.push_back( compliant_control::CompliantControl(
      arm_data_objects_.at(arm_index).stiffness_,
      arm_data_objects_.at(arm_index).deadband_,
      arm_data_objects_.at(arm_index).end_condition_wrench_,
      arm_data_objects_.at(arm_index).filter_param_,
      arm_data_objects_.at(arm_index).ft_data_,
      100.,
      50.
      ) );
  }

  // Sleep to allow the publishers to be created and FT data to stabilize
  ros::Duration(2.).sleep();
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
}