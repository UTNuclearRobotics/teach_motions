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

compliant_replay::CompliantReplay::CompliantReplay() :
  tf_listener_(tf_buffer_),
  action_server_(n_, "compliant_replay", boost::bind(&CompliantReplay::actionCB, this, _1), true)
{
  // Listen to the jog_arm warning topic. Exit if the jogger stops
 // jog_arm_warning_sub_ = n_.subscribe("jog_arm_server/warning", 1, &CompliantReplay::haltCB, this);
}

// An action server goal triggers this
void compliant_replay::CompliantReplay::actionCB(const teach_motions::CompliantReplayGoalConstPtr &goal)
{
  // Datafile name
  input_file_ = goal->file_prefix.data;

  setup();

  // Read the 6 nominal velocity components from a datafile.
  readTraj();

  double avg_timestep = ( arm_data_objects_[0].times_.back()-arm_data_objects_[0].times_[0] ) / arm_data_objects_[0].times_.size();
  ros::Rate rate(1/avg_timestep);

  // Store the outgoing Twist msg
  geometry_msgs::TwistStamped jog_cmd;

  int datapoint = 0;
  std::vector<double> velocity_nominal{0., 0., 0., 0., 0., 0.}, velocity_out{0., 0., 0., 0., 0., 0.};

  // Record data to csv file for future analysis
  std::ofstream replay_datafile;
  std::string path = ros::package::getPath("teach_motions");
  replay_datafile.open( path + "/data/log/log.csv" );
  std::string output_data_line;
  // Write a description of this datafile
  output_data_line = "This file saves data from the most recent trajectory.\n";
  replay_datafile << output_data_line;
  output_data_line.clear();
  // Write descriptive column headers
  output_data_line = "x_nom_vel,y_nom_vel,z_nom_vel,roll_nom_vel,pitch_nom_vel,yaw_nom_vel,x_compl_vel,y_compl_vel,z_compl_vel,roll_compl_vel,pitch_compl_vel,yaw_compl_vel,Fx,Fy,Fz,Tx,Ty,Tz\n";
  replay_datafile << output_data_line;
  output_data_line.clear();

  while (
    ros::ok() && 
    !jog_is_halted_ && 
    datapoint < arm_data_objects_[0].times_.size() &&
    !action_server_.isPreemptRequested()
    )
  {
    // For each arm
    for (int arm_index=0; arm_index<num_arms_; ++arm_index)
    {

      // Read the next velocity datapoint
      velocity_nominal[0] = arm_data_objects_.at(arm_index).x_dot_[datapoint];
      velocity_nominal[1] = arm_data_objects_.at(arm_index).y_dot_[datapoint];
      velocity_nominal[2] = arm_data_objects_.at(arm_index).z_dot_[datapoint];
      velocity_nominal[3] = arm_data_objects_.at(arm_index).roll_dot_[datapoint];
      velocity_nominal[4] = arm_data_objects_.at(arm_index).pitch_dot_[datapoint];
      velocity_nominal[5] = arm_data_objects_.at(arm_index).yaw_dot_[datapoint];

      // Send cmds to the robot(s)
      jog_cmd.header.frame_id = arm_data_objects_[arm_index].jog_command_frame_;
      jog_cmd.header.stamp = ros::Time::now();

      jog_cmd.twist.linear.x = velocity_out[0];
      jog_cmd.twist.linear.y = velocity_out[1];
      jog_cmd.twist.linear.z = velocity_out[2];
      jog_cmd.twist.angular.x = velocity_out[3];
      jog_cmd.twist.angular.y = velocity_out[4];
      jog_cmd.twist.angular.z = velocity_out[5];

      velocity_pubs_[arm_index].publish(jog_cmd);


      // Save data for this arm at this timestep
      output_data_line.append(
        std::to_string(velocity_nominal[0]) + "," +
        std::to_string(velocity_nominal[1]) + "," +
        std::to_string(velocity_nominal[2]) + "," +
        std::to_string(velocity_nominal[3]) + "," +
        std::to_string(velocity_nominal[4]) + "," +
        std::to_string(velocity_nominal[5]) + "," +
        std::to_string(velocity_out[0]) + "," +
        std::to_string(velocity_out[1]) + "," +
        std::to_string(velocity_out[2]) + "," +
        std::to_string(velocity_out[3]) + "," +
        std::to_string(velocity_out[4]) + "," +
        std::to_string(velocity_out[5]) + "," 
      );
    }
    // Terminate this line of data
    output_data_line.append("\n");
    replay_datafile << output_data_line;
    output_data_line.clear();

    // Go to the next line of datafile
    ++datapoint;
    rate.sleep();
  }

  replay_datafile.close();

  if ( jog_is_halted_ )
  {
    ROS_WARN_NAMED("compliant_replay", "Jogging was halted. Singularity, joint "
                                      "limit, or collision?");
    action_result_.successful.data = false;
  }
  else if ( action_server_.isPreemptRequested() )
    action_result_.successful.data = false;
  else
    action_result_.successful.data = true;

  action_server_.setSucceeded( action_result_ );
}

// CB for halt warnings from the jog_arm nodes
void compliant_replay::CompliantReplay::haltCB(const std_msgs::Bool::ConstPtr& msg)
{
  jog_is_halted_ = msg->data;
}


void compliant_replay::CompliantReplay::setup()
{
  // system() is bad practice, but loading parameters otherwise is tough.
  // Would need to parse the yaml file and load them individually.
  std::system( ("roslaunch teach_motions vaultbot_set_parameters.launch file_prefix:=" + input_file_ ).c_str() );
  ros::Duration(0.5).sleep();

  num_arms_ = get_ros_params::getIntParam("teach_motions/num_arms", n_);

  // Set up vectors to hold data/objects for each arm
  for (int arm_index=0; arm_index<num_arms_; ++arm_index)
  {
    arm_data_objects_.push_back( SingleArmData() );

    arm_data_objects_.at(arm_index).jog_command_frame_ = get_ros_params::getStringParam("teach_motions/ee" + std::to_string(arm_index) + "/ee_frame_name", n_);


    // To publish commands to robots
    ros::Publisher pub = n_.advertise<geometry_msgs::TwistStamped>(
      get_ros_params::getStringParam("teach_motions/ee" + std::to_string(arm_index) + "/jog_cmd_topic", n_),
      1);
    velocity_pubs_.push_back( pub );
  }

  // Sleep to allow the publishers to be created and FT data to stabilize
  ros::Duration(2.).sleep();
}

void compliant_replay::CompliantReplay::readTraj()
{
  std::string path = ros::package::getPath("teach_motions");

  std::string line, value;

  // Read vectors for each arm
  for (int arm_index=0; arm_index<num_arms_; ++arm_index)
  {
    std::ifstream input_file( path + "/data/" + input_file_ + "_arm" + std::to_string(arm_index) + "_processed.csv" );

    // Ignore the first line (headers)
    getline( input_file, line);

    while ( input_file.good() )
    {
      // Read a whole line
      getline( input_file, line);

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


int main(int argc, char** argv)
{
  ros::init(argc, argv, "compliant_replay");

  compliant_replay::CompliantReplay compliant_replay;
  ros::spin();

  return 0;
}
