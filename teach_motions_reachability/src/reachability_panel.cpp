#include "reachability_panel.h"

namespace teach_motions_reachability
{

// Here is the implementation of the ReachabilityPanel class.  ReachabilityPanel
// has these responsibilities:
//
// - Act as a container for GUI elements.
// - Saving and restoring internal state from a config file.
//
// We start with the constructor, doing the standard Qt thing of
// passing the optional *parent* argument on to the superclass
// constructor
ReachabilityPanel::ReachabilityPanel( QWidget* parent )
  : rviz::Panel( parent ), spinner_(1)
{
  // Next we lay out the "file_prefix" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  QHBoxLayout* file_prefix_layout = new QHBoxLayout;
  file_prefix_layout->addWidget( new QLabel( "File Prefix (e.g. 'cabinet_door1'):" ));
  file_prefix_editor_ = new QLineEdit;
  file_prefix_layout->addWidget( file_prefix_editor_ );

  // Stack stuff vertically
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout( file_prefix_layout, Qt::AlignTop );
  preview_button_ = new QPushButton( tr("Preview") );
  layout->addWidget( preview_button_, Qt::AlignTop );
  // Push other things towards the top with this expanding spacer
  layout->addStretch();
  setLayout( layout );

  // Next we make signal/slot connections.
  connect( file_prefix_editor_, SIGNAL( editingFinished() ), this, SLOT( updateFilePrefix() ));
  connect( preview_button_, SIGNAL( clicked() ), this, SLOT( previewTrajectory() ) );

  spinner_.start();
}

// Read the file name from the QLineEdit and call readChangeInPose() with the
// results.  This is connected to QLineEdit::editingFinished() which
// fires when the user presses Enter or Tab or otherwise moves focus
// away.
void ReachabilityPanel::updateFilePrefix()
{
  readChangeInPose( file_prefix_editor_->text() );
}

// Preview the trajectory when clicked.
void ReachabilityPanel::previewTrajectory()
{
  // Preview for each arm:
  // Get the current pose, add the change in pose from the datafile.
  // Plan a cartesian move to that target pose.
  // Visualize it in RViz.

  ros::Publisher display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;
  // The display requires this start state
  moveit_msgs::RobotState start_state;

  // TODO: check if current_pose and change_in_pose are in the same frame
  for (int arm_index=0; arm_index<arm_datas_.size(); arm_index++)
  {
    // Calculate the new target pose
    geometry_msgs::PoseStamped current_pose = arm_datas_.at(arm_index).move_group_ptr -> getCurrentPose();
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back( current_pose.pose );

    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = current_pose.header.frame_id;
    target_pose.pose.position.x = current_pose.pose.position.x + arm_datas_.at(arm_index).change_in_pose.pose.position.x;
    target_pose.pose.position.y = current_pose.pose.position.y + arm_datas_.at(arm_index).change_in_pose.pose.position.y;
    target_pose.pose.position.z = current_pose.pose.position.z + arm_datas_.at(arm_index).change_in_pose.pose.position.z;

    tf::Quaternion q_current, q_incremental, q_final;
    quaternionMsgToTF(current_pose.pose.orientation, q_current);
    quaternionMsgToTF(arm_datas_.at(arm_index).change_in_pose.pose.orientation, q_incremental);
    q_final = q_incremental*q_current;
    quaternionTFToMsg(q_final, target_pose.pose.orientation);

    // Plan to the new target pose
    waypoints.push_back( target_pose.pose );
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = arm_datas_.at(arm_index).move_group_ptr -> computeCartesianPath(waypoints, 0.005, 0.0, trajectory);

    ROS_INFO("Cartesian path %.2f%% achieved", fraction * 100.0);

    // Prepare to display the trajectory
    start_state.joint_state.name.insert( start_state.joint_state.name.end(), trajectory.joint_trajectory.joint_names.begin(), trajectory.joint_trajectory.joint_names.end() );
    // Add the joints to the initial state
    std::vector<double> initial_joints = arm_datas_.at(arm_index).move_group_ptr -> getCurrentJointValues();
    start_state.joint_state.position.insert( start_state.joint_state.position.end(), initial_joints.begin(), initial_joints.end() );

    display_trajectory.trajectory.push_back(trajectory);
  }

  display_trajectory.trajectory_start = start_state;
  display_publisher.publish(display_trajectory);
  // This sleep seems to be necessary to visualize the trajectories reliably
  ros::Duration(0.1).sleep();
}

// If user inputs new text, update the pose data
void ReachabilityPanel::readChangeInPose( const QString& new_file_prefix )
{
  // Only take action if the filename has changed.
  if( new_file_prefix != file_prefix_ )
  {
    file_prefix_ = new_file_prefix;

    // Clear the vector of arm data
    arm_datas_.clear();

    // There may only be one arm.
    std::string path = ros::package::getPath("teach_motions");
    std::string line, value;

    for (int arm_index=0; arm_index<2; arm_index++)
    {
      std::ifstream file( path + "/data/net_motion/" + file_prefix_.toStdString() + "_arm" + std::to_string(arm_index) + "_net_motion.csv" );
      if ( file.fail() )
      {
        ROS_INFO_STREAM("No datafile found for arm " << arm_index);
        continue;
      }

      // Record this arm's data in a member vector
      arm_pose_info arm_info;

      ROS_INFO_STREAM("Reading pose data for arm " << arm_index);
      geometry_msgs::PoseStamped pose;

      // Ignore the first line (headers)
      getline( file, line);

      while ( file.good() )
      {
        // Read a whole line
        getline( file, line);
        std::stringstream ss(line);

        // Read each value
        // Reject the first value in each row (it's a label)
        getline(ss, value, ',');

        if (value != "")  // Check for end of file
        {
          // Get change in pose
          pose.pose.position.x = std::stod( value );

          getline(ss, value, ',');
          pose.pose.position.y = std::stod( value );

          getline(ss, value, ',');
          pose.pose.position.z = std::stod( value );

          // Convert the rpy to quaternion
          getline(ss, value, ',');
          double roll = std::stod( value );

          getline(ss, value, ',');
          double pitch = std::stod( value );

          getline(ss, value, ',');
          double yaw = std::stod( value );

          tf::Quaternion q_tf = tf::createQuaternionFromRPY(roll, pitch, yaw);

          geometry_msgs::Quaternion q_msg;
          quaternionTFToMsg(q_tf , q_msg);
          pose.pose.orientation = q_msg;

          // Get MoveGroup
          getline(ss, value, ',');
          arm_info.move_group_name = value;
          // Create a new movegroup and store a pointer to it. Use shared_ptr so we don't worry about deleting it.
          arm_info.move_group_ptr.reset( new moveit::planning_interface::MoveGroupInterface(arm_info.move_group_name) );

          // Get frame_id
          getline(ss, value, ',');
          arm_info.frame_id = value; 
          pose.header.frame_id = value;   
          arm_info.change_in_pose = pose;    
        }
      }

      arm_datas_.push_back( arm_info );

      //ROS_INFO_STREAM( arm_datas_.back().change_in_pose );
      //ROS_INFO_STREAM( arm_datas_.back().frame_id );
      //ROS_INFO_STREAM( arm_datas_.back().move_group_name );
    }

    // rviz::Panel defines the configChanged() signal.  Emitting it
    // tells RViz that something in this panel has changed that will
    // affect a saved config file.  Ultimately this signal can cause
    // QWidget::setWindowModified(true) to be called on the top-level
    // rviz::VisualizationFrame, which causes a little asterisk ("*")
    // to show in the window's title bar indicating unsaved changes.
    Q_EMIT configChanged();
  }
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void ReachabilityPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "FilePrefix", file_prefix_ );
}

// Load all configuration data for this panel from the given Config object.
void ReachabilityPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString file_prefix;
  if( config.mapGetString( "FilePrefix", &file_prefix ))
  {
    file_prefix_editor_->setText( file_prefix );
    updateFilePrefix();
  }
}

} // end namespace teach_motions_reachability

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(teach_motions_reachability::ReachabilityPanel,rviz::Panel )