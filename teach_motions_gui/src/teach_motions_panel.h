#ifndef TEACH_MOTIONS_PANEL_H
#define TEACH_MOTIONS_PANEL_H

#ifndef Q_MOC_RUN

#include <actionlib/client/simple_action_client.h>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <stdio.h>
#include <sensor_msgs/JointState.h>
#include <string.h>
#include <teach_motions/CompliantReplayAction.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <utility>

#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>

#endif

class QLineEdit;

namespace teach_motions_gui
{

struct arm_pose_info {
  geometry_msgs::PoseStamped change_in_pose;
  std::string frame_id;
  std::string move_group_name;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_ptr;
};

class TeachMotionsPanel: public rviz::Panel
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  TeachMotionsPanel( QWidget* parent = 0 );

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

  // Next come a couple of public Qt slots.
public Q_SLOTS:
  // In this example setTopic() does not get connected to any signal
  // (it is called directly), but it is easy to define it as a public
  // slot instead of a private function in case it would be useful to
  // some other user.
  void readChangeInPose( const QString& topic );

  // Here we declare some internal slots.
protected Q_SLOTS:
  // updateFilePrefix() reads the file name from the QLineEdit and calls
  // readChangeInPose() with the result.
  void updateFilePrefix();

  // Slot to preview the trajectory
  void previewTrajectory();

  // Slot to execute the trajectory
  void executeTrajectory();

  // Slot to cancel trajectory execution
  void cancelExecution();

  // Then we finish up with protected member variables.
protected:
  // Temporarily disable buttons
  void disableButtons();
  void enableButtons();

  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    joint_states_.name = msg->name;
    joint_states_.position = msg->position;
  }

  // One-line text editor for entering the datafile name.
  QLineEdit* file_prefix_editor_;

  // The current file prefix, entered by user.
  QString file_prefix_;

  ros::NodeHandle nh_;

  // Pairs of arm indexes and PoseStampeds.
  // There may be 2 arms or just one, and the order isn't certain.
  std::vector< teach_motions_gui::arm_pose_info > arm_datas_;

  // Preview trajectory button
  QPushButton *preview_button_, *execute_button_, *cancel_button_;

  // MoveIt! requires an asynch spinner
  ros::AsyncSpinner spinner_;

  // Send service requests to trigger compliant motions
  ros::ServiceClient compliant_replay_client_;

  tf::TransformListener listener_;

  // Need this joint_states msg to display trajectories properly
  ros::Subscriber joint_states_sub_;
  sensor_msgs::JointState joint_states_;

  actionlib::SimpleActionClient<teach_motions::CompliantReplayAction> action_client_;
};

} // end namespace teach_motions_gui

#endif // TEACH_MOTIONS_PANEL_H