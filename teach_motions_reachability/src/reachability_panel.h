#ifndef REACHABILITY_PANEL_H
#define REACHABILITY_PANEL_H

#ifndef Q_MOC_RUN

#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <stdio.h>
#include <string.h>
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

namespace teach_motions_reachability
{

struct arm_pose_info {
  geometry_msgs::PoseStamped change_in_pose;
  std::string frame_id;
  std::string move_group_name;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_ptr;
};

class ReachabilityPanel: public rviz::Panel
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
  ReachabilityPanel( QWidget* parent = 0 );

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

  // Preview the trajectory when clicked.
  void previewTrajectory();

  // Then we finish up with protected member variables.
protected:
  // One-line text editor for entering the datafile name.
  QLineEdit* file_prefix_editor_;

  // The current file prefix, entered by user.
  QString file_prefix_;

  ros::NodeHandle nh_;

  // Pairs of arm indexes and PoseStampeds.
  // There may be 2 arms or just one, and the order isn't certain.
  std::vector< teach_motions_reachability::arm_pose_info > arm_datas_;

  // Preview trajectory button
  QPushButton* preview_button_;

  // MoveIt! requires an asynch spinner
  ros::AsyncSpinner spinner_;

  tf::TransformListener listener_;
};

} // end namespace teach_motions_reachability

#endif // REACHABILITY_PANEL_H