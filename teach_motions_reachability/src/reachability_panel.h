/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef TELEOP_PANEL_H
#define TELEOP_PANEL_H

#ifndef Q_MOC_RUN

#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <stdio.h>
#include <string.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
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
  std::string move_group;
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

  // The current file prefix.
  QString file_prefix_;

  ros::NodeHandle nh_;

  // Pairs of arm indexes and PoseStampeds.
  // There may be 2 arms or just one, and the order isn't certain.
  std::vector< teach_motions_reachability::arm_pose_info > arm_datas_;

  // Preview trajectory button
  QPushButton* preview_button_;
};

} // end namespace teach_motions_reachability

#endif // TELEOP_PANEL_H