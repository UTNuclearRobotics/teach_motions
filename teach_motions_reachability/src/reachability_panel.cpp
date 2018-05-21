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

#include <stdio.h>

#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>

#include "reachability_panel.h"

namespace teach_motions_reachability
{

// Here is the implementation of the ReachabilityPanel class.  ReachabilityPanel
// has these responsibilities:
//
// - Act as a container for GUI elements QLineEdit and LED indicators.
// - Saving and restoring internal state from a config file.
//
// We start with the constructor, doing the standard Qt thing of
// passing the optional *parent* argument on to the superclass
// constructor
ReachabilityPanel::ReachabilityPanel( QWidget* parent )
  : rviz::Panel( parent )
{
  // Next we lay out the "file_prefix" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  QHBoxLayout* file_prefix_layout = new QHBoxLayout;
  file_prefix_layout->addWidget( new QLabel( "File Prefix (e.g. 'cabinet_door1'):" ));
  file_prefix_editor_ = new QLineEdit;
  file_prefix_layout->addWidget( file_prefix_editor_ );

  // Lay out the topic field above the control widget.
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout( file_prefix_layout );
  setLayout( layout );

  // Next we make signal/slot connections.
  connect( file_prefix_editor_, SIGNAL( editingFinished() ), this, SLOT( updateFilePrefix() ));
}

// Read the topic name from the QLineEdit and call setFilePrefix() with the
// results.  This is connected to QLineEdit::editingFinished() which
// fires when the user presses Enter or Tab or otherwise moves focus
// away.
void ReachabilityPanel::updateFilePrefix()
{
  setFilePrefix( file_prefix_editor_->text() );
}

// Set the topic name we are publishing to.
void ReachabilityPanel::setFilePrefix( const QString& new_file_prefix )
{
  // Only take action if the name has changed.
  if( new_file_prefix != file_prefix_ )
  {
    file_prefix_ = new_file_prefix;
    ROS_INFO_STREAM( file_prefix_.toStdString() );

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