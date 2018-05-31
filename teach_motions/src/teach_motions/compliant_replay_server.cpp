#include "ros/ros.h"
#include "teach_motions/compliant_replay.h"
#include "teach_motions/RequestMotion.h"

bool requestMotion(teach_motions::RequestMotion::Request  &req,
         teach_motions::RequestMotion::Response &res)
{
  ROS_INFO_STREAM(req.file_prefix.data);

  compliant_replay::CompliantReplay replay_it;

  res.successful.data = true;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "compliant_replay_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("compliant_replay", requestMotion);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  while( ros::ok() )
    ros::Duration(0.01).sleep();

  return 0;
}