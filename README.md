**Usage:**

Save robot pose info when the robot is moved:

roslaunch teach_motions teach_motions.launch

Fit the pose datapoints with splines, take regularly-spaced derivatives, and save that derivative data into the /data folder:

roslaunch teach_motions postprocess.launch

Prepare to send the recorded trajectories to robot(s):

roslaunch teach_motions prepare_to_send_cmds.launch

Finally, actually send the recorded trajectories to robot(s):

rosrun teach_motions replay
