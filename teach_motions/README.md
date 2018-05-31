**Usage:**

Save robot pose info when the robot is moved:

`roslaunch teach_motions robotname_teach_motions.launch filename:=cabinet_door1`


Fit the pose datapoints with splines, take regularly-spaced derivatives, and save that derivative data into the /data folder:

`roslaunch teach_motions robotname_postprocess.launch filename:=cabinet_door1`


Prepare to send the recorded trajectories to robot(s):

`roslaunch teach_motions robotname_prepare_to_send_cmds.launch`

Load the parameters of the motion:

`roslaunch teach_motions robotname_set_parameters.launch filename:=cabinet_door1`

Make a service call to actually trigger the motion:

rosservice call compliant_replay "file_prefix: data: 'cabinet_door1'"

Or, do this all via the teach_motions_gui.