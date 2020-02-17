# Packages to be uploaded to the robot
This folder contains custom made packages to setup a robot for autonomous navigation. It is necessary to install additional packages from external repositories.
1. clover_driver contains the driver package to command the robot and manage the communication with Arduino. It makes use of additional packages that need to be installed like:
   joy, teleop_twist_joy, rosserial_arduino and rosserial. A special thanks to **danielsnider** for his simple_drive repository from which I took some scripts to multiplex command velocities
2. clover_odometry contains packages and nodes to retrieve odometry information. Odometry is obtained by merging with an Extended Kalman Filter package encoders odometry (read through Arduino) and Laser Scan Matching odometry.
   In particular, Laser Scan Matching odometry is obtained with the following packages: *laser_proc*, *urg_c*, *urg_node* and *rf2o_laser_odometry* (credits to **MAPIRlab**).
   Extended Kalman Filter is implemented with the *robot_localization* package.
3. clover_navigation is entirely based on the *ROS Navigation Stack* with the additional package *m-explore*
