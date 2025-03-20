# adjust mavros odom
rosservice call /mavros/set_message_interval 31 100 & sleep 3;
rosservice call /mavros/set_message_interval 32 100 & sleep 3;
# run ego
roslaunch ego_planner single_run_in_gazebo.launch & sleep 10;
roslaunch px4ctrl run_node.launch & sleep 10;
rosrun rqt_reconfigure rqt_reconfigure & sleep 10;
roslaunch ego_planner rviz.launch