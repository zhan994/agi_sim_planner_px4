# adjust mavros odom
rosservice call /mavros/set_message_interval 31 100 & sleep 3;
rosservice call /mavros/set_message_interval 32 100 & sleep 3;
# run ego
roslaunch plan_manage kino_replan_in_gazebo.launch & sleep 10;
roslaunch px4ctrl run_node.launch & sleep 10;
rosrun rqt_reconfigure rqt_reconfigure & sleep 10;
roslaunch plan_manage rviz.launch