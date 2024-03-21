
//////// To enter workspace ////////
cd Robot_project_ws/

//////// Used these commands everytimes before use every commands ////////
colcon build
source install/setup.bash 

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

<------------------------------------ For use robot in manual mode ------------------------------------>
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////// For open gazebo with robot model and world ///////
ros2 launch articubot_one launch_sim.launch.py world:=/home/aleecha/Robot_project_ws/src/articubot_one/worlds/Project.world
/////// For open teleop twist keyboard ////////
ros2 run articubot_one robot_control.py
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
--------------------------------------------------------------------------------------------------------

<------------------------------------ For use robot in auto mode ------------------------------------>
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////// For open gazebo with robot model and world ///////
ros2 launch articubot_one launch_sim.launch.py world:=/home/aleecha/Robot_project_ws/src/articubot_one/worlds/Project.world
/////// For open rviz2 ///////
rviz2
/////// For use twist_mux //////
ros2 run twist_mux twist_mux --ros-args --params-file ./src/articubot_one/config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel_unstamped
/////// For use slamtooolbox //////
ros2 launch slam_toolbox online_async_launch.py params_file:=./src/articubot_one/config/mapper_params_online_async.yaml use_sime_time:=true
/////// For use nav2 /////////
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
------------------------------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
