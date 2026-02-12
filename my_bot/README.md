## to launch gazebo with custom world
ros2 launch my_bot launch_sim.launch.py world:=/home/jd/dev_ws/src/my_bot/worlds/Dock.world 

#### localization
ros2 launch my_bot localization_launch.py map:=/home/jd/dev_ws/src/my_bot/map/Dock.yaml use_sim_time:=true

#### Navigation
ros2 launch my_bot navigation_launch.py use_sim_time:=true

#### Follow Path
python3 /home/jd/dev_ws/src/my_bot/scripts/route_follower.py  


##### nav2_DOCK 

ros2 topic pub --once dock_control std_msgs/String "data: 'dock'"
ros2 topic pub --once dock_control std_msgs/String "data: 'undock'"

## mapping with slam_toolbox
ros2 launch my_bot online_async_launch.py






##### DEVLOPMENT 
######################## APTIL_TAG_ALIGN 
python3 /home/jd/dev_ws/src/my_bot/scripts/docking.py

ros2 topic pub --once /align_dock/command std_msgs/String "data: 'dock_0'" --once

ros2 topic pub /align_dock/command std_msgs/String "data: 'undock'" --once

python3 /home/jd/dev_ws/src/my_bot/scripts/docking_nav2.py 