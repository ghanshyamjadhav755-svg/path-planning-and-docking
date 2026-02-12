# ROS2 Humble -- Path Planning & Docking (Simulation)

🎥 Demo Videos
Path Planning (Nav2 + Regulated Pure Pursuit Controller)

https://youtu.be/1kwWOEIahR0?si=epQHa5F5BqMiKqGi

AprilTag-Based Docking

https://youtu.be/pg-8IbTYcNY?si=yKqTrpMgpfvJQGif
## Clone & Build

``` bash
mkdir -p ~/amr_ws/src
cd ~/amr_ws/src
git clone https://github.com/ghanshyamjadhav755-svg/path-planning-and-docking.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

------------------------------------------------------------------------

## Basic Project Structure

    .
    ├── apriltag_ros
    │   └── launch / cfg / src
    │
    └── my_bot
        ├── config            → Nav2 parameters
        ├── description       → URDF
        ├── launch            → Simulation / Localization / Navigation
        ├── map               → Dock.yaml
        ├── scripts           → route_follower.py / docking.py
        ├── worlds            → Dock.world
        └── models

------------------------------------------------------------------------

## Nav2 Parameters Location

To upgrade or tune navigation:

    my_bot/config/nav2_params.yaml

------------------------------------------------------------------------

## Docking Logic Location

Docking scripts:

    my_bot/scripts/docking.py
    my_bot/scripts/docking_nav2.py

------------------------------------------------------------------------

# Execution Steps

## 1️⃣ Launch Simulation (Dock World Only)

``` bash
ros2 launch my_bot launch_sim.launch.py world:=/home/jd/dev_ws/src/my_bot/worlds/Dock.world
```

Designed for `Dock.world`.\
If using another world, update the world path.

------------------------------------------------------------------------

## 2️⃣ Localization (Dock Map Only)

``` bash
ros2 launch my_bot localization_launch.py map:=/home/jd/dev_ws/src/my_bot/map/Dock.yaml use_sim_time:=true
```

Designed for `Dock.yaml`.\
If using another map, update the map path.

------------------------------------------------------------------------

## 3️⃣ Start Navigation

``` bash
ros2 launch my_bot navigation_launch.py use_sim_time:=true
```

------------------------------------------------------------------------

# Path Following (Route Execution)

``` bash
python3 /home/jd/dev_ws/src/my_bot/scripts/route_follower.py
```

Behavior: - Executes predefined path - Runs forward - Automatically
reverses path - Loops continuously (forward + reverse)

If script does not execute:

``` bash
chmod +x /home/jd/dev_ws/src/my_bot/scripts/route_follower.py
```

### Changing Path

Edit:

    my_bot/scripts/route_follower.py

Modify waypoint coordinates directly inside the script.

If changing map: - Update coordinates accordingly - Ensure consistency
with selected map

------------------------------------------------------------------------

# Docking (Demo Mode)

Do NOT run docking together with route_follower.

Reason: - Route script already contains dock pose in path - Running both
causes conflict

------------------------------------------------------------------------

## Send Robot to Dock

``` bash
ros2 topic pub --once /dock_control std_msgs/String "data: 'dock'"
```

Undock:

``` bash
ros2 topic pub --once /dock_control std_msgs/String "data: 'undock'"
```

Docking works only in: - Dock.world - Dock.yaml

------------------------------------------------------------------------

# Development / Upgrade

For upgrades:

-   Nav2 tuning → `my_bot/config/nav2_params.yaml`
-   Dock alignment → `scripts/docking.py`
-   Route logic → `scripts/route_follower.py`
-   New maps → update launch paths + route coordinates
-   New world → update world path in launch command

Project validated for Dock.world and Dock.yaml.
