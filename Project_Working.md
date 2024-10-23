## Path Planning
- The tb3_nav2_params.yaml file inside navigation_tb3 config folder from the ROS2 Nav2 autonomous navigation tutorial holds changeable parameters for the path planning.
    - There are a few key areas: this shit can be found on the nav2 wiki, under config guide or tuning guide
        - amcl:
        - local costmap:
            - can change the width and height of the rolling window to change the area that the path planner uses to calculate the path. The bigger the area the more things are taken into account when doing the planning, however the larger the area the greater the computational complexity.
            - can change the inflation radius (also change the global costmap) which affects how the path planner sees obstacles and it can now drive closer to them, has a smaller radial obstacle avoidance the smaller the inflation radius


- Exploration steps:
    1. Mapping Run: Map the whole shop to create that layer around the objects that we will then be clearing throughout the second run. 
        1.1: Begin with nothing but a blank space, take in lidar scans and place waypoints on areas where the scan picks up the furthest explorable points.
            1.1.2: Can use lidar scan at maybe 60 points in a 360 degree radius and place the waypoints at any of those points that read max lidar scan. If there are two or more points within a certain radius of each other, place one waypoint in the centroid of that space


- Running everything:
    1. ros2 launch turtlebot3_gazebo open_track_maze.launch.py 
    2. /opt/ros/humble/share/nav2_bringup/rviz$ rviz2 -d nav2_default_view.rviz use_sim_time:=TRUE
    3. ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=TRUE
    4. ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=TRUE
    5. run turtlebot3_shop_navigator and turtlebot3_shop_keeper
    6. if i run ros2 launch nav2_bringup bringup.launch.py map:=map.yaml, thats why the localisation doesn't come up as active

- I FIGURED OUT WHERE THE turtlebot3_navigation2 navigation2.launch.py params files come from - ~/turtlebot3_ws/src/turtlebot3/turlebot3_navigation2/params/burger.yaml


