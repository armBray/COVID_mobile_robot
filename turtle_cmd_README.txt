!!!!!!!!
SE LA PRIMA COMPILAZIONE FALLISCE (causa detect.h), 
RICOMPILARE UNA SECONDA VOLTA
!!!!!!!!

______________________________________
WALL FOLLOWER

export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_house.launch 
------- THEN:
Do this:
    ----
    export TURTLEBOT3_MODEL=burger
    roslaunch exploration_pkg slam_nav.launch algorithm_name:=WF
    ---
    rosrun exploration_pkg wall_follower.py
or this:
    ---
    roslaunch exploration_pkg wall_follower.launch


---- a mappa completata
rosrun map_server map_saver -f src/turtlebot3_navigator/exploration_pkg/map/map

_______________________________________
FRONTIER

export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_house.launch 
------- THEN:
Do this:
    export TURTLEBOT3_MODEL=burger
    roslaunch exploration_pkg slam_nav.launch algorithm_name:=FE
    ---
    rosrun exploration_pkg controller.py 
    ---
    rosrun exploration_pkg frontier_exploration.py
or this:
    ---
    roslaunch exploration_pkg wall_follower.launch

---- a mappa completata
rosrun map_server map_saver -f src/turtlebot3_navigator/exploration_pkg/map/map

_______________________________________
NAVIGATION

export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_house.launch 
-------
the turtlebot has to lie in (-3,1) otherwise use:
roslaunch exploration_pkg spawn.launch 
-------
export TURTLEBOT3_MODEL=burger
roslaunch autonomous_nav aut_nav.launch first_launch:=true
-------
roslaunch autonomous_nav send_goals.launch

_______________________________________
COVID NAVIGATION

export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_house.launch 
-------
the turtlebot has to lie in (-3,1) otherwise use:
roslaunch exploration_pkg spawn.launch 
-------
export TURTLEBOT3_MODEL=burger
roslaunch autonomous_nav aut_nav.launch first_launch:=true
-------
roslaunch autonomous_nav covid_send_goals.launch
-------
rosrun autonomous_nav covid.py




















