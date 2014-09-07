# Husky Pursuit
The husky_pursuit package implements Pursuit and Evasion behaviors for ROS Robots and simulation.
The project was built during the summer of 2014 at the Naval Postgraduate School


## Description of project contents
    ─ scripts-----------------------------------Meat of the package
      ==================================Behaviors:
      ├── pure_seek.py--------------------------Move toward a goal relative position based on input (distance, bearing)
      ├── pure_flee.py--------------------------Move away given a RelativePosition (distance, bearing)
      ├── pursue.py-----------------------------Extrap
      ├── evade.py
      ├── offset_pursue.py
      ├── serpentine.py-------------------------Weave back and forth
      ├── wander.py-----------------------------Seek a random position
      ==================================
      ├── encoder_frame_remapper.py-------------Remap from encoder to odom, changing the frame id and child frame id for RViz vizualization of odometry
      ├── pose2d_to_odometry.py-----------------Wraps geometry_msgs/Pose2D as nav_msgs/Odometry for use with robot_pose_ekf (Pose2D from laser_scan_matcher)
      ├── safety_controller.py
      ├── tf_to_relative.py
      ├── utils.py------------------------------Mathematical utility methods
      ==================================Gun interface and robot control:
      ├── phidget_driver.py---------------------Uses phidgets.py. Subs to status/fire and fires on true. Pubs status/heath based on digital input.
      ├── phidgets.py---------------------------Uses the Phidget Python API to expose an interface kit object (Phidget 8/8/8 IO board)
      ├── shooter.py----------------------------Accepts RelativePosition messages and auto-fires when tolerances are met
      ├── manual_shooter.py---------------------Manually shoot with joystick trigger as long as the joy topic is published
      ├── leap_control.py-----------------------Control of the robot with the Leap Motion. Uses the official Python SDK
      ==================================Older files, not used/finished:
      └── get_data.py---------------------------Unused. Gets transform data from gazebo, stage etc. Replaced with tf_to_relative.py and RelativePosition.
      ├── goal_publisher.py---------------------Skeleton from original navigation stack architecture
      ├── state_publisher.py
      ├── model_spawner.sh
      ├── gmapping.bash
    ─ launch
      ├── amcl.launch
      ├── bringup.launch
      ├── camera
      │   ├── ar_track.launch-------------------
      │   └── camera.launch---------------------UVC camera launch. Can be included in launch/game/robot.launch, commented out as imagery is currently mostly unused
      ├── desktop
      │   ├── desktop.launch
      │   ├── sim.launch
      │   └── viz.launch
      ├── game----------------------------------Physical robot launch files. Indentation represents include structure, but this directory is flat.
      │   ├── system.launch---------------------Main launch for physical robot usage. Run this on your local machine after launching roscore on robot_0
      │   ├──    1 husky.machine
      │   ├──    2 arbiter.launch---------------Sets up initial "arena" with static transforms from world to robot_<num>/base_footprint. The format of this is args="x y z roll pitch yaw"
      │   ├──        1 dashboard.launch
      │   ├──    3 robot.launch----------------------At the moment, the camera include is commented out to avoid bogging down the network, but is functional if included.
      │   ├──        1 base.launch
      │   ├──    4 tf_to_relative.launch
      │   ├──    5 safety_controller.launch
      │   ├──    6 behaviors.launch
      │   ├──    7 gun.launch
      ├── gmapping.launch
      ├── gmapping_scan_matcher.launch
      ├── laser_splitter.launch
      ├── lms5xx.launch
      ├── localization.launch
      ├── pose.launch
      ├── scan_matcher.launch
      ├── simul
      │   ├── behaviors-------------------------Testbed launches comparing behaviors in simulation. <0 behavior>_<1 behavior>[_<2 behavior>].launch. Run in concert with stage.launch
      │   │   ├── evade_control_flee.launch
      │   │   ├── evade_pursue.launch
      │   │   ├── evade_pursue_wander.launch
      │   │   ├── flee_wander.launch
      │   │   ├── offset_pursue_control.launch
      │   │   ├── pursue_control.launch
      │   │   ├── pursue_control_seek.launch
      │   │   ├── pursue_serpentine.launch
      │   │   ├── seek_control_flee.launch
      │   │   ├── seek_serpentine_flee.launch
      │   │   ├── seek_serpentine.launch
      │   │   └── seek_wander.launch
      │   ├── control.launch
      │   ├── four_sim.launch
      │   ├── one_husky.launch
      │   ├── robots2.launch
      │   ├── robots4.launch
      │   ├── robots.launch
      │   ├── spawn_model.launch
      │   ├── stage.launch
      │   ├── stage_sky.launch
      │   ├── two_sim.launch
      │   └── unified_control.launch
      └── teleop
          ├── key_teleop.launch-----------------Depends on installation of the kobuki-keyop package (robot specific, but velocities are remapped to the husky topic).
          ├── joystick.launch-------------------Drive husky and shoot laser tag guns with a joystick. Includes manual shooter.py. To drive, hold the deadman button, and to shoot press the trigger.
          └── leap_teleop.launch----------------
    ─ maps--------------------------------------Maps generated with Gmapping at the Naval Postgraduate School
          ├── <map files>
    ─ msg
      └── RelativePosition.msg------------------Message used by the pure_seek, pure_flee and shooter nodes. Generated by tf_to_relative or <build your own new localization node>!
    ─ rviz
      ├── huskies.rviz--------------------------Physical robot system vizualization config. Used in launch/game/dashboard.launch to launch RViz.
      ├── navigation.rviz
      ├── robot_0.perspective-------------------rqt ('rosrun rqt_gui rqt_gui' or 'rqt') configuration
      └── stage.rviz----------------------------Simuation visualization configuration for 3 robots for RViz
    ─ worlds------------------------------------Stage world definitions
        ├── sky.world-----------------------------For mocked wing UAVs. More just experimental
        ├── square.png
        └── square.world--------------------------Main world simulation setup. Open, but boxed playing field with 3 robots (0, 1, 2)
    ─ setup.py
    ─ package.xml
    ─ README.md
    
    12 directories, 114 files

## General usage
### Stage behavior simulation
Set up your environment variables (`ROS_HOSTNAME` and `ROS_MASTER_URI`) for a local master as per the ROS wiki.
To toggle to a local master for just the current terminal, run
    export ROS_HOSTNAME=`hostname -I`; export ROS_MASTER_URI=http://$ROS_HOSTNAME:11311
If you followed the NPS ARSENL Yoda wiki for Robotic Laser tag to set up your environment variables, you should that aliased to `local_ros`, which I recommend you set if you are going to be toggling between physical and simulated robots. Otherwise, feel free to set those variables to localhost for pure simulation.

Launch Stage in one terminal (automatically starts roscore):
    roslaunch husky_pursuit stage.launch

Run a behavior in another terminal:
    roslaunch husky_pursuit seek_control_flee.launch
See the `Description of project contents` section for the game behavior launches. With a control launch, use a joystick plugged into your computer.

### Physical robot
First:
    ssh ros@192.168.1.125
    roscore
Then:
    roslaunch husky_pursuit system.launch
You can select the behaviors in system.launch when including behaviors.launch, as well as configure max linear and angular speed.

Recommended - also run `rqt_console` to monitor log messages and possibly `rqt_graph` to view node and topic connections.

## Future work
These are potential future work opportunities, but are not all-inclusive by any means.

### Localization

AR tag localization  
Computer vision  
* Color
* Feature recognition
LIDAR movement detection  
Fuse multiple outputs together - on loss of one (eg bad orientation), still have data. Also comp. viz might give a bearing, but not distance, and still making use of that

### Hardware

Automatic reloading of laser tag guns  
Designing a custom tag system  
Mounting laser tag guns and receivers on robots properly  
Measure actual bearing, yaw tolerances

### Behaviors

Movement manager
* Take relay responsibility from safety controller  
  * Safety controller instead continues to publish status  
  * Movement manager sucks in status and then responds (eg relaying, avoiding)  
* Compose behaviors, steering behaviors together, priority  

Obstacle avoidance
Arrival
Pursuit and Evasion on physical robots
Gazebo individual control launch file (straightforward, I just first need to fix some issue with four_sim.launch having robots fall through the ground)

### Simulation testing - testing different behaviors against each other

* Set up testing argument files - eg:
  * seek_vs_flee.csv: <pre>
max_lin, max_ang, max_lin, max_ang
0.5, 0.3, 0.3, 0.5
0.6, 0.2, 0.3, 0.5
...
</pre>
  * Simulate phidget_driver.py damage
  * Measure/compare time to destruction. Really testing deltas in velocity limits and the performance of behaviors/composed behaviors

### Architecture
Convert system to use rocon-multimaster for hydro.  
Dynamic reconfigure/parameter server support for nodes, especially the behaviors.  
Split behaviors off into a steering_behaviors package - this is a specialized usage of that

## Troubleshooting
### Joystick not working
If you have a CH Products flightstick, try running:
    sudo lsusb -v
Check joystick output with:
	jstest /dev/input/js0
