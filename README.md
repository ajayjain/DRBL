# Husky Pursuit
The husky_pursuit package implements Pursuit and Evasion behaviors for ROS Robots and simulation.
The project was built during the summer of 2014 at the Naval Postgraduate School


## Description of project contents
    ─ scripts-----------------------------------Meat of the package
      ==================================Behaviors:
      ├── evade.py
      ├── pure_seek.py
      ├── pure_flee.py
      ├── pursue.py
      ├── offset_pursue.py
      ├── serpentine.py
      ├── wander.py
      ==================================
      ├── encoder_frame_remapper.py-------------Remap from encoder to odom, changing the frame id and child frame id for RViz vizualization of odometry
      ├── pose2d_to_odometry.py-----------------Wraps geometry_msgs/Pose2D as nav_msgs/Odometry for use with robot_pose_ekf (Pose2D from laser_scan_matcher)
      ├── safety_controller.py
      ├── state_publisher.py
      ├── tf_to_relative.py
      ==================================Gun interface:
      ├── phidget_driver.py---------------------Uses phidgets.py. Subs to status/fire and fires on true. Pubs status/heath based on digital input.
      ├── phidgets.py---------------------------Uses the Phidget Python API to expose an interface kit object (Phidget 8/8/8 IO board)
      ├── shooter.py----------------------------Accepts RelativePosition messages and auto-fires when tolerances are met
      ├── manual_shooter.py---------------------Manually shoot with joystick trigger as long as the joy topic is published
      ==================================
      ├── utils.py------------------------------Mathematical utility methods
      ├── leap_control.py-----------------------Control of the robot with the Leap Motion. Uses the official Python SDK
      ==================================Older files, not used/finished:
      ├── goal_publisher.py---------------------Skeleton from original navigation stack architecture
      ├── model_spawner.sh
      ├── gmapping.bash
      └── get_data.py---------------------------Unused. Gets transform data from gazebo, stage etc. Replaced with tf_to_relative.py and RelativePosition.
    ─ launch
      ├── amcl.launch
      ├── bringup.launch
      ├── camera
      │   ├── ar_track.launch
      │   └── camera.launch
      ├── desktop
      │   ├── desktop.launch
      │   ├── sim.launch
      │   └── viz.launch
      ├── game----------------------------------Physical robot launch files. Indentation represents include structure, but this directory is flat.
      │   ├── system.launch---------------------Main component for physical robot usage.
      │   ├──    1 husky.machine
      │   ├──    2 arbiter.launch
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

## Future work
These are potential future work opportunities, but are not all-inclusive by any means.
### Architecture
Convert system to use rocon-multimaster for hydro.
Split behaviors off into a steering_behaviors package - this is a specialized usage of that