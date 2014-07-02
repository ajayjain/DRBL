#!/bin/bash
echo "Sleeping..."
sleep 4
echo "Spawning..."
rosrun gazebo_ros spawn_model $1 -unpause -urdf -param robot_description -model $2