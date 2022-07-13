# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/noetic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/noetic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in '/home/ninad/git_ws/AuE8230Spring22_ntarare/catkin_ws/devel;/home/ninad/catkin_ws/devel_isolated/turtlebot3_simulations;/home/ninad/catkin_ws/devel_isolated/turtlebot3_gazebo;/home/ninad/catkin_ws/devel_isolated/turtlebot3_autorace_driving;/home/ninad/catkin_ws/devel_isolated/turtlebot3_autorace_msgs;/home/ninad/catkin_ws/devel_isolated/turtlebot3_autorace_detect;/home/ninad/catkin_ws/devel_isolated/turtlebot3_autorace_core;/home/ninad/catkin_ws/devel_isolated/turtlebot3_autorace_camera;/home/ninad/catkin_ws/devel_isolated/turtlebot3_autorace_2020;/home/ninad/catkin_ws/devel_isolated/raspicam_node;/home/ninad/catkin_ws/devel_isolated/apriltag_ros;/home/ninad/catkin_ws/devel;/opt/ros/noetic'.split(';'):
        python_path = os.path.join(workspace, 'lib/python3/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/ninad/ninad_turtlebot3/devel/env.sh')

output_filename = '/home/ninad/ninad_turtlebot3/build/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    # print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
