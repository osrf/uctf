#!/usr/bin/env sh

arguments=$1
#todo actually call the executables in a known path
# sim_vehicle.py ${arguments}
/home/tfoote/uctf-ardu/ardupilot/build/sitl/bin/arducopter-quad -S -I0 --home -35.363261,149.165230,584,353 --model gazebo-iris --speedup 10 --defaults /home/tfoote/uctf-ardu/ardupilot/Tools/autotest/default_params/gazebo-iris.parm
