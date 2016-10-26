# Ardupilot integration

This document explains how to get and build the right versions of Gazebo and Ardupilot for use with U-CTF.
It's very much a work in progress and is primarily intended to assist the internal developer team to get up to speed.


## Get the code

Create a place to work:
~~~
export SRC_SPACE=~/uctf-ardu/src

mkdir -p ${SRC_SPACE}
~~~


Create a file `/tmp/gazebo_uctf.repos` with this content:
~~~
repositories:
  gazebo:
    type: hg
    url: ssh://hg@bitbucket.org/osrf/gazebo
    version: ardupilot
  gazebo_models:
    type: hg
    url: ssh://hg@bitbucket.org/osrf/gazebo_models
    version: zephyr_demos
  gazebo_ros_pkgs:
    type: git
    url: git@github.com:ros-simulation/gazebo_ros_pkgs.git
    version: kinetic-devel
  ign-math:
    type: hg
    url: https://bitbucket.org/ignitionrobotics/ign-math
    version: default
  ign-msgs:
    type: hg
    url: ssh://hg@bitbucket.org/ignitionrobotics/ign-msgs
    version: default
  ign-tools:
    type: hg
    url: ssh://hg@bitbucket.org/ignitionrobotics/ign-tools
    version: default
  sdformat:
    type: hg
    url: https://bitbucket.org/osrf/sdformat
    version: default
  uctf:
    type: git
    url: git@github.com:osrf/uctf
    version: add_ardupilot
~~~

Check out gazebo and get on the right branch:
~~~
vcs import --input /tmp/gazebo_uctf.repos ~/uctf-ardu/src
~~~

Fetch package.xml files:
~~~
curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_gazebo.xml > ${SRC_SPACE}/gazebo/package.xml
curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_ign-math.xml > ${SRC_SPACE}/ign-math/package.xml
curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_ign-msgs.xml > ${SRC_SPACE}/ign-msgs/package.xml
curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_ign-tools.xml > ${SRC_SPACE}/ign-tools/package.xml
curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_sdformat.xml > ${SRC_SPACE}/sdformat/package.xml
~~~

## Install prerequisites

~~~
rosdep install --from-path ${SRC_SPACE} --ignore-src
~~~

Additional new dependency needed is `libqwt-dev`.
~~~
sudo apt-get install libqwt-dev python-future
~~~

## Build the workspace

~~~
cd ${SRC_SPACE}
. /opt/ros/kinetic/setup.bash
catkin build
~~~

## Checkout Ardupilot

Check out ardupilot from a fork and get on the right branch:
~~~
cd ~/uctf-ardu
git clone https://github.com/tfoote/ardupilot.git -b uctf-dev
cd ardupilot
~~~

### Build ArduPilot SITL

~~~
export PATH=$PATH:~/uctf-ardu/ardupilot/Tools/autotest
cd ardupilot
./Tools/scripts/install-prereqs-ubuntu.sh
git submodule update --init --recursive
./waf configure
./waf
~~~

# Run Simulation


## Terminal setup

In each terminal instructed to open run the following commands.

~~~
. ~/uctf-ardu/devel/setup.bash
. ~/uctf-ardu/devel/share/gazebo-8/setup.sh
. ~/uctf-ardu/devel/share/uctf/setup.sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/uctf-ardu/src/gazebo_models
~~~

## Start Gazebo

Open a terminal source the above and run:

~~~
roslaunch ~/uctf-ardu/devel/share/uctf/launch/uctf.launch
~~~

## Launch Drones

Blue team in a new terminal 

~~~
spawn_blue --launch --delete 1 2 26 27
~~~

Gold team in a new terminal
~~~
spawn_gold --launch --delete 1 2 26 27
~~~

## Ground control
If you'd like to see your drones in a Ground Control Station. 

For the blue team connect to port: `14000`
For the gold team connect to port: `14001`

You can use `qgroundcontrol` or `apmplanner2`



# Deprecated Instructions


The run one of the below simulations
~~~
gazebo --verbose worlds/iris_arducopter_demo.world
~~~

~~~
gazebo --verbose worlds/zephyr_ardupilot_demo.world
~~~

### Start ArduPilot/ArduPlane
`-S` parameter will speedup the simulation
~~~
export PATH=$PATH:~/uctf-ardu/ardupilot/Tools/autotest
sim_vehicle.py -f gazebo-iris -S 10 -v ArduCopter -m --mav10
~~~

#### Test fly the vehicle manually
Inside the mavproxy you can now try flying it. 

The following will raise the ekf thresholds to account for simulation errors.
And the 2nd line will remove the timeout on being armed, otherwise you need to type very fast.

```
param set FS_EKF_THRESH 1
param set DISARM_DELAY 0
```

Switch to a GPS mode (simpler to command)
Now you can takeoff when simulated GPS is ready (about 10s)
```
mode guided
arm throttle
takeoff 5
```

~~~
export PATH=$PATH:~/uctf-ardu/ardupilot/Tools/autotest
sim_vehicle.py -f gazebo-zephyr -S 10 -v ArduPlane -m --mav10
~~~
