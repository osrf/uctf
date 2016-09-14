# Ardupilot integration

This document explains how to get and build the right versions of Gazebo and Ardupilot for use with U-CTF.
It's very much a work in progress and is primarily intended to assist the internal developer team to get up to speed.

## Install prerequisites

* Follow the Gazebo build-from-source instructions to [install prerequisites](http://gazebosim.org/tutorials?tut=install_from_source&cat=install#InstallRequiredDependencies), [build and install ignition math](http://gazebosim.org/tutorials?tut=install_from_source&cat=install#BuildAndInstallignitionmath), and [build and install sdformat](http://gazebosim.org/tutorials?tut=install_from_source&cat=install#BuildAndInstallSDFormat) (don't actually download and install Gazebo here; we'll do that below to get a particular branch).
* Note: you also need to build `ign-msgs` as you did `ign-math`. It's missing from the Gazebo tutorial: [ticket](https://bitbucket.org/osrf/gazebo_tutorials/issues/66/add-ign-msgs-to-build-from-source).

Additional new dependency needed is `libqwt-dev`.

## Get the code

Create a place to work:
~~~
mkdir ~/uctf-ardu
~~~

Check out gazebo and get on the right branch:
~~~
cd ~/uctf-ardu
hg clone ssh://hg@bitbucket.org/osrf/gazebo
cd gazebo
hg checkout ardupilot
~~~

Check out gazebo_models and get on the right branch:
~~~
cd ~/uctf-ardu
hg clone ssh://hg@bitbucket.org/osrf/gazebo_models
cd gazebo_models
hg checkout zephyr_demos
~~~

Check out ardupilot from a fork and get on the right branch:
~~~
cd ~/uctf-ardu
git clone https://github.com/hsu/ardupilot.git
cd ardupilot
git checkout gazebo_sitl
~~~

## Build everything

### Build ArduPilot SITL

~~~
export PATH=$PATH:[path to ardupilot]]/Tools/autotest
cd ardupilot
./Tools/scripts/install-prereqs-ubuntu.sh
git submodule checkout --init --recursive
(cd ArduCopter && make sitl)
(cd ArduPlane && make sitl)
~~~

## Run ArduPilot

### Start Gazebo

~~~
gazebo --verbose worlds/iris_arducopter_demo.world
~~~

~~~
gazebo --verbose worlds/zephyr_ardupilot_demo.world
~~~


### Start ArduPilot/ArduPlane

~~~
export PATH=$PATH:[path to ardupilot]]/Tools/autotest
sim_vehicles.py -f GazeboIris -S 1000 -v ArduCopter
~~~

#### Test fly the vehicle manually
Inside the mavproxy you can now try flying it. 

The following will raise the ekf thresholds to account for simulation errors.
And the 2nd line will remove the timeout on being armed, otherwise you need to type very fast.

```
param set FS_EKF_THRESH 1
param set DISARM_DELAY 0
```

Now you can takeoff
```
arm throttle
takeoff 5
```

~~~
export PATH=$PATH:[path to ardupilot]]/Tools/autotest
sim_vehicles.py -f GazeboZephyr -S 1000 -v ArduPlane
~~~


