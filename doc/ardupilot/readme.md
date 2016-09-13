# Ardupilot integration

This document explains how to get and build the right versions of Gazebo and Ardupilot for use with U-CTF.
It's very much a work in progress and is primarily intended to assist the internal developer team to get up to speed.

## Install prerequisites

* Follow the Gazebo build-from-source instructions to [install prerequisites](http://gazebosim.org/tutorials?tut=install_from_source&cat=install#InstallRequiredDependencies), [build and install ignition math](http://gazebosim.org/tutorials?tut=install_from_source&cat=install#BuildAndInstallignitionmath), and [build and install sdformat](http://gazebosim.org/tutorials?tut=install_from_source&cat=install#BuildAndInstallSDFormat) (don't actually download and install Gazebo here; we'll do that below to get a particular branch).

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

## Start ArduPilot/ArduPlane

~~~
export PATH=$PATH:[path to ardupilot]]/Tools/autotest
sim_vehicles.py -f GazeboIris -S 1000
~~~

~~~
export PATH=$PATH:[path to ardupilot]]/Tools/autotest
sim_vehicles.py -f GazeboZephyr -S 1000
~~~


