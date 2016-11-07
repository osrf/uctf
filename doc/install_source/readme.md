# Build package from source

For this tutorial we will assume you are still using ROS and Gazebo from binary packages.
Therefore first follow the instructions from the section *Install dependencies* in the [Install binary packages](../install_binary/readme.md) documentation.

If you want to look into building any of them from source please follow their tutorials ([ROS](http://wiki.ros.org/kinetic/Installation/Source), [Gazebo](http://gazebosim.org/tutorials?tut=install_from_source)).


## Get the code

Create a place to work:
~~~
export SRC_SPACE=~/uctf-ardu/src

mkdir -p ${SRC_SPACE}
~~~


Create a file `/tmp/gazebo_uctf.rosinstall` with this content:
~~~
- tar:
    local-name: gazebo
    uri: https://bitbucket.org/osrf/gazebo/get/594248df0a49.tar.gz
    version: osrf-gazebo-594248df0a49
- tar:
    local-name: gazebo_models
    uri: https://bitbucket.org/osrf/gazebo_models/get/ca17c6407082.tar.gz
    version: osrf-gazebo_models-ca17c6407082
- git:
    local-name: gazebo_ros_pkgs
    uri: git@github.com:ros-simulation/gazebo_ros_pkgs.git
    version: kinetic-devel
- hg:
    local-name: ign-math
    uri: https://bitbucket.org/ignitionrobotics/ign-math
    version: default
- hg:
    local-name: ign-msgs
    uri: https://bitbucket.org/ignitionrobotics/ign-msgs
    version: default
- hg:
    local-name: ign-tools
    uri: https://bitbucket.org/ignitionrobotics/ign-tools
    version: default
- hg:
    local-name: ign-tools
    uri: https://bitbucket.org/ignitionrobotics/ign-tools
    version: default
- hg:
    local-name: sdformat
    uri: https://bitbucket.org/osrf/sdformat
    version: default
- git:
    local-name: uctf
    uri: git@github.com:osrf/uctf
    version: add_ardupilot

~~~

If you're developing use this for gazebo instead of the tarball.
~~~
tar:
   local-name: gazebo
   uri: ssh://hg@bitbucket.org/osrf/gazebo
   version: ardupilot
~~~



Check out gazebo and get on the right branch:
~~~
rosinstall ~/uctf-ardu/src /tmp/gazebo_uctf.rosinstall
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
cd ${SRC_SPACE}/..
catkin config --init --extend /opt/ros/kinetic -i /opt/sasc --install --isolate-devel
sudo mkdir -p /opt/sasc
sudo chown -R $USER:$USER /opt/sasc
(cd src/uctf && git submodule update --init --recursive)
catkin build
cp -r ~/uctf-ardu/src/gazebo_models /opt/sasc/share
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
./waf configure --prefix=/opt/sasc
./waf
./waf install
~~~

### Install mavproxy

You will need mavproxy installed from pip on your system.
~~~
sudo pip install mavproxy
~~~

## Terminal setup

To use this installation in each terminal instructed to open run the following commands.

~~~
. /opt/sasc/setup.bash
. /opt/sasc/share/gazebo-8/setup.sh
. /opt/sasc/share/uctf/setup.sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/sasc/share/gazebo_models
~~~

---

Next: [Run an example](../run_example/readme.md).
