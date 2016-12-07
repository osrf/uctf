# Build package from source

For this tutorial we will assume you are still using ROS and Gazebo from binary packages.
Therefore first follow the instructions from the section *Install dependencies* in the [Install binary packages](../install_binary/readme.md) documentation.

If you want to look into building any of them from source please follow their tutorials ([ROS](http://wiki.ros.org/kinetic/Installation/Source), [Gazebo](http://gazebosim.org/tutorials?tut=install_from_source)).

## Avoid conflicts with system-installed gazebo and related packages

We have seen cases in which the build procedure described here can inadvertently cross-talk with a "regular" installation of gazebo and its support packages in `/usr`. We'll work on preventing that situation, but in the meantime we recommend avoiding it by removing any such packages from your system:
~~~
sudo apt-get remove gazebo* libignition* libsdformat*
~~~
If you experience mysterious segmentation faults, especially on startup, it's likely that the cause is cross-talk with your system-installed packages.

## Get the code

Create a place to work:
~~~
export SRC_SPACE=~/uctf-ardu/src
export INSTALL_SPACE=/opt/sasc-dev
mkdir -p ${SRC_SPACE}
~~~


Create a file `gazebo_uctf.rosinstall` with this command: 

~~~
wget https://github.com/osrf/uctf/raw/master/packaging/gazebo_uctf.rosinstall -O ${SRC_SPACE}/../gazebo_uctf.rosinstall
~~~

~~~
wget https://github.com/osrf/uctf/raw/master/packaging/other.rosinstall -O ${SRC_SPACE}/../other.rosinstall
~~~

### Download Source

Check out gazebo and get on the right branch:
~~~
wstool init ${SRC_SPACE} ${SRC_SPACE}/../gazebo_uctf.rosinstall
~~~

Fetch package.xml files:
~~~
curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_gazebo.xml > ${SRC_SPACE}/gazebo/package.xml
curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_ign-math.xml > ${SRC_SPACE}/ign-math/package.xml
curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_ign-msgs.xml > ${SRC_SPACE}/ign-msgs/package.xml
curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_ign-tools.xml > ${SRC_SPACE}/ign-tools/package.xml
curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_sdformat.xml > ${SRC_SPACE}/sdformat/package.xml
~~~

~~~
wstool init ${SRC_SPACE}/../other_src ${SRC_SPACE}/../other.rosinstall
~~~


## Install prerequisites

~~~
. /opt/ros/kinetic/setup.bash
rosdep update
rosdep install --from-path ${SRC_SPACE} --ignore-src
~~~

Add some other dependencies
~~~
sudo apt-get install libxslt1-dev libqwt-dev python-future libignition-transport-dev
~~~

## Build the workspace

~~~
cd ${SRC_SPACE}/..
catkin config --init --extend /opt/ros/kinetic -i ${INSTALL_SPACE} --install --isolate-devel
sudo mkdir -p ${INSTALL_SPACE}
sudo chown -R $USER:$USER ${INSTALL_SPACE}
(cd ${SRC_SPACE}/uctf && git submodule update --init --recursive)
catkin build
ln -s ${SRC_SPACE}/gazebo_models ${INSTALL_SPACE}/share/gazebo_models
~~~

## Checkout Ardupilot

Check out ardupilot from a fork and get on the right branch:
~~~
cd ${SRC_SPACE}/..
git clone https://github.com/tfoote/ardupilot.git -b uctf-dev
cd ardupilot
~~~

### Build ArduPilot SITL

~~~
export INSTALL_SPACE=/opt/sasc-dev
export SRC_SPACE=~/uctf-ardu/src
cd ${SRC_SPACE}/../ardupilot
./Tools/scripts/install-prereqs-ubuntu.sh
git submodule update --init --recursive
./waf configure --prefix=${INSTALL_SPACE}
./waf
./waf install
~~~

### Install mavproxy

You will need mavproxy installed from pip on your system.
~~~
pip install mavproxy --system --target=${INSTALL_SPACE}/lib/python/site-packages/ --install-option="--install-scripts=${INSTALL_SPACE}/bin"
~~~

### installing qgroundcontrol

```
wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage -O ${INSTALL_SPACE}/bin/qgroundcontrol
chmod +x ${INSTALL_SPACE}/bin/qgroundcontrol
```

### Arbiter

Since the arbiter uses Python 3 we setup a virtual environment:

#### Python 3 venv

```console
export VENV3=${INSTALL_SPACE}/venv3
mkdir -p ${VENV3}
pyvenv ${VENV3}
(. ${VENV3}/bin/activate && pip install wheel)
(. ${VENV3}/bin/activate && pip install image mavproxy netifaces numpy pyqt5 urllib3)

(. ${VENV3}/bin/activate && cd ${WS}/other_src/acs_lib && python setup.py install)
(. ${VENV3}/bin/activate && cd ${WS}/other_src/acs_dashboards && python setup.py install)
(. ${VENV3}/bin/activate && cd ${WS}/other_src/arbiter && python setup.py install)
(. ${VENV3}/bin/activate && cd ${WS}/other_src/swarmcommander && python setup.py install)
```

Install the following packages into the venv too: 

* acs_lib
* acs_dashboards
* arbiter

## Terminal setup

To use this installation in each terminal instructed to open run the following commands.

~~~
export INSTALL_SPACE=/opt/sasc-dev
. ${INSTALL_SPACE}/setup.bash
. ${INSTALL_SPACE}/share/gazebo-8/setup.sh
. ${INSTALL_SPACE}/share/uctf/setup.sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${INSTALL_SPACE}/share/gazebo_models
~~~

---

Next: [Run an example](../run_example/readme.md).
