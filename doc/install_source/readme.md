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

~~~
#!/bin/bash

set -o errexit


WORKSPACE=~/uctf-ardu
INSTALL_SPACE=/opt/sasc-dev


mkdir -p $WORKSPACE/src
cd $WORKSPACE/src && git clone https://github.com/osrf/uctf.git
$WORKSPACE/src/uctf/packaging/download.bash $WORKSPACE

rm -rf $INSTALL_SPACE
$WORKSPACE/src/uctf/packaging/build_and_install.bash $WORKSPACE $INSTALL_SPACE


~~~


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
