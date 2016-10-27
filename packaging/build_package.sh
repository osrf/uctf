#!/bin/bash

WS=`pwd`/ws
INSTALL=/opt/sasc-foo
REPOS=./gazebo_uctf.rosinstall

if test -d ${INSTALL}; then
  echo "Error: Install directory ${INSTALL} exists. Please delete it and try again."
  exit 1
fi

mkdir -p ${WS}/src

echo "Cloning from ${REPOS} into ${WS}/src..."
rosinstall ${WS}/src ${REPOS}
(cd ${WS}/src/uctf && git submodule update --init --recursive)

echo "Downloading pacakge.xml files for Gazebo pacakges..."
curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_gazebo.xml > ${WS}/src/gazebo/package.xml
curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_ign-math.xml > ${WS}/src/ign-math/package.xml
curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_ign-msgs.xml > ${WS}/src/ign-msgs/package.xml
curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_ign-tools.xml > ${WS}/src/ign-tools/package.xml
curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_sdformat.xml > ${WS}/src/sdformat/package.xml

echo "Installing dependencies with rosdep..."
. /opt/ros/kinetic/setup.bash
rosdep install --from-path ${WS}/src --ignore-src -y|| true

echo "Building Gazebo and friends..."
cd ${WS}
catkin config --init --extend /opt/ros/kinetic -i ${INSTALL} --install --isolate-devel
sudo mkdir -p ${INSTALL}
sudo chown -R ${USER}:${USER} ${INSTALL}
catkin build
cp -r ${WS}/src/gazebo_models ${INSTALL}/share

echo "Cloning Ardupilot..."
git clone https://github.com/tfoote/ardupilot.git -b uctf-dev
echo "Building Ardupilot..."
cd ardupilot
export PATH=${PATH}:${WS}/ardupilot/Tools/autotest
./Tools/scripts/install-prereqs-ubuntu.sh
git submodule update --init --recursive
./waf configure --prefix=${INSTALL}
./waf
./waf install
