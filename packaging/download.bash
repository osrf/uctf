#!/bin/bash

set -o errexit

if [ "$1" != "" ]; then
  WS=$1
else
  WS=`pwd`/ws
fi


SCRIPTDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPOS=${SCRIPTDIR}/gazebo_uctf.rosinstall
OTHER_REPOS=${SCRIPTDIR}/other.rosinstall

# TODO(tfoote) add proper certs for NPS servers
export GIT_SSL_NO_VERIFY=true

mkdir -p ${WS}/src

echo "Cloning from ${REPOS} into ${WS}/src..."
if [ -e ${WS}/src/.rosinstall ]; then
  wstool merge --merge-replace -t ${WS}/src ${REPOS} 
  wstool update -t ${WS}/src
else
  wstool init ${WS}/src ${REPOS}
fi
(cd ${WS}/src/uctf && git submodule update --init --recursive)

echo "Downloading pacakge.xml files for Gazebo packages..."
curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_gazebo.xml > ${WS}/src/gazebo/package.xml
curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_ign-math.xml > ${WS}/src/ign-math/package.xml
curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_ign-msgs.xml > ${WS}/src/ign-msgs/package.xml
curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_ign-tools.xml > ${WS}/src/ign-tools/package.xml
curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_sdformat.xml > ${WS}/src/sdformat/package.xml

mkdir -p ${WS}/other_src
echo "Cloning from ${OTHER_REPOS} into ${WS}/other_src..."
if [ -e ${WS}/other_src/.rosinstall ]; then
  wstool merge --merge-replace -t ${WS}/other_src ${OTHER_REPOS}
  wstool update -t ${WS}/other_src
else
  wstool init ${WS}/other_src ${OTHER_REPOS}
fi

cd ${WS}/other_src/ardupilot
git submodule update --init --recursive

cd ${WS}/other_src/mavlink
git submodule update --init --recursive

echo "Installing dependencies with rosdep..."
. /opt/ros/kinetic/setup.bash
rosdep install --from-path ${WS}/src --ignore-src -y|| true
echo "Installing other dependencies..."
sudo apt-get install -y libxslt1-dev libqwt-dev python-future libignition-transport-dev python-netifaces python-paramiko python3-espeak

# Things for venv3
sudo apt-get install -y python3-django python3-netifaces python3-numpy python3-pil python3-pyqt5 python3-urllib3

# things for venv
sudo apt-get install -y python-numpy python-jinja2
