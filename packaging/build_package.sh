#!/bin/bash

WS=`pwd`/ws
INSTALL_SPACE=/opt/sasc
REPOS=./gazebo_uctf.rosinstall

if test -d ${INSTALL_SPACE}; then
  echo "Error: Install directory ${INSTALL_SPACE} exists. Please delete it and try again."
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
echo "Installing other dependencies..."
sudo apt-get install -y libxslt1-dev libqwt-dev python-future libignition-transport-dev

echo "Building Gazebo and friends..."
cd ${WS}
catkin config --init --extend /opt/ros/kinetic -i ${INSTALL_SPACE} --install --isolate-devel
sudo mkdir -p ${INSTALL_SPACE}
sudo chown -R ${USER}:${USER} ${INSTALL_SPACE}
catkin build
cp -r ${WS}/src/gazebo_models ${INSTALL_SPACE}/share

echo "Cloning Ardupilot..."
git clone https://github.com/tfoote/ardupilot.git -b uctf-dev
echo "Building Ardupilot..."
cd ardupilot
export PATH=${PATH}:${WS}/ardupilot/Tools/autotest
# ./Tools/scripts/install-prereqs-ubuntu.sh
git submodule update --init --recursive
./waf configure --prefix=${INSTALL_SPACE}
./waf
./waf install

echo "Installing mavproxy"
pip install mavproxy --system --target=${INSTALL_SPACE}/lib/python/site-packages/ --install-option="--install-scripts=${INSTALL_SPACE}/bin"

echo "installing qgroundcontrol"

wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage -O /opt/sasc/bin/qgroundcontrol
chmod +x /opt/sasc/bin/qgroundcontrol

echo "generating control file"

cp ${WS}/src/uctf/packaging/sasc-control.base ${WS}/sasc-control
echo -n "Files:" >> ${WS}/sasc-control
find -L /opt/sasc -type f | xargs -I {} echo " {} /" >> ${WS}/sasc-control
cd ${WS}
equivs-build ${WS}/sasc-control
