#!/bin/bash

set -o errexit

if [ "$1" != "" ]; then
  WS=$1
else
  WS=`pwd`/ws
fi

if [ "$2" != "" ]; then
  INSTALL_SPACE=$2
else
  INSTALL_SPACE=/opt/sasc
fi

echo "Building Gazebo and friends..."
cd ${WS}
catkin config --init --extend /opt/ros/kinetic -i ${INSTALL_SPACE} --install --isolate-devel
sudo mkdir -p ${INSTALL_SPACE}
sudo chown -R ${USER}:${USER} ${INSTALL_SPACE}
catkin build --verbose

# Install Gazebo models
mkdir -p ${INSTALL_SPACE}/share/gazebo_models
for MODEL in iris_with_standoffs iris_with_standoffs_demo gimbal_small_2d zephyr_delta_wing zephyr_delta_wing_ardupilot_demo sun ground_plane
do
  rsync -av ${WS}/src/gazebo_models/${MODEL} ${INSTALL_SPACE}/share/gazebo_models
done



# echo "Cloning Ardupilot..."
# git clone https://github.com/tfoote/ardupilot.git -b uctf-dev
echo "Building Ardupilot..."
cd ${WS}/other_src/ardupilot
./waf configure --prefix=${INSTALL_SPACE}
./waf
./waf install

echo "installing lxml needed for mavlink"
pip install lxml future pyserial --system --target=${INSTALL_SPACE}/lib/python2.7/dist-packages/ --install-option="--install-scripts=${INSTALL_SPACE}/bin"


echo "Installing mavlink"
cd ${WS}/other_src/mavlink/pymavlink
PYTHONPATH=$PYTHONPATH:${INSTALL_SPACE}/lib/python2.7/dist-packages python setup.py install --prefix=${INSTALL_SPACE} --install-layout=deb

echo "Installing mavproxy"
cd ${WS}/other_src/mavproxy
PYTHONPATH=$PYTHONPATH:${INSTALL_SPACE}/lib/python2.7/dist-packages python setup.py install --prefix=${INSTALL_SPACE} --install-layout=deb


# echo "installing qgroundcontrol"
# wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage -O ${INSTALL_SPACE}/bin/qgroundcontrol
# chmod +x ${INSTALL_SPACE}/bin/qgroundcontrol

echo "Installing arbiter and dependencies"

VENV3=${INSTALL_SPACE}/venv3
mkdir -p ${VENV3}
pyvenv --system-site-packages ${VENV3} 
(. ${VENV3}/bin/activate && pip3 install wheel)
(. ${VENV3}/bin/activate && pip3 install future image mavproxy netifaces catkin-pkg rospkg)

(. ${VENV3}/bin/activate && cd ${WS}/other_src/acs_lib && python setup.py install)
(. ${VENV3}/bin/activate && cd ${WS}/other_src/acs_dashboards && python setup.py install)
(. ${VENV3}/bin/activate && cd ${WS}/other_src/arbiter && python setup.py install)
(. ${VENV3}/bin/activate && cd ${WS}/other_src/mavlink/pymavlink && python setup.py install)
(. ${VENV3}/bin/activate && cd ${WS}/src/autonomy-payload/ap_lib && python setup.py install)
(. ${VENV3}/bin/activate && cd ${WS}/other_src/swarmcommander && python setup.py install)

echo "Installing Game Director and dependencies"

GDI_VENV=${INSTALL_SPACE}/ugdi_venv
mkdir -p ${GDI_VENV}
virtualenv --system-site-packages ${GDI_VENV}
(. ${GDI_VENV}/bin/activate && cd ${WS}/other_src/ugdi && pip install -r requirements.txt )
(. ${GDI_VENV3}/bin/activate && cd ${WS}/other_src/acs_lib && python setup.py install)
(. ${GDI_VENV}/bin/activate && cd ${WS}/src/autonomy-payload/ap_lib && python setup.py install)
cp -r ${WS}/other_src/ugdi ${GDI_VENV}
