# Run your team in the cloud


On first login to all machines

```console
sudo apt-get update
sudo apt-get install sasc-gazebo-sitl
```

## On the Server


### Launch Gazebo
```
export INSTALL_SPACE=/opt/sasc
. ${INSTALL_SPACE}/setup.bash
. ${INSTALL_SPACE}/share/gazebo-8/setup.sh
. ${INSTALL_SPACE}/share/uctf/setup.sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${INSTALL_SPACE}/share/gazebo_models
roslaunch uctf uctf.launch gui:=false

```


### Launch Arbiter

SSH with X-Forwarding to the arbiter machine.
```
export INSTALL_SPACE=/opt/sasc
. ${INSTALL_SPACE}/setup.bash
. ${INSTALL_SPACE}/share/gazebo-8/setup.sh
. ${INSTALL_SPACE}/share/uctf/setup.sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${INSTALL_SPACE}/share/gazebo_models
. /opt/sasc-dev/venv3/bin/activate
PYTHONPATH=/opt/sasc/lib/python2.7/dist-packages:/opt/ros/kinetic/lib/python2.7/dist-packages:/usr/lib/python3/dist-packages arbiter_start.py -db br-blue -dr br-gold
```


## On payload spawn vehicles

parameterize acs_network_inteface local-ip gazebo-ip

```
spawn_blue 1 26 --acs br-blue --gazebo-ip 192.168.2.1 --local-ip 192.168.2.10
```

```
spawn_gold 1 26 --acs br-gold --gazebo-ip 192.168.3.1 --local-ip 192.168.3.10
```

## On OCU run fti.py or qgroundcontrol

Run fti.py or qgroundcontrol using the VPN tunnel.
Each drone has a unique ROS_MASTER_URI port. `11311 + MAV_SYS_ID`
