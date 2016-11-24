# Start your SASC round

1. Signup with a Google account at https://dev.cloudsim.io
1. Register for SASC. Wait for OSRF to approve your user as a SASC admin.
1. Log in to https://cloudsim.io/
1. Click on the SASC link (on the left menu bar, under the Dashboard).

SASC machines are grouped into Rounds. Click on the round button (with a + inside) to create a new round. 
You must supply a name for the Round, along with a usernames for each team.

1. Click on the LAUNCH button in the Arbiter box. This will launch a simulatopn cloud machine. This process takes about 2 minutes, and you should see the Status and machine ip update. You should also see a button appear to download the ssh key for the simulation machine.

1. You can launch a payload machine for each team by pressing the blue or gold LAUNCH A PAYLOAD button.
1. After a few second, payload information (local ip address) and ssh download links should appear.
1. Once you are finished with the round, press the FINISH ROUND button to kill the machines (simulation and payloads)


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
