# Start your SASC round

1. Signup with a Google account at https://cloudsim.io.
1. Register for SASC. Wait for OSRF to approve your user as a SASC admin.
1. Log in to https://cloudsim.io/
1. Click on the SASC link (on the left menu bar, under the Dashboard).

SASC machines are grouped into Rounds. Click on the round button (with a + inside) to create a new round. 
You must supply a name for the Round, along with a usernames for each team.

## Launch the simulation machine
1. Click on the LAUNCH button in the Arbiter box. This will launch a simulation cloud machine. This process takes about 2 minutes, and you should see the Status and machine ip update. You should also see a button appear to download the ssh key for the simulation machine. Given that key, you can ssh into the machine as user `ubuntu` (e.g., `ssh -i cloudsim.pem ubuntu@1.2.3.4`)

## Launch the payload machines
1. You can launch a payload machine for each team by pressing the blue or gold LAUNCH A PAYLOAD button.
1. After a few second, payload information (local IP address) and ssh download links should appear.
1. VPN: you can connect to a payload machine using a VPN. There are up to 5 openvpn keys that you can download for each team, named ocu-0 to ocu-4. Each OCU key bundle contains the necessary client configuration file and a private key. It is important not to use the same key on multiple OCUs (2 simultaneous connections with the same key will not work). To start the VPN, unpack the bundle that you downloaded and then run `openvpn --config openvpn.conf`.
1. After connecting to the VPN and downloading the SSH key for the Payload machine, you can ssh to its local IP address as user `ubuntu` (e.g., `ssh -i cloudsim.pem ubuntu@192.168.2.10`).

## Finish the round
1. Once you are finished with the round, press the FINISH ROUND button to terminate the machines (simulation and payloads). Machines are billed for each hour they are running.

# Run your team in the cloud


## On first login to all machines

```console
sudo apt-get update
sudo apt-get install sasc-gazebo-sitl
```

## On the Arbiter machine

### Launch Gazebo
```console
export INSTALL_SPACE=/opt/sasc
. ${INSTALL_SPACE}/setup.bash
. ${INSTALL_SPACE}/share/gazebo-8/setup.sh
. ${INSTALL_SPACE}/share/uctf/setup.sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${INSTALL_SPACE}/share/gazebo_models
export ROS_HOSTNAME=192.168.2.1
export ROS_MASTER_URI=http://192.168.2.1:11311
roslaunch uctf uctf.launch gui:=false

```

### Launch Arbiter

SSH with X-Forwarding to the arbiter machine (e.g., `ssh -XC -i cloudsim.pem ubuntu@1.2.3.4`).
```console
export INSTALL_SPACE=/opt/sasc
. ${INSTALL_SPACE}/setup.bash
. ${INSTALL_SPACE}/share/gazebo-8/setup.sh
. ${INSTALL_SPACE}/share/uctf/setup.sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${INSTALL_SPACE}/share/gazebo_models
. ${INSTALL_SPACE}/venv3/bin/activate
PYTHONPATH=${INSTALL_SPACE}/lib/python2.7/dist-packages:/opt/ros/kinetic/lib/python2.7/dist-packages:/usr/lib/python3/dist-packages arbiter_start.py -db br-blue -dr br-gold
```

## On each Payload machine

To spawn a blue plane from the blue Payload machine:
```console
export INSTALL_SPACE=/opt/sasc
. ${INSTALL_SPACE}/setup.bash
. ${INSTALL_SPACE}/share/gazebo-8/setup.sh
. ${INSTALL_SPACE}/share/uctf/setup.sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${INSTALL_SPACE}/share/gazebo_models
spawn_blue 1 26 --acs br-blue --gazebo-ip 192.168.2.1 --local-ip 192.168.2.10
```

To spawn a gold plane from the gold Payload machine:
```console
export INSTALL_SPACE=/opt/sasc
. ${INSTALL_SPACE}/setup.bash
. ${INSTALL_SPACE}/share/gazebo-8/setup.sh
. ${INSTALL_SPACE}/share/uctf/setup.sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${INSTALL_SPACE}/share/gazebo_models
spawn_gold 1 26 --acs br-gold --gazebo-ip 192.168.3.1 --local-ip 192.168.3.10
```
## On OCU run fti.py or qgroundcontrol

Run fti.py or qgroundcontrol using the VPN tunnel.
Each drone has a unique ROS_MASTER_URI port. `11311 + MAV_SYS_ID`
