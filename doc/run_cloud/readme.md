# Game Master

## One-time setup
1. Sign up with a Google account at https://cloudsim.io.
1. Email `uctf-internal@osrfoundation.org` to request admin access. Wait to hear confirmation, then log out and back in.

## Login to cloudsim.io
1. Log in to https://cloudsim.io/
1. Click on the SASC link (on the left menu bar, under the Dashboard).

## Start a SASC round

SASC machines are grouped into Rounds. Click on the round button (with a + inside) to create a new round. 
You must supply a name for the Round, along with one competitor username for each team.

## Launch the simulation/arbiter machine
1. Click on the LAUNCH button in the Arbiter box. This will launch a simulation cloud machine. This process takes about 2 minutes, and you should see the Status and machine ip update. You should also see a button appear to download the ssh key for the simulation machine. Given that key, you can ssh into the machine as user `ubuntu` (e.g., `ssh -i cloudsim.pem ubuntu@1.2.3.4`)

### Get the latest SASC package
ssh to the simulation/arbiter machine and make sure that you have the latest SASC package installed:
```console
sudo apt-get update
sudo apt-get install sasc-gazebo-sitl
```
Note that, if there is a newer package available, it will take quite some time to download and install, due to its size. Please be patient.

### Launch Gazebo
ssh to the simulation/arbiter machine and start Gazebo like so:
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

SSH with X-Forwarding enabled to the simulation/arbiter machine (e.g., `ssh -XC -i cloudsim.pem ubuntu@1.2.3.4`) and start the Arbiter like so:
```console
export INSTALL_SPACE=/opt/sasc
. ${INSTALL_SPACE}/venv3/bin/activate
. ${INSTALL_SPACE}/setup.bash
arbiter_start.py -db br-blue -dr br-gold
```

## Launch the payload machines
1. You can launch a payload machine for each team by pressing the blue or gold LAUNCH A PAYLOAD button.
1. After a few seconds, payload information (local IP address) and ssh download links should appear.

## Start a game
At this point, everything is ready for the teams to spawn their vehicles. You
should tell them that the game is on, who is playing which team (blue vs.
gold), and that they are free to start spawning.

**NOTE**: this step might be tied into the Game Director.

## Stop a game
To stop a game, just give a Ctrl-C to the `roslaunch` and `arbiter_start.py` commands that you started.

**NOTE**: this step might be tied into the Game Director.

## Start a new game
You can reuse the machines from a round for multiple games. Just stop and start Gazebo and the Arbiter.

## Finish the round
1. Once you are finished with the round, press the FINISH ROUND button to terminate the machines (simulation and payloads). Machines are billed for each hour they are running.

# Competitor

## One-time setup
1. Sign up with a Google account at https://cloudsim.io.
1. Email `uctf-internal@osrfoundation.org` to request competitor access. Wait to hear confirmation, then log out and back in.

## Login to cloudsim.io
1. Log in to https://cloudsim.io/
1. Click on the SASC link (on the left menu bar, under the Dashboard).

## Connect your VPN client
1. You can connect to your payload machine using a VPN. There are up to 5 openvpn keys that you can download, named ocu-0 to ocu-4. Each OCU key bundle contains the necessary client configuration file and a private key. It is important not to use the same key on multiple OCUs (2 simultaneous connections with the same key will not work). To start the VPN, unpack the bundle that you downloaded and then run `openvpn --config openvpn.conf`.
1. After connecting to the VPN and downloading the SSH key for the Payload machine, you can ssh to its local IP address as user `ubuntu` (e.g., `ssh -i cloudsim.pem ubuntu@192.168.2.10`).

## Get the latest SASC package
With your VPN connected, ssh to your payload machine and make sure that you have the latest SASC package installed:
```console
sudo apt-get update
sudo apt-get install sasc-gazebo-sitl
```
Note that, if there is a newer package available, it will take quite some time to download and install, due to its size. Please be patient.

## Check out your tactics code
With your VPN connected, ssh to your payload machine and fetch your tactics code, e.g.:
```console
git clone https://gitlab.nps.edu/myfork/scrimmage-templates
```

## Spawn vehicle(s)
We're assuming that your tactic repo is checked out to `$HOME/scrimmage-templates`, with your tactics modules sitting inside there, in `plugins/autonomy/python`. In the examples below, we're assuming that the tactic that you want to run is in a class called `MyClass` that is implemented in a file called `mymodule.py` (which, for completeness, is located at `$HOME/scrimmage-templates/plugins/autonomy/python/mymodule.py`).

### Blue team
To spawn a blue plane from the blue Payload machine:
```console
export INSTALL_SPACE=/opt/sasc
. ${INSTALL_SPACE}/setup.bash
. ${INSTALL_SPACE}/share/gazebo-8/setup.sh
. ${INSTALL_SPACE}/share/uctf/setup.sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${INSTALL_SPACE}/share/gazebo_models
export PYTHONPATH=/home/ubuntu/scrimmage-templates/plugins/autonomy/python:$PYTHONPATH
spawn_blue 26 --acs tap0 --gazebo-ip 192.168.2.1 --local-ip 192.168.2.10 --tactic-module mymodule --tactic-name MyClass
```

The `spawn_blue` script can be used to spawn multiple planes; just pass more integers in the range 26-50 (the ids 1-25 are for copters, which aren't yet supported). E.g., to launch 10 planes:
```console
spawn_blue 26 27 28 29 30 31 32 33 34 35 --acs tap0 --gazebo-ip 192.168.2.1 --local-ip 192.168.2.10 --tactic-module mymodule --tactic-name MyClass
```
The same tactic will be used by all planes.

### Gold team
To spawn a gold plane from the gold Payload machine:
```console
export INSTALL_SPACE=/opt/sasc
. ${INSTALL_SPACE}/setup.bash
. ${INSTALL_SPACE}/share/gazebo-8/setup.sh
. ${INSTALL_SPACE}/share/uctf/setup.sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${INSTALL_SPACE}/share/gazebo_models
export PYTHONPATH=/home/ubuntu/scrimmage-templates/plugins/autonomy/python:$PYTHONPATH
spawn_gold 26 --acs tap0 --gazebo-ip 192.168.3.1 --local-ip 192.168.3.10 --tactic-module mymodule --tactic-name MyClass
```

The `spawn_gold` script can be used to spawn multiple planes; just pass more integers in the range 26-50 (the ids 1-25 are for copters, which aren't yet supported). E.g., to launch 10 planes:
```console
spawn_gold 26 27 28 29 30 31 32 33 34 35 --acs tap0 --gazebo-ip 192.168.2.1 --local-ip 192.168.2.10 --tactic-module mymodule --tactic-name MyClass
```
The same tactic will be used by all planes.

## Run fti.py

TODO: document how to run fti.py locally, connecting via VPN, and what the startup sequence is.

## Run swarm_commander.py

TODO: document how to run swarm_commander.py locally, connecting via VPN, and what the startup sequence is.
