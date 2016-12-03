# Who are you?
* [Game Master](#game-master)
* [Competitor](#competitor)


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
1. Click on the LAUNCH button in the Arbiter box. This will launch a simulation
cloud machine. This process takes about 2 minutes, and you should see the
Status and machine ip update. You should also see a "KEYS" button appear; click
it to download the ssh key for the simulation machine. Given that key, you can
ssh into the machine as user `ubuntu` (e.g., `ssh -i cloudsim.pem
ubuntu@1.2.3.4`).  Note that you must make the key file not-world-readable
after you download it (e.g., `chmod 600 cloudsim.pem`); if you don't do this,
you'll get an error about permissions from `ssh`.

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

SSH with X-Forwarding enabled to the simulation/arbiter machine (e.g., `ssh -XC -i cloudsim.pem ubuntu@192.168.2.1`) and start the Arbiter like so:
```console
export INSTALL_SPACE=/opt/sasc
. ${INSTALL_SPACE}/venv3/bin/activate
. ${INSTALL_SPACE}/setup.bash
arbiter_start.py -db br-blue -dr br-gold
```

### Launch the Game Director

In another terminal on the simulation/arbiter machine:

Update the config file and update permissions

```
export INSTALL_SPACE=/opt/sasc
sudo chown ubuntu:ubuntu -R ${INSTALL_SPACE}/ugdi_venv/ugdi
nano  ${INSTALL_SPACE}/ugdi_venv/ugdi/config.py
```
Update NETWORK_1's device to be 'br-blue'
And NETWORK_2's device to be 'br-gold'

Save and exit. 

Now run the game director:
```
export INSTALL_SPACE=/opt/sasc
. ${INSTALL_SPACE}/ugdi_venv/bin/activate
cd ${INSTALL_SPACE}/ugdi_venv/ugdi && python run.py
```
The game director interface will be accessible at http://192.168.2.1:5001 for blue and http://192.168.2.1:5001 for gold


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

You should see the round(s) that you're invited to play in as a competitor.
Click on the round you want to play.

## Your local computer(s)

You'll be using one or more computers physically located with you (e.g., in
your lab) to get access to the competition. We call these computers OCUs, for
Operator Control Units.  We're assuming that these computers are all running
64-bit Ubuntu Xenial (e.g., the Predator laptops that you were provided with).

In case it's not already installed, you should install `openvpn` on each OCU:
```console
sudo apt-get install openvpn
```

You should also install the lastest `sasc-gazebo-sitl` package, which contains,
among other things, the graphical tools that you'll use later to startup and
control your vehicles:
```console
sudo apt-get update
sudo apt-get install sasc-gazebo-sitl
```

Note that, if there is a newer package available, it will take quite some time to download and install, due to its size. Please be patient.

## Connect your VPN client
There's a VPN server dedicated to for use by your team. To get shell access to
your Payload machines, and also to locally run graphical tools such as `fti.py`
and `swarm_commander.py`, you need to first connect the computer that you're
using (e.g., in your lab) to the VPN server. There are a number (currently 5)
of pre-generated VPN keys that you can download to make this connection, like so:

1. In the box for your team, click on the "OCU keys" drop-down and
select one of the keys. You'll be prompted to save a `.zip` file that contains the key.
1. Unzip the `.zip` file that you downloaded.
1. In a terminal, navigate to where you unzipped the file and start the
`openvpn` client using the configuration file that was in the `.zip` file:
```console
sudo openvpn --config openvpn.conf
```

You can connect multiple OCUs to your VPN server and they will all be able
to communicate with each other, as well as your Payload machine. **NOTE:**
Don't use the same VPN key simultaneously on two different computers; that
won't work. You need to download a different VPN key for each computer that
you're going to connect.

To disconnect from the VPN, just give Ctrl-C in the terminal where you started
`openvpn`.

## SSH to your Payload machine

To get shell access to your Payload machine, which is where you'll be cloning
and running your tactics, you need the SSH key for that machine, and you need
to be connected to the VPN server. You can download the SSH key by clicking on
the "KEYS" button in the box for the Payload machine. Note that you must make
the key file not-world-readable after you download it (e.g., `chmod 600
cloudsim.pem`); if you don't do this, you'll get an error about permissions
from `ssh`.

With the SSH key downloaded and after connecting your computer to the VPN
server, you can login to your Payload machine as the user `ubuntu` at the
"Local IP" address that is shown in the box for that machine (e.g., `ssh -i
cloudsim.pem ubuntu@192.168.2.10`).

### Get the latest SASC package
With your VPN connected, ssh to your payload machine and make sure that you have the latest SASC package installed:
```console
sudo apt-get update
sudo apt-get install sasc-gazebo-sitl
```
Note that, if there is a newer package available, it will take quite some time to download and install, due to its size. Please be patient.

### Check out your tactics code
With your VPN connected, ssh to your payload machine and fetch your tactics code, e.g.:
```console
git clone https://gitlab.nps.edu/myfork/scrimmage-templates
```

### Spawn vehicle(s)
With your VPN connected, ssh to your payload machine and start running vehicles
with your tactics.  We're assuming that your tactic repo is checked out to
`$HOME/scrimmage-templates`, with your tactics modules sitting inside there, in
`plugins/autonomy/python`. In the examples below, we're assuming that the
tactic that you want to run is in a class called `MyClass` that is implemented
in a file called `mymodule.py` (which, for completeness, is located at
`$HOME/scrimmage-templates/plugins/autonomy/python/mymodule.py`).

#### Blue team
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

#### Gold team
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
The same tactic will be used by all planes. You will see a string like below indicating that your tactic was found successfully. 

```
..INFO.. tactic_interface: Found tutorial_greedy_shooter_1 at: /home/ubuntu/scrimmage-templates/plugins/autonomy/python/tutorial_greedy_shooter.py
..INFO.. tactic_interface: Tactic Name: TutorialGreedyShooter
```

## Use local tools to control vehicles

After spawning vehicles, you need to run through a startup sequence using some
graphical tools that you will run on your local OCUs, which must be connected
to your VPN server.

### Flight Tech Interface (fti.py)

Temporary workaround until 1.0.11 of sasc-gazebo-sitl is released with this as an added dependency:
`sudo apt-get install python3-pyside`

From your OCU that is connected to the VPN, run `fti.py`:
```console
export INSTALL_SPACE=/opt/sasc
. ${INSTALL_SPACE}/venv3/bin/activate
PYTHONPATH=${INSTALL_SPACE}/lib/python2.7/dist-packages:/opt/ros/kinetic/lib/python2.7/dist-packages:/usr/lib/python3/dist-packages fti.py -d tap0 -z
```

A GUI should pop up and become populated by entries for each of your vehicles.
Go through the following sequence in that GUI:

1. Click "CAL PRESSURE ALL" and then "OK" to the popups that follow.
1. Click "FLIGHT READY ALL". The vehicle entrie should now be yellow.
1. Click "ARM ALL". Note that this command won't take effect until the vehicles
have had some time to initialize their simulated GPS sensors, which can take a
couple of minutes after spawning. The vehicle entries should now be green.

  You should see content like this from the payload for each machine, after which it will be armable.
    ```
[INFO] [1480712292.473821]: MAVLink STATUSTEXT: EKF2 IMU0 is using GPS
[INFO] [1480712292.474442]: MAVLink STATUSTEXT: EKF2 IMU1 is using GPS
```
1. For each vehicle:

    1. Select the vehicle in the list of entries.
    1. Fill in a "Stack number" and "Altitude above runway (m)", then click "Send Config" and then confirm in the following popup.
    1. Click "AUTO". The vehicle should launch, climb, and go to its designated waypoint.

There are a number of conditions after launch that can cause the vehicle
entries to become red, indicating a problem, including:

* Throttle failure. The autopilot sometimes determines that the simulated
throttle has failed and commands RTL. You can usually override this command
by clicking "AUTO" again for that vehicle.
* Lack of airspeed or airspeed too low. Either the autopilot isn't reporting
the airspeed properly, or the reported airspeed is too low. Either way, it
causes a warning/error in the GUI. There doesn't seem to be any effect on the
vehicle's behavior.

### Swarm Commander (swarm_commander.py)

From your OCU that is connected to the VPN,:
Create the file `~/.swarm_commander.ini`

```
[NETWORK]
device = tap0
port = 5554
acs_id = 250
```
Then run `swarm_commander.py`

```console
export INSTALL_SPACE=/opt/sasc
. ${INSTALL_SPACE}/venv3/bin/activate
. ${INSTALL_SPACE}/setup.bash
PYTHONPATH=$PYTHONPATH:${INSTALL_SPACE}/venv3/lib/python3.5/site-packages:/usr/lib/python3/dist-packages swarm_commander.py
```

A GUI should pop up and become populated by your vehicles. If you just went
through the startup sequence with `fti.py`, then your vehicles should all be in
the "Swarm Ingress" state, which means that they're approaching the swarm
rally/way point. Eventually, when they reach the designated location, they'll
start circling and their states will switch to "Swarm Ready". At this point,
you can engage your tactics, like so:

1. Select one or more vehicles, then pick a subswarm ID and click "Assign
Selected UAVs to Subswarm".
1. Pick that subswarm ID from the "Select Subswarm for Behavior Command" drop-down.
1. Pick "Tactic Interface" from the "Select Behavior to Initiate" drop-down.
1. Click "Specify Selected Behavior Parameters".

Your tactic should now start executing on the indicated vehicles.


### QGroundControl

If you would like to see what's happening on a moving map you can use QGroundControl

With the VPN connected run: 

```console
export INSTALL_SPACE=/opt/sasc
. ${INSTALL_SPACE}/setup.bash
qgroundcontrol
```

To connect to the vehicles use the following procedure:

1. Click on the Purple "Q" button in the top left.
2. Select the "Comm Links" entry in the left menu.
3. Click "Add"
4. Select Type: UDP then enter Blue as the name and give it port 14000 and click OK
5. Repeat for Gold using port 14001
6. Select the "Blue" connection and click "Connect" (or gold if you're gold team)
7. Click on the Paper Airplane Icon at the top to view the vehicles.
