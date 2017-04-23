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
You must also select the number of computers to make available to each team. Drones will be evenly distributed across the machines. It's recommended to have less than 10 drones per machine to keep the load to a reasonable level. 

Once you have configured the round press "Continue" to create the round.

## Launch the simulation/arbiter machine
1. Click on the LAUNCH button in the Arbiter box. This will launch a simulation
cloud machine. This process takes about 2 minutes, and you should see the
Status and machine ip update. You should also see a "KEYS" button appear; click
it to download the ssh key for the simulation machine.
It may take a minute or two for the key to become available, while the machine is provisioned.
Given that key, you can
ssh into the machine as user `ubuntu` (e.g., `ssh -i cloudsim.pem
ubuntu@1.2.3.4`).  Note that you must make the key file not-world-readable
after you download it (e.g., `chmod 600 cloudsim.pem`); if you don't do this,
you'll get an error about permissions from `ssh`.

### Get the latest SASC package
ssh to the simulation/arbiter machine and make sure that you have the latest SASC package installed:
```console
sudo apt-get update && sudo apt-get install sasc-gazebo-sitl
```
Note that, if there is a newer package available, it will take quite some time (~10 mintues) to download and install, due to its size. Please be patient.

### Launch Processes
ssh to the simulation/arbiter machine (e.g., `ssh -i cloudsim.pem
ubuntu@1.2.3.4`) and start Gazebo like so:
```console
/opt/sasc/bin/cloudsim_sasc_gazebo_env.sh roslaunch uctf cloudsim_game_master.launch
```


The Game Director web interface will be available on port 5001 of the public IP.
There will be a link in the CloudSim.io UI.


### Configure the Game Director

In another terminal on the simulation/arbiter machine (e.g., `ssh -i cloudsim.pem
ubuntu@1.2.3.4`):

Create an admin user (to login via the web interface)
```
/opt/sasc/bin/gdi_env.sh python /opt/sasc/ugdi_venv/ugdi/create_user.py
```

A Game Director user must first be created to control a game within the Game Director Interface, to create a Game Director user run:

  > create_user.py

If a user is already created in the SQLite database (gdi.db), the following message will appear:

  > A user already exists! Create another? (y/n):

Enter 'y', without the tick marks and press enter.


  > Password:

Type an password and press enter/return and then retype the password entered above when prompted.

  > Password (again):

Press enter/return and the password match and the user does not exist, the new user will be created.

Please see the [full game director documentation](game_director.md) for more details.


## Start a game
At this point, everything is ready for the teams to spawn their vehicles. You
should tell them that the game is on, who is playing which team (blue vs.
gold), and that they are free to start spawning.

**NOTE**: this step might be tied into the Game Director.

## Stop a game
To stop a game, just give a Ctrl-C to the `roslaunch`  commands that you started.


## Start a new game
You can reuse the machines from a round for multiple games. Just stop and start Gazebo and the Arbiter.

## Finish the round
1. Once you are finished with the round, press the FINISH ROUND button to terminate the machines (simulation and payloads). Machines are billed for each hour they are running.

## Remote debugging

If you wish to remotely debug your gazebo instance you can modify your launching technique to allow attchment of gzclient to get a 3D visualization of the world as well as interact with it remotely. [See these instructions](remote_debugging.md) 

# Competitor

## One-time setup
1. Sign up with a Google account at https://cloudsim.io.
1. Email `uctf-internal@osrfoundation.org` to request competitor access. Wait to hear confirmation, then log out and back in.

## Install sasc-gazebo-sitl on your local machines

You will need to follow the [binary install instructions](doc/install_binary/readme.md)


## Login to cloudsim.io
1. Log in to https://cloudsim.io/
1. Click on the SASC link (on the left menu bar, under the Dashboard).

You should see the round(s) that you're invited to play in as a competitor.
Click on the round you want to play.


### Start your payload computers. 

Click "LAUNCH ALL PAYLOADS" to launch all your payload machines.

Your cloud instances may have already been started by the game master, in which case you will see the payload machines listed below for your team.

After a few seconds, payload information and ssh download links should appear.
It make take up to 2 minutes for the machines to fully provision.

## Your local computer(s)

You'll be using one or more computers physically located with you (e.g., in
your lab) to get access to the competition. We call these computers OCUs, for
Operator Control Units.  We're assuming that these computers are all running
64-bit Ubuntu Xenial (e.g., the Predator laptops that you were provided with).

If you are not running on one of the SASC laptops please see [the binary installation instructions](docs/binary_install/readme.md)

Make sure to update `sasc-gazebo-sitl` to the latest package, which contains,
among other things, the graphical tools that you'll use later to startup and
control your vehicles:
```console
sudo apt-get update && sudo apt-get install sasc-gazebo-sitl
```

Note that, if there is a newer package available, it will take quite some time to download and install, due to its size. Please be patient.

## Connect your VPN client
There's a VPN server dedicated to for use by your team. To get shell access to
your Payload machines, and also to locally run graphical tools such as `fti.py`
and `swarm_commander.py`, you need to first connect the computer that you're
using (e.g., in your lab) to the VPN server. There are a number (currently 5)
of pre-generated VPN keys that you can download to make this connection, like so:

1. In the box for your team, click on the "OCU keys" drop-down and
select one of the keys. You'll be prompted to save a `.tar.gz` file that contains the key.
Generating the keys takes a minute or two you may have to wait before you can download it.
1. Untar the `.tar.gz` file that you downloaded.
1. In a terminal, navigate to where you unpacked the file and start the
`openvpn` client using the configuration file that was in the `.tar.gz` file:
```console
sudo openvpn --config openvpn.conf
```

If you are having trouble with rsync hanging, in another terminal:
```
sudo ifconfig tap0 mtu 1150
```

A successfully started VPN connection will show
```
Initialization Sequence Completed
```

You can connect multiple OCUs to your VPN server and they will all be able
to communicate with each other, as well as your Payload machine. **NOTE:**
Don't use the same VPN key simultaneously on two different computers; that
won't work. You need to download a different VPN key for each computer that
you're going to connect.

To disconnect from the VPN, just give Ctrl-C in the terminal where you started
`openvpn`.

## Setup your ssh agent

From the CloudSim UI you will have link to download the key. 
The key will be the same for all the payload machines.
The key takes a minute or two to generate and will not be downloadable immediately.
Add the key to your ssh-agent using the following command. Where the key is saved as cloudsim.pem.

```
chmod 600 ~/Download/cloudsim.pem
ssh-add ~/Download/cloudsim.pem
```

You will download these ssh keys but you will not need to use them directly.
The deploy script will use the keys to log into the remote servers for you.

## Deploy your tactics and start payload

Assuming that your tactics repository is checked out into `~/scrimmage-templates`.

This assumes you want to launch vehicles 26-31 and you're on team `blue`, the other option is `gold`.

To deploy your tactics and prepare the cloud machines use the following command.

Your game instances will have a number of payload hosts, the script needs to know how many the game is provisioned with to allocate the simulation appropriately.

```
. /opt/sasc/bin/cloudsim_sasc_gazebo_env.sh
sasc_deploy --hosts 3 --scrimm ~/scrimmage-templates blue --planes 15 --copters 10
```

The total number of vehicles should add up to less than 25. You can choose the balance between planes and copters as you wish.

The output of the above command will tell you the command to run the payloads remotely. It will look like this:

```
ROS_HOSTNAME=192.168.2.150 ROS_MASTER_URI=http://192.168.2.1:11311 roslaunch /tmp/sasc_remote.launch
```

If you get a timeout the first time try again.

To stop the payloads you can press `Ctrl-C`

This will also bring up two GUIs on your machine.


### Flight Tech Interface (fti.py)

A GUI should pop up and become populated by entries for each of your vehicles.
Go through the following sequence in that GUI:

1. Click "CAL PRESSURE ALL" and then "OK" to the popups that follow.
1. Click "FLIGHT READY ALL". The vehicle entrie should now be yellow.
1. For each vehicle:
    1. Select the vehicle in the list of entries.
    1. Fill in a "Stack number" (1 for Blue, 2 for Gold) and "Altitude above runway (m)", then click "Send Config" and then confirm in the following popup.
    1. Click "ARM" to arm the vehicle. You should see it change status to "ARMED" in the gui.
    1. Click "AUTO". The vehicle should launch, climb, and go to its designated waypoint.
       For planes make sure to go to AUTO soon after arming. An armed plane in simulation can get slightly airborn but if not in AUTO it will crash and become unrecoverable.
    1. Wait about 10 seconds or until you see Waypoint 1 in the payload console. This operation uses templates that are not safe to run in parallel..


There are a number of conditions after launch that can cause the vehicle
entries to become red, indicating a problem, including:

* Lack of airspeed or airspeed too low. Either the autopilot isn't reporting
the airspeed properly, or the reported airspeed is too low. Either way, it
causes a warning/error in the GUI. There doesn't seem to be any effect on the
vehicle's behavior.

#### Launch rate limiting

To simulate launching from a pair catapult only two planes can be launced every 30 seconds.
The 30 second countdown for the next plane starts after the previous plane has traveled out of the launching area.
You can do everything to the planes however they will be prevented from moving until their turn on the catapult.
Note that the catapult automatically iterates from lowest to highest in vehicle number.

### Swarm Commander (swarm_commander.py)

A GUI should pop up and become populated by your vehicles. If you just went
through the startup sequence with `fti.py`, then your vehicles should all be in
the "Ingress" state, which means that they're approaching the swarm
rally/way point. Eventually, when they reach the designated location, they'll
start circling and their states will switch to "Swarm Ready". At this point,
you can engage your tactics, like so:

1. Select one or more vehicles, then pick a subswarm ID and click "Assign
Selected UAVs to Subswarm".
1. Pick that subswarm ID from the "Select Subswarm for Behavior Command" drop-down.
1. Pick "Tactic Interface" from the "Select Behavior to Initiate" drop-down.
1. Click "Specify Selected Behavior Parameters".

Your tactic should now start executing on the indicated vehicles.
