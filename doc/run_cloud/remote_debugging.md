# Debugging in the cloud

If you want to debug remotely on the cloud and you are fulfilling the game master role.


You can modify how to launch gazebo such that you can connect gzclient to it remotely to get a 3D visualization and interaction environment for the simulation.

This will be phrased as changes to the 

## Remote debugging process

1. Connect to the VPN
1. Launch the gazebo instance with the following arguments `/opt/sasc/bin/cloudsim_sasc_gazebo_env.sh GAZEBO_IP=192.168.2.1 GAZEBO_MASTER_URI=http://192.168.2.1:12345 roslaunch uctf cloudsim_game_master.launch`
1. In another terminal locally,determine your local VPN ip and export it as the environment variable `LOCAL_VPN_IP`
1. run `/opt/sasc/bin/cloudsim_sasc_gazebo_env.sh GAZEBO_IP=$LOCAL_VPN_IP GAZEBO_MASTER_URI=http://192.168.2.1:12345 gzclient`

## Warnings

This will expose anyone on the VPN to be able to access and control gazebo. Do not do this during a competition.
