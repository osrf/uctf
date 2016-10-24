#!/usr/bin/env sh

mav_sys_id=$1
base_port=$2
rc_in_port=$3
gazebo_port_in=$4
gazebo_port_out=$5
default_params=$6

#todo actually call the executables in a known path
echo /home/tfoote/uctf-ardu/ardupilot/build/sitl/bin/arduplane -S -I $mav_sys_id --base-port $base_port --rc-in-port $rc_in_port --gazebo-port-in $gazebo_port_in --gazebo-port-out $gazebo_port_out --home -35.363261,149.165230,584,353 --model gazebo-zephyr --speedup 10 --defaults $default_params
/home/tfoote/uctf-ardu/ardupilot/build/sitl/bin/arduplane -S -I $mav_sys_id --base-port $base_port --rc-in-port $rc_in_port --gazebo-port-in $gazebo_port_in --gazebo-port-out $gazebo_port_out --home -35.363261,149.165230,584,353 --model gazebo-zephyr --speedup 10 --defaults $default_params
