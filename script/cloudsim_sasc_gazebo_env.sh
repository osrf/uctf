#!/usr/bin/env sh

. /opt/sasc/bin/sasc_gazebo_env.sh
export ROS_HOSTNAME=192.168.2.1
export ROS_MASTER_URI=http://192.168.2.1:11311

log="gazebo_log"
eval "$@" \&
PID=$!
echo "running" "$@" "in PID $PID"> $log
{ (cat <&3 3<&- >/dev/null; kill $PID; echo "killed" >> $log) & } 3<&0
trap "echo EXIT >> $log" EXIT
wait $PID
