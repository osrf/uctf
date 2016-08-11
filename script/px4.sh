#!/usr/bin/env sh

dir=`mktemp -d`
echo "temporary rootfs: $dir"
cp -R mixers $dir
cp -R rootfs $dir

cd $dir
rc_script=$1
px4 ${rc_script}
