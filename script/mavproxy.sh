#!/usr/bin/env sh

arguments=$@

echo mavproxy.py --mav10 --sitl 127.0.0.1:5501 $arguments
mavproxy.py --mav10 --sitl 127.0.0.1:5501 $arguments
