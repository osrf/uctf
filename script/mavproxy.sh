#!/usr/bin/env sh

arguments=$@

echo mavproxy.py --mav10 $arguments
mavproxy.py --mav10 $arguments
