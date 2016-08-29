# Build package from source

For this tutorial we will assume you are still using ROS and Gazebo from binary packages.
Therefore first follow the instructions from the section *Install dependencies* in the [Install binary packages](../install_binary.md) documentation.

If you want to look into building any of them from source please follow their tutorials ([ROS](http://wiki.ros.org/kinetic/Installation/Source), [Gazebo](http://gazebosim.org/tutorials?tut=install_from_source)).

## Install acs software

### Payload

The payload is being compiled in a catkin workspace names `ws_payload`.
Clone the following repositories and then build the workspace with `catkin_make`:

* `autonomy-payload`
* `autopilot_bridge`

### Arbiter

Since the arbiter uses Python 3 we setup a virtual environment:

```console
mkdir venv3
pyvenv venv3
. venv3/bin/activate.sh
```

Then install the following Python packages into the venv using `python setup.py develop`:

* acs_lib
* acs_dashboards
* arbiter

You will likely need addition dependencies in the venv:

```console
pip install image netifaces pyqt5 urllib3
```

In order to avoid a dependency on `atcommander` you need to patch the file
`acs_lib/acs_network/acs_network_ground.py` and move the module level import of
`atcommander` to a local scope where it is being used.

## Build uctf package

Install the dependencies to fetch and build the source code:

```console
sudo apt update
sudo apt install git protobuf-compiler
```

Get the source code of `uctf` (including its submodules):

```console
cd ~
git clone https://github.com/osrf/uctf.git
cd uctf
git submodule update --init --recursive
```

Build and install the package:

```console
. /opt/ros/kinetic/setup.bash
cd ~/uctf
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=~/uctf/install
make -j4
make install
```

The path `~/uctf/install` is the install prefix of `uctf`.
Whenever following instructions use `/usr/share/uctf/*` (which points to the files from the Debian package of `uctf`) you need to use `~/uctf/install/share/uctf/*` instead.

---

Next: [Run an example](../run_example/readme.md).
