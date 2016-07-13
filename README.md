# uctf
Unmanned Capture the Flag (U-CTF) project.

# Kick-off goals

1. Vehicles and environments
  1. Fixed wing model with aerodynamics
  1. Quad-copter model with aerodynamics
  1. Flat grass textured ground plane
    1. Pick a reasonable size that we can justify.
1. ROS vehicle control interface
  1. A plugin with public API to control the vehicles.
    1. Look at [swarm](https://bitbucket.org/osrf/swarm)
  1. Write an example controller that uses the plugin
    1. Demonstrate take off, simple flight pattern, and landing.
1. Gazebo-arbiter interface
  1. Get in contact with Tim Chung for access to their arbiter code.
1. Run single team in cloudsim
  1. Setup a docker container to run a single team of 50 vehicles in cloudsim.
1. Gazebo rules and scoring plugin
  1. Save this for last. The arbiter may implement this for us.

# References

1. [google group uctf-internal](https://groups.google.com/a/osrfoundation.org/forum/#!forum/uctf-internal)
