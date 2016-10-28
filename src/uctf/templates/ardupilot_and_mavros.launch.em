  <node
    name="ardupilot_@(vehicle_type)_@(mav_sys_id)"
    pkg="uctf"
    type="arduplane.sh"
    args="@(executable) @(mav_sys_id) @(base_port) @(rc_in_port) @(gazebo_port_in) @(gazebo_port_out) @(default_params) @(model)"
    cwd="ROS_HOME"
    ns="/@(vehicle_type)_@(mav_sys_id)"
    output="screen" />
  <node
    name="ardupilot_@(vehicle_type)_@(mav_sys_id)_mavproxy"
    pkg="uctf"
    type="mavproxy.sh"
    args="@(mavproxy_arguments)"
    cwd="ROS_HOME"
    ns="/@(vehicle_type)_@(mav_sys_id)"
    output="screen" />
  <include file="$(find mavros)/launch/apm.launch" ns="/@(vehicle_type)_@(mav_sys_id)">
    <arg name="fcu_url" value="udp://:@(ros_interface_port4)@@localhost:@(ros_interface_port3)" />
    <arg name="tgt_system" value="@(mav_sys_id)" />
  </include>
