  <node
    name="ardupilot_@(vehicle_type)_@(mav_sys_id)"
    pkg="uctf"
    type="arduplane.sh"
    args="@(executable) @(mav_sys_id) @(base_port) @(rc_in_port) @(gazebo_port_in) @(gazebo_port_out) @(default_params) @(model) @(home_str)"
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

<!-- TODO(tfoote) parameterize this in and out based on an option -->
<!-- TODO(tfoote) verify namespace is ok slightly different -->
<include file="$(find ap_master)/launch/sitl.launch" ns="/@(vehicle_type)_@(mav_sys_id)">
  <arg name="id" value="@(mav_sys_id)" />
  <!-- <arg name="name" value="@(vehicle_type)_@(mav_sys_id)"/> -->
  <arg name="sitl" value="@(sitl_base_url)" />
  <arg name="port" value="5554" />
  <arg name="dev" value="@(acs_network_inteface)" />
  <arg name="range" value="-1" />
</include>
<!--/usr/bin/python /opt/ros/kinetic/bin/roslaunch ap_master sitl.launch id:=101 name:=sitl101 sitl:=tcp:192.168.2.250:6772 port:=5554 ns:=sitl101 dev:=sitl_bridge_1 range:=-1 -->
