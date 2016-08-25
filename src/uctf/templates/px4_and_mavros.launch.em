  <node
    name="px4_@(vehicle_type)_@(mav_sys_id)"
    pkg="uctf"
    type="px4.sh"
    args="@(controller_config_path)"
    cwd="node"
    ns="/@(vehicle_type)_@(mav_sys_id)"
    output="screen" />
  <include file="$(find ap_master)/launch/master.launch">
    <arg name="id" value="@(mav_sys_id)" />
    <arg name="name" value="@(vehicle_type)_@(mav_sys_id)" />
    <arg name="ap_dev" value="udp:localhost:@(ros_interface_port4)" />
    <arg name="net_dev" value="lo" />
    <arg name="gps" default="0" />
    <arg name="do_bag" default="0" />
    <arg name="do_verify" default="0" />
  </include>
