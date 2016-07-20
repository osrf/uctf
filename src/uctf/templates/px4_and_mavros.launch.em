  <node
    name="px4_@(vehicle_type)_@(mav_sys_id)"
    pkg="uctf"
    type="mainapp.sh"
    args="@(controller_config_path)"
    cwd="node"
    ns="/@(vehicle_type)_@(mav_sys_id)"
    output="screen" />
  <include file="$(find mavros)/launch/px4.launch" ns="/@(vehicle_type)_@(mav_sys_id)">
    <arg name="fcu_url" value="udp://:@(ros_interface_port4)@@localhost:@(ros_interface_port3)" />
    <arg name="tgt_system" value="@(mav_sys_id)" />
  </include>
