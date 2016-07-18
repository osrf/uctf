<launch>
@[for v in vehicles]@
  <node
    name="px4_@(v['vehicle_type'])_@(v['mav_sys_id'])"
    pkg="uctf"
    type="mainapp.sh"
    args="@(v['controller_config_path'])"
    cwd="node"
    ns="/@(v['vehicle_type'])_@(v['mav_sys_id'])"
    output="screen" />
  <include file="$(find mavros)/launch/px4.launch" ns="/@(v['vehicle_type'])_@(v['mav_sys_id'])">
    <arg name="fcu_url" value="udp://:@(v['ros_interface_port4'])@@localhost:@(v['ros_interface_port3'])" />
    <arg name="tgt_system" value="@(v['mav_sys_id'])" />
  </include>
@[end for]@
</launch>
