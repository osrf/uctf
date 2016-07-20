from uctf import generate_init_script
from uctf import get_ground_control_port
from uctf import get_launch_snippet
from uctf import get_vehicle_pose
from uctf import spawn_model
from uctf import VEHICLE_BASE_PORT
from uctf import write_launch_file


def spawn_team(color):
    # ensure valid team color
    assert color in ['blue', 'gold']

    launch_snippet = ''

    # spawn 50 vehicles
    for i in range(50):
        # valid MAV_SYS_IDs 1 to 250
        # BLUE uses 1 to 50
        # GOLD uses 101 to 150
        mav_sys_id = 1 + i
        if color == 'gold':
            mav_sys_id += 100

        # the first 25 vehicles per team are iris
        # the second 25 vehicles per team are plane
        vehicle_type = 'iris' if i < 25 else 'plane'

        # each vehicle uses 4 consecutive ports
        # the first one is VEHICLE_BASE_PORT + MAV_SYS_IDs
        # the first two are used between the mavlink gazebo plugin and the px4
        # the second two are used between the px4 and and the mavros node
        vehicle_base_port = VEHICLE_BASE_PORT + mav_sys_id * 4

        # generate the vehicle specific init script
        init_script_path = generate_init_script(
            mav_sys_id, vehicle_type, vehicle_base_port,
            get_ground_control_port(color))

        # spawn the vehicle model in gazebo
        vehicle_pose = get_vehicle_pose(mav_sys_id, vehicle_type, color)
        spawn_model(
            mav_sys_id, vehicle_type, vehicle_base_port, color, vehicle_pose)

        launch_snippet += get_launch_snippet(
            mav_sys_id, vehicle_type, vehicle_base_port, init_script_path)

    launch_path = write_launch_file(launch_snippet)
    print('roslaunch %s' % launch_path)
