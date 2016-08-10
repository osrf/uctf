import argparse
import subprocess

from uctf import generate_init_script
from uctf import get_ground_control_port
from uctf import get_launch_snippet
from uctf import get_vehicle_pose
from uctf import delete_model
from uctf import spawn_model
from uctf import VEHICLE_BASE_PORT
from uctf import write_launch_file


def vehicle_id_type(value):
    """Validate the vehicle_id_type from string to int."""
    value = int(value)
    if value < 1 or value > 50:
        raise argparse.ArgumentTypeError('Vehicle id must be in [1, 50]')
    return value


def vehicle_type_and_mav_sys_id(vehicle_id, vehicle_color):
    """Get the vehicle_type and mav_sys_id from the vehicle's id and color."""
    # valid MAV_SYS_IDs 1 to 250

    # the first 25 vehicles per team are iris
    # the second 25 vehicles per team are plane
    vehicle_type = 'iris' if vehicle_id <= 25 else 'plane'

    # BLUE uses 1 to 50
    # GOLD uses 101 to 150
    mav_sys_id = vehicle_id
    if vehicle_color == 'gold':
        mav_sys_id += 100
    return vehicle_type, mav_sys_id


def spawn_team(color):
    parser = argparse.ArgumentParser('Spawn vehicle for one team.')
    parser.add_argument(
        'vehicle_id', nargs='*', metavar='VEHICLE_ID', type=vehicle_id_type,
        default=range(1, 51),
        help='The vehicle ids to spawn (default: 1-50)')
    parser.add_argument(
        '--gazebo-ros-master-uri',
        help='The uri used to spawn the models')
    parser.add_argument(
        '--mavlink-address',
        help='The IP address for mavlink (default: INADDR_ANY)')
    parser.add_argument(
        '--launch', action='store_true',
        help='Run generate launch file')
    parser.add_argument(
        '--delete', action='store_true',
        help='Despawn when killed')
    parser.add_argument('--debug', action='store_true')
    args = parser.parse_args()

    # ensure valid team color
    assert color in ['blue', 'gold']

    launch_snippet = ''

    # spawn 50 vehicles
    for i in args.vehicle_id:

        vehicle_type, mav_sys_id = vehicle_type_and_mav_sys_id(i, color)

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
            mav_sys_id, vehicle_type, vehicle_base_port, color, vehicle_pose,
            ros_master_uri=args.gazebo_ros_master_uri,
            mavlink_address=args.mavlink_address,
            debug=args.debug)

        launch_snippet += get_launch_snippet(
            mav_sys_id, vehicle_type, vehicle_base_port, init_script_path)

    launch_path = write_launch_file(launch_snippet)
    cmd = ['roslaunch', launch_path]
    print(' '.join(cmd))

    retcode = 0
    if args.launch:
        try:
            retcode = subprocess.call(cmd)
        except KeyboardInterrupt:
            pass
    if args.delete:
        for i in args.vehicle_id:
            _, mav_sys_id = vehicle_type_and_mav_sys_id(i, color)
            delete_model(
                mav_sys_id,
                vehicle_type,
                ros_master_uri=args.gazebo_ros_master_uri)
    return retcode
