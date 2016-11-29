import argparse
import os
import subprocess

from uctf import generate_config
from uctf import get_ground_control_port
from uctf import generate_launch_file
from uctf import get_vehicle_base_port
from uctf import get_vehicle_pose
from uctf import delete_model
from uctf import spawn_model


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
    # the second 25 vehicles per team are plane (delta_wing)
    vehicle_type = 'iris' if vehicle_id <= 25 else 'delta_wing'

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
        '--acs-network-interface', default='lo',
        help='The interface name for acs to talk on.')
    parser.add_argument(
        '--no-launch', action='store_false', dest='launch',
        help='Run generate launch file')
    parser.add_argument(
        '--no-delete', action='store_false', dest='delete',
        help='Despawn when killed')
    parser.add_argument(
        '--no-payload', action='store_false', dest='include_payload',
        help='Do not launch the payload.')
    parser.add_argument('--debug', action='store_true')
    parser.add_argument('--px4', action='store_true', default=False)
    parser.add_argument('--mavros', action='store_true', default=False)
    parser.add_argument('--gazebo-ip', default='127.0.0.1')
    parser.add_argument('--local-ip', default='127.0.0.1')
    args = parser.parse_args()
    autopilot = 'px4' if args.px4 else 'ardupilot'
    if not args.gazebo_ros_master_uri:
        args.gazebo_ros_master_uri = 'http://%s:11311' % args.gazebo_ip
    # ensure valid team color
    assert color in ['blue', 'gold']

    cmds = []

    # spawn 50 vehicles
    for i in args.vehicle_id:

        vehicle_type, mav_sys_id = vehicle_type_and_mav_sys_id(i, color)

        vehicle_base_port = get_vehicle_base_port(mav_sys_id)

        # generate the vehicle specific init script
        init_script_path = generate_config(
            mav_sys_id, vehicle_type, vehicle_base_port,
            get_ground_control_port(color), autopilot=autopilot)

        # spawn the vehicle model in gazebo
        vehicle_pose = get_vehicle_pose(mav_sys_id, vehicle_type, color)
        spawn_model(
            mav_sys_id, vehicle_type, vehicle_base_port, color, vehicle_pose,
            ros_master_uri=args.gazebo_ros_master_uri,
            mavlink_address=args.mavlink_address,
            debug=args.debug, autopilot=autopilot,
            gazebo_ip=args.gazebo_ip,
            local_ip=args.local_ip,
            )

        launch_path = generate_launch_file(
            mav_sys_id, vehicle_type, vehicle_base_port,
            init_script_path, args.debug,
            ground_port=get_ground_control_port(color),
            autopilot=autopilot,
            gazebo_ip=args.gazebo_ip,
            local_ip=args.local_ip,
            include_payload=args.include_payload,
            launch_mavros=args.mavros,
            acs_network_inteface=args.acs_network_interface,
            )

        ros_master_port = 11311 + mav_sys_id
        env = {'ROS_MASTER_URI': 'http://localhost:%d' % ros_master_port}
        cmd = ['roslaunch', launch_path]
        cmds.append((env, cmd))
        env_str = ' '.join(['%s=%s' % (k, v) for k, v in env.items()])
        print(env_str + ' ' + ' '.join(cmd))

    retcode = 0
    if args.launch:
        processes = []
        for (add_env, cmd) in cmds:
            env = dict(os.environ)
            env.update(add_env)
            p = subprocess.Popen(cmd, env=env)
            processes.append(p)
        try:
            for p in processes:
                p.wait()
        except KeyboardInterrupt:
            pass
        finally:
            for p in processes:
                try:
                    p.terminate()
                except OSError:
                    pass
    if args.delete:
        for i in args.vehicle_id:
            vehicle_type, mav_sys_id = vehicle_type_and_mav_sys_id(i, color)
            delete_model(
                mav_sys_id,
                vehicle_type,
                ros_master_uri=args.gazebo_ros_master_uri)
    return retcode
