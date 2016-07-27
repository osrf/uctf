from __future__ import division
from __future__ import print_function

import argparse
try:
    from cStringIO import StringIO
except ImportError:
    from io import StringIO
import math
import os
import re
import sys
import tempfile

from em import Interpreter
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import SpawnModelRequest
from rospy import ServiceProxy
from xacro import parse
from xacro import process_doc


CUBE_LENGTH = 500
# default location of spawned vehicles
MIN_LATITUDE = 47.397742
MAX_LATITUDE = 47.402251
MIN_LONGITUDE = 8.5389317
MAX_LONGITUDE = 8.5455939

VEHICLE_BASE_PORT = 14000
GROUND_CONTROL_PORT_BLUE = 14000
GROUND_CONTROL_PORT_GOLD = 14001


def get_ground_control_port(color):
    return GROUND_CONTROL_PORT_BLUE \
        if color == 'blue' else GROUND_CONTROL_PORT_GOLD


def get_vehicle_base_port(mav_sys_id):
    return VEHICLE_BASE_PORT + mav_sys_id * 4


def get_vehicle_pose(mav_sys_id, vehicle_type, color):
    inteam_id = mav_sys_id % 100
    DISTANCE_FROM_CUBE = 5.0
    x = -DISTANCE_FROM_CUBE if color == 'blue' else \
        (CUBE_LENGTH + DISTANCE_FROM_CUBE)
    y = CUBE_LENGTH - 50 - inteam_id * (2 if vehicle_type == 'iris' else 5)
    yaw = 0 if color == 'blue' else 3.1416
    return (x, y, yaw)


def mav_sys_id_type(value):
    value = int(value)
    if value < 1 or value > 250:
        raise argparse.ArgumentTypeError('MAV_SYS_ID must be in [1, 250]')
    return value


def spawn_one():
    parser = argparse.ArgumentParser(
        'Spawn vehicle.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('mav_sys_id', type=mav_sys_id_type)
    parser.add_argument('--vehicle-type', choices=['iris', 'plane'])
    parser.add_argument('--baseport', type=int)
    parser.add_argument('--color', choices=['blue', 'gold'])
    parser.add_argument('--groundport', type=int)
    parser.add_argument('-x', type=float)
    parser.add_argument('-y', type=float)
    parser.add_argument('--debug', action='store_true')
    args = parser.parse_args()

    # choose some nice defaults based on the id
    if args.vehicle_type is None:
        args.vehicle_type = 'iris' if args.mav_sys_id % 2 else 'plane'
    if args.baseport is None:
        args.baseport = 14000 + args.mav_sys_id * 4
    if args.color is None:
        if args.mav_sys_id < 101:
            args.color = 'blue'
        elif args.mav_sys_id < 201:
            args.color = 'gold'
    if args.groundport is None:
        args.groundport = 14000
        if args.color == 'blue':
            args.groundport += 1
        if args.color == 'gold':
            args.groundport += 2
    if args.x is None and args.y is None:
        # arrange in 10 by 10 blocks
        offset_x = ((args.mav_sys_id - 1) // 10) % 10
        offset_y = ((args.mav_sys_id - 1) % 10)
        if args.mav_sys_id > 200:
            args.x = 1.0 + offset_x
            args.y = offset_y - 5.0
        else:
            args.x = -offset_x
            args.y = 1.0 + offset_y
            if args.mav_sys_id <= 100:
                args.y = -args.y
    if args.x is None:
        args.x = 0.0
    if args.y is None:
        args.y = 0.0

    config_path = generate_init_script(
        args.mav_sys_id, args.vehicle_type, args.baseport, args.groundport,
        args.debug)

    spawn_model(
        args.mav_sys_id,
        args.vehicle_type, args.baseport, args.color,
        (args.x, args.y, 0), args.debug)

    launch_file = generate_launch_file(
        args.mav_sys_id,
        args.vehicle_type, args.baseport,
        config_path,
        args.debug)

    spawn_launch_file(launch_file)


def generate_init_script(
    mav_sys_id, vehicle_type, baseport, ground_port, debug=False
):
    # read vehicle specific init file
    init_filename = os.path.join(
        os.path.dirname(__file__), '..', '..', '..', '..',
        'share', 'uctf', 'SITLinit',
        'rcS_gazebo_%s' % vehicle_type)
    with open(init_filename, 'r') as h:
        init_data = h.read()
    init_lines = init_data.splitlines()

    # add MAV_SYS_ID
    prefix = 'param set MAV_TYPE'
    for i, line in enumerate(init_lines):
        if line.startswith(prefix):
            init_lines[i + 1:i + 1] = ['param set MAV_SYS_ID %d' % mav_sys_id]
            break
    else:
        raise RuntimeError("Could not find line starting with '%s'" % prefix)

    # add simulator port
    match = 'simulator start -s'
    for i, line in enumerate(init_lines):
        if line == match:
            init_lines[i] += ' -u %d' % baseport
            break
    else:
        raise RuntimeError("Could not find line with '%s'" % match)

    # update relative path to mixers file
    prefix = 'mixer load'
    for i, line in enumerate(init_lines):
        if line.startswith(prefix):
            init_lines[i] = re.sub(
                re.escape(' ../../../../ROMFS/') + '(px4fmu_common|sitl)' +
                re.escape('/mixers/'),
                ' mixers/', line, 1)
            break
    else:
        raise RuntimeError("Could not find line starting with '%s'" % prefix)

    # update hard coded ports and add ground control port
    prefix = 'mavlink start -u 14556'
    for i, line in enumerate(init_lines):
        if line.startswith(prefix):
            init_lines[i] = line.replace('14556', str(baseport + 1), 1)
            init_lines[i] += ' -o %d' % ground_port
            break
    else:
        raise RuntimeError("Could not find line starting with '%s'" % prefix)

    # update hard coded ports
    prefix = 'mavlink start -u 14557'
    for i, line in enumerate(init_lines):
        if line.startswith(prefix):
            line = line.replace('14557', str(baseport + 2), 1)
            init_lines[i] = line.replace('14540', str(baseport + 3), 1)
            break
    else:
        raise RuntimeError("Could not find line starting with '%s'" % prefix)

    # update hard coded ports
    prefix = 'mavlink stream'
    for i, line in enumerate(init_lines):
        if line.startswith(prefix):
            init_lines[i] = line.replace('14556', str(baseport + 1), 1)

    if debug:
        print('\n'.join(init_lines))

    fd, path = tempfile.mkstemp(prefix='%s_%d_' % (vehicle_type, mav_sys_id))
    with os.fdopen(fd, 'w') as h:
        h.write('\n'.join(init_lines) + '\n')

    return path


def spawn_model(mav_sys_id, vehicle_type, baseport, color, pose, debug=False):
    x, y, yaw = pose
    srv = ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    model_filename = os.path.join(
        os.path.dirname(__file__), '..', '..', '..', '..',
        'share', 'uctf', 'models',
        '%s' % vehicle_type, '%s.sdf' % vehicle_type)
    with open(model_filename, 'r') as h:
        model_xml = h.read()

    kwargs = {
        'mappings': {
            'mavlink_udp_port': str(baseport),
            'mavlink_udp_port_2': str(baseport + 1),
        },
    }
    if color:
        material_name = 'Gazebo/%s' % color.capitalize()
        kwargs['mappings']['visual_material'] = material_name
    model_xml = xacro(model_xml, **kwargs)
    if debug:
        print(model_xml)

    req = SpawnModelRequest()
    unique_name = vehicle_type + '_' + str(mav_sys_id)
    req.model_name = unique_name
    req.model_xml = model_xml
    req.robot_namespace = unique_name
    req.initial_pose.position.x = x
    req.initial_pose.position.y = y
    req.initial_pose.position.z = 50.0
    req.initial_pose.orientation.x = 0.0
    req.initial_pose.orientation.y = 0.0
    req.initial_pose.orientation.z = math.sin(yaw / 2.0)
    req.initial_pose.orientation.w = math.cos(yaw / 2.0)
    req.reference_frame = ''

    resp = srv(req)
    if resp.success:
        print(resp.status_message, '(%s)' % unique_name)
        return 0
    else:
        print(resp.status_message, file=sys.stderr)
        return 1


def xacro(template_xml, **kwargs):
    doc = parse(template_xml)
    process_doc(doc, **kwargs)
    xml = doc.toprettyxml(indent='  ')
    xml = xml.replace(' xmlns:xacro="http://ros.org/wiki/xacro"', '', 1)
    return xml


def generate_launch_file(
    mav_sys_id, vehicle_type, baseport, config_path, debug
):
    launch_snippet = get_launch_snippet(mav_sys_id, vehicle_type, baseport, config_path, debug)
    if debug:
        print(launch_snippet)
    return write_launch_file(launch_snippet)


def get_launch_snippet(
    mav_sys_id, vehicle_type, vehicle_base_port, init_script_path, debug=False
):
    pkg_share_path = os.path.normpath(os.path.join(
        os.path.dirname(__file__), '..', '..', '..', '..',
        'share', 'uctf'))
    if debug:
        print('For manual invocation run:')
        print('  cd %s && mainapp %s' % (pkg_share_path, init_script_path))
        print(
            '  roslaunch px4 mavros.launch fcu_url:=udp://:%d@localhost:%d '
            'tgt_system:=%d ns:=/%s_%d' % (
                vehicle_base_port + 3, vehicle_base_port + 2,
                mav_sys_id, vehicle_type, mav_sys_id))

    data = {
        'controller_config_path': init_script_path,
        'ros_interface_port3': vehicle_base_port + 2,
        'ros_interface_port4': vehicle_base_port + 3,
        'vehicle_type': vehicle_type,
        'mav_sys_id': mav_sys_id,
        'pkg_share_path': pkg_share_path,
    }
    return empy('px4_and_mavros.launch.em', data)


def write_launch_file(launch_snippet):
    fd, path = tempfile.mkstemp(suffix='.launch')
    with os.fdopen(fd, 'w') as h:
        h.write('<launch>\n')
        h.write(launch_snippet)
        h.write('</launch>\n')
    return path


def empy(template_name, data, options=None):
    template_path = os.path.join(
        os.path.dirname(__file__), 'templates', template_name)
    output = StringIO()
    try:
        interpreter = Interpreter(output=output, options=options)
        with open(template_path, 'r') as h:
            content = h.read()
        interpreter.string(content, template_path, locals=data)
        value = output.getvalue()
        return value
    except Exception as e:
        print("%s processing template '%s'" %
              (e.__class__.__name__, template_name), file=sys.stderr)
        raise
    finally:
        interpreter.shutdown()


def spawn_launch_file(launch_file):
    print('roslaunch %s' % launch_file)


def global_to_cube(latitude, longitude):
    up_fraction = (latitude - MIN_LATITUDE) / (MAX_LATITUDE - MIN_LATITUDE)
    right_fraction = (longitude - MIN_LONGITUDE) / \
        (MAX_LONGITUDE - MIN_LONGITUDE)
    left_fraction = 1.0 - right_fraction
    return left_fraction * CUBE_LENGTH, up_fraction * CUBE_LENGTH


def cube_to_global(x, y):
    left_fraction = x / CUBE_LENGTH
    up_fraction = y / CUBE_LENGTH
    right_fraction = 1.0 - left_fraction
    latitude = up_fraction * (MAX_LATITUDE - MIN_LATITUDE) + MIN_LATITUDE
    longitude = right_fraction * (MAX_LONGITUDE - MIN_LONGITUDE) + MIN_LONGITUDE
    return latitude, longitude
