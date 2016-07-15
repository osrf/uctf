from __future__ import print_function

import argparse
import os
import re
import sys
import tempfile

from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import SpawnModelRequest
from rospy import ServiceProxy
from xacro import parse
from xacro import process_doc


def mav_sys_id_type(value):
    value = int(value)
    if value < 1 or value > 250:
        raise argparse.ArgumentTypeError('MAV_SYS_ID must be in [1, 250]')
    return value


def main():
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

    config_path = generate_controller_config(
        args.mav_sys_id, args.vehicle_type, args.baseport, args.groundport,
        args.debug)

    spawn_model(
        args.mav_sys_id,
        args.vehicle_type, args.baseport, args.color,
        args.x, args.y, args.debug)

    spawn_controller(config_path)


def generate_controller_config(
    mav_sys_id, vehicle_type, baseport, ground_port, debug
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


def spawn_model(mav_sys_id, vehicle_type, baseport, color, x, y, debug):
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
    req.initial_pose.position.z = 0.0
    req.initial_pose.orientation.x = 0.0
    req.initial_pose.orientation.y = 0.0
    req.initial_pose.orientation.z = 0.0
    req.initial_pose.orientation.w = 1.0
    req.reference_frame = ''

    resp = srv(req)
    if resp.success:
        print(resp.status_message)
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


def spawn_controller(config_path):
    pkg_share_path = os.path.normpath(os.path.join(
        os.path.dirname(__file__), '..', '..', '..', '..',
        'share', 'uctf'))
    print('cd %s' % pkg_share_path)
    print('mainapp %s' % config_path)
