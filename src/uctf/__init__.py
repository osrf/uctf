from __future__ import print_function

import argparse
import os
import sys

from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import SpawnModelRequest
from rospy import ServiceProxy
from xacro import parse
from xacro import process_doc


def main():
    parser = argparse.ArgumentParser(
        'Spawn vehicle.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('vehicle_type', choices=['iris', 'plane'])
    parser.add_argument('suffix', default='')
    parser.add_argument('-x', type=float, default=0.0)
    parser.add_argument('-y', type=float, default=0.0)
    parser.add_argument('--color', choices=['blue', 'gold'])
    parser.add_argument('--debug', action='store_true')
    parser.add_argument('--mavlink', type=int, default=14560)
    parser.add_argument('--mavlink2', type=int, default=14556)
    args = parser.parse_args()
    spawn(args.vehicle_type, args.suffix, args.x, args.y, args.color,
          args.debug, args.mavlink, args.mavlink2)


def spawn(vehicle_type, suffix, x, y, color, debug, mavlink, mavlink2):
    srv = ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    model_filename = os.path.join(
        os.path.dirname(__file__), '..', '..', '..', '..',
        'share', 'uctf', 'models',
        '%s' % vehicle_type, '%s.sdf' % vehicle_type)
    with open(model_filename, 'r') as h:
        model_xml = h.read()

    kwargs = {
        'mappings': {
            'mavlink_udp_port': str(mavlink),
            'mavlink_udp_port_2': str(mavlink2),
        },
    }
    if color:
        material_name = 'Gazebo/%s' % color.capitalize()
        kwargs['mappings']['visual_material'] = material_name
    model_xml = xacro(model_xml, **kwargs)
    if debug:
        print(model_xml)

    req = SpawnModelRequest()
    unique_name = vehicle_type + suffix
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
