import socket
import struct
from threading import Thread

from python_qt_binding.QtCore import QMutex, QMutexLocker, QTimer
from qt_gui.plugin import Plugin

from pymavlink.mavutil import mavlink_connection
from pymavlink.dialects.v10.ardupilotmega \
    import MAVLink_global_position_int_message
from pymavlink.dialects.v10.ardupilotmega \
    import MAVLink_param_value_message

from uctf import GROUND_CONTROL_PORT_BLUE
from uctf import GROUND_CONTROL_PORT_GOLD
from uctf.widget import Widget


class View(Plugin):

    def __init__(self, context):
        super(View, self).__init__(context)
        self.setObjectName('UCTFView')

        self._widget = Widget()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() +
                (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        self._update_queue = []
        self._mutex = QMutex()

        self.subscribers = {}
        self._vehicle_types = {}
        self._received_msg_types = []
        self._subscribe('blue', GROUND_CONTROL_PORT_BLUE)
        self._subscribe('gold', GROUND_CONTROL_PORT_GOLD)

        self._timer = QTimer()
        self._timer.timeout.connect(self._update_model)
        self._timer.start(40)

    def _subscribe(self, color, port):
        device = 'udpin:localhost:%d' % port
        print('listening to %s' % device)
        conn = mavlink_connection(device)

        def run():
            while True:
                try:
                    msg = conn.recv_match(blocking=True, timeout=1.0)
                except socket.error:
                    return
                if msg:
                    self._message_callback(color, msg)

        thread = Thread(target=run, name=color)
        thread.start()
        self.subscribers[color] = (conn, thread)

    def _message_callback(self, color, msg):
        # MAVLink_altitude_message
        # MAVLink_attitude_message
        # MAVLink_attitude_quaternion_message
        # MAVLink_attitude_target_message
        # MAVLink_battery_status_message
        # MAVLink_extended_sys_state_message
        # MAVLink_global_position_int_message
        # MAVLink_gps_raw_int_message
        # MAVLink_heartbeat_message
        # MAVLink_highres_imu_message
        # MAVLink_home_position_message
        # MAVLink_local_position_ned_message
        # MAVLink_nav_controller_output_message
        # MAVLink_sys_status_message
        # MAVLink_vfr_hud_message
        # MAVLink_vibration_message
        # MAVLink_wind_cov_message
        msg_type_name = type(msg).__name__
        if msg_type_name not in self._received_msg_types:
            # print(msg_type_name)
            self._received_msg_types.append(msg_type_name)

        if isinstance(msg, MAVLink_param_value_message):
            if msg.param_id == 'MAV_TYPE':
                src_system = msg.get_srcSystem()
                mav_type = self._float_to_int(msg.param_value)
                print('vehicle #%d is of type %s' % (src_system, mav_type))
                self._vehicle_types[src_system] = mav_type

        if isinstance(msg, MAVLink_global_position_int_message):
            src_system = msg.get_srcSystem()
            if src_system not in self._vehicle_types:
                self._vehicle_types[src_system] = None
                self._request_vehicle_type(color, src_system)
                return
            if self._vehicle_types[src_system]:
                update = (
                    color,
                    src_system,
                    self._vehicle_types[src_system],
                    {
                        'lat': 1.0 * msg.lat / 10000000,
                        'lon': 1.0 * msg.lon / 10000000,
                    }
                )
                with QMutexLocker(self._mutex):
                    self._update_queue.append(update)

    def _float_to_int(self, value):
        p = struct.pack('<f', value)
        val = struct.unpack('I', p)
        return val[0]

    def _request_vehicle_type(self, color, src_system):
        print('requesting vehicle type of %s #%d' % (color, src_system))
        (conn, _) = self.subscribers[color]
        conn.mav.param_request_read_send(src_system, 1, 'MAV_TYPE', -1)

    def _update_model(self):
        with QMutexLocker(self._mutex):
            updates = self._update_queue
            self._update_queue = []
        for update in updates:
            self._widget.update_vehicle(*update)

    def shutdown_plugin(self):
        for (conn, thread) in self.subscribers.values():
            conn.close()
            thread.join(1)
