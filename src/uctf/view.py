from python_qt_binding.QtCore import QMutex, QMutexLocker, QTimer
from qt_gui.plugin import Plugin

import rospy
from sensor_msgs.msg import NavSatFix

from uctf.control import get_namespaces
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

        self.subscribers = []
        self._subscribe('blue')
        self._subscribe('gold')

        self._timer = QTimer()
        self._timer.timeout.connect(self._update_model)
        self._timer.start(40)

    def _subscribe(self, color):
        print('_subscribe')
        namespaces = get_namespaces(color)
        for namespace in namespaces:
            subscriber = rospy.Subscriber(
                '%s/mavros/global_position/global' % namespace, NavSatFix,
                callback=self._position_callback,
                callback_args=[color, namespace])
            self.subscribers.append(subscriber)
        print('_subscribe done')

    def _position_callback(self, msg, color_and_namespace):
        color, namespace = color_and_namespace
        with QMutexLocker(self._mutex):
            update = (namespace, msg, color)
            self._update_queue.append(update)

    def _update_model(self):
        with QMutexLocker(self._mutex):
            updates = self._update_queue
            self._update_queue = []
        for update in updates:
            self._widget.update_vehicle(*update)

    def shutdown_plugin(self):
        while self.subscribers:
            self.subscribers.pop().unregister()
