from python_qt_binding.QtCore import QPointF, Qt
from python_qt_binding.QtGui import QBrush, QColor, QPen, QPolygonF
from python_qt_binding.QtWidgets import QGraphicsEllipseItem, \
    QGraphicsPolygonItem, QGraphicsRectItem, QGraphicsScene, QGraphicsView

from uctf import CUBE_LENGTH
from uctf import global_to_cube

SUPPORT_AREA_DEPTH = 50


class Widget(QGraphicsView):

    def __init__(self, parent=None):
        super(Widget, self).__init__(parent)
        self.setWindowTitle('U-CTF View')

        self._scene = QGraphicsScene()
        self._scene.setBackgroundBrush(Qt.white)

        # game cube grass area
        grass_color = QColor(0, 255, 0, 50)
        self._scene.addRect(
            0, 0, CUBE_LENGTH, CUBE_LENGTH,
            QPen(grass_color), QBrush(grass_color))
        # blue team area
        blue_color = QColor(0, 0, 255, 50)
        self._scene.addRect(
            0, -SUPPORT_AREA_DEPTH, CUBE_LENGTH, SUPPORT_AREA_DEPTH,
            QPen(blue_color), QBrush(blue_color))
        # gold team area
        gold_color = QColor(255, 215, 0, 50)
        self._scene.addRect(
            0, CUBE_LENGTH, CUBE_LENGTH, SUPPORT_AREA_DEPTH,
            QPen(gold_color), QBrush(gold_color))
        # penalty area
        orange_color = QColor(0, 200, 0, 50)
        self._scene.addRect(
            CUBE_LENGTH, 0, SUPPORT_AREA_DEPTH, CUBE_LENGTH,
            QPen(orange_color), QBrush(orange_color))

        # rotate view to match the coordinate system of the game cube
        self.rotate(180)
        self.setScene(self._scene)

        # pens and brushes for the vehicles
        self._pens = {
            'blue': QPen(Qt.blue, 2),
            'gold': QPen(QColor(191, 151, 0), 2),
        }
        self._brushes = {
            'blue': QBrush(Qt.blue),
            'gold': QBrush(QColor(191, 151, 0)),
        }

        self._vehicles = {}

    def update_vehicle(self, namespace, msg, color):
        if namespace not in self._vehicles:
            item = self._create_vehicle_item(namespace, color)
            self._scene.addItem(item)
            self._vehicles[namespace] = item
        else:
            item = self._vehicles[namespace]
        cube_pos = global_to_cube(msg.latitude, msg.longitude)
        item.setPos(*cube_pos)

        # set visible area
        padding = 10
        self.fitInView(
            -SUPPORT_AREA_DEPTH - padding, -SUPPORT_AREA_DEPTH - padding,
            CUBE_LENGTH + 2.0 * (SUPPORT_AREA_DEPTH + 2),
            CUBE_LENGTH + 2.0 * (SUPPORT_AREA_DEPTH + padding),
            Qt.KeepAspectRatio)

    def _create_vehicle_item(self, namespace, color):
        if namespace.startswith('iris'):
            # draw cross
            item = QGraphicsPolygonItem(QPolygonF([
                QPointF(0, 0),
                QPointF(-3, -3),
                QPointF(3, 3),
                QPointF(0, 0),
                QPointF(-3, 3),
                QPointF(3, -3),
                QPointF(0, 0),
            ]))
        elif namespace.startswith('delta_wing'):
            # draw circle
            item = QGraphicsEllipseItem(-3, -3, 6, 6)
        else:
            # draw square
            item = QGraphicsRectItem(-3, -3, 6, 6)
        item.setBrush(self._brushes[color])
        item.setPen(self._pens[color])
        item.setToolTip(namespace)
        return item
