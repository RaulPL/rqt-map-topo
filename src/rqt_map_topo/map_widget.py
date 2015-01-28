__author__ = 'Raul Peralta-Lozada'
__email__ = 'raulpl25@gmail.com'

"""
This rqt_plugin was mostly based on the rqt_bag and rqt_nav_view plugins.
"""


import os
import yaml

import rospy
import rospkg
import numpy

from math import radians, degrees, cos, sin

from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, qWarning, Signal, QRectF, QLineF, QEvent, QObject
from python_qt_binding.QtGui import (QFileDialog, QGraphicsView, QIcon, QWidget,
                                     QMessageBox, QPixmap, qRgb, QImage, QGraphicsScene,
                                     QGraphicsTextItem, QColor, QPen, QFont)


class MapTopoGraphicsView(QGraphicsView):
    def __init__(self, parent=None):
        super(MapTopoGraphicsView, self).__init__()


class MapTopoScene(QGraphicsScene):
    def __init__(self,):
        super(MapTopoScene, self).__init__()


class MapTopoWidget(QWidget):
    """
    Widget for use with MapTopo class to display and save semantic locations
    Handles all widget callbacks
    """
    map_read = Signal()
    def __init__(self, context):
        """
        :param context: plugin context hook to enable adding widgets as a ROS_GUI pane, ''PluginContext''
        """
        super(MapTopoWidget, self).__init__()

        self.edit_mode = False

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_map_topo'), 'resource', 'map_widget.ui')
        loadUi(ui_file, self, {'MapTopoGraphicsView': MapTopoGraphicsView})

        self.setObjectName('MapTopoWidget')
        self._scene = MapTopoScene()
        self.graphics_view.setScene(self._scene)
        self.places_dict = {}  # places to be stored in a yaml file
        self.map = None
        self._clicked_pos = None
        self._grid_pos = None
        self._sceneItems = {}  # map that will store all the scene items for each place
        self._mag = 10  # magnitude in pixels of the line drawn

        self.load_button.setIcon(QIcon.fromTheme('document-open'))
        self.save_button.setIcon(QIcon.fromTheme('document-save'))
        self.theta_spin.setRange(-359.99, +359.99)

        self.add_button.clicked[bool].connect(self._handle_add_clicked)
        self.load_button.clicked[bool].connect(self._handle_load_clicked)
        self.save_button.clicked[bool].connect(self._handle_save_clicked)
        self.map_button.clicked[bool].connect(self._handle_read_map)
        self.get_button.clicked[bool].connect(self._handle_get_clicked)
        self.remove_button.clicked[bool].connect(self._handle_remove_clicked)
        self.theta_spin.valueChanged.connect(self._theta_changed)
        self.x_spin.valueChanged.connect(self._x_changed)
        self.y_spin.valueChanged.connect(self._y_changed)
        self.graphics_view.mousePressEvent = self.on_mouse_down
        self.graphics_view.wheelEvent = self.on_wheel_event

        self.add_button.setEnabled(False)
        self.load_button.setEnabled(False)
        self.save_button.setEnabled(False)
        self.name_edit.setEnabled(False)
        self.x_spin.setEnabled(False)
        self.y_spin.setEnabled(False)
        self.theta_spin.setEnabled(False)
        self.get_button.setEnabled(False)
        self.remove_button.setEnabled(False)
        self.graphics_view.setEnabled(False)

        self.map_read.connect(self._update_map)


    def _theta_changed(self, value):
        name = self.name_edit.text()
        if (len(name)!=0) and (name in self.places_dict.keys()):
            self._scene.removeItem(self._sceneItems[name]["line"])
            grid_pos = self.places_dict[name]
            scene_pos = self._gridToScene(grid_pos[0], grid_pos[1], numpy.asarray(grid_pos[3:7]))
            line = QLineF(scene_pos[0], scene_pos[1],
                          (scene_pos[0] - self._mag*cos(radians(value))),
                          (scene_pos[1] + self._mag*sin(radians(value)))
            )
            self._sceneItems[name]["line"] = self._scene.addLine(line, pen=QPen(Qt.red, 2))
            q = quaternion_from_euler(0.0, 0.0, radians(value))
            self.places_dict[name][3] = float(q[0])
            self.places_dict[name][4] = float(q[1])
            self.places_dict[name][5] = float(q[2])
            self.places_dict[name][6] = float(q[3])

    def _x_changed(self, value):
        name = self.name_edit.text()
        if (len(name)!=0) and (name in self.places_dict.keys()):
            self._scene.removeItem(self._sceneItems[name]["line"])
            self._scene.removeItem(self._sceneItems[name]["rec"])
            grid_pos = self.places_dict[name]
            scene_pos = self._gridToScene(value, grid_pos[1], numpy.asarray(grid_pos[3:7]))
            x_c = scene_pos[0] - self._sceneItems[name]["text"].boundingRect().width()/2.0
            self._sceneItems[name]["text"].setPos(x_c, scene_pos[1])
            self._sceneItems[name]["rec"] = self._scene.addRect(QRectF(
                (scene_pos[0] - 2), (scene_pos[1] - 2), 4, 4))
            line = QLineF(scene_pos[0], scene_pos[1],
                          (scene_pos[0] - self._mag*cos(radians(scene_pos[3]))),
                          (scene_pos[1] + self._mag*sin(radians(scene_pos[3])))
            )
            self._sceneItems[name]["line"] = self._scene.addLine(line, pen=QPen(Qt.red, 2))
            self.places_dict[name][0] = value


    def _y_changed(self, value):
        name = self.name_edit.text()
        if (len(name)!=0) and (name in self.places_dict.keys()):
            self._scene.removeItem(self._sceneItems[name]["line"])
            self._scene.removeItem(self._sceneItems[name]["rec"])
            grid_pos = self.places_dict[name]
            scene_pos = self._gridToScene(grid_pos[0], value, numpy.asarray(grid_pos[3:7]))
            x_c = scene_pos[0] - self._sceneItems[name]["text"].boundingRect().width()/2.0
            self._sceneItems[name]["text"].setPos(x_c, scene_pos[1])
            self._sceneItems[name]["rec"] = self._scene.addRect(QRectF(
                (scene_pos[0] - 2), (scene_pos[1] - 2), 4, 4))
            line = QLineF(scene_pos[0], scene_pos[1],
                          (scene_pos[0] - self._mag*cos(radians(scene_pos[3]))),
                          (scene_pos[1] + self._mag*sin(radians(scene_pos[3])))
            )
            self._sceneItems[name]["line"] = self._scene.addLine(line, pen=QPen(Qt.red, 2))
            self.places_dict[name][1] = value

    def _handle_add_clicked(self):
        name = self.name_edit.text()
        if len(name)!=0:
            # remove and re-draw it
            if name in self.places_dict.keys():
                self.places_dict.pop(name)
                for item in self._sceneItems[name].keys():
                    self._scene.removeItem(self._sceneItems[name][item])
            try:
                # storing the values in the dict
                x = self.x_spin.value()
                y = self.y_spin.value()
                theta = self.theta_spin.value()
                q = quaternion_from_euler(0.0, 0.0, theta)
                self.places_dict[str(name)] = [x, y, 0.0, float(q[0]), float(q[1]), float(q[2]), float(q[3])]

                # drawing the items
                self._sceneItems[name] = {"text":QGraphicsTextItem()}
                self._sceneItems[name]["text"].setDefaultTextColor(QColor(0, 0, 255))
                self._sceneItems[name]["text"].setFont(QFont("Times", 10))
                self._sceneItems[name]["text"].setPlainText(name)
                scene_pos = self._gridToScene(x, y, q)
                x_c = scene_pos[0] - self._sceneItems[name]["text"].boundingRect().width()/2.0
                self._sceneItems[name]["text"].setPos(x_c, scene_pos[1])
                self._scene.addItem(self._sceneItems[name]["text"])
                self._sceneItems[name]["rec"] = self._scene.addRect(QRectF(
                    (scene_pos[0] - 2), (scene_pos[1] - 2), 4, 4))
                line = QLineF(scene_pos[0], scene_pos[1],
                              (scene_pos[0] - self._mag*cos(radians(scene_pos[3]))),
                              (scene_pos[1] + self._mag*sin(radians(scene_pos[3])))
                )
                self._sceneItems[name]["line"] = self._scene.addLine(line, pen=QPen(Qt.red, 2))

            except ValueError:
                QMessageBox.critical(self, "Error!", "You must insert a valid value.")
        else:
            QMessageBox.critical(self, "Error!", "You have to insert a name and a valid position.")

    def _handle_load_clicked(self):
        filename = QFileDialog.getOpenFileName(
            self, self.tr('Load from File'), '.', self.tr('YAML files {.yaml} (*.yaml)'))
        if filename[0] != '':
            with open(filename[0], 'r') as infile:
                try:
                    self.places_dict = yaml.load(infile)
                    for k in self._sceneItems.keys():
                        for item in self._sceneItems[k].keys():
                            self._scene.removeItem(self._sceneItems[k][item])
                    for name in self.places_dict.keys():
                        #
                        q = numpy.asarray(self.places_dict[name][3:7])
                        scene_pos = self._gridToScene(self.places_dict[name][0], self.places_dict[name][1], q)

                        # drawing the items
                        self._sceneItems[name] = {"text":QGraphicsTextItem()}
                        self._sceneItems[name]["text"].setDefaultTextColor(QColor(0, 0, 255))
                        self._sceneItems[name]["text"].setFont(QFont("Times", 10))
                        self._sceneItems[name]["text"].setPlainText(name)
                        x_c = scene_pos[0] - self._sceneItems[name]["text"].boundingRect().width()/2.0
                        self._sceneItems[name]["text"].setPos(x_c, scene_pos[1])
                        self._scene.addItem(self._sceneItems[name]["text"])
                        self._sceneItems[name]["rec"] = self._scene.addRect(QRectF(
                            (scene_pos[0] - 2), (scene_pos[1] - 2), 4, 4))
                        line = QLineF(scene_pos[0], scene_pos[1],
                                      (scene_pos[0] - self._mag*cos(radians(scene_pos[3]))),
                                      (scene_pos[1] + self._mag*sin(radians(scene_pos[3])))
                        )
                        self._sceneItems[name]["line"] = self._scene.addLine(line, pen=QPen(Qt.red, 2))
                except yaml.scanner.ScannerError:
                    QMessageBox.critical(self, "Error!", "Invalid YAML file.")

    def _handle_get_clicked(self):
        name = self.name_edit.text()
        if (name in self.places_dict.keys()):
            self.x_spin.setValue(self.places_dict[name][0])
            self.y_spin.setValue(self.places_dict[name][1])
            theta = euler_from_quaternion(numpy.asarray(self.places_dict[name][3:7]))[2]
            self.theta_spin.setValue(degrees(theta))
        else:
            QMessageBox.critical(self, "Error!", "This places has not been added.")

    def _gridToScene(self, x, y, q):
        x = x - self.map.info.origin.position.x
        y = y - self.map.info.origin.position.y
        x = x * (1.0/self.map.info.resolution)
        y = y * (1.0/self.map.info.resolution)
        x_s = (self.map.info.width/2.0) + (self.map.info.width/2.0) - x
        theta = euler_from_quaternion(q)[2]
        return [int(x_s), int(y), 0.0, degrees(theta)]

    def _sceneToGrid(self, x, y, theta):
        theta = radians(theta)
        q = quaternion_from_euler(0.0, 0.0, theta)
        x_g = ((self.map.info.width/2.0) - x) + (self.map.info.width/2.0)
        map_g = (x_g * self.map.info.resolution, y * self.map.info.resolution)
        ox = self.map.info.origin.position.x
        oy = self.map.info.origin.position.y
        return  [(map_g[0] + ox), (map_g[1] + oy), 0.0, float(q[0]), float(q[1]), float(q[2]), float(q[3])]

    def _handle_save_clicked(self):
        filename = QFileDialog.getSaveFileName(
            self, self.tr('Save to file...'), '.', self.tr('YAML files {.yaml} (*.yaml)'))
        if filename[0] != '':
            with open(filename[0], 'w') as outfile:
                outfile.write(yaml.dump(self.places_dict, default_flow_style=False))

    def _handle_remove_clicked(self):
        name = self.name_edit.text()
        if name in self.places_dict.keys():
            self.places_dict.pop(name)
            for item in self._sceneItems[name].keys():
                self._scene.removeItem(self._sceneItems[name][item])
        else:
            QMessageBox.critical(self, "Error!", "Place does not exist.")

    def on_wheel_event(self, event):
        if event.delta() > 0:
            self.graphics_view.scale(1.1, 1.1)
        else:
            self.graphics_view.scale(0.9, 0.9)

    def on_mouse_down(self, event):
        if event.buttons() == Qt.LeftButton:
            clicked_pos = self.graphics_view.mapToScene(event.pos().x(), event.pos().y())
            grid_pos = self._sceneToGrid(clicked_pos.x(), clicked_pos.y(), 0.0)
            self.x_spin.setValue(grid_pos[0])
            self.y_spin.setValue(grid_pos[1])

    def _handle_read_map(self):
        try:
            self.map = rospy.wait_for_message("/map", OccupancyGrid, timeout=5)
            self.add_button.setEnabled(True)
            self.load_button.setEnabled(True)
            self.save_button.setEnabled(True)
            self.name_edit.setEnabled(True)
            self.x_spin.setEnabled(True)
            self.y_spin.setEnabled(True)
            self.theta_spin.setEnabled(True)
            self.get_button.setEnabled(True)
            self.remove_button.setEnabled(True)
            self.graphics_view.setEnabled(True)
            self.graphics_view.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
            self.graphics_view.setDragMode(QGraphicsView.ScrollHandDrag)
            self.graphics_view.setInteractive(True)
            self.map_read.emit()
        except rospy.exceptions.ROSException:
            QMessageBox.critical(self, "Error!", "Unable to read the map. Make sure to run the map_server node.")

    def _update_map(self):
        """
        Method used in the rqt_nav_view plugin to read the image from the map server.
        """
        a = numpy.array(self.map.data, dtype=numpy.uint8, copy=False, order='C')
        a = a.reshape((self.map.info.height, self.map.info.width))
        if self.map.info.width % 4:
            e = numpy.empty((self.map.info.height, 4 - self.map.info.width % 4), dtype=a.dtype, order='C')
            a = numpy.append(a, e, axis=1)
        image = QImage(a.reshape((a.shape[0] * a.shape[1])),
                       self.map.info.width, self.map.info.height, QImage.Format_Indexed8)

        for i in reversed(range(101)):
            image.setColor(100 - i, qRgb(i* 2.55, i * 2.55, i * 2.55))
        image.setColor(101, qRgb(255, 0, 0))  # not used indices
        image.setColor(255, qRgb(200, 200, 200))  # color for unknown value -1
        self._map = image
        self.graphics_view.setSceneRect(0, 0, self.map.info.width, self.map.info.height)

        pixmap = QPixmap.fromImage(self._map)
        self._map_item = self._scene.addPixmap(pixmap)

        # Everything must be mirrored
        self._map_item.scale(-1, 1)
        self._map_item.translate(-1*self.map.info.width, 0)

        # Add drag and drop functionality
        # self.add_dragdrop(self._map_item)

        self.graphics_view.centerOn(self._map_item)
        self.graphics_view.show()
