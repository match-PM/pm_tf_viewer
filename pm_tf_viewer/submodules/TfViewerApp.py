from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QThread, pyqtSlot, QRunnable, QObject, QThreadPool
from PyQt6.QtWidgets import (QScrollArea, 
                             QMessageBox, 
                             QDialog, 
                             QHBoxLayout, 
                             QDialog, 
                             QInputDialog, 
                             QTreeWidget, 
                             QTreeWidgetItem, 
                             QApplication, 
                             QGridLayout, 
                             QFrame, 
                             QMainWindow, 
                             QListWidget, 
                             QListWidgetItem, 
                             QDoubleSpinBox, 
                             QWidget, 
                             QVBoxLayout, 
                             QPushButton, 
                             QCheckBox, 
                             QLineEdit, 
                             QComboBox, 
                             QTextEdit,
                             QLabel,
                             QSlider, 
                             QSpinBox, 
                             QFontDialog, 
                             QFileDialog,
                             QTableWidgetItem,
                            QTableWidget,
                            )

from scipy.spatial.transform import Rotation as R
from PyQt6.QtGui import QColor, QTextCursor, QFont, QAction
from functools import partial
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException

from geometry_msgs.msg import TransformStamped, Transform

import rclpy
import yaml


class TfViewerAppConfig():
    def __init__(self):
        self._parent_frame = 'world'
        self._existing_frames:list[str] = []
        self._lateral_unit = 'm'
        self._angular_unit = 'rad'
        self._lateral_unit_list = ['m', 'cm', 'mm', 'um', 'nm']
        self._angular_unit_list = ['rad', 'deg', 'Quaternion']
        self._lateral_unit_conversion = {
            'm': 1.0,
            'cm': 1e2,
            'mm': 1e3,
            'um': 1e6,
            'nm': 1e9
        }
        self._angular_unit_conversion = {
            'rad': 1.0,
            'deg': 3.141592653589793 / 180.0,
            'Quaternion': None
        }

    def get_parent_frame(self):
        return self._parent_frame
    
    def set_parent_frame(self, parent_frame):
        if parent_frame in self._existing_frames:
            self._parent_frame = parent_frame
        else:
            self._parent_frame = 'world'
        
    def get_lateral_unit(self): 
        return self._lateral_unit
    
    def set_lateral_unit(self, lateral_unit):
        if lateral_unit in self._lateral_unit_list:
            self._lateral_unit = lateral_unit
        else:
            raise ValueError(f"Invalid lateral unit: {lateral_unit}")
        
    def get_angular_unit(self):
        return self._angular_unit
    
    def set_angular_unit(self, angular_unit):
        if angular_unit in self._angular_unit_list:
            self._angular_unit = angular_unit
        else:
            raise ValueError(f"Invalid angular unit: {angular_unit}")
        
    def get_lateral_unit_conversion(self):
        return self._lateral_unit_conversion[self._lateral_unit]
    
    def get_angular_unit_conversion(self):
        return self._angular_unit_conversion[self._angular_unit]

    def update_existing_frames(self, frames:list[str]):
        self._existing_frames = frames
        # add world at the beginning
        if 'world' not in self._existing_frames:
            self._existing_frames.insert(0, 'world')
    
    def get_lateral_unit_list(self):
        return self._lateral_unit_list
    
    def get_angular_unit_list(self):
        return self._angular_unit_list
    
    def get_existing_frames(self):
        return self._existing_frames
    
class TfViewer():
    def __init__(self, ros_node:Node, 
                 config: TfViewerAppConfig):
        self.config = config
        self.ros_node = ros_node
        self.buffer = Buffer(node=ros_node)
        self.listener = TransformListener(self.buffer, ros_node)
        self._frames: list[TransformStamped] = []

    def update_tf_frames(self):
        try:
            frames:dict = yaml.safe_load(self.buffer.all_frames_as_yaml())
            
            self._frames = []
            keys = list(frames.keys())

            for key in keys:
                parent_frame = self.config.get_parent_frame()
                child_frame = key

                try:
                    trans = self.buffer.lookup_transform(parent_frame, child_frame, rclpy.time.Time())
                    trans.transform.translation.x *= self.config.get_lateral_unit_conversion()
                    trans.transform.translation.y *= self.config.get_lateral_unit_conversion()
                    trans.transform.translation.z *= self.config.get_lateral_unit_conversion()
                    self._frames.append(trans)

                except Exception as e:
                    pass
                    self.ros_node.get_logger().warn(f"Failed to lookup transform from {parent_frame} to {child_frame}: {e}")

        except Exception as e:
            self.ros_node.get_logger().warn(f"Failed to get frames: {e}")
            self._frames = []

    def get_frames(self):
        return self._frames
    
    def get_existing_frame_names(self)->list[str]:
        """
        Returns a list of existing frame names.
        """
        try:
            frames = yaml.safe_load(self.buffer.all_frames_as_yaml())
            return list(frames.keys())
        except Exception as e:
            self.ros_node.get_logger().warn(f"Failed to get frames: {e}")
            return []

def create_readonly_lineedit(value: str) -> QLineEdit:
    le = QLineEdit(value)
    le.setReadOnly(True)
    le.setFrame(False)  # optional: make it look more like a label
    le.setStyleSheet("background: transparent;")  # optional: transparent bg
    return le

class TfTransformViewerItem(QWidget):
    def __init__(self, tf:TransformStamped, 
                 app_config: TfViewerAppConfig):
        super().__init__()
        self.tf = tf
        self.main_layout = QVBoxLayout()
        self.setLayout(self.main_layout)
        self.app_config = app_config

        # Create widgets for displaying transform data
        self.child_frame_id_label = QLabel(f"Frame: {tf.child_frame_id}")

        # Add widgets to layout
        #self.main_layout.addWidget(self.frame_id_label)
        self.main_layout.addWidget(self.child_frame_id_label)

        transform_layout = QGridLayout()
        # Section headers
        transform_layout.addWidget(QLabel("Translation:"), 0, 0, 1, 2)
        transform_layout.addWidget(QLabel("Rotation:"),    0, 2, 1, 2)

        # Translation labels and values
        transform_layout.addWidget(QLabel("x:"), 1, 0)
        transform_layout.addWidget(create_readonly_lineedit(f"{tf.transform.translation.x:.9f}"), 1, 1)
        transform_layout.addWidget(QLabel("y:"), 2, 0)
        transform_layout.addWidget(create_readonly_lineedit(f"{tf.transform.translation.y:.9f}"), 2, 1)
        transform_layout.addWidget(QLabel("z:"), 3, 0)
        transform_layout.addWidget(create_readonly_lineedit(f"{tf.transform.translation.z:.9f}"), 3, 1)

        if self.app_config.get_angular_unit() == 'Quaternion':
            # Rotation labels and values
            transform_layout.addWidget(QLabel("x:"), 1, 2)
            transform_layout.addWidget(create_readonly_lineedit(f"{tf.transform.rotation.x:.9f}"), 1, 3)
            transform_layout.addWidget(QLabel("y:"), 2, 2)
            transform_layout.addWidget(create_readonly_lineedit(f"{tf.transform.rotation.y:.9f}"), 2, 3)
            transform_layout.addWidget(QLabel("z:"), 3, 2)
            transform_layout.addWidget(create_readonly_lineedit(f"{tf.transform.rotation.z:.9f}"), 3, 3)
            transform_layout.addWidget(QLabel("w:"), 4, 2)
            transform_layout.addWidget(create_readonly_lineedit(f"{tf.transform.rotation.w:.9f}"), 4, 3)
        else:
            # calculate_euler_angles from quaternion
            # Rotation labels and values
            qx = tf.transform.rotation.x
            qy = tf.transform.rotation.y
            qz = tf.transform.rotation.z
            qw = tf.transform.rotation.w

            # Create a Rotation object from the quaternion
            r = R.from_quat([qx, qy, qz, qw])

            # Convert to Euler angles (in radians by default)
            # You can use 'xyz' or 'zyx' depending on your convention
            if self.app_config.get_angular_unit() == 'rad':
                roll, pitch, yaw = r.as_euler('xyz', degrees=False)
            elif self.app_config.get_angular_unit() == 'deg':
                roll, pitch, yaw = r.as_euler('xyz', degrees=True)
            else:
                raise ValueError("Invalid angular unit")
            transform_layout.addWidget(QLabel("roll:"), 1, 2)
            transform_layout.addWidget(create_readonly_lineedit(f"{roll:.9f}"), 1, 3)
            transform_layout.addWidget(QLabel("pitch:"), 2, 2)
            transform_layout.addWidget(create_readonly_lineedit(f"{pitch:.9f}"), 2, 3)
            transform_layout.addWidget(QLabel("yaw:"), 3, 2)
            transform_layout.addWidget(create_readonly_lineedit(f"{yaw:.9f}"), 3, 3)

        # Add transform layout to main layout
        self.main_layout.addLayout(transform_layout)
        self.main_layout.addStretch()


class TfTransformViewerListWidget(QListWidget):
    def __init__(self, tf_list:list[TransformStamped], 
                 app_config: TfViewerAppConfig):
        super().__init__()
        self.app_config = app_config
        #self.tf_list = tf_list

    def update_tf_list(self, tf_list: list[TransformStamped]):
        self.clear()
        for tf in tf_list:
            item = QListWidgetItem(self)
            widget = TfTransformViewerItem(tf,self.app_config)

            # Important: set the size hint!
            item.setSizeHint(widget.sizeHint())

            self.addItem(item)
            self.setItemWidget(item, widget)


class TfViewerApp(QMainWindow):
    def __init__(self, ros_node:Node):
        super().__init__()
        self.service_node = ros_node

        self.config = TfViewerAppConfig()
        self.tf_viewer = TfViewer(ros_node,
                                  self.config)

        self.setWindowTitle("TF Viewer")
        self.main_layout = QGridLayout()
        self.main_widget = QWidget()
        self.main_widget.setLayout(self.main_layout)
        self.setCentralWidget(self.main_widget)

            # def refresh button
        self.refresh_button = QPushButton("Refresh Transforms")
        self.refresh_button.clicked.connect(self.refresh_button_event)
        self.main_layout.addWidget(self.refresh_button)

        # create drop down menu with sting list
        self.parent_frame_combo = QComboBox(self)
        self.parent_frame_combo.activated.connect(self.on_parent_frame_activated)
        self.main_layout.addWidget(self.parent_frame_combo)

        self.lateral_unit_combo = QComboBox(self)
        self.lateral_unit_combo.addItems(self.config.get_lateral_unit_list())
        self.lateral_unit_combo.setCurrentText(self.config.get_lateral_unit())
        self.lateral_unit_combo.currentTextChanged.connect(self.lateral_unit_changed_event)
        self.main_layout.addWidget(self.lateral_unit_combo)
        self.angular_unit_combo = QComboBox(self)
        self.angular_unit_combo.addItems(self.config.get_angular_unit_list())
        self.angular_unit_combo.setCurrentText(self.config.get_angular_unit())
        self.angular_unit_combo.currentTextChanged.connect(self.angular_unit_changed_event)
        self.main_layout.addWidget(self.angular_unit_combo)

        self.tf_viewer_list = TfTransformViewerListWidget(self.tf_viewer._frames, 
                                                         self.config)
        
        self.main_layout.addWidget(self.tf_viewer_list)


        # update the list initially
        self.update_list()

        ## Timer to update table
        #self.timer = QTimer(self)
        #self.timer.timeout.connect(self.update_list)
        #self.timer.start(5000)  # update every second

    def on_parent_frame_activated(self, index: int):
        selected_text = self.parent_frame_combo.itemText(index)
        self.parent_frame_changed_event(selected_text)

    def angular_unit_changed_event(self):
        self.config.set_angular_unit(self.angular_unit_combo.currentText())
        self.update_list()
        self.angular_unit_combo.setCurrentText(self.config.get_angular_unit())

    def lateral_unit_changed_event(self):
        self.config.set_lateral_unit(self.lateral_unit_combo.currentText())
        self.update_list()
        self.lateral_unit_combo.setCurrentText(self.config.get_lateral_unit())

    def update_parent_list(self):
        self.parent_frame_combo.clear()
        self.parent_frame_combo.addItems(self.config.get_existing_frames())
        self.parent_frame_combo.setCurrentText(self.config.get_parent_frame())

    def update_list(self):
        self.tf_viewer.update_tf_frames()
        self.tf_viewer_list.update_tf_list(self.tf_viewer.get_frames())
        self.config.update_existing_frames(self.tf_viewer.get_existing_frame_names())
        
    def parent_frame_changed_event(self, parent_frame_name:str):
        current_parent_frame = parent_frame_name
        self.config.set_parent_frame(current_parent_frame)
        self.update_list()
        self.parent_frame_combo.setCurrentText(self.config.get_parent_frame())

    def refresh_button_event(self):
        self.update_list()
        self.update_parent_list()