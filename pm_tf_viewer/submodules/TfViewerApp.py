from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QThread, pyqtSlot, QRunnable, QObject, QThreadPool, QSize
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
                             QSizePolicy,
                             QTabWidget,
                             QAbstractItemView,
                             QGroupBox,
                             QStyle
                            )


from PyQt6.QtGui import QCursor, QColor, QPalette
from scipy.spatial.transform import Rotation as R
from PyQt6.QtGui import QTextCursor, QFont, QAction
from functools import partial
from rclpy.node import Node
from datetime import datetime
import copy

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
            #return sorted(list(frames.keys())) # Alphabetical order /doesnt work
            return list(frames.keys()) 
        except Exception as e:
            self.ros_node.get_logger().warn(f"Failed to get frames: {e}")
            return []

def create_readonly_lineedit(value: str) -> QLineEdit:
    le = QLineEdit(value)
    le.setReadOnly(True)
    le.setFrame(False) # optional: make it look more like a label
    le.setStyleSheet("background: transparent;") # optional: transparent bg
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
        self.child_frame_id_label = QLabel(f"{tf.child_frame_id}")

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
    
    # transform_selected = pyqtSignal(float, float, float, float, float, float, str)
    transform_ID_selected = pyqtSignal(str)
    
    def __init__(self, tf_list:list[TransformStamped], 
                 app_config: TfViewerAppConfig, 
                 ros_node: Node):
        super().__init__()
        self.itemDoubleClicked.connect(self.tf_data_on_click)  # Connect item click to method /// click twice
        #self.itemClicked.connect(self.tf_data_on_click)  # Clicked once 
        self.app_config = app_config
        self.ros_node = ros_node

    def sort_key(self, tf: TransformStamped): #Blueprint for sorting
        name = tf.child_frame_id
        if not name:
            return (2, "")
        first_char = name[0]
        if first_char.isalpha():
            return (0, name.lower())
        elif first_char.isdigit():
            return (1, name.lower())
        else:
            return (2, name.lower())

    def update_tf_list(self, tf_list: list[TransformStamped]):
        self.clear()
        sorted_tf_list = sorted(tf_list, key=self.sort_key)  # Second Attempt at sorting
        for tf in sorted_tf_list:
            item = QListWidgetItem(self)
            widget = TfTransformViewerItem(tf, self.app_config)

            # Important: set the size hint!
            item.setSizeHint(widget.sizeHint())
            item.setData(Qt.ItemDataRole.UserRole, tf)
            self.addItem(item)
            self.setItemWidget(item, widget)

    def tf_data_on_click(self, item: QListWidgetItem):

        try:
            transform: TransformStamped = item.data(Qt.ItemDataRole.UserRole)
            tf_id = transform.child_frame_id
            self.ros_node.get_logger().info(f"Transform ID selected: {tf_id}")  # Fixed: using ros_node
 
            self.transform_ID_selected.emit(tf_id)  

        except Exception as e:
            self.ros_node.get_logger().error(f"Error retrieving transform ID: {str(e)}")  
            pass  # Handle the case where transform data is not available or item is invalid

class AutoSizedWindow(QMainWindow):
    def __init__(self, ros_node: Node):
        QMainWindow.__init__(self)
        
        self.ros_node = ros_node  # Consistent naming
        self.ros_node.get_logger().info("AutoSizedWindow initialized, waiting for content to be set up.")
        
        # Central widget setup
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)
        
        # Configure window sizing behavior
        self.setup_window_sizing()
        
    
    def setup_window_sizing(self):
        """Configures automatic sizing with horizontal expansion and vertical limits"""
        try:
            # Calculate initial size
            self.adjustSize()
            
            # Get screen dimensions
            screen = QApplication.primaryScreen().availableGeometry()
            max_height = int(screen.height() * 0.8)  # 80% of screen height
            max_width = int(screen.width() * 0.9)    # 90% of screen width
            
            # Apply calculated dimensions
            width = min(self.width(), max_width)
            height = min(self.height(), max_height)
            
            self.resize(width, height)
            self.ros_node.get_logger().info(f"Window resized to {width}x{height}")  # Fixed: using ros_node
            
            # Set size constraints
            self.setMinimumWidth(width)  # Prevent narrower than content
            self.setMinimumHeight(200)   # Absolute minimum height
            self.setMaximumHeight(max_height)  # Cap vertical expansion
            
            # Size policy for central widget
            self.central_widget.setSizePolicy(
                QSizePolicy.Policy.Expanding,
                QSizePolicy.Policy.MinimumExpanding
            )
            
        except Exception as e:
            self.ros_node.get_logger().error(f"Window sizing failed: {str(e)}")
    
    def update_content(self):
        
        try:
            self.adjustSize()
            current_height = self.height()
            max_height = int(QApplication.primaryScreen().availableGeometry().height() * 0.9)
            
            if current_height > max_height:
                self.setFixedHeight(max_height)
            else:
                self.setMinimumHeight(current_height)
                self.setMaximumHeight(max_height)
                
            self.ros_node.get_logger().debug(f"Content updated, new height: {current_height}")
            
        except Exception as e:
            self.ros_node.get_logger().error(f"Content update failed: {str(e)}")

class CalculationPanel(QWidget):

    def __init__(self, app_config: TfViewerAppConfig, ros_node:Node):
        super().__init__()
        self.app_config = app_config
        self.ros_node = ros_node
        self.buffer = Buffer(node=ros_node)  # Define the buffer attribute
        self.listener = TransformListener(self.buffer, ros_node)
        self._frames: list[TransformStamped] = []
        QTimer.singleShot(500, self.update_tf_frames_live_data)  # Initial delay to allow TF data to populate
        
        self.frozen_data = {}

        # Hauptlayout: Horizontal (Textfeld links, Dropdowns rechts)
        self.side_layout = QHBoxLayout()  # <<-- vorher QVBoxLayout is aber hässlich also QHBoxLayout
        self.setLayout(self.side_layout)
        self.calc_slides = []  # Store all calculation slides
        # UI aufbauen
        
        self.setup_calc_panel()  # Textfeld für Ergebnisse
        self.setup_right_panel()
    
    def get_frozen_tf_data_for_slide_checkbox(self, slide_id: int, checkbox_id: str, label_text: str):
        return self.frozen_data.get(slide_id, {}).get(checkbox_id, {}).get(label_text, None)
    
    def freeze_tf_data_for_slide_checkbox(self, slide_id: int, checkbox_id: str):
        if slide_id not in self.frozen_data.keys():
            self.frozen_data[slide_id] = {}
            
        
        self.frozen_data[slide_id][checkbox_id] = copy.deepcopy(self.updated_frames_by_id)
        self.ros_node.get_logger().info(f"Frozen data for slide {slide_id}, checkbox {checkbox_id}")
    
    def unfreeze_tf_data_for_slide_checkbox(self, slide_id: int, checkbox_id: str):
        if slide_id in self.frozen_data and checkbox_id in self.frozen_data[slide_id]:
            self.frozen_data.get(slide_id).pop(checkbox_id, None)
    
    def remove_frozen_data_for_slide(self, slide_id: int):
        self.frozen_data.pop(slide_id, None)

    def setup_calc_panel(self):
        self.calc_panel = QWidget()
        self.calc_panel.setMinimumHeight(350)
        self.calc_layout = QVBoxLayout(self.calc_panel)
        self.calc_layout.stretch(1) # 
        # Create a scroll area
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)  # important to resize with content

        # Container inside scroll area
        self.slides_container = QWidget()
        self.slides_layout = QVBoxLayout(self.slides_container)
        self.slides_layout.addStretch()  # placeholder stretch

        self.scroll_area.setWidget(self.slides_container)
        self.calc_layout.addWidget(self.scroll_area)

        # Add the whole calc panel to the main layout
        self.side_layout.addWidget(self.calc_panel, 3)

    def setup_right_panel(self):
        # Create a container widget for the right panel
        right_panel_widget = QWidget()
        right_panel_widget.setFixedWidth(260)  # Set fixed width here
        
        # Your existing layout
        right_layout = QVBoxLayout()
        right_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        
        title_label = QLabel("Berechnungen und Einstellungen")
        title_label.setAlignment(Qt.AlignmentFlag.AlignTop)
        title_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        right_layout.addWidget(title_label)

        self.dropdown_ID_storage = QComboBox()
        self.dropdown_ID_storage.setToolTip("Select an ID via double clicking a Frame to store in the dropdowns")
        
        # Dropdowns mit Beschriftung
        right_layout.addWidget(QLabel("Translation ID Storage:"))
        right_layout.addWidget(self.dropdown_ID_storage)

        # Buttons
        self.translation_button = QPushButton("Calculate Translations")
        self.add_slide_button = QPushButton("Add Calculation Slide")
        # self.remove_slide_button = QPushButton("Remove Last Calculation Slide")
        self.clear_button = QPushButton("Clear Everything")
        # self.remove_all_slides_button = QPushButton("Delete all Slides")
        self.clear_dropdown_button = QPushButton("Clear ID Storage")
        
        right_layout.addWidget(self.translation_button)
        right_layout.addWidget(self.add_slide_button)
        # right_layout.addWidget(self.remove_slide_button)
        # right_layout.addWidget(self.remove_all_slides_button)
        right_layout.addWidget(self.clear_dropdown_button)
        right_layout.addWidget(self.clear_button)
        
        # Set the layout on the container widget
        right_panel_widget.setLayout(right_layout)
        
        # Add the container widget to your side layout instead of right_layout directly
        self.side_layout.addWidget(right_panel_widget, 1)

        # Events verbinden
        self.clear_button.clicked.connect(self.on_clear)
        self.translation_button.clicked.connect(self.auto_calculations) 
        self.add_slide_button.clicked.connect(self.add_calc_slide)
        # self.remove_slide_button.clicked.connect(self.remove_last_calc_slide)
        # self.remove_all_slides_button.clicked.connect(self.delete_slides)
        self.clear_dropdown_button.clicked.connect(self.dropdown_ID_storage.clear)

    def add_calc_slide(self, title: str = None):
        slide_number = len(self.calc_slides)
        title = title or f"Calculation Slide {slide_number + 1}"

        # Remove trailing stretch
        self.slides_layout.takeAt(self.slides_layout.count() - 1)

        # Pass self (the CalculationPanel instance) as the first parameter
        slide = CalcSlides(self, id=slide_number, title=title)  # <-- Add self here
        slide.setFixedHeight(260)
        slide.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.calc_slides.append(slide)
        self.slides_layout.addWidget(slide)

        # Add stretch again
        self.slides_layout.addStretch()

        self.update_calc_slide_dropdowns()

    def remove_last_calc_slide(self):
        if not self.calc_slides:
            return
        slide = self.calc_slides.pop()
        self.remove_frozen_data_for_slide(slide.slide_id)
        self.calc_layout.removeWidget(slide)
        slide.deleteLater()
    
    def delete_slides(self):
        number_of_slides = len(self.calc_slides)
        for _ in range(number_of_slides):
            self.remove_last_calc_slide()
        
        
    def receive_translation_data(self, frame_name: str):
        if not frame_name:
            frame_name = "No ID for Frame"

        self.clicked_frame = frame_name
        self.ros_node.get_logger().info(f"Transform ID selected: {frame_name}")
        
        if self.clicked_frame not in [self.dropdown_ID_storage.itemText(i) for i in range(self.dropdown_ID_storage.count())]:
            # Add to dropdowns only if not already present
            self.dropdown_ID_storage.addItem(self.clicked_frame)
            self.update_calc_slide_dropdowns()
    
    def get_current_timestamp(self):
        now = datetime.now()
        current_time = now.strftime("%H:%M:%S")
        return current_time

    def auto_calculations(self):
        self.ros_node.get_logger().info("Starting auto-calculations...")

        try:
            if len(self.calc_slides) > 0 and self.updated_frames_by_id:
                slide: CalcSlides
                for slide in self.calc_slides:
                    label1_text = slide.dropdown1.currentText()
                    label2_text = slide.dropdown2.currentText()

                    current_data = self.updated_frames_by_id

                    frame1 = self.get_frozen_tf_data_for_slide_checkbox(
                        slide.slide_id, slide.freeze1.checkbox_id, label1_text
                    )
                    frame2 = self.get_frozen_tf_data_for_slide_checkbox(
                        slide.slide_id, slide.freeze2.checkbox_id, label2_text
                    )

                    if frame1 is None:
                        self.ros_node.get_logger().info(
                            f"Using live data for slide {slide.slide_id + 1}, dropdown1: {label1_text}"
                        )
                        frame1 = current_data.get(label1_text)

                    if frame2 is None:
                        self.ros_node.get_logger().info(
                            f"Using live data for slide {slide.slide_id + 1}, dropdown2: {label2_text}"
                        )
                        frame2 = current_data.get(label2_text)

                    if frame1 is None or frame2 is None:
                        slide.clear_results()
                        continue

                    try:
                        # --- Extract translations ---
                        x1, y1, z1 = (
                            frame1.transform.translation.x,
                            frame1.transform.translation.y,
                            frame1.transform.translation.z,
                        )
                        x2, y2, z2 = (
                            frame2.transform.translation.x,
                            frame2.transform.translation.y,
                            frame2.transform.translation.z,
                        )

                        # --- Extract quaternions ---
                        qx1, qy1, qz1, qw1 = (
                            frame1.transform.rotation.x,
                            frame1.transform.rotation.y,
                            frame1.transform.rotation.z,
                            frame1.transform.rotation.w,
                        )
                        qx2, qy2, qz2, qw2 = (
                            frame2.transform.rotation.x,
                            frame2.transform.rotation.y,
                            frame2.transform.rotation.z,
                            frame2.transform.rotation.w,
                        )

                        # --- Apply unit conversion ---
                        lateral_conversion = self.app_config.get_lateral_unit_conversion()
                        x1 *= lateral_conversion
                        y1 *= lateral_conversion
                        z1 *= lateral_conversion
                        x2 *= lateral_conversion
                        y2 *= lateral_conversion
                        z2 *= lateral_conversion

                        lateral_unit = self.app_config.get_lateral_unit()

                        # --- Vector calculations ---
                        dx, dy, dz = x2 - x1, y2 - y1, z2 - z1
                        vector_length = (dx**2 + dy**2 + dz**2) ** 0.5

                        # --- Angular calculations ---
                        from scipy.spatial.transform import Rotation as R

                        rot1 = R.from_quat([qx1, qy1, qz1, qw1])
                        rot2 = R.from_quat([qx2, qy2, qz2, qw2])
                        rot_rel = rot2 * rot1.inv()

                        angular_unit = self.app_config.get_angular_unit()

                        if angular_unit.lower() in ["deg", "degree", "degrees"]:
                            roll, pitch, yaw = rot_rel.as_euler("xyz", degrees=True)
                            roll_text = f"ΔRoll = {roll:>8.2f}°"
                            pitch_text = f"ΔPitch = {pitch:>7.2f}°"
                            yaw_text = f"ΔYaw = {yaw:>10.2f}°"

                        elif angular_unit.lower() in ["rad", "radian", "radians"]:
                            roll, pitch, yaw = rot_rel.as_euler("xyz", degrees=False)
                            roll_text = f"ΔRoll = {roll:>8.3f} rad"
                            pitch_text = f"ΔPitch = {pitch:>7.3f} rad"
                            yaw_text = f"ΔYaw = {yaw:>10.3f} rad"
                            
                        elif angular_unit.lower() == "quaternion":
                            qx_rel, qy_rel, qz_rel, qw_rel = rot_rel.as_quat()
                            roll_text = f"Δqx = {qx_rel:>8.4f}"
                            pitch_text = f"Δqy = {qy_rel:>8.4f}"
                            yaw_text = f"Δqz = {qz_rel:>8.4f}"
                            qw_text = f"Δqw = {qw_rel:>8.4f}"
                        
                        else:
                            roll_text = f"ΔRoll = N/A"
                            pitch_text = f"ΔPitch = N/A"
                            yaw_text = f"ΔYaw = N/A"

                        # --- Update result labels ---
                        slide.calc_title_label.setText(f"Parent Frame = [{self.app_config.get_parent_frame()}]")

                        slide.pos1_label.setText(f"Frame 1 = [{label1_text}] at ({x1:.2f}, {y1:.2f}, {z1:.2f}) {lateral_unit}")
                        slide.pos2_label.setText(f"Frame 2 = [{label2_text}] at ({x2:.2f}, {y2:.2f}, {z2:.2f}) {lateral_unit}")
                        
                        slide.dx_label.setText(f"Δx = {dx:>8.3f} {lateral_unit}")
                        slide.dy_label.setText(f"Δy = {dy:>8.3f} {lateral_unit}")
                        slide.dz_label.setText(f"Δz = {dz:>8.3f} {lateral_unit}")
                        
                        slide.roll_label.setText(roll_text)
                        slide.pitch_label.setText(pitch_text)
                        slide.yaw_label.setText(yaw_text)
                        slide.qw_label.setText(qw_text if angular_unit.lower() == "quaternion" else "")
                        
                        slide.vector_length_label.setText(f"Vector length: |V| = {vector_length:.3f} {lateral_unit}")
                        slide.units_label.setText(f"Units: Lateral: [{lateral_unit}], Angular: [{angular_unit}]")

                    except (KeyError, AttributeError) as e:
                        slide.clear_results()
                        self.ros_node.get_logger().warn(f"Calculation error for slide {slide.slide_id}: {str(e)}")

            else:
                for slide in self.calc_slides:
                    slide.clear_results()

        except Exception as e:
            self.ros_node.get_logger().error(f"Auto calculation error: {str(e)}")
            for slide in self.calc_slides:
                slide.clear_results()

    def update_tf_frames_live_data(self):
            """Live-updates transform frame data and stores it in a name-based dictionary."""

            
            self.updated_frames_by_id = {}  # Reset existing
                

            try:
                frames: dict = yaml.safe_load(self.buffer.all_frames_as_yaml())
                keys = list(frames.keys())

                for key in keys:
                    parent_frame = self.app_config.get_parent_frame()
                    child_frame = key

                    try:
                        trans = self.buffer.lookup_transform(parent_frame, child_frame, rclpy.time.Time())

                        # Unit conversion
                        #trans.transform.translation.x *= self.app_config.get_lateral_unit_conversion()
                        #trans.transform.translation.y *= self.app_config.get_lateral_unit_conversion()
                        #trans.transform.translation.z *= self.app_config.get_lateral_unit_conversion()

                        # Save transform to dictionary by child_frame (key)
                        self.updated_frames_by_id[child_frame] = trans
                        

                    except Exception as e:
                        self.ros_node.get_loggerslide().warn(
                            f"Failed to lookup transform from {parent_frame} to {child_frame}: {e}"
                        )

            except Exception as e:
                self.ros_node.get_logger().warn(f"Failed to get frames: {e}")
    
    def update_calc_slide_dropdowns(self):
        """
        Refresh all dropdowns in every calculation slide based on self.dropdown_ID_storage.
        Keeps previous selections if still valid.
        """
        # Get current available names (Dropdown 1, Dropdown 2, ...)
        available_names = [self.dropdown_ID_storage.itemText(i) for i in range(self.dropdown_ID_storage.count())]

        # Loop through all slides
        for slide in self.calc_slides:
            # FIX: Access the dropdowns from each slide, not from self
            for dropdown in [slide.dropdown1, slide.dropdown2]:
                # Save current selection
                current_text = dropdown.currentText()

                # Block signals to avoid triggering connected events during update
                dropdown.blockSignals(True)
                dropdown.clear()
                dropdown.addItems(available_names)

                # Try to restore selection if still available
                if current_text in available_names:
                    dropdown.setCurrentText(current_text)

                dropdown.blockSignals(False)

    def on_clear(self):
        self.dropdown_ID_storage.clear()
        number_of_slides = len(self.calc_slides)
        for _ in range(number_of_slides):
            self.remove_last_calc_slide()
        self.frozen_data = {}
    
    def debug(self):
        
        #self.result_display.append("Debug Mode --> Translation data will be displayed.")
       #Problem noch nicht gelöst, dass der Debug Button nicht wie gewünscht funktioniert
        return True

class CalcSlides(QGroupBox):
    def __init__(self, calculation_panel: CalculationPanel, id: int, title: str = "Calculation Slide"):
        super().__init__(title)

        self.slide_id = id
        self.calculation_panel = calculation_panel
        
        # Create main layout
        main_layout = QVBoxLayout()
        
        # Create grid layout for better alignment
        grid_layout = QGridLayout()
        grid_layout.setColumnStretch(0, 0)  # Frame 1 column
        grid_layout.setColumnStretch(1, 0)  # Frame 2 column  
        grid_layout.setColumnStretch(2, 1)  # Results (takes most space)
        grid_layout.setHorizontalSpacing(15)  # Add spacing between columns
        
        # Row 0: Headers
        # grid_layout.addWidget(QLabel("Frame 1"), 0, 0)
        # grid_layout.addWidget(QLabel("Frame 2"), 0, 1)
        
        # Row 1: Dropdowns with freezes on the right
        frame1_layout = QHBoxLayout()
        self.dropdown1 = QComboBox()
        self.dropdown1.setFixedHeight(40)
        self.dropdown1.setFixedWidth(150)
        self.freeze1 = ClickableLabel(calculation_panel, calc_slide=self, checkbox_id="checkbox1", dropdown=self.dropdown1)
        self.freeze1.setToolTip("♻️ = Using live data /// ❄️ = Freeze current position")
        frame1_layout.addWidget(self.dropdown1)
        frame1_layout.addWidget(self.freeze1)
        frame1_layout.addSpacing(10)  # Small space between dropdown and freeze
        
        frame2_layout = QHBoxLayout()
        self.dropdown2 = QComboBox()
        self.dropdown2.setFixedWidth(150)
        self.dropdown2.setFixedHeight(40)
        self.freeze2 = ClickableLabel(calculation_panel, calc_slide=self, checkbox_id="checkbox2", dropdown=self.dropdown2)
        self.freeze2.setToolTip("♻️ = Using live data /// ❄️ = Freeze current position")
        frame2_layout.addWidget(self.dropdown2)
        frame2_layout.addWidget(self.freeze2)
        frame2_layout.addSpacing(10)  # Small space between dropdown and freeze
        
        # Create container widgets for the layouts
        frame1_widget = QWidget()
        frame1_widget.setLayout(frame1_layout)
        frame2_widget = QWidget()
        frame2_widget.setLayout(frame2_layout)
        
        grid_layout.addWidget(frame1_widget, 0, 0)
        grid_layout.addWidget(frame2_widget, 1, 0)
        
        # Result display (spans both rows for height consistency)
        self.result_display = QWidget()
        self.result_display.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.result_display.setMinimumWidth(550)
        self.result_display.setMinimumHeight(150)
        
        # Create a grid layout for the results
        self.results_grid = QGridLayout(self.result_display)
        self.results_grid.setVerticalSpacing(5)
        self.results_grid.setHorizontalSpacing(15)  # Increased spacing
        
        # Set column stretch to keep angles close to vectors
        self.results_grid.setColumnStretch(0, 0)  # Vector labels - won't expand
        self.results_grid.setColumnStretch(1, 0)  # Vector values - won't expand  
        self.results_grid.setColumnStretch(2, 0)  # Spacing column - fixed
        self.results_grid.setColumnStretch(3, 0)  # Rotation labels - won't expand
        self.results_grid.setColumnStretch(4, 0)  # Rotation values - won't expand
        
        grid_layout.addWidget(self.result_display, 0, 2, 2, 1)  # spans 2 rows from row 0
        
        main_layout.addLayout(grid_layout)
        self.setLayout(main_layout)
        
        # Initialize result labels
        self.init_result_labels()

    def init_result_labels(self):
        """Initialize all the labels for displaying results in grid format"""
        # Clear existing widgets
        for i in reversed(range(self.results_grid.count())):
            widget = self.results_grid.itemAt(i).widget()
            if widget:
                widget.setParent(None)
        
        # Create labels for transformation info
        row = 0
        
        # Title
        self.calc_title_label = QLabel("Parent_frame:")
        self.calc_title_label.setStyleSheet("font-weight: bold;")
        self.results_grid.addWidget(self.calc_title_label, row, 0, 1, 5)
        row += 1
        
        # Position labels
        self.pos1_label = QLabel("Frame 1 = ")
        self.pos2_label = QLabel("Frame 2 = ")
        self.results_grid.addWidget(self.pos1_label, row, 0, 1, 5)
        row += 1
        self.results_grid.addWidget(self.pos2_label, row, 0, 1, 5)
        row += 1
        
        # Empty row for spacing
        self.results_grid.addWidget(QLabel(""), row, 0)
        row += 1
        
        # Section headers
        vector_header = QLabel("Vector Difference:")
        rotation_header = QLabel("Rotation Difference:")
        vector_header.setStyleSheet("font-weight: bold;")
        rotation_header.setStyleSheet("font-weight: bold;")
        
        self.results_grid.addWidget(vector_header, row, 0, 1, 1)  # Span 2 columns
        self.results_grid.addWidget(rotation_header, row, 3, 1, 1)  # Span 2 columns
        row += 1
        
        # Vector and rotation values - now in separate columns for better control
        self.dx_label = QLabel("Δx = ")
        self.dy_label = QLabel("Δy = ")
        self.dz_label = QLabel("Δz = ")
        self.roll_label = QLabel("ΔRoll = ")
        self.pitch_label = QLabel("ΔPitch = ")
        self.yaw_label = QLabel("ΔYaw = ")
        self.qw_label = QLabel("Δqw = ")
        
        # Vector values (left side)
        self.results_grid.addWidget(self.dx_label, row, 0)    # Label
        self.results_grid.addWidget(QLabel(), row, 1)         # Value will be updated
        row += 1
        
        self.results_grid.addWidget(self.dy_label, row, 0)    # Label  
        self.results_grid.addWidget(QLabel(), row, 1)         # Value will be updated
        row += 1
        
        self.results_grid.addWidget(self.dz_label, row, 0)    # Label
        self.results_grid.addWidget(QLabel(), row, 1)         # Value will be updated
        row += 1
        
        # Reset row for rotation values (right side)
        row = 5  # Same row as vectors start
        
        # Rotation values (right side)
        self.results_grid.addWidget(self.roll_label, row, 3)  # Label
        self.results_grid.addWidget(QLabel(), row, 4)         # Value will be updated
        row += 1
        
        self.results_grid.addWidget(self.pitch_label, row, 3) # Label
        self.results_grid.addWidget(QLabel(), row, 4)         # Value will be updated
        row += 1
        
        self.results_grid.addWidget(self.yaw_label, row, 3)   # Label  
        self.results_grid.addWidget(QLabel(), row, 4)         # Value will be updated
        row += 1

        self.results_grid.addWidget(self.qw_label, row, 3)   # Label  
        self.results_grid.addWidget(QLabel(), row, 4)         # Value will be updated
        row += 1
        
        # Vector length and units
        self.vector_length_label = QLabel("Vector length: |V| = ")
        self.units_label = QLabel("Units: ")
        self.results_grid.addWidget(self.vector_length_label, row, 0, 1, 2)
        self.results_grid.addWidget(self.units_label, row, 3, 1, 2)
        
    def clear_results(self):
        """Clear all result displays"""
        self.pos1_label.setText("Frame 1 = ")
        self.pos2_label.setText("Frame 2 = ")
        self.dx_label.setText("Δx = ")
        self.dy_label.setText("Δy = ")
        self.dz_label.setText("Δz = ")
        self.roll_label.setText("ΔRoll = ")
        self.pitch_label.setText("ΔPitch = ")
        self.yaw_label.setText("ΔYaw = ")
        self.vector_length_label.setText("Vector length: |V| = ")
        self.units_label.setText("Units: ")



class ClickableLabel(QLabel):  # Remove CalculationPanel inheritance
    """Label that toggles between two symbols with a checkbox-style border."""
    def __init__(self, calculation_panel: CalculationPanel, calc_slide: CalcSlides, checkbox_id: str, dropdown: QComboBox, symbol_unchecked="♻️", symbol_checked="❄️",
                 color_unchecked="red", color_checked="blue", border_color="white"):
        super().__init__(symbol_unchecked)
        
        # Store reference to the actual CalculationPanel instance
        self.calculation_panel = calculation_panel
        self.calc_slide = calc_slide
        
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setFixedSize(35, 35)
        self.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
        self._checked = False
        self.symbol_unchecked = symbol_unchecked
        self.symbol_checked = symbol_checked
        self.color_unchecked = color_unchecked
        self.color_checked = color_checked
        self.border_color = border_color

        self.checkbox_id = checkbox_id
        self.dropdown = dropdown # Reference to the associated dropdown

        self.update_style()

    def mousePressEvent(self, event):
        self._checked = not self._checked
        self.update_style()
        if self._checked == True:
            self.calculation_panel.freeze_tf_data_for_slide_checkbox(self.calc_slide.slide_id, self.checkbox_id)
            self.calculation_panel.ros_node.get_logger().info(f"Freeze enabled for {self.checkbox_id} at slide {self.calc_slide.slide_id + 1}, using {self.dropdown.currentText()}.")
        elif self._checked == False:
            self.calculation_panel.unfreeze_tf_data_for_slide_checkbox(self.calc_slide.slide_id, self.checkbox_id)
            self.calculation_panel.ros_node.get_logger().info(f"Freeze disabled for {self.checkbox_id} at slide {self.calc_slide.slide_id + 1}, using live data for calculations.")
    # ... rest of the class remains the same
    def update_style(self):
        if self._checked:
            symbol = self.symbol_checked
            color = self.color_checked
        else:
            symbol = self.symbol_unchecked
            color = self.color_unchecked

        # Set text and styles including border and background
        self.setText(symbol)
        self.setStyleSheet(f"""
            color: {color};
            font-size: 20px;
            background-color: black;
            border: 2px solid {self.border_color};
            border-radius: 5px;
        """)

    def is_checked(self):
        return self._checked

class TfViewerApp(AutoSizedWindow):
    def __init__(self, ros_node:Node):
        AutoSizedWindow.__init__(self, ros_node)
        self.config = TfViewerAppConfig()
        
        
        self.service_node = ros_node
        self.tf_viewer = TfViewer(ros_node, self.config)
        
        self.setWindowTitle("TF Viewer")
        self.main_layout = QGridLayout()
        self.main_widget = QWidget()
        self.main_widget.setLayout(self.main_layout)
        self.setCentralWidget(self.main_widget)

        # Add CalculationPanel as a tab
        
        self.calc_panel = CalculationPanel(self.config, ros_node)
        self.tabs = QTabWidget(self)
        self.tabs.addTab(self.calc_panel, "Calculation Panel")
        self.main_layout.addWidget(self.tabs)

        # def refresh button
        self.refresh_button = QPushButton("Refresh Transforms")
        self.refresh_button.clicked.connect(self.refresh_button_event)
        self.main_layout.addWidget(self.refresh_button)

        # create drop down menu with sting listCalculationPanel.update_tf_frames_live_data(self)
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
        self.tf_viewer_list = TfTransformViewerListWidget(self.tf_viewer._frames, self.config, self.service_node)
        
        
        self.main_layout.addWidget(self.tf_viewer_list)
        
        self.tf_viewer_list.transform_ID_selected.connect(self.calc_panel.receive_translation_data)


        # update the list initially
        self.update_list()
        self.resize(1000, 800)  # Larger default size
        self.setMinimumSize(1300, 700)  # Prevent making it too small
        ## Timer to update table
        #self.timer = QTimer(self)
        #self.timer.timeout.connect(self.update_list)
        #self.timer.start(5000)  # update every second
        QTimer.singleShot(0, self.update_list)
        QTimer.singleShot(0, self.update_parent_list)
        #QTimer.singleShot(0, CalculationPanel.update_tf_frames_live_data(self))  # Initial update after setup
    
    def on_parent_frame_activated(self, index: int):
        selected_text = self.parent_frame_combo.itemText(index)
        self.parent_frame_changed_event(selected_text)
        self.calc_panel.update_tf_frames_live_data()
        self.calc_panel.auto_calculations()

    def angular_unit_changed_event(self):
        self.config.set_angular_unit(self.angular_unit_combo.currentText())
        self.update_list()
        self.calc_panel.update_tf_frames_live_data()
        self.calc_panel.auto_calculations()
        self.angular_unit_combo.setCurrentText(self.config.get_angular_unit())

    def lateral_unit_changed_event(self):
        self.config.set_lateral_unit(self.lateral_unit_combo.currentText())
        self.update_list()
        self.calc_panel.update_tf_frames_live_data()
        self.calc_panel.auto_calculations()
        self.lateral_unit_combo.setCurrentText(self.config.get_lateral_unit())
        

    def update_parent_list(self):
        self.parent_frame_combo.clear()
        self.parent_frame_combo.addItems(self.config.get_existing_frames())
        self.parent_frame_combo.setCurrentText(self.config.get_parent_frame())
        self.calc_panel.update_tf_frames_live_data()
        self.calc_panel.auto_calculations()

    def update_list(self):
        self.tf_viewer.update_tf_frames()
        self.tf_viewer_list.update_tf_list(self.tf_viewer.get_frames())
        self.config.update_existing_frames(self.tf_viewer.get_existing_frame_names())
        self.calc_panel.update_tf_frames_live_data()
        self.calc_panel.auto_calculations()

    def parent_frame_changed_event(self, parent_frame_name:str):
        current_parent_frame = parent_frame_name
        self.config.set_parent_frame(current_parent_frame)
        self.update_list()
        self.parent_frame_combo.setCurrentText(self.config.get_parent_frame())
        self.calc_panel.update_tf_frames_live_data()
        self.calc_panel.auto_calculations()
        
    def refresh_button_event(self):
        self.update_list()
        self.update_parent_list()
        self.calc_panel.update_tf_frames_live_data()
        self.calc_panel.auto_calculations()
        