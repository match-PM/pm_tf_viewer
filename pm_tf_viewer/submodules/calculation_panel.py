"""Compare frame poses expressed in a shared parent frame."""

from copy import deepcopy
import json
from math import isfinite, sqrt
from pathlib import Path

from PyQt6.QtCore import QSaveFile, QIODevice, Qt, pyqtSignal
from geometry_msgs.msg import TransformStamped
from PyQt6.QtWidgets import (
    QApplication, QComboBox, QFileDialog, QGroupBox, QHBoxLayout, QLabel, QMessageBox,
    QPushButton, QScrollArea, QToolButton, QVBoxLayout, QWidget,
)
from scipy.spatial.transform import Rotation


def pose_difference(frame1, frame2):
    """Return displacement and active rotation from pose 1 to 2 in parent axes."""
    p1, p2 = frame1.transform.translation, frame2.transform.translation
    delta = (p2.x - p1.x, p2.y - p1.y, p2.z - p1.z)
    q1, q2 = frame1.transform.rotation, frame2.transform.rotation
    rotation1 = Rotation.from_quat([q1.x, q1.y, q1.z, q1.w])
    rotation2 = Rotation.from_quat([q2.x, q2.y, q2.z, q2.w])
    return delta, rotation2 * rotation1.inv()


class CalculationPanel(QWidget):
    """Select available frames and manage independent calculation slides."""

    settings_loaded = pyqtSignal()

    def __init__(self, config, ros_node):
        super().__init__()
        self.config = config
        self.ros_node = ros_node
        self.frames = {}
        self.parent_frame = config.get_parent_frame()
        self.calc_slides = []
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)
        controls = QHBoxLayout()
        add_button = QToolButton()
        add_button.setText('+ Add calculation')
        add_button.setToolTip('Add another pair of frames to compare')
        add_button.clicked.connect(self.add_slide)
        controls.addWidget(add_button)
        hint = QLabel('Select two frames to compare. Use Refresh Transforms to update.')
        hint.setWordWrap(True)
        controls.addWidget(hint, 1)
        for text, callback in (('Save…', self.save_dialog), ('Load…', self.load_dialog)):
            button = QToolButton()
            button.setText(text)
            button.clicked.connect(callback)
            controls.addWidget(button)
        clear_button = QToolButton()
        clear_button.setText('Clear all')
        clear_button.setToolTip('Remove all calculation slides')
        clear_button.clicked.connect(self.clear_all)
        controls.addWidget(clear_button)
        layout.addLayout(controls)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        container = QWidget()
        self.slides_layout = QVBoxLayout(container)
        self.slides_layout.addStretch()
        scroll.setWidget(container)
        layout.addWidget(scroll, 1)

    def save_dialog(self):
        path, _ = QFileDialog.getSaveFileName(
            self, 'Save calculation slides', '', 'Calculation slides (*.json)')
        if not path:
            return
        if not Path(path).suffix:
            path += '.json'
        try:
            self.save_slides(path)
        except (OSError, ValueError) as error:
            QMessageBox.warning(self, 'Cannot save slides', str(error))

    def load_dialog(self):
        path, _ = QFileDialog.getOpenFileName(
            self, 'Load calculation slides', '', 'Calculation slides (*.json)')
        if not path:
            return
        try:
            self.load_slides(path)
        except (OSError, ValueError, TypeError, KeyError) as error:
            QMessageBox.warning(self, 'Cannot load slides', str(error))

    def save_slides(self, path):
        """Save selections, SI snapshots, and shared display settings atomically."""
        slides = []
        for slide in self.calc_slides:
            entries = []
            for dropdown, snapshot in zip(slide.dropdowns, slide.snapshots):
                pose = None
                if snapshot is not None:
                    p, q = snapshot.transform.translation, snapshot.transform.rotation
                    pose = {'translation': [p.x, p.y, p.z],
                            'rotation': [q.x, q.y, q.z, q.w]}
                entries.append({'frame': dropdown.currentText(), 'snapshot': pose})
            slides.append(entries)
        data = {'version': 1, 'parent_frame': self.parent_frame,
                'translation_unit': self.config.get_lateral_unit(),
                'rotation_unit': self.config.get_angular_unit(), 'slides': slides}
        payload = json.dumps(data, indent=2, allow_nan=False).encode('utf-8')
        target = QSaveFile(str(path))
        if not target.open(QIODevice.OpenModeFlag.WriteOnly):
            raise OSError(target.errorString())
        if target.write(payload) != len(payload):
            target.cancelWriting()
            raise OSError(target.errorString())
        if not target.commit():
            raise OSError(target.errorString())

    def load_slides(self, path):
        """Validate an entire file before replacing the current slides."""
        data = json.loads(Path(path).read_text(encoding='utf-8'))
        if not isinstance(data, dict) or data.get('version') != 1:
            raise ValueError('Unsupported calculation slides file version.')
        parent = data.get('parent_frame')
        lateral, angular = data.get('translation_unit'), data.get('rotation_unit')
        if not isinstance(parent, str) or not parent:
            raise ValueError('A parent frame is required.')
        if (lateral not in self.config.get_lateral_unit_list()
                or angular not in self.config.get_angular_unit_list()):
            raise ValueError('Invalid display units.')
        if not isinstance(data.get('slides'), list):
            raise ValueError('The file must contain a list of slides.')
        restored = []
        for entries in data['slides']:
            if not isinstance(entries, list) or len(entries) != 2:
                raise ValueError('Each slide must contain two frame selections.')
            selections = []
            for entry in entries:
                if not isinstance(entry, dict) or not isinstance(entry.get('frame'), str):
                    raise ValueError('Invalid frame selection.')
                name, pose = entry['frame'], entry.get('snapshot')
                snapshot = None
                if pose is not None:
                    if not name or not isinstance(pose, dict):
                        raise ValueError('Invalid frozen pose.')
                    position, quaternion = pose.get('translation'), pose.get('rotation')
                    for vector, size in ((position, 3), (quaternion, 4)):
                        if (not isinstance(vector, list) or len(vector) != size
                                or any(type(v) not in (int, float) or not isfinite(v)
                                       for v in vector)):
                            raise ValueError('Invalid frozen pose coordinates.')
                    Rotation.from_quat(quaternion)  # Reject zero quaternions.
                    snapshot = TransformStamped()
                    snapshot.header.frame_id = parent
                    snapshot.child_frame_id = name
                    p, q = snapshot.transform.translation, snapshot.transform.rotation
                    p.x, p.y, p.z = map(float, position)
                    q.x, q.y, q.z, q.w = map(float, quaternion)
                selections.append((name, snapshot))
            restored.append(selections)
        if parent != self.parent_frame:
            self.frames = {}
        self.config.update_existing_frames(
            list(dict.fromkeys(self.config.get_existing_frames() + [parent])))
        self.config.set_parent_frame(parent)
        self.config.set_lateral_unit(lateral)
        self.config.set_angular_unit(angular)
        self.parent_frame = parent
        self.clear_all()
        self.settings_loaded.emit()
        for selections in restored:
            self.add_slide()
            slide = self.calc_slides[-1]
            for index, (name, snapshot) in enumerate(selections):
                dropdown, button = slide.dropdowns[index], slide.freeze_buttons[index]
                dropdown.blockSignals(True)
                if dropdown.findText(name) < 0:
                    dropdown.addItem(name)
                dropdown.setCurrentText(name)
                dropdown.blockSignals(False)
                slide.snapshots[index] = snapshot
                button.blockSignals(True)
                button.setChecked(snapshot is not None)
                button.blockSignals(False)
                button.setText('Frozen' if snapshot is not None else 'Freeze')
                dropdown.setEnabled(snapshot is None)
            slide.update_results()

    def receive_translation_data(self, name):
        """Select a double-clicked frame in the most recent calculation slide."""
        if name not in self.frames:
            return
        if not self.calc_slides:
            self.add_slide()
        slide = self.calc_slides[-1]
        for index, dropdown in enumerate(slide.dropdowns):
            if slide.snapshots[index] is None:
                dropdown.setCurrentText(name)
                break

    def update_dropdowns(self):
        names = sorted(self.frames, key=str.casefold)
        for slide in self.calc_slides:
            for dropdown in slide.dropdowns:
                previous = dropdown.currentText()
                dropdown.blockSignals(True)
                dropdown.clear()
                dropdown.addItems(names)
                if previous:
                    # Keep unavailable selections so refresh cannot silently
                    # substitute another frame, including for frozen poses.
                    if previous not in names:
                        dropdown.addItem(previous)
                    dropdown.setCurrentText(previous)
                dropdown.blockSignals(False)
            slide.update_results()

    def add_slide(self):
        slide = CalculationSlide(self)
        self.calc_slides.append(slide)
        self.slides_layout.insertWidget(self.slides_layout.count() - 1, slide)
        self.update_titles()
        self.update_dropdowns()

    def remove_slide(self, slide):
        self.calc_slides.remove(slide)
        self.slides_layout.removeWidget(slide)
        slide.hide()
        slide.deleteLater()
        self.update_titles()

    def update_titles(self):
        for index, slide in enumerate(self.calc_slides, 1):
            slide.setTitle(f'Calculation Slide {index}')

    def clear_all(self):
        for slide in list(self.calc_slides):
            self.remove_slide(slide)

    def update_frames(self, frames):
        """Use the viewer's SI poses; reset snapshots when parent axes change."""
        parent = self.config.get_parent_frame()
        if parent != self.parent_frame:
            for slide in self.calc_slides:
                slide.reset_freezes()
            self.parent_frame = parent
        self.frames = {frame.child_frame_id: frame for frame in frames}
        self.update_dropdowns()


class CalculationSlide(QGroupBox):
    """Compare two live or independently frozen poses."""

    def __init__(self, panel):
        super().__init__()
        self.panel = panel
        self.snapshots = [None, None]
        self.dropdowns = []
        self.freeze_buttons = []
        layout = QVBoxLayout(self)
        toolbar = QHBoxLayout()
        self.copy_button = QToolButton()
        self.copy_button.setText('Copy results')
        self.copy_button.setEnabled(False)
        self.copy_button.clicked.connect(self.copy_results)
        toolbar.addStretch()
        toolbar.addWidget(self.copy_button)
        delete = QToolButton()
        delete.setText('×')
        delete.setToolTip('Delete this calculation slide')
        delete.clicked.connect(lambda: panel.remove_slide(self))
        toolbar.addWidget(delete)
        layout.addLayout(toolbar)
        for index in range(2):
            row = QHBoxLayout()
            row.addWidget(QLabel(f'Frame {index + 1}:'))
            dropdown = QComboBox()
            dropdown.setMinimumWidth(180)
            dropdown.currentTextChanged.connect(self.update_results)
            self.dropdowns.append(dropdown)
            row.addWidget(dropdown, 1)
            freeze = QPushButton('Freeze')
            freeze.setCheckable(True)
            freeze.setToolTip('Freeze this pose until switched back to live data')
            freeze.toggled.connect(
                lambda checked, i=index: self.toggle_freeze(i, checked))
            self.freeze_buttons.append(freeze)
            row.addWidget(freeze)
            layout.addLayout(row)
        self.result_label = QLabel()
        self.result_label.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(self.result_label)
        self.update_results()

    def toggle_freeze(self, index, checked):
        frame = self.panel.frames.get(self.dropdowns[index].currentText())
        if checked and frame is None:
            button = self.freeze_buttons[index]
            button.blockSignals(True)
            button.setChecked(False)
            button.blockSignals(False)
            self.update_results()
            return
        self.snapshots[index] = deepcopy(frame) if checked else None
        self.dropdowns[index].setEnabled(not checked)
        self.freeze_buttons[index].setText('Frozen' if checked else 'Freeze')
        self.update_results()

    def reset_freezes(self):
        for index, button in enumerate(self.freeze_buttons):
            button.blockSignals(True)
            button.setChecked(False)
            button.blockSignals(False)
            self.snapshots[index] = None
            self.dropdowns[index].setEnabled(True)
            button.setText('Freeze')

    def update_results(self):
        config = self.panel.config
        frames = [self.snapshots[i] if self.snapshots[i] is not None
                  else self.panel.frames.get(dropdown.currentText())
                  for i, dropdown in enumerate(self.dropdowns)]
        self.copy_button.setEnabled(False)
        if any(frame is None for frame in frames):
            self.result_label.setText('Select two available frames to calculate.')
            return
        try:
            delta, rotation = pose_difference(*frames)
        except ValueError as error:
            self.result_label.setText(f'Cannot calculate rotation: {error}')
            return
        scale = config.get_lateral_unit_conversion()
        unit = config.get_lateral_unit()
        angular_unit = config.get_angular_unit()
        lines = [f'Parent Frame: {self.panel.parent_frame}',
                 'Differences: Frame 2 − Frame 1, expressed in parent axes.']
        for index, frame in enumerate(frames):
            p = frame.transform.translation
            state = 'frozen' if self.snapshots[index] is not None else 'live'
            lines.append(
                f'Frame {index + 1}: {frame.child_frame_id} ({state}) at '
                f'({p.x * scale:.9f}, {p.y * scale:.9f}, '
                f'{p.z * scale:.9f}) {unit}')
        for axis, value in zip('xyz', delta):
            lines.append(f'Δ{axis} = {value * scale:.9f} {unit}')
        length = sqrt(sum(value * value for value in delta)) * scale
        lines.append(f'Vector length: {length:.9f} {unit}')
        if angular_unit == 'Quaternion':
            labels, values = ('qx', 'qy', 'qz', 'qw'), rotation.as_quat()
            suffix = ''
        else:
            labels = ('Roll', 'Pitch', 'Yaw')
            values = rotation.as_euler('xyz', degrees=angular_unit == 'deg')
            suffix = f' {angular_unit}'
        lines.extend(f'Δ{label} = {value:.9f}{suffix}'
                     for label, value in zip(labels, values))
        self.result_label.setText('\n'.join(lines))
        self.copy_button.setEnabled(True)

    def copy_results(self):
        if self.copy_button.isEnabled():
            QApplication.clipboard().setText(
                f'{self.title()}\n{self.result_label.text()}')
