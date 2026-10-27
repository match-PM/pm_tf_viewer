"""Regression tests for the manually ported calculation panel."""

import os

import pytest
from geometry_msgs.msg import TransformStamped
from PyQt6.QtWidgets import QApplication
from scipy.spatial.transform import Rotation

from pm_tf_viewer.submodules.TfViewerApp import (
    TfTransformViewerListWidget, TfViewerAppConfig,
)
from pm_tf_viewer.submodules.calculation_panel import (
    CalculationPanel, pose_difference,
)


@pytest.fixture(scope='module')
def app():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    instance = QApplication.instance() or QApplication([])
    yield instance


def frame(name, xyz=(0., 0., 0.), rotation=None):
    result = TransformStamped()
    result.header.frame_id = 'world'
    result.child_frame_id = name
    p = result.transform.translation
    p.x, p.y, p.z = map(float, xyz)
    q = result.transform.rotation
    q.x, q.y, q.z, q.w = (
        rotation.as_quat() if rotation is not None else (0., 0., 0., 1.))
    return result


@pytest.fixture
def panel(app):
    widget = CalculationPanel(TfViewerAppConfig(), None)
    widget.update_frames([frame('a'), frame('b', (1, 2, 2))])
    yield widget
    widget.close()
    widget.deleteLater()


def add_slide(panel):
    panel.add_slide()
    slide = panel.calc_slides[-1]
    slide.dropdowns[1].setCurrentText('b')
    return slide


def test_parent_axes_rotation_and_displacement():
    r1 = Rotation.from_euler('x', 45, degrees=True)
    r2 = Rotation.from_euler('y', 60, degrees=True)
    delta, rotation = pose_difference(
        frame('a', (1, 2, 3), r1), frame('b', (4, 6, 3), r2))
    assert delta == (3, 4, 0)
    assert (rotation * r1).as_matrix() == pytest.approx(r2.as_matrix())


def test_freeze_is_independent_of_deleting_and_adding_slides(panel):
    first = add_slide(panel)
    second = add_slide(panel)
    second.freeze_buttons[1].click()
    panel.remove_slide(first)
    third = add_slide(panel)
    panel.update_frames([frame('a'), frame('b', (9, 0, 0))])
    assert second.title() == 'Calculation Slide 1'
    assert second.snapshots[1].transform.translation.x == 1
    assert third.snapshots == [None, None]
    assert 'Δx = 1.000000000 m' in second.result_label.text()
    assert 'Δx = 9.000000000 m' in third.result_label.text()


def test_units_convert_frozen_si_data_and_clipboard(panel, app):
    slide = add_slide(panel)
    slide.freeze_buttons[1].click()
    panel.config.set_lateral_unit('mm')
    panel.config.set_angular_unit('Quaternion')
    panel.update_frames([])
    # Freeze the other endpoint before losing live transforms.
    assert not slide.copy_button.isEnabled()
    panel.update_frames([frame('a')])
    assert 'Δx = 1000.000000000 mm' in slide.result_label.text()
    assert 'Δqw = 1.000000000' in slide.result_label.text()
    slide.copy_button.click()
    assert 'Δqw = 1.000000000' in app.clipboard().text()
    panel.config.set_angular_unit('deg')
    slide.update_results()
    assert 'Δqw' not in slide.result_label.text()
    assert 'ΔYaw = 0.000000000 deg' in slide.result_label.text()


def test_parent_changes_release_snapshots_and_clear_all(panel):
    slide = add_slide(panel)
    slide.freeze_buttons[0].click()
    panel.config.update_existing_frames(['other'])
    panel.config.set_parent_frame('other')
    panel.update_frames([])
    assert slide.snapshots == [None, None]
    assert slide.dropdowns[0].isEnabled()
    assert not slide.freeze_buttons[0].isChecked()
    assert not slide.copy_button.isEnabled()
    panel.clear_all()
    assert not panel.calc_slides
    assert panel.slides_layout.count() == 1


def test_unavailable_freeze_and_invalid_rotation(panel):
    slide = add_slide(panel)
    panel.update_frames([])
    slide.freeze_buttons[0].click()
    assert not slide.freeze_buttons[0].isChecked()
    bad = frame('b')
    bad.transform.rotation.w = 0.
    panel.update_frames([frame('a'), bad])
    assert 'Cannot calculate rotation' in slide.result_label.text()
    assert not slide.copy_button.isEnabled()


def test_sorting_and_double_click_selection(app):
    widget = TfTransformViewerListWidget([], TfViewerAppConfig())
    names = ['_last', '2frame', 'beta', 'Alpha']
    widget.update_tf_list([frame(name) for name in names])
    assert [widget.item(i).data(256) for i in range(widget.count())] == [
        'Alpha', 'beta', '2frame', '_last']
    selected = []
    widget.transform_ID_selected.connect(selected.append)
    widget.itemDoubleClicked.emit(widget.item(0))
    assert selected == ['Alpha']
    widget.close()


def test_frames_available_without_double_click_and_refresh_keeps_selection(panel):
    slide = add_slide(panel)
    assert [slide.dropdowns[0].itemText(i)
            for i in range(slide.dropdowns[0].count())] == ['a', 'b']
    assert slide.copy_button.isEnabled()
    panel.update_frames([frame('c'), frame('b'), frame('a')])
    assert slide.dropdowns[1].currentText() == 'b'
    assert slide.dropdowns[0].findText('c') >= 0
    panel.update_frames([frame('c')])
    assert slide.dropdowns[1].currentText() == 'b'
    assert not slide.copy_button.isEnabled()
    panel.update_frames([frame('a'), frame('b')])
    assert slide.copy_button.isEnabled()


def test_save_load_roundtrip_with_offline_snapshots(panel, tmp_path):
    first = add_slide(panel)
    first.freeze_buttons[0].click()
    first.freeze_buttons[1].click()
    second = add_slide(panel)
    panel.config.set_lateral_unit('mm')
    panel.config.set_angular_unit('Quaternion')
    path = tmp_path / 'slides.json'
    panel.save_slides(path)
    second.dropdowns[1].setCurrentText('a')
    panel.clear_all()
    panel.update_frames([])
    panel.config.update_existing_frames(['other'])
    panel.config.set_parent_frame('other')
    panel.update_frames([])
    panel.load_slides(path)
    assert panel.config.get_parent_frame() == 'world'
    assert panel.config.get_lateral_unit() == 'mm'
    assert panel.config.get_angular_unit() == 'Quaternion'
    assert len(panel.calc_slides) == 2
    frozen, live = panel.calc_slides
    assert all(button.isChecked() for button in frozen.freeze_buttons)
    assert all(not dropdown.isEnabled() for dropdown in frozen.dropdowns)
    assert 'Δx = 1000.000000000 mm' in frozen.result_label.text()
    assert frozen.copy_button.isEnabled()
    assert live.dropdowns[1].currentText() == 'b'
    assert not live.copy_button.isEnabled()
    panel.update_frames([frame('a'), frame('b', (8, 0, 0))])
    assert 'Δx = 1000.000000000 mm' in frozen.result_label.text()
    assert 'Δx = 8000.000000000 mm' in live.result_label.text()


@pytest.mark.parametrize('contents', [
    '{broken json',
    '{"version": 2}',
    '{"version": 1, "parent_frame": "world", "translation_unit": "m", '
    '"rotation_unit": "rad", "slides": [[{"frame": "a", "snapshot": '
    '{"translation": [0, 0, 0], "rotation": [0, 0, 0, 0]}}, {"frame": "b"}]]}',
])
def test_invalid_load_keeps_existing_slides(panel, tmp_path, contents):
    slide = add_slide(panel)
    before = slide.result_label.text()
    path = tmp_path / 'invalid.json'
    path.write_text(contents)
    with pytest.raises(ValueError):
        panel.load_slides(path)
    assert panel.calc_slides == [slide]
    assert slide.result_label.text() == before


def test_empty_slides_file(panel, tmp_path):
    path = tmp_path / 'empty.json'
    panel.save_slides(path)
    add_slide(panel)
    panel.load_slides(path)
    assert not panel.calc_slides
