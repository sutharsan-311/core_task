"""Unit tests for the pure detection math."""
from core_task_perception.detection import summarize


def test_no_boxes_reports_not_found():
    r = summarize([], 'person', 640, 480)
    assert r['found'] is False
    assert r['cx'] == 0.0 and r['area'] == 0.0


def test_wrong_class_is_ignored():
    boxes = [('chair', 0.9, 0, 0, 100, 100)]
    assert summarize(boxes, 'person', 640, 480)['found'] is False


def test_centred_target_has_zero_offset():
    # Box centred in a 640x480 image.
    boxes = [('person', 0.8, 300, 220, 340, 260)]
    r = summarize(boxes, 'person', 640, 480)
    assert r['found'] is True
    assert abs(r['cx']) < 1e-9
    assert abs(r['cy']) < 1e-9


def test_target_on_the_right_is_positive_cx():
    boxes = [('person', 0.8, 500, 220, 600, 260)]
    assert summarize(boxes, 'person', 640, 480)['cx'] > 0


def test_target_on_the_left_is_negative_cx():
    boxes = [('person', 0.8, 40, 220, 140, 260)]
    assert summarize(boxes, 'person', 640, 480)['cx'] < 0


def test_area_grows_as_target_nears():
    small = summarize([('person', 0.8, 300, 220, 340, 260)], 'person', 640, 480)
    big = summarize([('person', 0.8, 200, 100, 440, 380)], 'person', 640, 480)
    assert big['area'] > small['area']
    assert 0.0 < big['area'] <= 1.0


def test_most_confident_target_wins():
    boxes = [('person', 0.5, 40, 220, 140, 260),     # left, low conf
             ('person', 0.9, 500, 220, 600, 260)]    # right, high conf
    r = summarize(boxes, 'person', 640, 480)
    assert r['conf'] == 0.9
    assert r['cx'] > 0        # picked the right-hand, higher-confidence one


def test_zero_image_size_is_safe():
    assert summarize([('person', 0.9, 0, 0, 10, 10)], 'person', 0, 0)['found'] \
        is False


def test_configurable_target_label():
    boxes = [('backpack', 0.7, 300, 220, 340, 260)]
    assert summarize(boxes, 'backpack', 640, 480)['found'] is True
