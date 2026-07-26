"""Pure, ROS-free detection math.

Turns raw YOLO boxes into a single normalised target report the follow
controller can act on, with no rclpy or torch imports so it unit-tests without
a ROS graph or a model. The node in target_detector.py does the I/O.
"""


def summarize(boxes, target_label, img_w, img_h):
    """Pick the best box matching `target_label` and normalise it.

    Args:
        boxes: iterable of (label, conf, x1, y1, x2, y2) in pixels.
        target_label: class name to follow, e.g. 'person'.
        img_w, img_h: image size in pixels.

    Returns:
        {
            "found": bool,
            "label": str,            # the target label (echoed)
            "conf": float,           # best box confidence, 0.0 if none
            "cx": float,             # horizontal offset, -1 (left)..+1 (right)
            "cy": float,             # vertical offset, -1 (top)..+1 (bottom)
            "area": float,           # box area / image area, 0..1 (distance cue)
        }
        `cx`/`cy`/`area` are 0.0 when nothing is found.
    """
    result = {"found": False, "label": target_label, "conf": 0.0,
              "cx": 0.0, "cy": 0.0, "area": 0.0}
    if img_w <= 0 or img_h <= 0:
        return result

    # Highest-confidence box of the requested class. Following the most
    # confident target avoids flip-flopping between two people in frame.
    best = None
    for label, conf, x1, y1, x2, y2 in boxes:
        if label != target_label:
            continue
        if best is None or conf > best[1]:
            best = (label, conf, x1, y1, x2, y2)
    if best is None:
        return result

    _, conf, x1, y1, x2, y2 = best
    box_cx = (x1 + x2) / 2.0
    box_cy = (y1 + y2) / 2.0
    result["found"] = True
    result["conf"] = float(conf)
    # Map pixel centre to [-1, 1] about the image centre.
    result["cx"] = (box_cx - img_w / 2.0) / (img_w / 2.0)
    result["cy"] = (box_cy - img_h / 2.0) / (img_h / 2.0)
    result["area"] = max(0.0, (x2 - x1) * (y2 - y1)) / float(img_w * img_h)
    return result
