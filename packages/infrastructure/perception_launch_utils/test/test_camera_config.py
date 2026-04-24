# Copyright 2026 perspective_grasp
# SPDX-License-Identifier: Apache-2.0
"""Unit tests for perception_launch_utils.camera_config."""

import os
import textwrap
import tempfile

import pytest

from perception_launch_utils import (
    compose_topic,
    load_cameras,
    load_config,
)


def _write_yaml(text: str) -> str:
    fd, path = tempfile.mkstemp(suffix='.yaml')
    os.write(fd, textwrap.dedent(text).encode())
    os.close(fd)
    return path


def test_missing_path_returns_single_camera_fallback():
    cfg = load_config(None)
    assert len(cfg.cameras) == 1
    assert cfg.cameras[0].namespace == ''
    assert cfg.base_frame == 'ur5e_base_link'


def test_nonexistent_path_returns_single_camera_fallback():
    cfg = load_config('/nonexistent/path/camera_config.yaml')
    assert len(cfg.cameras) == 1
    assert cfg.cameras[0].namespace == ''


def test_empty_string_returns_single_camera_fallback():
    cfg = load_config('')
    assert len(cfg.cameras) == 1


def test_two_camera_yaml_parses_to_two_cameras():
    path = _write_yaml('''
        perception_system:
          ros__parameters:
            base_frame: ur5e_base_link
            cameras:
              - id: 0
                namespace: /cam0
                type: eye_in_hand
                frame_id: cam0_color_optical_frame
              - id: 1
                namespace: /cam1
                type: eye_to_hand
                frame_id: cam1_color_optical_frame
    ''')
    try:
        cfg = load_config(path)
        assert len(cfg.cameras) == 2
        assert cfg.cameras[0].id == 0
        assert cfg.cameras[0].namespace == '/cam0'
        assert cfg.cameras[0].ns_clean == 'cam0'
        assert cfg.cameras[0].type == 'eye_in_hand'
        assert cfg.cameras[1].ns_clean == 'cam1'
        assert cfg.cameras[1].type == 'eye_to_hand'
        assert cfg.namespaces == ['/cam0', '/cam1']
    finally:
        os.unlink(path)


def test_load_cameras_convenience_wrapper():
    path = _write_yaml('''
        perception_system:
          ros__parameters:
            cameras:
              - id: 0
                namespace: /cam0
                type: eye_in_hand
                frame_id: cam0_color_optical_frame
    ''')
    try:
        cams = load_cameras(path)
        assert len(cams) == 1
        assert cams[0].namespace == '/cam0'
    finally:
        os.unlink(path)


def test_empty_cameras_list_falls_back_to_default():
    path = _write_yaml('''
        perception_system:
          ros__parameters:
            cameras: []
    ''')
    try:
        cfg = load_config(path)
        assert len(cfg.cameras) == 1
        assert cfg.cameras[0].namespace == ''
    finally:
        os.unlink(path)


def test_missing_fields_use_defaults():
    path = _write_yaml('''
        perception_system:
          ros__parameters:
            cameras:
              - namespace: /cam0
    ''')
    try:
        cfg = load_config(path)
        cam = cfg.cameras[0]
        assert cam.id == 0  # defaulted from index
        assert cam.type == 'eye_in_hand'
        assert cam.frame_id == 'camera_color_optical_frame'
        assert cam.fps == 30
    finally:
        os.unlink(path)


@pytest.mark.parametrize('ns, suffix, expected', [
    ('', 'yolo/detections', '/yolo/detections'),
    ('/cam0', 'yolo/detections', '/cam0/yolo/detections'),
    ('cam0', 'yolo/detections', '/cam0/yolo/detections'),
    ('/cam0/', '/yolo/detections', '/cam0/yolo/detections'),
    ('', '/yolo/detections', '/yolo/detections'),
])
def test_compose_topic(ns, suffix, expected):
    assert compose_topic(ns, suffix) == expected
