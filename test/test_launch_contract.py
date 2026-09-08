"""Evaluate the installed upstream launch without starting hardware processes."""
from pathlib import Path
from unittest.mock import patch

import pytest
import yaml
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('depth', [None, 'true', 'false'])
def test_resolved_camera_identity_and_parameters(depth):
    resolved = []

    def capture(node, context):
        params = {}
        for item in evaluate_parameters(context, node._Node__parameters):
            assert isinstance(item, dict)
            params.update(item)
        resolved.append((
            perform_substitutions(context, normalize_to_list_of_substitutions(node._Node__node_name)),
            perform_substitutions(context, normalize_to_list_of_substitutions(node._Node__node_namespace)),
            params,
        ))
        return None

    arguments = {} if depth is None else {'enable_depth': depth}
    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(ROOT / 'launch/dual_realsense.launch.py')),
            launch_arguments=arguments.items(),
        ),
    ]))
    with patch.object(Node, 'execute', capture):
        assert service.run() == 0
    assert len(resolved) == 2
    assert {name for name, _, _ in resolved} == {'wrist_camera', 'external_camera'}
    serials = {'wrist_camera': '_342222073510', 'external_camera': '_244222076317'}
    for name, namespace, params in resolved:
        assert namespace == 'camera'
        assert params['camera_name'] == name
        assert params['serial_no'] == serials[name]
        assert params['enable_depth'] is (depth == 'true')
        assert params['enable_color'] is True
        assert params['rgb_camera.color_profile'] == '1280x720x30'
        assert params['rgb_camera.color_format'] == 'RGB8'
        for key in ('enable_gyro', 'enable_accel', 'enable_infra', 'enable_infra1',
                    'enable_infra2', 'pointcloud.enable', 'enable_sync', 'align_depth.enable'):
            assert params[key] is False


def test_research_image_displays():
    config = yaml.safe_load((ROOT / 'rviz/fr3_lab_stack.rviz').read_text())
    displays = config['Visualization Manager']['Displays']
    images = [d for d in displays if d['Class'] == 'rviz_default_plugins/Image']
    assert {d['Name']: d['Topic']['Value'] for d in images} == {
        'Wrist RGB': '/camera/wrist_camera/color/image_raw',
        'External RGB': '/camera/external_camera/color/image_raw',
    }
    assert all(d['Enabled'] for d in images)
    assert any(d['Class'] == 'moveit_rviz_plugin/MotionPlanning' and d['Enabled']
               for d in displays)
