"""Check that the mapping launch differs only by omitting the unused path smoother."""
import ast
import importlib.util
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import yaml
from launch import LaunchContext

PROJECT = Path(__file__).resolve().parents[1]


class WithoutPathSmoother(ast.NodeTransformer):
    def visit_List(self, node):
        # Remove the two process/component declarations and lifecycle membership.
        node.elts = [item for item in node.elts if not (
            isinstance(item, ast.Constant) and item.value == 'smoother_server' or
            isinstance(item, ast.Call) and any(
                keyword.arg == 'package' and isinstance(keyword.value, ast.Constant)
                and keyword.value.value == 'nav2_smoother' for keyword in item.keywords))]
        return self.generic_visit(node)


class MappingNavigationLaunchTest(unittest.TestCase):
    def test_only_path_smoother_is_removed_from_upstream_launch(self):
        upstream = Path(get_package_share_directory('nav2_bringup')) / 'launch/navigation_launch.py'
        expected = WithoutPathSmoother().visit(ast.parse(upstream.read_text()))
        actual = ast.parse((PROJECT / 'launch/mapping_navigation.launch.py').read_text())
        self.assertEqual(ast.dump(actual), ast.dump(expected))

    def test_mapping_layer_override_is_a_list_and_preserves_shared_config(self):
        spec = importlib.util.spec_from_file_location(
            'mapping_launch', PROJECT / 'launch/tiago_apartment_mapping.launch.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        source = PROJECT / 'nav2_params_jazzy.yaml'
        original = source.read_text()
        rewritten = module.MappingRewrittenYaml(
            source_file=str(source), convert_types=True,
            param_rewrites={
                'local_costmap.local_costmap.ros__parameters.plugins':
                    '["static_layer", "voxel_layer", "inflation_layer"]'})
        generated = Path(rewritten.perform(LaunchContext()))
        try:
            params = yaml.safe_load(generated.read_text())
            self.assertEqual(params['local_costmap']['local_costmap']['ros__parameters']['plugins'],
                             ['static_layer', 'voxel_layer', 'inflation_layer'])
            self.assertEqual(params['global_costmap'], yaml.safe_load(original)['global_costmap'])
            self.assertEqual(source.read_text(), original)
        finally:
            generated.unlink()

    def test_mapping_default_behavior_tree_does_not_require_smoother(self):
        params = yaml.safe_load((PROJECT / 'nav2_params_jazzy.yaml').read_text())
        navigator = params['bt_navigator']['ros__parameters']
        self.assertFalse(navigator.get('default_nav_to_pose_bt_xml'))
        tree = (Path(get_package_share_directory('nav2_bt_navigator')) /
                'behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml')
        tags = {element.tag for element in ET.parse(tree).iter()}
        self.assertIn('ComputePathToPose', tags)
        self.assertIn('FollowPath', tags)
        self.assertNotIn('SmoothPath', tags)
        # If upstream adds an external/subtree reference it needs a fresh audit.
        self.assertNotIn('include', tags)
        self.assertNotIn('SubTree', tags)


if __name__ == '__main__':
    unittest.main()
