"""
Tests for ROS 2 code examples.
These tests verify syntax and basic structure without requiring ROS 2 installation.
"""
import pytest
from pathlib import Path

def test_ros2_examples_directory_exists():
    """Verify ROS 2 examples directory exists."""
    ros2_dir = Path(__file__).parent.parent.parent / 'static' / 'code-examples' / 'ros2'
    assert ros2_dir.exists(), "ROS 2 examples directory should exist"

def test_hello_node_syntax():
    """Test that hello_node.py has valid Python syntax."""
    hello_node_path = Path(__file__).parent.parent.parent / 'static' / 'code-examples' / 'ros2' / 'hello_node.py'

    if hello_node_path.exists():
        with open(hello_node_path, 'r') as f:
            code = f.read()

        # Verify it compiles without syntax errors
        try:
            compile(code, str(hello_node_path), 'exec')
        except SyntaxError as e:
            pytest.fail(f"Syntax error in hello_node.py: {e}")
    else:
        pytest.skip("hello_node.py not yet created")

# Additional tests will be added as code examples are created
