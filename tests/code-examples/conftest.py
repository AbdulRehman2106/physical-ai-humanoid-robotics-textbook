import pytest
import sys
from pathlib import Path

# Add static/code-examples to Python path for imports
code_examples_dir = Path(__file__).parent.parent.parent / 'static' / 'code-examples'
sys.path.insert(0, str(code_examples_dir))

@pytest.fixture
def mock_ros2_environment(monkeypatch):
    """Mock ROS 2 environment for testing without actual ROS 2 installation."""
    # Mock rclpy imports
    monkeypatch.setattr('sys.modules', {
        **sys.modules,
        'rclpy': type('MockModule', (), {}),
        'rclpy.node': type('MockModule', (), {}),
    })
    return True

@pytest.fixture
def mock_gazebo_environment(monkeypatch):
    """Mock Gazebo environment for testing."""
    return True

@pytest.fixture
def mock_isaac_sim_environment(monkeypatch):
    """Mock Isaac Sim environment for testing."""
    return True
