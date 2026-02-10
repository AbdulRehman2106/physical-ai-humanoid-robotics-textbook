# Code Example Testing Framework

This directory contains tests for all code examples in the textbook to ensure they are runnable and produce expected outputs.

## Structure

```
tests/code-examples/
├── test_ros2_examples.py
├── test_gazebo_examples.py
├── test_isaac_sim_examples.py
├── test_vla_examples.py
├── test_sim_to_real_examples.py
└── conftest.py
```

## Running Tests

```bash
# Install pytest
pip install pytest pytest-timeout

# Run all tests
pytest tests/code-examples/

# Run specific test file
pytest tests/code-examples/test_ros2_examples.py

# Run with verbose output
pytest -v tests/code-examples/
```

## Test Requirements

- All code examples must be complete and runnable
- Tests verify syntax correctness and basic functionality
- Mock external dependencies (ROS 2, Gazebo, Isaac Sim) when needed
- Tests should complete within reasonable time limits
