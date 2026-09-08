# Contributing

Keep changes focused on the joint position action contract, server, clients, or serial backend.
Add permanent tests for behavior changes and run the package build and test paths before submitting a change.
Contributions are provided under the Apache License 2.0 used by this package.

## Development checks

Source the ROS 2 environment before running the hooks because the local `ament` hooks use tools from
that environment.

Install the hooks once in each clone:

```bash
pre-commit install
```

Run every hook against the repository before requesting a review:

```bash
pre-commit run --all-files
```

Ruff applies safe fixes, formats Python, and sorts imports.
ClangFormat formats C and C++.
The `ament` hooks then validate Python, C++, CMake, and XML using the ROS 2 tooling.

Python code uses a 100-character line limit.
C and C++ code use a 120-character line limit.
Both local hooks and package tests use `ament_flake8.ini` so Flake8 accepts Ruff's slice spacing.

Run the package tests after changing behavior or a public interface:

```bash
colcon test --packages-select joint_position_controller_server
colcon test-result --verbose
```
