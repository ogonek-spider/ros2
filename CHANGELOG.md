# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

### Added
- VSCode build task (`.vscode/tasks.json`) — *Spider: launch Mujoco simulation (WSL)*, marked as the default build task and bound to `Ctrl+Shift+B`. It calls `wsl.exe` to enter `/home/spider/projects/ogonek-spider/ros2`, activates the `pixi -e kilted` environment, sources `install/local_setup.bash`, and runs `ros2 launch spider_ros_control spider-mujoco.launch.py`.
- Pre-launch task *Spider: sync sources to WSL*: before each simulation run it rsyncs the working copy from `/mnt/c/Users/user/vs_code_projects/ogonek-spider/ros2/` to the WSL clone at `/home/spider/projects/ogonek-spider/ros2/` (excluding `.git`, `build/`, `install/`, `log/`, the pixi cache, `__pycache__`, `.vscode`, `*.zip`). After the sync the launch task runs an incremental `colcon build --symlink-install`, so URDF/MJCF/Python edits take effect without manual rebuilds or commits.
- The Mujoco scene now uses a heightfield terrain (`Rolling Hills`) instead of a flat floor. The heightmap is unpacked into `spider_description/mujoco/assets/rolling_hills/` and wired up via `<hfield>` in `scene.xml` (30×30 m area, max height 0.6 m).

## [0.0.38] - 2026-04-10

### Added
- Windows support via WSL2 (Ubuntu 24.04).
- `scripts/setup-wsl.sh` — idempotent bootstrap script that provisions the WSL environment: system dependencies, pixi, repository clone with submodules, `twai_proto`, and the `kilted` pixi env.
- `docs/wsl2-setup.md` — step-by-step guide for installing and running the project on Windows + WSL2 (including `.wslconfig` with `networkingMode=mirrored`, USB passthrough via `usbipd-win` for real hardware, and Mujoco GUI through WSLg).
- `README.md` now links to `docs/wsl2-setup.md` for the Windows + WSL2 setup path.

### Notes
- A `~/.wslconfig` with `networkingMode=mirrored` is required on the Windows host — it fixes "Network is unreachable" inside WSL when VPN/Docker Desktop is active and exposes host interfaces (including VPN) to the guest.
- On Linux the serial port is named differently than on macOS: replace `/dev/tty.usbmodem101` with `/dev/ttyACM0` (or `ttyUSB0`) in `spider_ros_control/description/urdf/spider.urdf.xacro` when running on real hardware.
