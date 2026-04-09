# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

## [0.1.0] - 2026-04-10

### Added
- Windows support via WSL2 (Ubuntu 24.04).
- `scripts/setup-wsl.sh` — idempotent bootstrap script that provisions the WSL environment: system dependencies, pixi, repository clone with submodules, `twai_proto`, and the `kilted` pixi env.
- `docs/wsl2-setup.md` — step-by-step guide for installing and running the project on Windows + WSL2 (including `.wslconfig` with `networkingMode=mirrored`, USB passthrough via `usbipd-win` for real hardware, and Mujoco GUI through WSLg).
- `README.md` now links to `docs/wsl2-setup.md` for the Windows + WSL2 setup path.

### Notes
- A `~/.wslconfig` with `networkingMode=mirrored` is required on the Windows host — it fixes "Network is unreachable" inside WSL when VPN/Docker Desktop is active and exposes host interfaces (including VPN) to the guest.
- On Linux the serial port is named differently than on macOS: replace `/dev/tty.usbmodem101` with `/dev/ttyACM0` (or `ttyUSB0`) in `spider_ros_control/description/urdf/spider.urdf.xacro` when running on real hardware.
