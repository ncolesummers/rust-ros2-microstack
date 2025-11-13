# Setup Guide

Complete setup instructions and troubleshooting for the Rust ROS2 Microstack.

## Prerequisites

### System Requirements

- **OS**: Ubuntu 24.04 LTS (recommended) or 22.04 LTS
- **RAM**: 4GB minimum, 8GB recommended
- **Disk**: 10GB free space for ROS2 + Rust toolchain

### Required Software

1. **ROS2 Jazzy**
2. **Rust** (stable, edition 2024 support)
3. **Gazebo** (for simulation)
4. **Build tools** (clang, cmake, pkg-config)

---

## Installation

### 1. Install ROS2 Jazzy

Follow the official guide: [ROS2 Jazzy Installation](https://docs.ros.org/en/jazzy/Installation.html)

**Ubuntu Quick Install:**

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy desktop
sudo apt update
sudo apt install ros-jazzy-desktop

# Install development tools
sudo apt install ros-dev-tools
```

**Verify installation:**

```bash
source /opt/ros/jazzy/setup.bash
ros2 --version
# Should print: ros2 doctor <version>
```

### 2. Install Rust

```bash
# Install rustup
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Reload shell or source
source $HOME/.cargo/env

# Verify installation
rustc --version
cargo --version

# Ensure stable toolchain with edition 2024 support
rustup update stable
rustup default stable
```

**Check edition 2024 support:**

```bash
rustc --version
# Should be 1.85.0 or newer for edition 2024
```

### 3. Install Gazebo

```bash
# Gazebo Harmonic (compatible with ROS2 Jazzy)
sudo apt install ros-jazzy-ros-gz

# Verify
gz sim --version
```

### 4. Install Build Dependencies

r2r requires system libraries for ROS2 integration:

```bash
# Core build tools
sudo apt install build-essential clang llvm-dev libclang-dev cmake pkg-config

# ROS2 message packages (required for r2r code generation)
sudo apt install ros-jazzy-std-msgs ros-jazzy-geometry-msgs ros-jazzy-sensor-msgs
```

---

## Project Setup

### Clone and Build

```bash
# Clone repository
git clone <your-repo-url>
cd rust-ros2-microstack

# Load ROS2 environment
source scripts/dev_env.sh

# Build workspace
cargo build --workspace

# Run tests
cargo test

# Run a node
cargo run -p apps/teleop_mux -- --help
```

### Environment Setup

**Option 1: Manual sourcing (each terminal)**

```bash
source /opt/ros/jazzy/setup.bash
```

**Option 2: Use project script (recommended)**

```bash
source scripts/dev_env.sh
```

**Option 3: Auto-source in shell RC (permanent)**

```bash
# Add to ~/.bashrc or ~/.zshrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Verify ROS2 environment is loaded:**

```bash
env | grep ROS
# Should see ROS_VERSION=2, ROS_DISTRO=jazzy, etc.
```

---

## Troubleshooting

### Build Errors

#### Error: `fatal error: 'rclcpp/rclcpp.hpp' file not found`

**Cause**: ROS2 environment not sourced before build.

**Fix**:

```bash
source /opt/ros/jazzy/setup.bash
cargo clean
cargo build
```

#### Error: `could not find native static library rcl`

**Cause**: Missing ROS2 development packages.

**Fix**:

```bash
sudo apt install ros-jazzy-ros-core ros-dev-tools
```

#### Error: `edition 2024 is unstable`

**Cause**: Rust toolchain too old.

**Fix**:

```bash
rustup update stable
rustc --version  # Verify >= 1.85.0
```

### Runtime Errors

#### Error: `No executable found`

**Cause**: Binary crate not specified correctly.

**Fix**:

```bash
# For apps (binaries)
cargo run -p apps/teleop_mux

# For nodes (libraries) - not runnable directly
# Run from apps/ or examples/ instead
```

#### Error: Topic not connecting

**Symptom**: Publisher and subscriber exist but no messages received.

**Cause**: QoS policy mismatch.

**Debug**:

```bash
# Check topic info
ros2 topic info /topic_name -v

# Should show matching QoS between publishers and subscribers
```

**Fix**: Ensure both use compatible QoS (both reliable or both best_effort).

#### Error: `Failed to create node: DomainParticipantQosPolicy is immutable`

**Cause**: Multiple r2r contexts or conflicting DDS configuration.

**Fix**:

```bash
# Clear DDS environment
unset RMW_IMPLEMENTATION

# Use default (CycloneDDS)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Gazebo Issues

#### Gazebo doesn't start

**Check installation**:

```bash
gz sim --version
ros2 pkg list | grep ros_gz
```

**If missing**:

```bash
sudo apt install ros-jazzy-ros-gz-sim
```

#### `/clock` topic not publishing

**Cause**: Gazebo not started with ROS2 bridge.

**Fix**:

```bash
# Start with bridge
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="shapes.sdf"

# Or use project smoke test
bash scripts/sim_smoke.sh
```

### Permission Errors

#### Error: `Permission denied` when running scripts

**Fix**:

```bash
chmod +x scripts/*.sh
```

### Logging Issues

#### No logs appearing

**Cause**: `RUST_LOG` not set.

**Fix**:

```bash
RUST_LOG=info cargo run -p apps/teleop_mux
```

**Set globally** (add to `~/.bashrc`):

```bash
export RUST_LOG=info
```

---

## Development Tools (Optional)

### Pre-commit Hooks

Pre-commit hooks automatically run code quality checks before each commit, reducing CI failures and maintaining consistent code quality.

**What gets checked:**

- `cargo fmt` - Formats all Rust code
- `cargo clippy` - Lints code and catches common mistakes
- Additional checks - Trailing whitespace, YAML/TOML syntax, merge conflicts

**Installation:**

```bash
# Install pre-commit (requires Python 3.8+)
uv tool install pre-commit
uv tool update-shell

# Install the hooks into your local repository
cd /path/to/rust-ros2-microstack
pre-commit install
```

**Usage:**

```bash
# Hooks run automatically on git commit
git commit -m "your message"

# Run manually on all files
pre-commit run --all-files

# Run manually on staged files only
pre-commit run

# Skip hooks temporarily (use sparingly!)
git commit --no-verify -m "your message"
```

**Troubleshooting:**

If `cargo clippy` fails in pre-commit but works normally:

```bash
# Ensure ROS2 environment is sourced
source /opt/ros/jazzy/setup.bash  # or setup.zsh

# Run pre-commit again
pre-commit run --all-files
```

**Updating hooks:**

```bash
# Update to latest hook versions
pre-commit autoupdate
```

### IDE Setup

**VS Code (recommended)**

Install extensions:

- rust-analyzer
- CodeLLDB (for debugging)
- ROS (for launch files, URDF syntax)

**Settings** (`.vscode/settings.json`):

```json
{
  "rust-analyzer.cargo.features": "all",
  "rust-analyzer.checkOnSave.command": "clippy",
  "rust-analyzer.server.extraEnv": {
    "LD_LIBRARY_PATH": "/opt/ros/jazzy/lib"
  }
}
```

**CLion / RustRover**

Settings → Languages & Frameworks → Rust → External Linter: `clippy`

### Debugging

**GDB with Rust symbols:**

```bash
cargo build
rust-gdb target/debug/teleop_mux
```

**ROS2 topic inspection:**

```bash
# List all topics
ros2 topic list

# Echo messages
ros2 topic echo /cmd_vel

# Get topic info
ros2 topic info /cmd_vel -v

# Publish test message
ros2 topic pub /heartbeat std_msgs/msg/Empty '{}'
```

---

## Environment Variables Reference

| Variable | Purpose | Example |
|----------|---------|---------|
| `ROS_DOMAIN_ID` | Isolate ROS2 networks | `export ROS_DOMAIN_ID=42` |
| `RMW_IMPLEMENTATION` | Choose DDS vendor | `rmw_cyclonedds_cpp` |
| `RUST_LOG` | Set log level | `info`, `debug`, `trace` |
| `RUST_BACKTRACE` | Enable backtraces | `1` or `full` |

---

## Next Steps

1. **Verify setup**: Run `bash scripts/sim_smoke.sh`
2. **Explore examples**: `cargo run --example hello_publisher`
3. **Read architecture**: [docs/architecture.md](architecture.md)
4. **Learn ROS2 concepts**: [docs/ros2-primer.md](ros2-primer.md)

---

## Getting Help

- **ROS2 Answers**: [answers.ros.org](https://answers.ros.org)
- **r2r Issues**: [github.com/sequenceplanner/r2r/issues](https://github.com/sequenceplanner/r2r/issues)
- **Rust Forums**: [users.rust-lang.org](https://users.rust-lang.org)
- **Project Issues**: [your-repo-issues]

---

**Pro Tip**: Add `source /opt/ros/jazzy/setup.bash` to your shell RC file to avoid sourcing manually each session.
