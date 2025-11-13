# Rust ROS2 Microstack

> Learn ROS2 in Rust through practical, production-ready examples

A learning-focused portfolio project demonstrating real-world ROS2 patterns using the [r2r](https://github.com/sequenceplanner/r2r) Rust bindings. This microstack implements common robotics components: safety watchdogs, teleop multiplexers, and sensor filters.

## Why This Stack?

This project uses **r2r** for ROS2 integration in Rust. Why r2r?

- **Async-first design**: Native tokio integration for ergonomic concurrent programming
- **Type-safe**: Leverages Rust's type system to catch ROS2 errors at compile time
- **Lightweight**: Minimal overhead compared to C++ rclcpp
- **Active development**: Well-maintained with strong community support

For a detailed comparison of r2r vs rclrs vs rosrust, design decisions, and architecture diagrams, see [docs/architecture.md](docs/architecture.md).

## Prerequisites

- **ROS2 Jazzy** ([Installation guide](https://docs.ros.org/en/jazzy/Installation.html))
- **Rust** (stable toolchain, edition 2024)
- **Gazebo** (for simulation and testing)

## Project Structure

This workspace uses a **mixed structure** to balance reusability and clarity:

```
rust-ros2-microstack/
├── apps/          # User-facing executables (teleop_mux)
├── nodes/         # Reusable library crates (safety_watchdog, sensor_filter)
├── examples/      # Standalone r2r learning examples
├── docs/          # Architecture and learning resources
└── scripts/       # Development and testing utilities
```

**Why apps/ vs nodes/?**

- `apps/` contains binaries meant to be run directly (e.g., `cargo run -p apps/teleop_mux`)
- `nodes/` are libraries exposing node logic, allowing reuse and easier unit testing
- This pattern separates interface (binary) from implementation (library)

See [CONTRIBUTING.md](CONTRIBUTING.md) for more details on adding new components.

## Quickstart (60 seconds)

```bash
# 1. Load ROS2 environment
source scripts/dev_env.sh

# 2. Verify ROS2 is available
which ros2  # Should print a path
ros2 topic list  # Should run without error

# 3. Build the workspace
cargo build --workspace

# 4. Run the smoke test (optional - verifies Gazebo + ROS bridge)
bash scripts/sim_smoke.sh

# 5. Run a node
cargo run -p apps/teleop_mux -- --priority teleop
```

See [docs/setup.md](docs/setup.md) for detailed setup instructions and troubleshooting.

## Nodes & Interfaces

### Topic/QoS Reference

| Node | Subscribes To | Publishes To | QoS Policy | Rate (Hz) |
|------|---------------|--------------|------------|-----------|
| **safety_watchdog** | `/heartbeat` (std_msgs/Empty) | `/cmd_vel` (geometry_msgs/Twist) | Sub: best_effort<br>Pub: reliable, depth 10 | On timeout |
| **teleop_mux** | `/teleop/cmd_vel` (Twist)<br>`/nav/cmd_vel` (Twist) | `/cmd_vel` (Twist) | All: reliable, depth 10 | ~10 |
| **sensor_filter** | `/scan` (sensor_msgs/LaserScan) | `/scan/filtered` (LaserScan) | All: reliable, depth 10 | ~10 |

### Node Descriptions

**Safety Watchdog** (`nodes/safety_watchdog`)
Monitors heartbeat messages and publishes zero velocity commands when heartbeats stop. Demonstrates timer-based safety patterns and QoS configuration.

**Teleop Mux** (`apps/teleop_mux`)
Multiplexes teleop and navigation command velocities based on configurable priority. Shows multi-subscription patterns and runtime configuration with clap.

**Sensor Filter** (`nodes/sensor_filter`)
Clamps LaserScan ranges to remove outliers. Illustrates sensor message processing and metadata preservation.

See [docs/nodes.md](docs/nodes.md) for implementation deep-dives.

## Learning Path

New to r2r or ROS2 in Rust? Follow this path:

1. **ROS2 Primer**: Read [docs/ros2-primer.md](docs/ros2-primer.md) for ROS2 concepts from a Rust perspective
2. **Architecture**: Review [docs/architecture.md](docs/architecture.md) to understand design decisions
3. **Examples**: Run standalone examples in `examples/` (hello_publisher, hello_subscriber, simple_timer)
4. **Nodes**: Study node implementations in `apps/` and `nodes/` to see patterns in action
5. **Testing**: Explore `tests/integration/` for ROS2 testing strategies

## Examples

Standalone minimal examples for learning r2r patterns:

```bash
# Publisher example
cargo run --example hello_publisher

# Subscriber example (in another terminal)
cargo run --example hello_subscriber

# Timer-based publishing
cargo run --example simple_timer

# Service example
cargo run --example basic_service
```

Each example demonstrates a single r2r pattern with extensive inline documentation. See [examples/README.md](examples/README.md) for details.

## Development

### Running Tests

```bash
# Unit tests
cargo test

# Integration tests (requires ROS2 environment)
source scripts/dev_env.sh
cargo test --test '*' -- --test-threads=1

# Benchmarks
cargo bench
```

### Code Quality

```bash
# Format check
cargo fmt --check

# Lints
cargo clippy --workspace --all-targets -- -D warnings
```

**Pre-commit hooks (optional but recommended):**

Automatically run format and lint checks before each commit:

```bash
pipx install pre-commit
pre-commit install
```

See [docs/setup.md](docs/setup.md#pre-commit-hooks) for detailed instructions and troubleshooting.

### Recording and Replaying ROS Bags

```bash
# Record sensor data (5 seconds)
bash scripts/bag_record.sh /scan 5

# Replay
bash scripts/bag_play.sh <bag_path>
```

## Documentation

- [Architecture & Design](docs/architecture.md) - Why r2r, system design, trade-offs
- [ROS2 Primer for Rustaceans](docs/ros2-primer.md) - ROS2 concepts explained for Rust developers
- [Setup Guide](docs/setup.md) - Detailed installation and troubleshooting
- [Node Deep Dive](docs/nodes.md) - Implementation patterns and best practices

## Contributing

Contributions are welcome! Please read [CONTRIBUTING.md](CONTRIBUTING.md) for:

- Code style guidelines
- How to add new nodes
- Testing requirements
- Documentation standards

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

**Built with:** Rust 🦀 | ROS2 Jazzy 🤖 | r2r 📡
