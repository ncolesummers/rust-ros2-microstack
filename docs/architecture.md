# Architecture & Design

## Overview

This microstack demonstrates production-ready ROS2 patterns in Rust using the r2r bindings. The architecture prioritizes **safety**, **observability**, and **educational clarity** to help developers learn ROS2 + Rust integration.

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     ROS2 DDS Network                         │
└─────────────────────────────────────────────────────────────┘
           │                    │                    │
           ▼                    ▼                    ▼
    ┌──────────────┐   ┌──────────────┐   ┌──────────────┐
    │   Safety     │   │   Teleop     │   │   Sensor     │
    │  Watchdog    │   │     Mux      │   │   Filter     │
    └──────────────┘   └──────────────┘   └──────────────┘
           │                    │                    │
    Subs: /heartbeat    Subs: /teleop/cmd_vel    Subs: /scan
    Pubs: /cmd_vel           /nav/cmd_vel       Pubs: /scan/filtered
                        Pubs: /cmd_vel


Flow: External sources → Sensor Filter → Perception
      Telop/Nav → Teleop Mux → Safety Watchdog → Robot
```

### Component Responsibilities

**Safety Watchdog** (`nodes/safety_watchdog`)

- **Purpose**: Emergency stop on heartbeat timeout
- **Pattern**: Timeout-based safety interlock
- **Key Learning**: Timer-based logic, QoS profiles (best_effort vs reliable)

**Teleop Mux** (`apps/teleop_mux`)

- **Purpose**: Arbitrate between manual and autonomous control
- **Pattern**: Priority-based message selection
- **Key Learning**: Multi-subscription, clap configuration, tokio concurrency

**Sensor Filter** (`apps/sensor_filter`)

- **Purpose**: Clamp lidar outliers for stable perception
- **Pattern**: Stateless message transformation
- **Key Learning**: sensor_msgs handling, metadata preservation, benchmarking

## Why r2r?

### Rust ROS2 Binding Landscape

| Feature | r2r | rclrs | rosrust |
|---------|-----|-------|---------|
| **ROS Version** | ROS2 | ROS2 | ROS1 only |
| **Async Model** | tokio-native | Manual | Callbacks |
| **Type Safety** | Strong | Strong | Moderate |
| **Message Gen** | Build-time (r2r_msg_gen) | Build-time | Runtime |
| **Maturity** | Active | Official but early | Mature (ROS1) |
| **Learning Curve** | Moderate | Steep | Low |
| **Production Ready** | Yes | Experimental | Yes (ROS1) |

### Why We Chose r2r

1. **Async-first design**: r2r embraces tokio, making concurrent node logic natural in Rust
2. **Ecosystem compatibility**: Works seamlessly with existing Rust async libraries
3. **Build-time safety**: Message types are generated at build time, catching errors early
4. **Active development**: Regular updates and responsive maintainers
5. **Real-world usage**: Deployed in production robotics systems

### Trade-offs

**Advantages:**

- Ergonomic async/await patterns for node logic
- Excellent interop with tokio ecosystem (tracing, anyhow, etc.)
- Minimal boilerplate compared to rclrs
- Strong community examples and documentation

**Disadvantages:**

- Not the official ROS2 Rust binding (rclrs is)
- Smaller community than rclcpp or rclpy
- Some advanced rcl features may lag behind C++ API
- Requires understanding of tokio runtime model

For this portfolio project, r2r's ergonomics and async-first design make it ideal for teaching modern Rust + ROS2 patterns.

## Design Decisions

### Binary vs Library Crates

**nodes/** are **libraries** with logic exposed for:

- Unit testing without spawning processes
- Reuse across multiple binaries
- Example: `safety_watchdog` could be imported by integration tests

**apps/** are **binaries** meant to:

- Be run directly via `cargo run`
- Provide user-facing CLI interfaces
- Example: `teleop_mux` is an end-user tool

This mixed structure balances testability and clarity.

### When to Use tokio vs r2r Spin

**Use r2r's event loop (spin)** when:

- Node logic is purely reactive (callbacks only)
- Simple pub/sub without complex async coordination

**Use tokio runtime** when:

- Timeout detection (safety_watchdog)
- Concurrent subscriptions with selection logic (teleop_mux)
- Integration with async Rust libraries (HTTP clients, databases)

All nodes in this project use **tokio** to demonstrate async patterns.

### QoS Profile Selection

**Best Effort** (safety_watchdog heartbeat subscriber):

- Low latency, tolerates packet loss
- Suitable for high-rate health monitoring
- Missed heartbeat triggers safety action anyway

**Reliable** (all publishers, most subscribers):

- Guaranteed delivery (retries on packet loss)
- Depth 10: Handles temporary subscriber lag
- Standard for command and sensor topics

See [ros2-primer.md](ros2-primer.md) for QoS deep dive.

## Module Organization

```
workspace/
├── apps/
│   └── teleop_mux/        # Binary crate with main.rs + CLI
├── nodes/
│   ├── safety_watchdog/   # Library crate exposing WatchdogNode
│   └── sensor_filter/     # Library crate exposing FilterNode
├── examples/              # Standalone learning examples
└── tests/
    └── integration/       # ROS2 message flow tests
```

**Rationale**: Libraries in `nodes/` allow `tests/integration/` to import and test node logic without subprocess management.

## Technology Choices

### Core Stack

- **r2r**: ROS2 bindings
- **tokio**: Async runtime (matches r2r's async model)
- **tracing**: Structured logging (better than println!, integrates with ROS2 ecosystems)
- **anyhow**: Error handling (ergonomic error propagation)
- **clap**: CLI parsing (derive macros for maintainability)

### Testing

- **criterion**: Micro-benchmarking (sensor_filter performance)
- **tokio::time::pause**: Deterministic async testing (watchdog timeout tests)

### Observability

- **tracing-subscriber**: Configurable log output (RUST_LOG env var)
- Future: OpenTelemetry integration for distributed tracing

## Future Enhancements

- **Action servers**: Demonstrate long-running tasks with feedback
- **Parameter server**: Runtime reconfiguration without restart
- **Lifecycle nodes**: Managed startup/shutdown for complex systems
- **Component composition**: Multiple nodes in single process for efficiency
- **Cross-compilation**: Deploy to ARM targets (e.g., Raspberry Pi)

## References

- [r2r GitHub](https://github.com/sequenceplanner/r2r)
- [ROS2 Humble Docs](https://docs.ros.org/en/humble/) (concepts apply to Jazzy)
- [Tokio Documentation](https://tokio.rs)
- [Rust API Guidelines](https://rust-lang.github.io/api-guidelines/)

---

**Next**: See [ros2-primer.md](ros2-primer.md) for ROS2 concepts explained for Rust developers.
