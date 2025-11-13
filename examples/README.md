# r2r Examples - Learning Path

This directory contains minimal, well-documented examples demonstrating core [r2r](https://github.com/sequenceplanner/r2r) patterns for ROS2 in Rust. Each example is a standalone binary designed to teach specific concepts.

## Prerequisites

Before running examples, ensure you have:

1. ROS2 Jazzy installed (see [setup.md](../docs/setup.md))
2. Rust toolchain (edition 2024)
3. Environment sourced: `source scripts/dev_env.sh`

## Examples

| Example | What You'll Learn |
|---------|-------------------|
| [`hello_publisher`](hello_publisher.rs) | • Creating a ROS2 node with r2r<br>• Publishing messages to topics<br>• Configuring QoS profiles<br>• Using tokio intervals for periodic tasks |
| [`hello_subscriber`](hello_subscriber.rs) | • Subscribing to ROS2 topics<br>• Async message handling with r2r<br>• Processing incoming messages |
| [`simple_timer`](simple_timer.rs) | • Timer-based periodic publishing<br>• Using `tokio::select!` for concurrent operations<br>• Timeout detection patterns<br>• Multi-task coordination |
| [`basic_service`](basic_service.rs) | • Creating service servers<br>• Calling services as a client<br>• Request/response communication<br>• Running server and client in one binary |

## Usage

Run any example with:

```bash
# Source ROS2 environment
source scripts/dev_env.sh

# Run an example
cargo run --example hello_publisher

# In another terminal (for subscriber/service examples)
cargo run --example hello_subscriber
```

## Logging

All examples use structured logging with OpenTelemetry support. Configure via environment variables:

```bash
# Show debug logs
RUST_LOG=debug cargo run --example hello_publisher

# JSON output (for log aggregation)
LOG_FORMAT=json cargo run --example hello_publisher

# OpenTelemetry export (requires collector)
LOG_FORMAT=otlp OTLP_ENDPOINT=http://localhost:4317 cargo run --example hello_publisher
```

**See [docs/logging.md](../docs/logging.md) for comprehensive documentation** including:

- All configuration options
- Output format examples
- Distributed tracing setup
- Best practices

## Learning Path

We recommend following this order:

1. **hello_publisher** - Start here to understand basic publishing
2. **hello_subscriber** - Learn to receive messages
3. **simple_timer** - Explore timer-based patterns
4. **basic_service** - Understand request/response communication

## Testing Examples

For integration testing of multiple examples:

```bash
./scripts/sim_smoke.sh
```

## Further Reading

- [ROS2 Primer for Rustaceans](../docs/ros2-primer.md)
- [Node Implementation Guide](../docs/nodes.md)
- [Architecture Overview](../docs/architecture.md)
