# Structured Logging Guide

This guide explains how to use structured logging in the rust-ros2-microstack project.

## Table of Contents

- [Quick Start](#quick-start)
- [Log Levels](#log-levels)
- [Structured Fields](#structured-fields)
- [Configuration](#configuration)
- [Output Formats](#output-formats)
- [Distributed Tracing](#distributed-tracing)
- [Best Practices](#best-practices)
- [Examples](#examples)

## Quick Start

Add the logging module to your dependencies (already included in workspace members):

```toml
[dependencies]
rust-ros2-microstack = { path = "../.." }
tracing = { workspace = true }
```

Initialize logging in your main function:

```rust
use rust_ros2_microstack::logging;
use tracing::info;

fn main() -> anyhow::Result<()> {
    logging::init();

    info!("Application started");

    // Your code here...

    Ok(())
}
```

That's it! You now have structured logging with environment-based configuration.

## Log Levels

Use appropriate log levels for different types of messages:

### `error!` - Unrecoverable Failures

Use for errors that prevent normal operation:

```rust
use tracing::error;

if let Err(e) = critical_operation() {
    error!(error = %e, "Failed to initialize ROS2 node");
    return Err(e);
}
```

### `warn!` - Recoverable Issues

Use for problems that don't stop execution but indicate potential issues:

```rust
use tracing::warn;

if elapsed > timeout {
    warn!(
        elapsed_ms = elapsed.as_millis(),
        timeout_ms = timeout.as_millis(),
        "Heartbeat timeout detected"
    );
}
```

### `info!` - Major State Changes

Use for important events that happen infrequently:

```rust
use tracing::info;

info!("Publisher created on topic: /chatter");
info!(count = message_count, "Sent all messages");
```

### `debug!` - Detailed Execution Flow

Use for diagnostic information during development:

```rust
use tracing::debug;

debug!(topic = %topic_name, qos = ?qos_profile, "Subscribing to topic");
debug!(msg_id = msg.id, "Processing message");
```

### `trace!` - Very Verbose

Use sparingly for extremely detailed debugging (disabled by default):

```rust
use tracing::trace;

trace!(bytes_read = n, "Read data from socket");
```

## Structured Fields

Unlike printf-style logging, structured logging attaches key-value pairs to messages:

### Basic Fields

```rust
info!(count = 42, "Published message");
// Output: Published message count=42
```

### Display Formatting (`%`)

Use `%` for types implementing `Display`:

```rust
info!(topic = %topic_name, "Subscribed");
// Good for strings, numbers, etc.
```

### Debug Formatting (`?`)

Use `?` for types implementing `Debug`:

```rust
debug!(config = ?node_config, "Loaded configuration");
// Good for complex structs
```

### Multiple Fields

Combine multiple fields in one log statement:

```rust
info!(
    node = "safety_watchdog",
    timeout_ms = 1000,
    topic = %topic_name,
    "Node initialized"
);
```

### Computed Fields

Fields can be expressions:

```rust
info!(
    duration_ms = start.elapsed().as_millis(),
    success = result.is_ok(),
    "Operation completed"
);
```

## Configuration

All configuration is done through environment variables.

### `RUST_LOG` - Filter Directives

Control which logs are shown:

```bash
# Show only warnings and errors
RUST_LOG=warn cargo run --example hello_publisher

# Show all info and above
RUST_LOG=info cargo run

# Show debug logs
RUST_LOG=debug cargo run

# Show trace logs for specific modules
RUST_LOG=rust_ros2_microstack=trace cargo run

# Multiple filters
RUST_LOG=rust_ros2_microstack=debug,r2r=info cargo run
```

### `LOG_FORMAT` - Output Format

Choose the output format (default: `pretty`):

```bash
# Pretty console output (default, good for development)
cargo run --example hello_publisher

# JSON output (good for production/log aggregation)
LOG_FORMAT=json cargo run --example hello_publisher

# OpenTelemetry export (send to collector)
LOG_FORMAT=otlp OTLP_ENDPOINT=http://localhost:4317 cargo run
```

### `OTLP_ENDPOINT` - Telemetry Collector

When using `LOG_FORMAT=otlp`, specify the collector endpoint:

```bash
OTLP_ENDPOINT=http://localhost:4317 LOG_FORMAT=otlp cargo run
```

### `SERVICE_NAME` - Service Identifier

Override the service name for distributed tracing (default: `rust-ros2-microstack`):

```bash
SERVICE_NAME=my-robot-node LOG_FORMAT=otlp OTLP_ENDPOINT=http://localhost:4317 cargo run
```

## Output Formats

### Pretty Format (Default)

Human-readable output for development:

```
2025-11-12T10:30:45.123Z  INFO rust_ros2_microstack: Publisher created on topic: /chatter
2025-11-12T10:30:45.234Z  INFO hello_publisher: Published message count=1 message="Hello, ROS2!"
```

### JSON Format

Machine-readable output for log aggregation:

```json
{"timestamp":"2025-11-12T10:30:45.123Z","level":"INFO","target":"rust_ros2_microstack","fields":{"message":"Publisher created on topic: /chatter"}}
{"timestamp":"2025-11-12T10:30:45.234Z","level":"INFO","target":"hello_publisher","fields":{"count":1,"message":"Hello, ROS2!","log_message":"Published message"}}
```

Perfect for:
- Log aggregation systems (ELK, Splunk, etc.)
- Cloud logging (CloudWatch, Stackdriver)
- Automated log analysis

### OTLP Format

Exports traces to OpenTelemetry-compatible backends:

```bash
# Start Jaeger (supports OpenTelemetry)
docker run -d --name jaeger \
  -p 4317:4317 \
  -p 16686:16686 \
  jaegertracing/all-in-one:latest

# Run your application
LOG_FORMAT=otlp OTLP_ENDPOINT=http://localhost:4317 cargo run

# View traces at http://localhost:16686
```

## Distributed Tracing

Use spans to track operations across async boundaries:

### Basic Span

```rust
use tracing::info_span;

let span = info_span!("process_message", msg_id = msg.id);
let _enter = span.enter();

// All logs within this scope are associated with the span
info!("Processing started");
do_work().await;
info!("Processing completed");
```

### Instrument Macro

Automatically create spans for functions:

```rust
use tracing::instrument;

#[instrument(skip(ctx), fields(node_name = %node_name))]
async fn initialize_node(ctx: Context, node_name: &str) -> Result<Node> {
    info!("Initializing node");
    // Function entry/exit automatically logged
    let node = Node::create(ctx, node_name, "")?;
    Ok(node)
}
```

### Async Spans

Spans work across async boundaries:

```rust
use tracing::{info_span, Instrument};

async fn process_messages() {
    let messages = vec![1, 2, 3];

    for msg in messages {
        // Each message processed in its own span
        async {
            info!("Processing message");
            tokio::time::sleep(Duration::from_millis(100)).await;
            info!("Message processed");
        }
        .instrument(info_span!("process_msg", msg_id = msg))
        .await;
    }
}
```

### Span Fields

Add contextual information to spans:

```rust
use tracing::info_span;

let span = info_span!(
    "publish_message",
    topic = %topic_name,
    qos = "reliable",
    msg_size = msg.data.len()
);
```

## Best Practices

### 1. Choose the Right Log Level

```rust
// ❌ Don't use info for frequent events
for msg in messages {
    info!("Processing message {}", msg.id);  // Too noisy!
}

// ✅ Use debug for frequent events
for msg in messages {
    debug!(msg_id = msg.id, "Processing message");
}

// ✅ Use info for summaries
info!(processed = messages.len(), "Processed all messages");
```

### 2. Use Structured Fields, Not String Formatting

```rust
// ❌ Don't use string formatting
info!("Published message count={}", count);

// ✅ Use structured fields
info!(count = count, "Published message");
```

### 3. Log Errors with Context

```rust
// ❌ Don't lose error details
if let Err(e) = operation() {
    error!("Operation failed");
}

// ✅ Include error information
if let Err(e) = operation() {
    error!(error = %e, "Operation failed");
}
```

### 4. Don't Log Sensitive Data

```rust
// ❌ Don't log passwords, tokens, etc.
debug!(password = %user.password, "User login");

// ✅ Log only non-sensitive information
debug!(username = %user.username, "User login");
```

### 5. Use Spans for Long Operations

```rust
// ❌ Don't use separate log calls
info!("Starting initialization");
initialize().await;
info!("Initialization complete");

// ✅ Use a span to group related logs
async {
    info!("Initializing");
    initialize().await;
    info!("Complete");
}
.instrument(info_span!("initialization"))
.await;
```

### 6. Add Context to Error Returns

```rust
use anyhow::Context;

// ❌ Don't return errors without context
let file = File::open("config.yaml")?;

// ✅ Add context to errors
let file = File::open("config.yaml")
    .context("Failed to open configuration file")?;
```

## Examples

### Example 1: Simple Publisher

```rust
use rust_ros2_microstack::logging;
use tracing::{info, warn};
use anyhow::Result;

fn main() -> Result<()> {
    logging::init();

    info!("Starting publisher");

    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "publisher", "")?;

    info!(node = "publisher", "Node created");

    // Publishing loop...

    Ok(())
}
```

### Example 2: Data Processing with Debug Logs

```rust
use tracing::{debug, info, instrument};

#[instrument(skip(data))]
fn process_sensor_data(sensor_id: u32, data: &[f32]) -> Vec<f32> {
    info!(sensor_id = sensor_id, samples = data.len(), "Processing sensor data");

    let filtered: Vec<f32> = data.iter()
        .filter(|&&x| x > 0.0 && x < 10.0)
        .map(|&x| {
            debug!(sensor_id = sensor_id, raw_value = x, "Filtering value");
            x
        })
        .collect();

    info!(
        sensor_id = sensor_id,
        input_samples = data.len(),
        output_samples = filtered.len(),
        "Processing complete"
    );

    filtered
}
```

### Example 3: Error Handling with Context

```rust
use tracing::{error, info};
use anyhow::{Context, Result};

async fn publish_message(publisher: &Publisher, msg: Message) -> Result<()> {
    info!(msg_id = msg.id, "Publishing message");

    publisher
        .publish(&msg)
        .await
        .context("Failed to publish message")?;

    info!(msg_id = msg.id, "Message published successfully");
    Ok(())
}

// In main loop:
if let Err(e) = publish_message(&publisher, msg).await {
    error!(error = %e, msg_id = msg.id, "Publish failed");
}
```

### Example 4: Watchdog with Structured Warnings

```rust
use tracing::{info, warn, instrument};
use std::time::{Duration, Instant};

#[instrument(skip(timeout))]
async fn monitor_heartbeat(timeout: Duration) {
    let mut last_heartbeat = Instant::now();

    info!(timeout_ms = timeout.as_millis(), "Starting heartbeat monitor");

    loop {
        tokio::time::sleep(Duration::from_millis(100)).await;

        let elapsed = last_heartbeat.elapsed();
        if elapsed > timeout {
            warn!(
                elapsed_ms = elapsed.as_millis(),
                timeout_ms = timeout.as_millis(),
                "Heartbeat timeout detected"
            );

            // Publish safety command...
            info!("Published zero velocity command");
        }
    }
}
```

### Example 5: Custom Configuration

```rust
use rust_ros2_microstack::logging::{LoggingConfig, LogFormat};
use tracing::{info, Level};
use anyhow::Result;

fn main() -> Result<()> {
    // Custom configuration
    let config = LoggingConfig::new("my-custom-node")
        .with_format(LogFormat::Json)
        .with_default_level(Level::DEBUG);

    rust_ros2_microstack::logging::init_with_config(config)?;

    info!("Custom logging initialized");

    Ok(())
}
```

## Integration with ROS2

### Logging ROS2 Events

```rust
use tracing::{info, debug};

// Node lifecycle
info!(node = %node_name, namespace = %namespace, "Node created");

// Topic operations
info!(topic = %topic_name, qos = ?qos_profile, "Publisher created");
debug!(topic = %topic_name, msg_seq = seq, "Message published");

// Subscriptions
info!(topic = %topic_name, "Subscription created");
debug!(topic = %topic_name, msg_size = msg.data.len(), "Message received");

// Services
info!(service = %service_name, "Service created");
debug!(service = %service_name, request_id = id, "Service call received");
```

### Logging Performance Metrics

```rust
use tracing::info;
use std::time::Instant;

let start = Instant::now();

// Do work...

info!(
    operation = "publish_batch",
    duration_ms = start.elapsed().as_millis(),
    message_count = messages.len(),
    "Batch published"
);
```

## Troubleshooting

### Logs Not Appearing

Check your `RUST_LOG` setting:

```bash
# Make sure it's not filtered out
RUST_LOG=debug cargo run

# Or enable everything
RUST_LOG=trace cargo run
```

### Too Many Logs

Reduce verbosity:

```bash
# Only show warnings and errors
RUST_LOG=warn cargo run

# Or filter by module
RUST_LOG=rust_ros2_microstack=info cargo run
```

### OTLP Connection Fails

Verify the collector is running:

```bash
# Check if collector is accessible
curl http://localhost:4317

# Check logs for connection errors
RUST_LOG=debug LOG_FORMAT=otlp OTLP_ENDPOINT=http://localhost:4317 cargo run
```

## Further Reading

- [Tracing Documentation](https://docs.rs/tracing/)
- [OpenTelemetry Specification](https://opentelemetry.io/)
- [CONTRIBUTING.md](../CONTRIBUTING.md) - Project contribution guidelines
- [Examples](../examples/) - Working code examples

## Summary

- Use `logging::init()` in your main function
- Choose appropriate log levels (`error!`, `warn!`, `info!`, `debug!`, `trace!`)
- Use structured fields instead of string formatting
- Configure via environment variables (`RUST_LOG`, `LOG_FORMAT`)
- Use spans (`#[instrument]`) for distributed tracing
- Follow best practices for production-ready logging
