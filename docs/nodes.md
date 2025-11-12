# Node Implementation Deep Dive

Detailed implementation patterns, design choices, and learning objectives for each node in the microstack.

## Table of Contents

- [Safety Watchdog](#safety-watchdog)
- [Teleop Mux](#teleop-mux)
- [Sensor Filter](#sensor-filter)

---

## Safety Watchdog

**Location**: `nodes/safety_watchdog`
**Type**: Library crate
**Purpose**: Emergency stop on heartbeat timeout

### Overview

The safety watchdog monitors heartbeat messages and publishes zero-velocity commands when heartbeats stop arriving. This pattern is critical for preventing uncontrolled robot motion during communication failures or operator absence.

### Learning Objectives

- Implement timer-based safety interlocks
- Configure different QoS profiles (best_effort subscriber, reliable publisher)
- Use tokio for async timeout detection
- Handle concurrent subscription and timer logic

### Architecture

```
┌─────────────────┐
│   External      │
│   Heartbeat     │
│   Publisher     │
└────────┬────────┘
         │ /heartbeat (std_msgs/Empty)
         │ QoS: best_effort
         ▼
┌─────────────────┐
│     Safety      │
│   Watchdog      │
│   Node          │
└────────┬────────┘
         │ /cmd_vel (geometry_msgs/Twist)
         │ QoS: reliable, depth 10
         ▼
┌─────────────────┐
│   Robot Base    │
│   Controller    │
└─────────────────┘
```

### Key Implementation Patterns

#### 1. Timeout Detection with Tokio

```rust
use tokio::time::{interval, Duration};

// Track last heartbeat time
let mut last_heartbeat = Instant::now();

// Check timeout periodically
let mut check_interval = interval(Duration::from_millis(timeout_ms));

loop {
    tokio::select! {
        // Heartbeat received
        Some(_) = heartbeat_sub.next() => {
            last_heartbeat = Instant::now();
        }

        // Timeout check tick
        _ = check_interval.tick() => {
            if last_heartbeat.elapsed() > Duration::from_millis(timeout_ms) {
                // Publish zero velocity
                publish_zero_twist(&publisher)?;
            }
        }
    }
}
```

**Why tokio::select!?**: Allows concurrent waiting on heartbeat messages and timer ticks without blocking.

#### 2. QoS Configuration

**Heartbeat Subscriber**: Best Effort
- **Rationale**: Heartbeats are high-rate; missing one is acceptable (timeout triggers anyway)
- **Trade-off**: Lower latency vs guaranteed delivery

**Cmd Vel Publisher**: Reliable, Depth 10
- **Rationale**: Critical commands must not be lost
- **Depth 10**: Handles temporary subscriber lag without dropping messages

```rust
use r2r::QosProfile;

// Best effort for high-rate monitoring
let heartbeat_qos = QosProfile::best_effort();

// Reliable for critical commands
let cmd_vel_qos = QosProfile::default().reliable(); // depth 10
```

#### 3. Zero-Velocity Message Construction

```rust
use r2r::geometry_msgs::msg::Twist;

fn zero_twist() -> Twist {
    Twist {
        linear: Vector3 { x: 0.0, y: 0.0, z: 0.0 },
        angular: Vector3 { x: 0.0, y: 0.0, z: 0.0 },
    }
}
```

**Best Practice**: Create a helper function to ensure consistent zero-velocity messages.

### CLI Interface

```bash
cargo run -p nodes/safety_watchdog -- \
  --heartbeat /heartbeat \
  --timeout-ms 200 \
  --output /cmd_vel
```

**Flags**:
- `--heartbeat`: Topic name for heartbeat subscription
- `--timeout-ms`: Timeout duration before publishing zero velocity
- `--output`: Topic name for velocity command publication

### Testing Strategies

**Manual Test**:
```bash
# Terminal 1: Start watchdog
cargo run -p nodes/safety_watchdog -- --timeout-ms 200

# Terminal 2: Publish heartbeats
ros2 topic pub /heartbeat std_msgs/msg/Empty '{}' -r 5

# Terminal 3: Monitor cmd_vel
ros2 topic echo /cmd_vel

# Stop Terminal 2 → cmd_vel should publish zeros
```

**Integration Test** (see `tests/integration/test_safety_watchdog.rs`):
- Use `tokio::time::pause()` for deterministic async testing
- Spawn watchdog in background task
- Publish heartbeats, verify silence
- Stop heartbeats, verify zero-velocity publication

### Edge Cases

- **Startup**: Watchdog publishes zeros until first heartbeat
- **Rapid toggle**: Rate-limit zero-velocity publications (once per timeout window)
- **Shutdown**: Graceful cleanup of subscriptions and publishers

---

## Teleop Mux

**Location**: `apps/teleop_mux`
**Type**: Binary crate
**Purpose**: Arbitrate between teleop and navigation commands

### Overview

The teleop mux subscribes to both manual teleop commands and autonomous navigation commands, forwarding the higher-priority stream based on runtime configuration. This allows operators to override autonomy on demand.

### Learning Objectives

- Subscribe to multiple topics of the same type
- Implement priority-based message selection
- Use clap for CLI configuration
- Handle concurrent subscriptions with tokio::select!

### Architecture

```
┌──────────────┐        ┌──────────────┐
│   Teleop     │        │     Nav      │
│   Commands   │        │   Commands   │
└──────┬───────┘        └──────┬───────┘
       │ /teleop/cmd_vel       │ /nav/cmd_vel
       │ Twist                 │ Twist
       ▼                       ▼
    ┌────────────────────────────┐
    │      Teleop Mux Node       │
    │   Priority: teleop|nav     │
    └───────────┬────────────────┘
                │ /cmd_vel
                ▼
         ┌──────────────┐
         │  Robot Base  │
         └──────────────┘
```

### Key Implementation Patterns

#### 1. Multi-Subscription with Priority

```rust
enum Priority {
    Teleop,
    Nav,
}

let mut teleop_sub = node.create_subscription("/teleop/cmd_vel", qos)?;
let mut nav_sub = node.create_subscription("/nav/cmd_vel", qos)?;

let mut latest_teleop: Option<Twist> = None;
let mut latest_nav: Option<Twist> = None;

loop {
    tokio::select! {
        Some(msg) = teleop_sub.next() => {
            latest_teleop = Some(msg);
            if matches!(priority, Priority::Teleop) {
                publisher.publish(&msg)?;
            }
        }
        Some(msg) = nav_sub.next() => {
            latest_nav = Some(msg);
            if matches!(priority, Priority::Nav) {
                publisher.publish(&msg)?;
            }
        }
    }
}
```

**Why store latest messages?**: Allows switching priority without waiting for new messages.

#### 2. CLI Configuration with Clap

```rust
use clap::Parser;

#[derive(Parser)]
#[command(name = "teleop_mux")]
#[command(about = "Multiplex teleop and nav commands by priority")]
struct Args {
    #[arg(long, default_value = "teleop")]
    priority: String, // "teleop" or "nav"

    #[arg(long, default_value = "/teleop/cmd_vel")]
    teleop_topic: String,

    #[arg(long, default_value = "/nav/cmd_vel")]
    nav_topic: String,

    #[arg(long, default_value = "/cmd_vel")]
    output_topic: String,
}
```

**Derive macros**: Clap generates help text and parsing logic automatically.

#### 3. QoS Consistency

All topics use **Reliable, Depth 10**:
- **Rationale**: Command velocities are critical; packet loss is unacceptable
- **Depth 10**: Standard for command topics

### CLI Interface

```bash
# Priority: teleop (default)
cargo run -p apps/teleop_mux -- --priority teleop

# Priority: nav
cargo run -p apps/teleop_mux -- --priority nav

# Custom topics
cargo run -p apps/teleop_mux -- \
  --teleop-topic /manual/cmd_vel \
  --nav-topic /auto/cmd_vel \
  --output /robot/cmd_vel \
  --priority teleop
```

### Testing Strategies

**Manual Test**:
```bash
# Terminal 1: Start mux (teleop priority)
cargo run -p apps/teleop_mux -- --priority teleop

# Terminal 2: Publish teleop commands
ros2 topic pub /teleop/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}}' -r 5

# Terminal 3: Publish nav commands
ros2 topic pub /nav/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}' -r 5

# Terminal 4: Monitor output
ros2 topic echo /cmd_vel
# Should show x: 0.1 (teleop priority)

# Restart Terminal 1 with --priority nav
# Should show x: 0.5 (nav priority)
```

**Integration Test**:
- Publish to both input topics
- Verify output matches priority setting
- Change priority, verify switch

### Design Considerations

**State vs Stateless**:
- **Stateless** (current): Publish immediately on receipt
- **Stateful** (future): Track "recent" messages (e.g., last 100ms) and switch on timeout

**Arbitration Strategies**:
- **Priority** (current): Simple, explicit
- **Timeout-based**: Favor teleop if active, fall back to nav
- **Velocity-based**: Favor non-zero commands

---

## Sensor Filter

**Location**: `nodes/sensor_filter`
**Type**: Library crate
**Purpose**: Clamp LaserScan outliers

### Overview

The sensor filter subscribes to raw lidar scans, clamps range values to configurable bounds, and republishes filtered scans. This stabilizes downstream perception algorithms by removing noise and outliers.

### Learning Objectives

- Work with complex sensor messages (LaserScan)
- Implement stateless message transformations
- Preserve metadata (headers, timestamps, angle info)
- Write micro-benchmarks with criterion

### Architecture

```
┌──────────────┐
│    Lidar     │
│   Hardware   │
└──────┬───────┘
       │ /scan (sensor_msgs/LaserScan)
       ▼
┌──────────────────┐
│  Sensor Filter   │
│  Clamp [min,max] │
└──────┬───────────┘
       │ /scan/filtered
       ▼
┌──────────────────┐
│   Perception     │
│   Pipeline       │
└──────────────────┘
```

### Key Implementation Patterns

#### 1. Message Transformation

```rust
use r2r::sensor_msgs::msg::LaserScan;

fn clamp_scan(mut scan: LaserScan, min: f32, max: f32) -> LaserScan {
    scan.ranges = scan.ranges.iter()
        .map(|&r| r.clamp(min, max))
        .collect();

    // Preserve all metadata
    scan // header, angle_min, angle_max, etc. unchanged
}
```

**Critical**: Preserve `header` (timestamp, frame_id) for downstream TF2 lookups.

#### 2. Subscription + Transformation + Publication

```rust
let mut sub = node.create_subscription::<LaserScan>("/scan", qos)?;
let pub = node.create_publisher::<LaserScan>("/scan/filtered", qos)?;

while let Some(msg) = sub.next().await {
    let filtered = clamp_scan(msg, args.min, args.max);
    pub.publish(&filtered)?;
}
```

**Pipeline pattern**: Subscribe → Transform → Publish (common in ROS2).

#### 3. Benchmarking with Criterion

**Goal**: Ensure clamp operation is fast enough for 10Hz lidar.

```rust
// benches/sensor_filter.rs
use criterion::{black_box, criterion_group, criterion_main, Criterion};

fn benchmark_clamp(c: &mut Criterion) {
    let scan = LaserScan {
        ranges: vec![1.0; 360], // Typical 360-degree scan
        // ... other fields
    };

    c.bench_function("clamp_scan", |b| {
        b.iter(|| clamp_scan(black_box(scan.clone()), 0.1, 8.0))
    });
}

criterion_group!(benches, benchmark_clamp);
criterion_main!(benches);
```

**Run**: `cargo bench`

**Acceptable**: < 1ms for 360-point scan (allows 10Hz with headroom).

### CLI Interface

```bash
cargo run -p nodes/sensor_filter -- \
  --input /scan \
  --output /scan/filtered \
  --min 0.12 \
  --max 8.0
```

**Flags**:
- `--input`: Input topic name
- `--output`: Output topic name
- `--min`: Minimum valid range (meters)
- `--max`: Maximum valid range (meters)

### Testing Strategies

**Rosbag Test**:
```bash
# Record real lidar data
ros2 bag record /scan -o test_bag

# Replay while running filter
ros2 bag play test_bag &
cargo run -p nodes/sensor_filter -- --min 0.1 --max 10.0

# Verify filtered output
ros2 topic echo /scan/filtered | head
```

**Unit Test**:
```rust
#[test]
fn test_clamp_preserves_header() {
    let scan = LaserScan {
        header: Header { frame_id: "laser".into(), .. },
        ranges: vec![0.05, 5.0, 15.0],
        // ...
    };

    let filtered = clamp_scan(scan.clone(), 0.1, 10.0);

    assert_eq!(filtered.header.frame_id, "laser");
    assert_eq!(filtered.ranges, vec![0.1, 5.0, 10.0]);
}
```

### Performance Considerations

**Avoid Allocations**:
```rust
// ❌ Slow: Creates new Vec
scan.ranges = scan.ranges.iter().map(|&r| r.clamp(min, max)).collect();

// ✅ Fast: In-place mutation
scan.ranges.iter_mut().for_each(|r| *r = r.clamp(min, max));
```

**SIMD Opportunities** (future):
- Use `packed_simd` for vectorized clamping
- Expected 4-8x speedup on large scans

### Edge Cases

- **Empty scan**: ranges.len() == 0 → Publish as-is
- **Infinity values**: `f32::INFINITY` → Clamp to max
- **NaN values**: Preserve or replace with max? (Design choice)

---

## Common Patterns Across Nodes

### Error Handling

```rust
use anyhow::{Context, Result};

fn run() -> Result<()> {
    let node = r2r::Node::create(ctx, "my_node", "")
        .context("Failed to create node")?;
    // ...
}

fn main() {
    if let Err(e) = run() {
        tracing::error!("Node failed: {:?}", e);
        std::process::exit(1);
    }
}
```

**Why anyhow?**: Rich error messages with context for debugging.

### Structured Logging

```rust
use tracing::{info, warn, error, debug};

// Initialize once in main
tracing_subscriber::fmt::init();

// Use throughout
info!("Node started");
debug!(topic = %topic_name, "Subscribed to topic");
warn!(timeout_ms = %timeout, "Heartbeat timeout");
error!(error = %e, "Failed to publish");
```

**Levels**: Set via `RUST_LOG=debug cargo run ...`

### Graceful Shutdown

```rust
use tokio::signal;

tokio::select! {
    _ = signal::ctrl_c() => {
        info!("Shutting down gracefully");
    }
    _ = run_node() => {}
}
```

**Best Practice**: Listen for Ctrl+C and clean up resources.

---

## Further Reading

- [ROS2 Best Practices](https://docs.ros.org/en/jazzy/The-ROS2-Project/Contributing/Developer-Guide.html)
- [r2r Examples](https://github.com/sequenceplanner/r2r/tree/master/examples)
- [Tokio Tutorial](https://tokio.rs/tokio/tutorial)
- [Criterion Benchmarking](https://bheisler.github.io/criterion.rs/book/)

---

**Next**: Explore `examples/` for isolated learning examples of each pattern.
