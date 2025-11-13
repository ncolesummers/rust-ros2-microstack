# ROS2 Primer for Rustaceans

A guide to ROS2 concepts explained through a Rust lens. If you're coming from pure Rust, this doc bridges the gap to distributed robotics systems.

## What is ROS2?

**ROS2** (Robot Operating System 2) is a framework for building distributed robotics applications. Think of it as:

- **Microservices for robots**: Nodes are independent processes communicating over a network
- **Typed message passing**: Publishers and subscribers with schema-enforced messages
- **Discovery-based**: Nodes find each other automatically (no central broker like Kafka)

Unlike Rust's ownership model (compile-time safety), ROS2 provides **runtime safety** through type-checked messaging and lifecycle management.

## Core Concepts

### Nodes

A **node** is an independent process or computation. In Rust terms:

- Like a separate `tokio::task` or process
- Has its own event loop
- Communicates only via messages (no shared memory by default)

```rust
// r2r example: Creating a node
let ctx = r2r::Context::create()?;
let mut node = r2r::Node::create(ctx, "my_node", "")?;
```

**Why separate nodes?**

- **Isolation**: Crash in one node doesn't kill the system
- **Language agnostic**: Mix Rust, Python, C++ nodes
- **Distributed**: Nodes can run on different machines

### Topics (Pub/Sub)

**Topics** are named channels for asynchronous message passing. Similar to:

- Rust's `tokio::sync::mpsc` but **network-wide**
- Erlang's message passing
- Go channels across processes

```rust
// Publisher
let publisher = node.create_publisher::<std_msgs::msg::String>("/chatter", QosPolicyReliable)?;
publisher.publish(&msg)?;

// Subscriber
let subscriber = node.create_subscription::<std_msgs::msg::String>("/chatter", QosPolicyReliable)?;
tokio::spawn(async move {
    while let Some(msg) = subscriber.next().await {
        println!("Received: {:?}", msg);
    }
});
```

**Key difference from Rust channels:**

- **Many-to-many**: Multiple publishers, multiple subscribers
- **Type-safe**: Message type enforced (like Rust generics)
- **Discoverable**: No need to pass channel handles; subscribe by topic name

### Services (RPC)

**Services** are request/response patterns. Like Rust's:

- `async fn` with a return value
- HTTP endpoints
- tonic gRPC (but ROS2-native)

```rust
// Service server (responds to requests)
let service = node.create_service::<example_interfaces::srv::AddTwoInts>("/add_two_ints")?;

// Service client (makes requests)
let client = node.create_client::<example_interfaces::srv::AddTwoInts>("/add_two_ints")?;
let response = client.request(&request).await?;
```

**Use case**: Operations requiring acknowledgment (e.g., "start motor" → "motor started").

### Actions (Long-Running Tasks)

**Actions** are for tasks with progress feedback. Similar to:

- Rust's `Stream` of progress updates
- Async iterators with cancellation
- Tokio's `CancellationToken` + progress channel

**Example**: "Navigate to waypoint" → streams progress → completes or is canceled.

### QoS (Quality of Service)

**QoS** policies control message delivery guarantees. Rust doesn't have a direct equivalent, but think:

- **Reliability**: TCP (reliable) vs UDP (best-effort)
- **Durability**: Keep last N messages for late joiners (like a buffered channel)
- **Liveliness**: Detect dead publishers

#### Common QoS Profiles

| Profile | Reliability | History Depth | Use Case |
|---------|-------------|---------------|----------|
| **Best Effort** | UDP-like | 1 | High-rate sensor data (lidar @ 10Hz) |
| **Reliable** | TCP-like | 10 | Commands, low-rate sensors |
| **System Default** | Reliable | 10 | Most topics |
| **Sensor Data** | Best Effort | 5 | Camera, IMU streams |

**Example in r2r:**

```rust
use r2r::QosProfile;

// Best effort (low latency, tolerates loss)
let qos = QosProfile::best_effort();

// Reliable (guaranteed delivery)
let qos = QosProfile::default(); // Usually reliable
```

**Why it matters**: Mismatched QoS → subscribers won't connect!

- Publisher: Reliable, Subscriber: Best Effort → ✗ No connection
- Both Reliable or both Best Effort → ✓ Connected

### DDS (Data Distribution Service)

**DDS** is the middleware ROS2 uses for discovery and transport. Think:

- **mDNS + ZeroMQ**: Auto-discovery + message passing
- **Under the hood**: You rarely interact with DDS directly

**What Rust devs should know:**

- DDS handles serialization (like `serde` but for network)
- Discovery is automatic (no Consul/etcd needed)
- Multiple vendors (CycloneDDS, FastDDS) - usually transparent

## Message Types

Messages are like **Rust structs with serde** but ROS2-specific:

```rust
// ROS2 message definition (.msg file)
// std_msgs/String.msg
// string data

// Generated Rust type (via r2r_msg_gen)
pub struct String {
    pub data: std::string::String,
}
```

**Common message packages:**

- `std_msgs`: Primitives (String, Int32, Bool, Header)
- `geometry_msgs`: Poses, Twists, Transforms
- `sensor_msgs`: LaserScan, Image, Imu
- `nav_msgs`: Odometry, Path

**Custom messages**: Define `.msg` files, r2r generates Rust structs at build time.

## Clocks & Time

ROS2 has **simulated time** for testing. In Rust terms:

- Like `tokio::time::pause()` but system-wide
- Gazebo publishes `/clock` → all nodes use sim time

```rust
// Get current time (respects /clock if use_sim_time=true)
let now = node.get_clock()?.now()?;
```

**Why?** Replay rosbags at different speeds, run tests deterministically.

## Parameters

**Parameters** are runtime configuration values. Like:

- Environment variables
- Config files (TOML/YAML)
- But **dynamically reconfigurable** without restart

```rust
// Declare parameter with default
node.declare_parameter("timeout_ms", 200)?;

// Get value
let timeout: i64 = node.get_parameter("timeout_ms")?;

// Set from CLI
// ros2 run my_pkg my_node --ros-args -p timeout_ms:=500
```

**When to use:**

- Values that change between deployments (topic names, thresholds)
- Avoid: Large data, secrets (use env vars or files instead)

## Lifecycle Nodes

**Lifecycle nodes** have explicit state transitions. Similar to:

- Finite state machines
- Rust's typestate pattern
- Managed startup/shutdown

**States**: Unconfigured → Inactive → Active → Finalized

**Use case**: Ensure hardware is initialized before accepting commands.

## Coordinate Frames (TF2)

**TF2** manages coordinate transformations. Think:

- 3D transform tree (parent-child relationships)
- "Where is the camera relative to the robot base?"

**Not covered in this microstack** but essential for real robots.

## ROS2 vs Rust Idioms

| ROS2 Concept | Rust Equivalent | Key Difference |
|--------------|-----------------|----------------|
| Topic | `tokio::mpsc` channel | Network-wide, discoverable |
| Service | `async fn` RPC | Request/response over network |
| Node | `tokio::task` | Independent process, crash-isolated |
| QoS | TCP vs UDP | Configurable per-topic |
| Message | `struct` with `serde` | IDL-generated, cross-language |
| Parameter | Environment variable | Dynamically reconfigurable |

## Best Practices for Rust + ROS2

1. **Use `tracing` not `println!`**: Integrate with ROS2 logging levels
2. **Avoid `unwrap()` in callbacks**: Use `anyhow::Result` and log errors
3. **QoS matters**: Always verify publisher/subscriber QoS compatibility
4. **Respect shutdown signals**: Listen for `r2r`'s shutdown event
5. **Test with sim time**: Use `/clock` topic in integration tests
6. **Type safety**: Let r2r's generated types catch schema errors at compile time

## Common Pitfalls

**QoS Mismatch**

```rust
// Publisher: reliable
let pub = node.create_publisher::<String>("/topic", QosProfile::default())?;

// Subscriber: best_effort
let sub = node.create_subscription::<String>("/topic", QosProfile::best_effort())?;
// ❌ No connection! Incompatible QoS
```

**Blocking in Callbacks**

```rust
subscriber.for_each(|msg| {
    std::thread::sleep(Duration::from_secs(10)); // ❌ Blocks event loop!
    // Use tokio::spawn for long tasks
});
```

**Forgetting to Spin**

```rust
let mut node = r2r::Node::create(ctx, "my_node", "")?;
let _sub = node.create_subscription::<String>("/topic", QosProfile::default())?;
// ❌ Node must spin to process messages
loop {
    node.spin_once(Duration::from_millis(100));
}
```

## Learning Resources

- **ROS2 Official Docs**: [docs.ros.org](https://docs.ros.org/en/jazzy/)
- **r2r Examples**: [github.com/sequenceplanner/r2r/examples](https://github.com/sequenceplanner/r2r/tree/master/examples)
- **DDS Spec**: [omg.org/spec/DDS](https://www.omg.org/spec/DDS/) (optional deep dive)
- **This project's examples/**: Standalone r2r patterns

## Next Steps

1. Read [architecture.md](architecture.md) for system design
2. Run `examples/hello_publisher` and `examples/hello_subscriber`
3. Study `nodes/safety_watchdog` for timer patterns
4. Explore `apps/teleop_mux` for multi-subscription logic

---

**Remember**: ROS2 is **distributed-first**, Rust is **safety-first**. r2r brings them together with compile-time checks for runtime message passing.
