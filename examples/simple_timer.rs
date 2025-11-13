//! # Simple Timer
//!
//! ## What you'll learn
//! - Timer-based periodic publishing
//! - Using `futures::select!` for concurrent operations
//! - Timeout detection patterns
//! - Multi-task coordination
//! - The r2r event loop pattern (spin_once + async executor)
//!
//! ## Usage
//! ```bash
//! source scripts/dev_env.sh
//! cargo run --example simple_timer
//! ```
//!
//! This example demonstrates a safety watchdog pattern: a heartbeat
//! publisher that also monitors for timeout conditions.

use anyhow::Result;
use futures::{executor::LocalPool, select, task::LocalSpawnExt, FutureExt};
use r2r::{QosProfile, std_msgs::msg::Empty};
use tracing::{info, warn};
use tracing_subscriber;

fn main() -> Result<()> {
    // Initialize tracing for structured logging
    tracing_subscriber::fmt::init();

    info!("Starting simple_timer example");

    // Create ROS2 context and node
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "simple_timer", "")?;

    // Configure QoS - using BEST_EFFORT for high-frequency heartbeat
    // This is appropriate for data that can tolerate occasional loss
    let qos = QosProfile::default().best_effort().keep_last(1);

    // Create publishers for heartbeat and timeout warnings
    let heartbeat_pub = node.create_publisher::<Empty>("/heartbeat", qos.clone())?;
    let timeout_pub = node.create_publisher::<Empty>("/timeout_warning", qos)?;

    info!("Publishers created:");
    info!("  - /heartbeat (500ms interval)");
    info!("  - /timeout_warning (triggered if no spin for >2s)");

    // Create multiple r2r wall timers with different periods
    // heartbeat_timer: Fast periodic publish (500ms)
    let mut heartbeat_timer = node.create_wall_timer(std::time::Duration::from_millis(500))?;

    // watchdog_timer: Monitors for timeout conditions (2 seconds)
    let mut watchdog_timer = node.create_wall_timer(std::time::Duration::from_secs(2))?;

    // Track last successful operation for timeout detection
    let mut last_heartbeat = std::time::Instant::now();
    let timeout_threshold = std::time::Duration::from_secs(2);

    info!("Timers started. Press Ctrl+C to stop.");

    // Set up the async task executor
    // r2r uses futures::executor::LocalPool for lightweight async task execution
    let mut pool = LocalPool::new();
    let spawner = pool.spawner();

    // Spawn a task to handle timer events concurrently
    spawner.spawn_local(async move {
        loop {
            // futures::select! runs multiple async operations concurrently
            // The first one to complete will be handled
            select! {
                // Heartbeat timer: publish periodic Empty messages
                result = heartbeat_timer.tick().fuse() => {
                    match result {
                        Ok(_) => {
                            let msg = Empty {};
                            if let Err(e) = heartbeat_pub.publish(&msg) {
                                warn!(error = %e, "Failed to publish heartbeat");
                            } else {
                                info!("Heartbeat sent");
                                // Update last activity time
                                last_heartbeat = std::time::Instant::now();
                            }
                        }
                        Err(e) => {
                            warn!(error = %e, "Heartbeat timer error");
                        }
                    }
                }

                // Watchdog timer: check for timeout conditions
                result = watchdog_timer.tick().fuse() => {
                    match result {
                        Ok(_) => {
                            let elapsed = last_heartbeat.elapsed();
                            if elapsed > timeout_threshold {
                                warn!(
                                    elapsed_ms = elapsed.as_millis(),
                                    threshold_ms = timeout_threshold.as_millis(),
                                    "Timeout detected! Publishing warning."
                                );

                                // Publish timeout warning
                                let msg = Empty {};
                                if let Err(e) = timeout_pub.publish(&msg) {
                                    warn!(error = %e, "Failed to publish timeout warning");
                                }
                            }
                        }
                        Err(e) => {
                            warn!(error = %e, "Watchdog timer error");
                        }
                    }
                }
            }
        }
    })?;

    // Main event loop: the key pattern for r2r
    // This combines ROS2 event processing with async task execution
    loop {
        // spin_once() processes ROS2 events (timer ticks, ensures messages
        // are sent over DDS, graph changes, etc.)
        node.spin_once(std::time::Duration::from_millis(100));

        // run_until_stalled() executes pending async tasks
        // (processes timer events, publishes messages, etc.)
        pool.run_until_stalled();
    }
}
