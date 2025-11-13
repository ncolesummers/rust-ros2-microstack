//! # Hello Publisher
//!
//! ## What you'll learn
//! - Creating a ROS2 node with r2r
//! - Publishing messages to topics
//! - Configuring QoS profiles
//! - Using r2r wall timers for periodic tasks
//! - The r2r event loop pattern (spin_once + async executor)
//!
//! ## Usage
//! ```bash
//! source scripts/dev_env.sh
//! cargo run --example hello_publisher
//! ```
//!
//! In another terminal, run the subscriber:
//! ```bash
//! cargo run --example hello_subscriber
//! ```

use anyhow::Result;
use futures::{executor::LocalPool, task::LocalSpawnExt};
use r2r::{QosProfile, std_msgs::msg::String as StringMsg};
use tracing::{info, warn};

fn main() -> Result<()> {
    // Initialize tracing for structured logging
    tracing_subscriber::fmt::init();

    info!("Starting hello_publisher example");

    // Create ROS2 context - this is the entry point for r2r
    let ctx = r2r::Context::create()?;

    // Create a node - the fundamental unit in ROS2
    let mut node = r2r::Node::create(ctx, "hello_publisher", "")?;

    // Configure QoS (Quality of Service) profile
    // - RELIABLE: Guarantees message delivery (vs BEST_EFFORT)
    // - keep_last(10): Keep last 10 messages in queue
    let qos = QosProfile::default().reliable().keep_last(10);

    // Create a publisher for String messages on the "chatter" topic
    // The publisher is typed - it can only send StringMsg
    let publisher = node.create_publisher::<StringMsg>("/chatter", qos)?;

    // Create a wall timer for periodic publishing (1 second)
    // r2r provides timers that integrate with the event loop
    let mut timer = node.create_wall_timer(std::time::Duration::from_secs(1))?;

    info!("Publisher created on topic: /chatter");
    info!("Publishing messages every 1 second. Press Ctrl+C to stop.");

    // Set up the async task executor
    // r2r uses futures::executor::LocalPool for lightweight async task execution
    let mut pool = LocalPool::new();
    let spawner = pool.spawner();

    // Spawn a task to publish messages on timer events
    spawner.spawn_local(async move {
        let mut count = 0u32;
        loop {
            // Wait for the timer to tick
            match timer.tick().await {
                Ok(_) => {
                    // Create a new message
                    let msg = StringMsg {
                        data: format!("Hello, ROS2! Message #{}", count),
                    };

                    // Publish the message
                    // In r2r, publishing is synchronous (no await needed)
                    match publisher.publish(&msg) {
                        Ok(_) => {
                            info!(count = count, message = %msg.data, "Published message");
                        }
                        Err(e) => {
                            warn!(error = %e, "Failed to publish message");
                        }
                    }

                    count += 1;
                }
                Err(e) => {
                    warn!(error = %e, "Timer error");
                }
            }
        }
    })?;

    // Main event loop: the key pattern for r2r
    // This combines ROS2 event processing with async task execution
    loop {
        // spin_once() processes ROS2 events (timer ticks, graph changes,
        // ensures messages are sent over DDS, etc.)
        node.spin_once(std::time::Duration::from_millis(100));

        // run_until_stalled() executes pending async tasks
        // (processes timer ticks, publishes messages, etc.)
        pool.run_until_stalled();
    }
}
