//! # Hello Subscriber
//!
//! ## What you'll learn
//! - Subscribing to ROS2 topics
//! - Async message handling with r2r
//! - Processing incoming messages
//! - The r2r event loop pattern (spin_once + async executor)
//!
//! ## Usage
//! First, start the publisher in one terminal:
//! ```bash
//! source scripts/dev_env.sh
//! cargo run --example hello_publisher
//! ```
//!
//! Then run this subscriber in another terminal:
//! ```bash
//! cargo run --example hello_subscriber
//! ```

use anyhow::Result;
use futures::{executor::LocalPool, future, stream::StreamExt, task::LocalSpawnExt};
use r2r::{QosProfile, std_msgs::msg::String as StringMsg};
use tracing::info;
use tracing_subscriber;

fn main() -> Result<()> {
    // Initialize tracing for structured logging
    tracing_subscriber::fmt::init();

    info!("Starting hello_subscriber example");

    // Create ROS2 context and node
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "hello_subscriber", "")?;

    // Configure QoS to match the publisher
    // Using RELIABLE ensures we don't miss messages
    let qos = QosProfile::default().reliable().keep_last(10);

    // Create a subscriber for String messages on the "chatter" topic
    // This returns a stream of messages that we can iterate over
    let subscriber = node.subscribe::<StringMsg>("/chatter", qos)?;

    info!("Subscriber created on topic: /chatter");
    info!("Waiting for messages. Press Ctrl+C to stop.");

    // Set up the async task executor
    // r2r uses futures::executor::LocalPool for lightweight async task execution
    let mut pool = LocalPool::new();
    let spawner = pool.spawner();

    // Spawn a task to process incoming messages
    spawner.spawn_local(async move {
        subscriber
            .for_each(|message| {
                // Successfully received a message
                info!(
                    data = %message.data,
                    "Received message"
                );
                future::ready(())
            })
            .await
    })?;

    // Main event loop: the key pattern for r2r
    // This combines ROS2 event processing with async task execution
    loop {
        // spin_once() processes ROS2 events (receives messages from DDS,
        // pushes them into subscriber streams, etc.)
        node.spin_once(std::time::Duration::from_millis(100));

        // run_until_stalled() executes pending async tasks
        // (processes messages from subscriber streams, runs timers, etc.)
        pool.run_until_stalled();
    }
}
