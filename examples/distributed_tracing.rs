//! # Distributed Tracing Example
//!
//! ## What you'll learn
//! - Using `#[instrument]` for automatic span creation
//! - Creating custom spans with structured fields
//! - Tracing operations across async boundaries
//! - Exporting traces to OpenTelemetry collectors
//! - Visualizing distributed traces
//!
//! ## Usage
//!
//! ### Run with console output (default):
//! ```bash
//! source scripts/dev_env.sh
//! RUST_LOG=debug cargo run --example distributed_tracing
//! ```
//!
//! ### Run with OpenTelemetry (Jaeger):
//! ```bash
//! # Start Jaeger
//! docker run -d --name jaeger \
//!   -p 4317:4317 \
//!   -p 16686:16686 \
//!   jaegertracing/all-in-one:latest
//!
//! # Run example with OTLP export
//! LOG_FORMAT=otlp OTLP_ENDPOINT=http://localhost:4317 \
//!   RUST_LOG=debug cargo run --example distributed_tracing
//!
//! # View traces at http://localhost:16686
//! # - Search for service: "rust-ros2-microstack"
//! # - Look for operation: "distributed_tracing_workflow"
//! ```
//!
//! ## Key Concepts
//!
//! This example demonstrates how spans allow you to:
//! 1. Track operations across async tasks
//! 2. Measure timing and performance
//! 3. Add contextual information to logs
//! 4. Debug complex async workflows
//! 5. Visualize execution flow in distributed systems

use anyhow::Result;
use r2r::{QosProfile, std_msgs::msg::String as StringMsg};
use rust_ros2_microstack::logging;
use std::time::Duration;
use tracing::{Instrument, debug, info, info_span, instrument, warn};

/// Main workflow - automatically creates a span named "distributed_tracing_workflow"
#[instrument]
async fn distributed_tracing_workflow() -> Result<()> {
    info!("Starting distributed tracing workflow");

    // Simulate a complex workflow with multiple steps
    let sensor_data = collect_sensor_data().await?;
    let processed = process_data(sensor_data).await?;
    publish_results(processed).await?;

    info!("Workflow completed successfully");
    Ok(())
}

/// Collect sensor data - demonstrates automatic span creation with fields
#[instrument(fields(sensor_count = 3))]
async fn collect_sensor_data() -> Result<Vec<f32>> {
    info!("Collecting sensor data");

    // Simulate reading from multiple sensors
    let mut data = Vec::new();

    for sensor_id in 1..=3 {
        let reading = read_sensor(sensor_id).await?;
        data.push(reading);
    }

    info!(samples = data.len(), "Sensor data collected");
    Ok(data)
}

/// Read from a single sensor - demonstrates nested spans
#[instrument]
async fn read_sensor(sensor_id: u32) -> Result<f32> {
    debug!(sensor_id = sensor_id, "Reading sensor");

    // Simulate sensor read delay
    tokio::time::sleep(Duration::from_millis(50)).await;

    // Simulate sensor value
    let value = 10.0 + (sensor_id as f32) * 2.5;

    debug!(sensor_id = sensor_id, value = value, "Sensor read complete");
    Ok(value)
}

/// Process data - demonstrates spans with custom fields
#[instrument(skip(data), fields(input_size = data.len()))]
async fn process_data(data: Vec<f32>) -> Result<Vec<f32>> {
    info!("Processing sensor data");

    // Create a custom span for the filtering operation
    let filtered = async {
        debug!("Filtering data");

        let filtered: Vec<f32> = data
            .iter()
            .filter(|&&x| {
                let valid = x > 5.0 && x < 50.0;
                if !valid {
                    warn!(value = x, "Value out of range, filtering out");
                }
                valid
            })
            .copied()
            .collect();

        info!(
            input_count = data.len(),
            output_count = filtered.len(),
            "Filtering complete"
        );

        filtered
    }
    .instrument(info_span!(
        "filter_data",
        threshold_min = 5.0,
        threshold_max = 50.0
    ))
    .await;

    // Create a custom span for normalization
    let normalized = async {
        debug!("Normalizing data");

        let max = filtered.iter().copied().fold(f32::MIN, f32::max);
        let normalized: Vec<f32> = filtered.iter().map(|&x| x / max).collect();

        info!(max_value = max, "Normalization complete");
        normalized
    }
    .instrument(info_span!("normalize_data"))
    .await;

    info!(output_size = normalized.len(), "Processing complete");
    Ok(normalized)
}

/// Publish results - demonstrates ROS2 integration with tracing
#[instrument(skip(data), fields(data_points = data.len()))]
async fn publish_results(data: Vec<f32>) -> Result<()> {
    info!("Publishing results");

    // Create ROS2 context and node within a span
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "tracing_publisher", "")?;

    let qos = QosProfile::default();
    let publisher = node.create_publisher::<StringMsg>("/processed_data", qos)?;

    info!(topic = "/processed_data", "Publisher created");

    // Publish each data point in its own span
    for (idx, &value) in data.iter().enumerate() {
        async {
            let msg = StringMsg {
                data: format!("Sample {}: {:.3}", idx, value),
            };

            debug!(index = idx, value = value, "Publishing data point");

            if let Err(e) = publisher.publish(&msg) {
                warn!(error = %e, index = idx, "Failed to publish");
            } else {
                debug!(index = idx, "Published successfully");
            }

            // Simulate some processing time
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        .instrument(
            info_span!("publish_sample", sample_index = idx, value = %format!("{:.3}", value)),
        )
        .await;
    }

    info!(published_count = data.len(), "All results published");
    Ok(())
}

/// Demonstrate manual span creation and entry
async fn manual_span_example() {
    // Create a span manually
    let span = info_span!("manual_operation", operation_id = 42, user = "example");

    // Enter the span
    let _enter = span.enter();

    info!("Inside manual span");
    tokio::time::sleep(Duration::from_millis(100)).await;
    info!("Manual span complete");

    // Span exits when _enter is dropped
}

/// Demonstrate concurrent operations with spans
#[instrument]
async fn concurrent_operations_example() -> Result<()> {
    info!("Starting concurrent operations");

    // Use tokio::join! to run operations concurrently
    // Each operation gets its own span
    let (result1, result2, result3) = tokio::join!(
        async {
            info!("Task 1 starting");
            tokio::time::sleep(Duration::from_millis(100)).await;
            info!("Task 1 complete");
            "result1"
        }
        .instrument(info_span!("concurrent_task", task_id = 1)),
        async {
            info!("Task 2 starting");
            tokio::time::sleep(Duration::from_millis(150)).await;
            info!("Task 2 complete");
            "result2"
        }
        .instrument(info_span!("concurrent_task", task_id = 2)),
        async {
            info!("Task 3 starting");
            tokio::time::sleep(Duration::from_millis(80)).await;
            info!("Task 3 complete");
            "result3"
        }
        .instrument(info_span!("concurrent_task", task_id = 3)),
    );

    info!(
        results = ?[result1, result2, result3],
        "All concurrent operations complete"
    );

    Ok(())
}

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize logging (supports OTLP export for distributed tracing)
    logging::init();

    info!("=== Distributed Tracing Example ===");
    info!("This example demonstrates span usage for distributed tracing");

    // Run the main workflow with instrumentation
    async {
        // Run main workflow
        if let Err(e) = distributed_tracing_workflow().await {
            warn!(error = %e, "Workflow failed");
        }

        // Wait a bit
        tokio::time::sleep(Duration::from_millis(500)).await;

        // Demonstrate manual spans
        manual_span_example().await;

        // Wait a bit more
        tokio::time::sleep(Duration::from_millis(500)).await;

        // Demonstrate concurrent operations
        if let Err(e) = concurrent_operations_example().await {
            warn!(error = %e, "Concurrent operations failed");
        }

        info!("=== Example Complete ===");
        info!("Check your tracing backend (e.g., Jaeger) to visualize the spans");
    }
    .instrument(info_span!("main"))
    .await;

    // Shutdown tracing (flushes spans to collector)
    if let Err(e) = logging::shutdown() {
        warn!(error = %e, "Failed to shutdown logging cleanly");
    }

    Ok(())
}
