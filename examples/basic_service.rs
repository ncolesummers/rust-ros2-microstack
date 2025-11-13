//! # Basic Service
//!
//! ## What you'll learn
//! - Creating service servers
//! - Calling services as a client
//! - Request/response communication
//! - Running server and client in one binary
//!
//! ## Usage
//! ```bash
//! source scripts/dev_env.sh
//! cargo run --example basic_service
//! ```
//!
//! This example demonstrates ROS2 services by running both a service
//! server (that adds two integers) and a client (that calls the service)
//! in a single binary.

use anyhow::Result;
use futures::{executor::LocalPool, stream::StreamExt, task::LocalSpawnExt};
use r2r::QosProfile;
use r2r::example_interfaces::srv::AddTwoInts;
use tracing::{info, warn};
use tracing_subscriber;

fn main() -> Result<()> {
    // Initialize tracing for structured logging
    tracing_subscriber::fmt::init();

    info!("Starting basic_service example");

    // Create ROS2 context and nodes
    // We create two separate nodes: one for the server, one for the client
    let ctx = r2r::Context::create()?;
    let mut server_node = r2r::Node::create(ctx.clone(), "add_two_ints_server", "")?;
    let mut client_node = r2r::Node::create(ctx, "add_two_ints_client", "")?;

    // Create QoS profile for services (uses default service QoS)
    let qos = QosProfile::services_default();

    // Create a service server
    // The server will respond to AddTwoInts requests
    let mut service =
        server_node.create_service::<AddTwoInts::Service>("/add_two_ints", qos.clone())?;

    info!("Service server created on: /add_two_ints");

    // Create a service client
    // The client will make requests to the AddTwoInts service
    let client = client_node.create_client::<AddTwoInts::Service>("/add_two_ints", qos)?;

    info!("Service client created");

    info!("Starting demonstration.");

    // Set up the async task executor
    // r2r uses futures::executor::LocalPool for lightweight async task execution
    let mut pool = LocalPool::new();
    let spawner = pool.spawner();

    // Spawn a task to handle service requests
    // This runs concurrently with the client calls
    spawner.spawn_local(async move {
        loop {
            // Wait for incoming service requests
            match service.next().await {
                Some(service_request) => {
                    // Extract the request message
                    let request = &service_request.message;

                    // Process the request: add two integers
                    let result = request.a + request.b;

                    info!(
                        a = request.a,
                        b = request.b,
                        result = result,
                        "Processing service request"
                    );

                    // Send the response back to the client
                    let response = AddTwoInts::Response { sum: result };
                    if let Err(e) = service_request.respond(response) {
                        warn!(error = %e, "Failed to send response");
                    }
                }
                None => break,
            }
        }
    })?;

    // Spawn a task to make client requests
    spawner.spawn_local(async move {
        // Give the service a moment to initialize
        // Use a simple counter-based delay since we're in async context
        for _ in 0..10 {
            // This will be processed during pool.run_until_stalled()
            futures::future::ready(()).await;
        }

        // Make several client requests to demonstrate the service
        for i in 0..5 {
            // Prepare a request
            let request = AddTwoInts::Request { a: i, b: i * 2 };

            info!(a = request.a, b = request.b, "Sending service request");

            // Call the service and await the response
            // client.request() returns a Result<Future>, so we need to unwrap then await
            match client.request(&request) {
                Ok(future) => match future.await {
                    Ok(response) => {
                        info!(
                            a = request.a,
                            b = request.b,
                            sum = response.sum,
                            "Received service response"
                        );
                    }
                    Err(e) => {
                        warn!(error = %e, "Service call failed");
                    }
                },
                Err(e) => {
                    warn!(error = %e, "Failed to create service request");
                }
            }

            // Wait between requests using a simple delay loop
            // In a real application, you might use a timer
            for _ in 0..1000 {
                futures::future::ready(()).await;
            }
        }

        info!("Client finished making requests");
    })?;

    // Main event loop: the key pattern for r2r
    // This combines ROS2 event processing with async task execution
    // We need to spin BOTH nodes to process their respective ROS2 events
    loop {
        // spin_once() on both nodes to process ROS2 events
        // (service requests, service responses, graph changes, etc.)
        server_node.spin_once(std::time::Duration::from_millis(100));
        client_node.spin_once(std::time::Duration::from_millis(100));

        // run_until_stalled() executes pending async tasks
        // (processes service requests, sends client requests, etc.)
        pool.run_until_stalled();
    }
}
