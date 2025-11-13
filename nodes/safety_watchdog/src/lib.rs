//! # Safety Watchdog Node
//!
//! Monitors heartbeat messages and publishes zero velocity commands on timeout.
//!
//! This is a skeleton implementation with structured logging infrastructure.
//! Full implementation tracked in GitHub issue #6.
//!
//! ## Logging Examples
//!
//! This module demonstrates proper use of structured logging and distributed
//! tracing spans for async operations.

use anyhow::Result;
use tracing::{debug, info, instrument, warn};

/// Configuration for the safety watchdog
#[derive(Debug, Clone)]
pub struct WatchdogConfig {
    /// Timeout duration in milliseconds
    pub timeout_ms: u64,
    /// Input topic to monitor
    pub input_topic: String,
    /// Output topic for safety commands
    pub output_topic: String,
}

impl Default for WatchdogConfig {
    fn default() -> Self {
        Self {
            timeout_ms: 1000,
            input_topic: "/heartbeat".to_string(),
            output_topic: "/cmd_vel".to_string(),
        }
    }
}

/// Safety watchdog node
pub struct SafetyWatchdog {
    config: WatchdogConfig,
}

impl SafetyWatchdog {
    /// Create a new safety watchdog with the given configuration
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use safety_watchdog::{SafetyWatchdog, WatchdogConfig};
    /// use tracing::info;
    ///
    /// let config = WatchdogConfig::default();
    /// let watchdog = SafetyWatchdog::new(config);
    /// info!(timeout_ms = watchdog.config().timeout_ms, "Watchdog created");
    /// ```
    #[instrument(skip(config), fields(timeout_ms = config.timeout_ms))]
    pub fn new(config: WatchdogConfig) -> Self {
        info!(
            timeout_ms = config.timeout_ms,
            input_topic = %config.input_topic,
            output_topic = %config.output_topic,
            "Creating safety watchdog"
        );
        Self { config }
    }

    /// Get the current configuration
    pub fn config(&self) -> &WatchdogConfig {
        &self.config
    }

    /// Run the watchdog node
    ///
    /// This is a placeholder for the actual implementation.
    ///
    /// # Examples of Structured Logging
    ///
    /// ```no_run
    /// use tracing::{info, warn, error, debug};
    ///
    /// // Info: Major state changes
    /// info!(node = "safety_watchdog", "Node started");
    ///
    /// // Warn: Recoverable issues
    /// warn!(timeout_ms = 1000, elapsed_ms = 1500, "Heartbeat timeout detected");
    ///
    /// // Error: Failures
    /// error!(error = %anyhow::anyhow!("test"), "Failed to publish safety command");
    ///
    /// // Debug: Detailed execution flow
    /// debug!(heartbeat_count = 42, "Heartbeat received");
    /// ```
    #[instrument(skip(self))]
    pub async fn run(&self) -> Result<()> {
        info!("Starting safety watchdog node");
        debug!(config = ?self.config, "Configuration loaded");

        // TODO: Implement watchdog logic
        // - Create ROS2 node
        // - Subscribe to heartbeat topic
        // - Set up timer for timeout detection
        // - Publish zero velocity on timeout

        info!("Safety watchdog node ready");
        warn!("Note: This is a placeholder. Full implementation tracked in GitHub issue #6");

        Ok(())
    }
}

/// Helper function to demonstrate span usage across async boundaries
///
/// Spans allow you to track operations across async tasks and see timing
/// information in distributed tracing systems.
#[instrument]
pub async fn example_async_operation() -> Result<()> {
    debug!("Starting async operation");

    // Simulate some work
    tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;

    debug!("Async operation completed");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = WatchdogConfig::default();
        assert_eq!(config.timeout_ms, 1000);
        assert_eq!(config.input_topic, "/heartbeat");
        assert_eq!(config.output_topic, "/cmd_vel");
    }

    #[test]
    fn test_watchdog_creation() {
        let config = WatchdogConfig::default();
        let watchdog = SafetyWatchdog::new(config.clone());
        assert_eq!(watchdog.config().timeout_ms, config.timeout_ms);
    }
}
