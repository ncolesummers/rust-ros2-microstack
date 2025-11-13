//! Structured logging with OpenTelemetry support
//!
//! This module provides a unified logging initialization system with support for:
//! - Multiple output formats (pretty console, JSON, OTLP)
//! - Environment-based configuration
//! - OpenTelemetry distributed tracing
//! - RUST_LOG filtering
//!
//! # Quick Start
//!
//! ```no_run
//! use rust_ros2_microstack::logging;
//!
//! logging::init();
//! tracing::info!("Application started");
//! ```
//!
//! # Configuration
//!
//! Environment variables:
//! - `RUST_LOG`: Filter directives (e.g., `info`, `debug`, `my_crate=trace`)
//! - `LOG_FORMAT`: Output format (`pretty`, `json`, `otlp`) - defaults to `pretty`
//! - `OTLP_ENDPOINT`: OpenTelemetry collector endpoint (e.g., `http://localhost:4317`)
//!
//! # Examples
//!
//! ```bash
//! # Pretty console output (default)
//! cargo run --example hello_publisher
//!
//! # JSON output for production
//! LOG_FORMAT=json cargo run --example hello_publisher
//!
//! # Send traces to OpenTelemetry collector
//! LOG_FORMAT=otlp OTLP_ENDPOINT=http://localhost:4317 cargo run --example hello_publisher
//!
//! # Debug logging
//! RUST_LOG=debug cargo run --example hello_publisher
//! ```

use anyhow::{Context, Result};
use opentelemetry::trace::TracerProvider as _;
use opentelemetry_otlp::WithExportConfig;
use opentelemetry_sdk::Resource;
use opentelemetry_sdk::trace::{RandomIdGenerator, Sampler};
use std::env;
use tracing::Level;
use tracing_subscriber::layer::SubscriberExt;
use tracing_subscriber::util::SubscriberInitExt;
use tracing_subscriber::{EnvFilter, Layer, fmt};

/// Output format for log messages
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LogFormat {
    /// Human-readable pretty console output (default for development)
    Pretty,
    /// JSON structured output (recommended for production)
    Json,
    /// OpenTelemetry Protocol - sends traces to collector
    Otlp,
}

impl LogFormat {
    /// Parse log format from environment variable
    pub fn from_env() -> Self {
        match env::var("LOG_FORMAT")
            .unwrap_or_default()
            .to_lowercase()
            .as_str()
        {
            "json" => LogFormat::Json,
            "otlp" => LogFormat::Otlp,
            _ => LogFormat::Pretty,
        }
    }
}

/// Configuration for the logging system
#[derive(Debug, Clone)]
pub struct LoggingConfig {
    /// Output format
    pub format: LogFormat,
    /// Service name for distributed tracing
    pub service_name: String,
    /// OpenTelemetry endpoint (only used with OTLP format)
    pub otlp_endpoint: Option<String>,
    /// Default log level when RUST_LOG is not set
    pub default_level: Level,
}

impl Default for LoggingConfig {
    fn default() -> Self {
        Self {
            format: LogFormat::from_env(),
            service_name: env::var("SERVICE_NAME")
                .unwrap_or_else(|_| "rust-ros2-microstack".to_string()),
            otlp_endpoint: env::var("OTLP_ENDPOINT").ok(),
            default_level: Level::INFO,
        }
    }
}

impl LoggingConfig {
    /// Create a new configuration with custom service name
    pub fn new(service_name: impl Into<String>) -> Self {
        Self {
            service_name: service_name.into(),
            ..Default::default()
        }
    }

    /// Set the output format
    pub fn with_format(mut self, format: LogFormat) -> Self {
        self.format = format;
        self
    }

    /// Set the OpenTelemetry endpoint
    pub fn with_otlp_endpoint(mut self, endpoint: impl Into<String>) -> Self {
        self.otlp_endpoint = Some(endpoint.into());
        self
    }

    /// Set the default log level
    pub fn with_default_level(mut self, level: Level) -> Self {
        self.default_level = level;
        self
    }
}

/// Initialize logging with default configuration from environment
///
/// This is the simplest way to set up logging. It reads configuration from
/// environment variables:
/// - `RUST_LOG` for filtering
/// - `LOG_FORMAT` for output format
/// - `OTLP_ENDPOINT` for telemetry endpoint
///
/// # Panics
///
/// Panics if the subscriber cannot be initialized (e.g., called multiple times)
///
/// # Example
///
/// ```no_run
/// use rust_ros2_microstack::logging;
///
/// logging::init();
/// tracing::info!("Ready to go!");
/// ```
pub fn init() {
    init_with_config(LoggingConfig::default()).expect("Failed to initialize logging");
}

/// Initialize logging with custom configuration
///
/// Use this for more control over logging behavior.
///
/// # Errors
///
/// Returns an error if:
/// - The OpenTelemetry exporter cannot be configured
/// - The tracing subscriber cannot be set as global default
///
/// # Example
///
/// ```no_run
/// use rust_ros2_microstack::logging::{LoggingConfig, LogFormat};
/// use tracing::Level;
///
/// let config = LoggingConfig::new("my-node")
///     .with_format(LogFormat::Json)
///     .with_default_level(Level::DEBUG);
///
/// rust_ros2_microstack::logging::init_with_config(config)?;
/// tracing::info!("Custom logging initialized");
/// # Ok::<(), anyhow::Error>(())
/// ```
pub fn init_with_config(config: LoggingConfig) -> Result<()> {
    // Create env filter with fallback to default level
    // When RUST_LOG is not set, use the default level for all targets
    let env_filter = EnvFilter::try_from_default_env()
        .unwrap_or_else(|_| EnvFilter::new(config.default_level.to_string()));

    match config.format {
        LogFormat::Pretty => {
            // Pretty console output for development
            tracing_subscriber::registry()
                .with(env_filter)
                .with(
                    fmt::layer()
                        .pretty()
                        .with_target(true)
                        .with_thread_ids(false)
                        .with_thread_names(false),
                )
                .init();
        }
        LogFormat::Json => {
            // JSON output for production/log aggregation
            tracing_subscriber::registry()
                .with(env_filter)
                .with(fmt::layer().json())
                .init();
        }
        LogFormat::Otlp => {
            // OpenTelemetry for distributed tracing
            let endpoint = config
                .otlp_endpoint
                .as_ref()
                .context("OTLP_ENDPOINT must be set when using LOG_FORMAT=otlp")?;

            // Configure OpenTelemetry tracer pipeline
            // Using install_simple() for immediate export (better for examples/debugging)
            // For production, consider install_batch() for better performance
            let tracer_provider = opentelemetry_otlp::new_pipeline()
                .tracing()
                .with_exporter(
                    opentelemetry_otlp::new_exporter()
                        .tonic()
                        .with_endpoint(endpoint),
                )
                .with_trace_config(
                    opentelemetry_sdk::trace::Config::default()
                        .with_resource(Resource::new(vec![opentelemetry::KeyValue::new(
                            "service.name",
                            config.service_name.clone(),
                        )]))
                        .with_id_generator(RandomIdGenerator::default())
                        .with_sampler(Sampler::AlwaysOn),
                )
                .install_simple()
                .context("Failed to initialize OTLP tracer")?;

            // Get a tracer from the provider
            let tracer = tracer_provider.tracer("rust-ros2-microstack");

            // Set up tracing subscriber with OpenTelemetry layer
            let telemetry = tracing_opentelemetry::layer().with_tracer(tracer);

            tracing_subscriber::registry()
                .with(env_filter)
                .with(telemetry)
                .with(
                    fmt::layer()
                        .with_target(true)
                        .with_filter(tracing_subscriber::filter::LevelFilter::INFO),
                )
                .init();

            // Set the provider globally so shutdown() can flush traces
            opentelemetry::global::set_tracer_provider(tracer_provider);
        }
    }

    Ok(())
}

/// Shutdown the global tracer provider
///
/// Call this before application exit to ensure all spans are flushed.
/// Only necessary when using OTLP format.
///
/// # Example
///
/// ```no_run
/// use rust_ros2_microstack::logging;
///
/// logging::init();
///
/// // ... application logic ...
///
/// logging::shutdown()?;
/// # Ok::<(), anyhow::Error>(())
/// ```
pub fn shutdown() -> Result<()> {
    opentelemetry::global::shutdown_tracer_provider();
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_log_format_from_env() {
        // Test default
        unsafe {
            env::remove_var("LOG_FORMAT");
        }
        assert_eq!(LogFormat::from_env(), LogFormat::Pretty);

        // Test json
        unsafe {
            env::set_var("LOG_FORMAT", "json");
        }
        assert_eq!(LogFormat::from_env(), LogFormat::Json);

        // Test OTLP
        unsafe {
            env::set_var("LOG_FORMAT", "otlp");
        }
        assert_eq!(LogFormat::from_env(), LogFormat::Otlp);

        // Test case insensitivity
        unsafe {
            env::set_var("LOG_FORMAT", "JSON");
        }
        assert_eq!(LogFormat::from_env(), LogFormat::Json);

        // Cleanup
        unsafe {
            env::remove_var("LOG_FORMAT");
        }
    }

    #[test]
    fn test_config_builder() {
        let config = LoggingConfig::new("test-service")
            .with_format(LogFormat::Json)
            .with_default_level(Level::DEBUG)
            .with_otlp_endpoint("http://localhost:4317");

        assert_eq!(config.service_name, "test-service");
        assert_eq!(config.format, LogFormat::Json);
        assert_eq!(config.default_level, Level::DEBUG);
        assert_eq!(
            config.otlp_endpoint,
            Some("http://localhost:4317".to_string())
        );
    }
}
