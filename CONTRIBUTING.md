# Contributing to Rust ROS2 Microstack

Thank you for your interest in contributing! This document provides guidelines for contributing code, documentation, and examples to this learning-focused portfolio project.

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [Getting Started](#getting-started)
- [Code Style Guidelines](#code-style-guidelines)
- [Adding New Nodes](#adding-new-nodes)
- [Testing Requirements](#testing-requirements)
- [Documentation Standards](#documentation-standards)
- [Pull Request Process](#pull-request-process)

---

## Code of Conduct

This project aims to be welcoming, inclusive, and educational. We expect all contributors to:
- Be respectful and constructive in discussions
- Focus on learning and teaching others
- Provide clear explanations for design choices
- Help newcomers understand Rust + ROS2 patterns

---

## Getting Started

### Prerequisites

1. Complete the setup in [docs/setup.md](docs/setup.md)
2. Verify you can build and test:
   ```bash
   source scripts/dev_env.sh
   cargo build --workspace
   cargo test
   ```
3. Read the architecture docs: [docs/architecture.md](docs/architecture.md)

### Finding Work

- Check [open issues](../../issues) for tasks labeled `good-first-issue` or `help-wanted`
- Review the [backlog.txt](backlog.txt) for planned features
- Propose new ideas by opening an issue first for discussion

---

## Code Style Guidelines

### Rust Formatting

**Always** run `cargo fmt` before committing:

```bash
cargo fmt --all
```

**Check formatting** in CI:
```bash
cargo fmt --all -- --check
```

### Linting

**Fix all clippy warnings** before submitting:

```bash
cargo clippy --workspace --all-targets -- -D warnings
```

**Exception**: If a clippy lint is incorrect, use `#[allow(clippy::lint_name)]` with a comment explaining why.

### Naming Conventions

- **Crates**: `snake_case` (e.g., `safety_watchdog`, `sensor_filter`)
- **Types**: `PascalCase` (e.g., `WatchdogNode`, `FilterConfig`)
- **Functions**: `snake_case` (e.g., `create_node`, `clamp_scan`)
- **Constants**: `SCREAMING_SNAKE_CASE` (e.g., `DEFAULT_TIMEOUT_MS`)

### Error Handling

**Use `anyhow::Result` for application code**:
```rust
use anyhow::{Context, Result};

fn run() -> Result<()> {
    let node = create_node().context("Failed to create node")?;
    // ...
    Ok(())
}
```

**Never use `unwrap()` or `expect()` in production code** (examples are OK).

### Logging

**Use `tracing` not `println!`**:

```rust
use tracing::{info, warn, error, debug};

info!("Node started");
debug!(topic = %topic_name, "Subscribed to topic");
warn!(timeout_ms = %timeout, "Heartbeat timeout");
error!(error = %e, "Failed to publish");
```

**Log Levels**:
- `error`: Unrecoverable failures
- `warn`: Recoverable issues (e.g., timeout detected)
- `info`: Major state changes (e.g., node started)
- `debug`: Detailed execution flow
- `trace`: Very verbose (rarely needed)

---

## Adding New Nodes

### When to Add a Node

Nodes should demonstrate a distinct ROS2 pattern or solve a real robotics problem. Before adding:

1. **Check for existing nodes** that could be extended
2. **Open an issue** describing the node's purpose and learning objectives
3. **Get feedback** from maintainers on design

### Node Structure Decision

**Binary crate (`apps/`)** if:
- Meant to be run directly by end users
- Has a CLI interface
- Example: `teleop_mux`

**Library crate (`nodes/`)** if:
- Reusable logic that can be tested independently
- May be used by multiple binaries
- Example: `safety_watchdog`, `sensor_filter`

**Uncertain?** Default to library crate (can always add a thin binary wrapper later).

### Step-by-Step: Adding a New Node

1. **Create the crate**:
   ```bash
   # For library node
   cargo new --lib nodes/my_node

   # For binary app
   cargo new apps/my_app
   ```

2. **Update workspace `Cargo.toml`**:
   ```toml
   [workspace]
   members = [
       # ... existing members
       "nodes/my_node",
   ]
   ```

3. **Update node's `Cargo.toml`**:
   ```toml
   [package]
   name = "my_node"
   version = "0.1.0"
   edition = "2024"

   [dependencies]
   r2r = { workspace = true }
   tokio = { workspace = true }
   tracing = { workspace = true }
   tracing-subscriber = { workspace = true }
   anyhow = { workspace = true }
   clap = { workspace = true, features = ["derive"] }
   ```

4. **Add module-level documentation**:
   ```rust
   //! # My Node
   //!
   //! Brief description of what this node does.
   //!
   //! ## Learning Objectives
   //! - Pattern 1
   //! - Pattern 2
   //!
   //! ## Usage
   //! ```bash
   //! cargo run -p my_node -- --help
   //! ```
   ```

5. **Implement with clear comments**:
   ```rust
   /// Creates the node and configures subscriptions/publishers.
   ///
   /// # Arguments
   /// - `ctx`: r2r context
   /// - `config`: Node configuration
   ///
   /// # Returns
   /// Configured node or error
   pub fn create_node(ctx: r2r::Context, config: Config) -> anyhow::Result<r2r::Node> {
       // Implementation with inline comments explaining r2r patterns
   }
   ```

6. **Add unit tests**:
   ```rust
   #[cfg(test)]
   mod tests {
       use super::*;

       #[test]
       fn test_node_creation() {
           // Test logic
       }
   }
   ```

7. **Update `docs/nodes.md`** with implementation deep dive

8. **Update README's topic table** with new subscriptions/publications

9. **Add to backlog** or create GitHub issue for integration tests

### Required for Node Approval

- [ ] Module-level documentation with learning objectives
- [ ] Inline doc comments explaining r2r patterns
- [ ] Unit tests covering core logic
- [ ] CLI interface with `--help` text
- [ ] Updated `docs/nodes.md`
- [ ] Updated README topic table
- [ ] Passes `cargo fmt` and `cargo clippy`
- [ ] Integration test or manual test plan

---

## Testing Requirements

### Unit Tests

**Location**: `src/lib.rs` or `src/tests/` in each crate

**Coverage**: Core logic, message transformations, configuration parsing

**Example**:
```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_clamp_ranges() {
        let result = clamp(5.0, 0.0, 10.0);
        assert_eq!(result, 5.0);
    }

    #[test]
    fn test_clamp_min() {
        let result = clamp(-1.0, 0.0, 10.0);
        assert_eq!(result, 0.0);
    }
}
```

**Run**: `cargo test`

### Integration Tests

**Location**: `tests/integration/`

**Purpose**: Test actual ROS2 message flow between nodes

**Example**:
```rust
// tests/integration/test_my_node.rs
use r2r::{Context, Node};
use tokio::time::{timeout, Duration};

#[tokio::test]
async fn test_my_node_publishes() {
    let ctx = Context::create().unwrap();
    let mut node = Node::create(ctx, "test_node", "").unwrap();

    // Test logic with actual r2r pub/sub
    // ...
}
```

**Run**: `cargo test --test '*'`

**Note**: Integration tests may require ROS2 environment (run after `source scripts/dev_env.sh`).

### Benchmarks

**Location**: `benches/` in crate root

**When required**: For performance-critical operations (e.g., sensor processing)

**Example**:
```rust
// benches/my_benchmark.rs
use criterion::{black_box, criterion_group, criterion_main, Criterion};

fn benchmark_operation(c: &mut Criterion) {
    c.bench_function("operation", |b| {
        b.iter(|| my_operation(black_box(input)))
    });
}

criterion_group!(benches, benchmark_operation);
criterion_main!(benches);
```

**Run**: `cargo bench`

---

## Documentation Standards

### Code Documentation

**All public items must have doc comments**:

```rust
/// Represents a safety watchdog node.
///
/// Monitors heartbeat messages and publishes zero-velocity commands
/// when heartbeats stop arriving within the configured timeout.
///
/// # Examples
/// ```no_run
/// let node = WatchdogNode::new(config)?;
/// node.run().await?;
/// ```
pub struct WatchdogNode {
    // ...
}
```

**Explain "why" not just "what"**:
```rust
// ❌ Bad: States the obvious
/// Sets the timeout.
pub fn set_timeout(&mut self, timeout: Duration) { ... }

// ✅ Good: Explains rationale
/// Sets the timeout for heartbeat monitoring.
///
/// After this duration without heartbeats, the node publishes
/// zero-velocity commands to safely stop the robot.
pub fn set_timeout(&mut self, timeout: Duration) { ... }
```

### Learning-Focused Comments

**Explain r2r patterns for learners**:

```rust
// Create a subscriber with best-effort QoS.
// Best-effort is used here because heartbeats are high-rate;
// missing one is acceptable since the timeout will trigger anyway.
let subscriber = node.create_subscription::<Empty>(
    "/heartbeat",
    QosProfile::best_effort()
)?;
```

### Architecture Documentation

**When adding significant features**, update:
- [docs/architecture.md](docs/architecture.md) - Design decisions
- [docs/nodes.md](docs/nodes.md) - Implementation patterns
- [README.md](README.md) - User-facing overview

### Examples Documentation

**Standalone examples** should:
- Be < 100 lines of code
- Have extensive inline comments
- Include a "What you'll learn" comment at the top
- Be runnable with `cargo run --example <name>`

---

## Pull Request Process

### Before Submitting

1. **Branch from `main`**:
   ```bash
   git checkout -b feature/my-feature
   ```

2. **Write clear commit messages**:
   ```
   Add sensor_filter node with LaserScan clamping

   - Implements configurable min/max range clamping
   - Adds benchmarks showing <1ms processing time
   - Updates docs/nodes.md with implementation details

   Closes #42
   ```

3. **Run quality checks**:
   ```bash
   cargo fmt --all
   cargo clippy --workspace --all-targets -- -D warnings
   cargo test --workspace
   cargo doc --no-deps --workspace
   ```

4. **Update documentation** (if applicable):
   - [ ] README.md
   - [ ] docs/nodes.md
   - [ ] Inline doc comments
   - [ ] backlog.txt or GitHub issues

### Submitting

1. **Push your branch**:
   ```bash
   git push origin feature/my-feature
   ```

2. **Open a Pull Request** with:
   - **Clear title**: "Add sensor_filter node"
   - **Description**: What changes, why, learning objectives
   - **Linked issues**: "Closes #42"
   - **Checklist**: Use the PR template (see `.github/PULL_REQUEST_TEMPLATE.md`)

3. **Respond to feedback** promptly and respectfully

### PR Checklist (Auto-populated from template)

- [ ] Code follows style guidelines (`cargo fmt`, `cargo clippy`)
- [ ] Unit tests added for new logic
- [ ] Integration tests added (if applicable)
- [ ] Documentation updated (README, docs/, inline comments)
- [ ] Learning objectives clearly explained
- [ ] All CI checks pass
- [ ] Manual testing performed and described

### Review Process

1. **Automated checks** run (CI: build, fmt, clippy)
2. **Maintainer review** (usually within 3 days)
3. **Requested changes** (if any)
4. **Approval** → **Merge**

---

## Questions?

- **General questions**: Open a [GitHub Discussion](../../discussions)
- **Bug reports**: Open an [Issue](../../issues)
- **Feature proposals**: Open an [Issue](../../issues) for discussion first

---

Thank you for contributing to Rust ROS2 learning! Your work helps others learn ROS2 in Rust. 🦀🤖
