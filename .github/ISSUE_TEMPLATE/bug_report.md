---
name: Bug Report
about: Report a bug or unexpected behavior
title: '[BUG] '
labels: 'bug'
assignees: ''
---

## Bug Description

A clear and concise description of what the bug is.

## Steps to Reproduce

1. Source ROS2 environment: `source scripts/dev_env.sh`
2. Run command: `cargo run -p ...`
3. Observe error: ...

## Expected Behavior

What you expected to happen.

## Actual Behavior

What actually happened. Include error messages if applicable.

```
Paste error messages here
```

## Environment

- **OS**: (e.g., Ubuntu 24.04)
- **ROS2 Distro**: (run `echo $ROS_DISTRO`)
- **Rust Version**: (run `rustc --version`)
- **r2r Version**: (check Cargo.toml)

## Additional Context

Add any other context about the problem here. Screenshots, logs, or related issues.

## Checklist

- [ ] I have sourced the ROS2 environment (`source scripts/dev_env.sh`)
- [ ] I have run `cargo build` successfully before encountering this bug
- [ ] I have checked existing issues for duplicates
- [ ] I have included error messages and logs
