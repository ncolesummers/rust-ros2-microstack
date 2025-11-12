# Pull Request

## Description

<!-- Provide a clear description of what this PR does -->

## Type of Change

<!-- Check all that apply -->

- [ ] Bug fix (non-breaking change that fixes an issue)
- [ ] New feature (non-breaking change that adds functionality)
- [ ] Breaking change (fix or feature that would cause existing functionality to not work as expected)
- [ ] Documentation update
- [ ] Code refactoring (no functional changes)
- [ ] Performance improvement
- [ ] Tests

## Learning Objectives

<!-- What r2r or ROS2 patterns does this code teach users? -->

-
-

## Related Issues

<!-- Link related issues (e.g., Closes #42, Fixes #123) -->

Closes #

## Changes Made

<!-- Detailed list of changes -->

-
-
-

## Testing

<!-- Describe how you tested these changes -->

### Manual Testing

```bash
# Commands you ran to test
source scripts/dev_env.sh
cargo build --workspace
cargo run -p <package> -- <args>
```

**Results**:

### Automated Testing

- [ ] Unit tests pass (`cargo test`)
- [ ] Integration tests pass (if applicable)
- [ ] Benchmarks run without regressions (if applicable)

## Documentation

<!-- Check all that were updated -->

- [ ] Inline code comments added/updated
- [ ] Module-level documentation updated
- [ ] README.md updated (if user-facing changes)
- [ ] docs/nodes.md updated (if node implementation changed)
- [ ] docs/architecture.md updated (if design changed)
- [ ] CONTRIBUTING.md updated (if contributor process changed)

## Code Quality Checklist

- [ ] Code follows style guidelines (`cargo fmt --all`)
- [ ] No clippy warnings (`cargo clippy --workspace --all-targets -- -D warnings`)
- [ ] All tests pass (`cargo test --workspace`)
- [ ] Documentation builds without warnings (`cargo doc --no-deps --workspace`)
- [ ] Commit messages are clear and descriptive
- [ ] Learning objectives clearly explained

## Breaking Changes

<!-- If breaking changes, describe migration path -->

## Screenshots / Logs

<!-- If applicable, add screenshots or logs showing the feature working -->

```
Paste relevant logs here
```

## Additional Notes

<!-- Any other information that would be helpful for reviewers -->
