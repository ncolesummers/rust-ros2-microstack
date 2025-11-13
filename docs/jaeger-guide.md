# Understanding Jaeger Traces

## Quick Guide to Reading the distributed_tracing Example

When you run the distributed_tracing example with OTLP, you're sending trace data to Jaeger. Here's how to understand what you see.

## Jaeger UI Overview

Access at: http://localhost:16686

### Main Sections

1. **Search Panel (Left)**: Filter traces by service, operation, time range
2. **Trace List (Center)**: Shows all matching traces with duration
3. **Trace Detail (Right)**: Detailed timeline view of a selected trace

## Understanding the Trace Timeline

### Our Example Trace Structure

When you run:
```bash
LOG_FORMAT=otlp OTLP_ENDPOINT=http://localhost:4317 cargo run --example distributed_tracing
```

You'll see a trace that looks like this in Jaeger:

```
main (total duration: ~1.5s)
│
├─ distributed_tracing_workflow (620ms)
│  │
│  ├─ collect_sensor_data (154ms)
│  │  │
│  │  ├─ read_sensor (sensor_id=1) (51ms)
│  │  ├─ read_sensor (sensor_id=2) (51ms)
│  │  └─ read_sensor (sensor_id=3) (51ms)
│  │
│  ├─ process_data (25ms)
│  │  │
│  │  ├─ filter_data (12ms)
│  │  └─ normalize_data (8ms)
│  │
│  └─ publish_results (45ms)
│     │
│     ├─ publish_sample (sample_index=0) (13ms)
│     ├─ publish_sample (sample_index=1) (13ms)
│     └─ publish_sample (sample_index=2) (13ms)
│
├─ 500ms sleep
│
├─ manual_operation (100ms)
│  └─ operation_id=42, user=example
│
├─ 500ms sleep
│
└─ concurrent_operations_example (152ms)
   │
   ├─ concurrent_task (task_id=1) (100ms) ─┐
   ├─ concurrent_task (task_id=2) (150ms) ─┤ These run in parallel!
   └─ concurrent_task (task_id=3) (80ms)  ─┘
```

## How to Navigate Jaeger

### Step 1: Find Your Trace

1. **Service dropdown**: Select `rust-ros2-microstack`
2. **Operation dropdown**: Select `main` (or leave as "all")
3. Click **"Find Traces"** button

You'll see a list of traces. Each line represents one execution of your program.

### Step 2: Click on a Trace

You'll see:
- **Service & Operation** at the top
- **Duration** (how long the entire trace took)
- **Number of spans** (how many operations)
- **Timestamp** (when it ran)

### Step 3: Expand the Timeline

Click the trace to see the detailed timeline view with horizontal bars:

```
Trace Timeline Visualization:

main                    |████████████████████████████████████| 1.5s
├─ workflow             |███████|                              620ms
│  ├─ collect           |██|                                   154ms
│  │  ├─ read_sensor    |█|                                    51ms
│  │  ├─ read_sensor     |█|                                   51ms
│  │  └─ read_sensor      |█|                                  51ms
│  ├─ process                |█|                               25ms
│  │  ├─ filter              |_|                               12ms
│  │  └─ normalize            |_|                              8ms
│  └─ publish                    |█|                           45ms
└─ concurrent                            |████|                152ms
   ├─ task_1                             |███|                 100ms
   ├─ task_2                             |█████|               150ms
   └─ task_3                             |██|                  80ms
```

**Key observations:**
- Longer bars = more time spent
- Nested bars = function calls
- Overlapping bars at same level = parallel execution

## Reading Span Details

### Click on Any Span to See:

**1. Span Information**
- Operation name: `read_sensor`, `process_data`, etc.
- Duration: How long it took
- Start time: When it started (relative to trace start)

**2. Tags (Structured Fields)**
These come from your code! For example:

In `read_sensor`, you'll see:
```rust
// From code:
#[instrument]
async fn read_sensor(sensor_id: u32) -> Result<f32>

// In Jaeger:
Tags:
  sensor_id: 1
```

In `collect_sensor_data`:
```rust
// From code:
#[instrument(fields(sensor_count = 3))]

// In Jaeger:
Tags:
  sensor_count: 3
```

**3. Logs (Within Spans)**
These are your `info!()`, `debug!()`, etc. calls:
```rust
debug!(sensor_id = sensor_id, value = value, "Sensor read complete");
```

Shows up as a log entry with timestamp within the span.

**4. Process Information**
- Service name: `rust-ros2-microstack`
- Process tags: hostname, runtime info

## What Makes Each Span Useful?

### Example 1: Performance Analysis

Look at `collect_sensor_data` span:
```
collect_sensor_data (154ms)
├─ read_sensor (sensor_id=1) (51ms)
├─ read_sensor (sensor_id=2) (51ms)
└─ read_sensor (sensor_id=3) (51ms)
```

**What you learn:**
- Each sensor read takes ~51ms (consistent performance)
- They run sequentially (not in parallel) - you can tell because they don't overlap
- Total time = 3 × 51ms ≈ 154ms

**Optimization opportunity:** You could parallelize these reads!

### Example 2: Parallel Execution

Look at `concurrent_operations_example`:
```
concurrent_operations_example (152ms)
├─ concurrent_task (task_id=1) (100ms) ─┐
├─ concurrent_task (task_id=2) (150ms) ─┤ Overlapping!
└─ concurrent_task (task_id=3) (80ms)  ─┘
```

**What you learn:**
- Tasks run in parallel (bars overlap in timeline)
- Task 2 is slowest (150ms) so it determines total time
- If they ran sequentially: 100 + 150 + 80 = 330ms
- Running in parallel: max(100, 150, 80) = 150ms
- **Speedup: 2.2x** from parallelization!

### Example 3: Finding Bottlenecks

If you see:
```
process_data (1000ms)
├─ filter_data (10ms)
└─ normalize_data (985ms)    ← This is the bottleneck!
```

You immediately know where to optimize.

## Practical Use Cases

### 1. Performance Debugging
**Question:** "Why is my robot's navigation slow?"

**Jaeger shows:**
```
navigation (500ms)
├─ get_sensor_data (5ms)
├─ compute_path (400ms)      ← Found it!
└─ send_commands (10ms)
```

### 2. Understanding Execution Flow
**Question:** "What happens when I publish a message?"

**Jaeger shows:** The complete chain of operations with timing.

### 3. Debugging Race Conditions
**Question:** "Why do my async tasks sometimes fail?"

**Jaeger shows:** Exactly when tasks start/end and how they overlap.

### 4. Comparing Runs
**Question:** "Did my optimization actually help?"

**Before optimization:**
```
Trace 1: process_data (1000ms)
```

**After optimization:**
```
Trace 2: process_data (250ms)  ← 4x faster!
```

## Interactive Features in Jaeger

### 1. Filtering
In search panel:
- **Min/Max Duration**: Find slow traces
- **Tags**: Find traces with specific values
  - Example: `sensor_id=2` finds only traces that read sensor 2
- **Lookback**: "Last 1 hour", "Last 15 minutes", etc.

### 2. Comparison
- Click "Compare" checkbox on multiple traces
- See side-by-side comparison
- Identify differences in behavior

### 3. Deep Links
- URL contains trace ID
- Share exact trace with teammates
- Bookmark problematic traces

### 4. Statistics
- Shows percentiles (p50, p95, p99)
- Identifies outliers
- Helps set alerts

## Connecting to Your Code

### From Code to Jaeger

**Your code:**
```rust
#[instrument(skip(data), fields(input_size = data.len()))]
async fn process_data(data: Vec<f32>) -> Result<Vec<f32>> {
    info!("Processing sensor data");

    let filtered = async {
        debug!("Filtering data");
        // ... filtering logic ...
        info!(input_count = data.len(), "Filtering complete");
    }
    .instrument(info_span!("filter_data", threshold = 5.0))
    .await;

    Ok(filtered)
}
```

**In Jaeger, you'll see:**

**Span:** `process_data`
- **Tags:**
  - `input_size: 3`
- **Child span:** `filter_data`
  - **Tags:**
    - `threshold: 5.0`
  - **Logs:**
    - `"Filtering data"` (at span start)
    - `"Filtering complete"` with `input_count: 3` (at span end)

## Common Patterns to Recognize

### Sequential Operations (Waterfall)
```
|─────A─────|
            |─────B─────|
                        |─────C─────|
```
Each waits for previous to complete.

### Parallel Operations (Concurrent)
```
|─────A─────|
|─────B─────|
|─────C─────|
```
All run at the same time.

### Fan-out/Fan-in
```
        ┌─────task1─────┐
|─start─┼─────task2─────┼─end─|
        └─────task3─────┘
```
Start one operation, spawn many, wait for all to complete.

### Request-Response
```
client |────request────>|
                         |────process────|
       |<────response────|
```

## Tips for Effective Tracing

### 1. Use Meaningful Span Names
```rust
// ❌ Bad
#[instrument]
async fn process() { }

// ✅ Good
#[instrument]
async fn process_laser_scan_data() { }
```

### 2. Add Context with Fields
```rust
// ❌ Bad
#[instrument]
async fn read_sensor(id: u32) { }

// ✅ Good - 'id' automatically becomes a tag!
#[instrument]
async fn read_sensor(id: u32) { }
```

### 3. Log Important Events
```rust
#[instrument]
async fn compute_path() {
    info!("Starting path computation");
    // ... computation ...
    info!(nodes_explored = 1000, "Path found");
}
```

### 4. Use Spans for All Async Operations
```rust
// Each async operation should have its own span
async fn complex_workflow() {
    operation_1().instrument(info_span!("op1")).await;
    operation_2().instrument(info_span!("op2")).await;
}
```

## Troubleshooting

### "I don't see any traces"

1. Check Jaeger is running: `docker ps | grep jaeger`
2. Check OTLP_ENDPOINT is correct: `echo $OTLP_ENDPOINT`
3. Check logs for errors: Run with `RUST_LOG=debug`
4. Verify time range in Jaeger search (default is last 1 hour)

### "Spans are missing"

1. Make sure `#[instrument]` is on the function
2. For manual spans, use `.instrument(span).await`
3. Check that functions are actually being called

### "Tags don't show up"

1. Function parameters automatically become tags
2. Use `fields(key = value)` in `#[instrument]`
3. Use structured fields in logs: `info!(key = value, "message")`

### "Traces are incomplete"

1. Call `logging::shutdown()` before exit
2. Give time for async export (few seconds)
3. Check for panics/early exits

## Next Steps

### Learn More

1. Run the example multiple times with different inputs
2. Try adding your own spans to other examples
3. Compare traces before/after code changes
4. Set up alerts on slow operations

### Advanced Features (Not Covered Here)

- Service dependencies graph
- Custom span kinds (client, server, producer, consumer)
- Baggage propagation (context across services)
- Sampling strategies (for high-volume production)

## Summary

**Jaeger shows you:**
- ✅ How long each operation takes
- ✅ Which operations run in parallel vs sequential
- ✅ Where your bottlenecks are
- ✅ Exact execution order of async operations
- ✅ Structured data at each step (via tags and logs)

**Use it when:**
- Debugging performance issues
- Understanding complex async flows
- Optimizing your code
- Learning how your system behaves

**The key insight:** Instead of guessing or using print statements, you get a visual, interactive timeline of exactly what happened in your program!
