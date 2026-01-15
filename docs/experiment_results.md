# Experiment Results

## Research Question

**In a controlled 6×6 grid environment with limited sensing (forward ultrasonic only), how does greedy best-first pathfinding compare to human RC control in collision avoidance and positioning accuracy?**

*Note: This is a proof-of-concept study for grid-based navigation, not a claim about real-world autonomous vehicles.*

## Methodology

| Parameter | Value |
|-----------|-------|
| Grid Size | 6×6 (36 cells, 24cm × 24cm per cell) |
| Trials | 3 per condition |
| Terrain | Normal (smooth indoor) + Dirt (outdoor uneven) |
| Control | Human-operated RC car |
| Operator | 17 y/o with limited practice time |
| **Collision Definition** | **Physical contact with obstacle (cup ramen)** |
| **Position Error** | **Euclidean distance from vehicle center to goal center** |
| **Measurement Tool** | **Ruler (±0.5cm precision)** |

## Results

### Normal Terrain Performance

| Trial | Autonomous Collisions | RC Collisions | Autonomous Error | RC Error |
|-------|----------------------|---------------|------------------|----------|
| 1 | 0 | 2 | 0.8 cm | 12.5 cm |
| 2 | 0 | 3 | 1.2 cm | 15.0 cm |
| 3 | 0 | 1 | 0.9 cm | 14.8 cm |
| **Mean** | **0** | **2.0** | **~1.0 cm** | **~14.1 cm** |

### Dirt Terrain Performance

| Trial | Autonomous Collisions | RC Collisions | Notes |
|-------|----------------------|---------------|-------|
| 1 | 1 | 0 | Wheel slippage affected turns |
| 2 | 2 | 0 | Human adapted to terrain |
| 3 | 1 | 1 | Both struggled |
| **Mean** | **1.3** | **0.3** | Autonomous degraded |

### Summary by Terrain

| Terrain | Winner | Reason |
|---------|--------|--------|
| **Normal** | ✅ Autonomous | Consistent, precise, zero collisions |
| **Dirt** | ⚠️ Human | Adapted to slippage, intuitive control |

## Analysis

### Why Autonomous Won on Normal Terrain
- Digital motor commands = no human tremor
- Grid-based positioning = consistent accuracy
- Pre-planned paths = predictable behavior

### Why Human Won on Dirt Terrain
- Real-time visual feedback
- Intuitive compensation for slippage
- Adaptive control (no fixed calibration)

## Limitations

1. **Operator profile**: Limited practice time may have disadvantaged human baseline
2. **Sample size**: Only 3 trials per condition
3. **Terrain-specific calibration**: Robot tuned for normal terrain only
4. **Single sensor**: Forward-only obstacle detection
5. **Data retention**: Raw per-trial logs were not preserved (project completed ~1 year ago); results transcribed from recorded trial notes

## Conclusion

**Hypothesis partially confirmed.**

- ✅ **Normal terrain:** Autonomous significantly safer and more reliable
- ❌ **Dirt terrain:** Human outperformed due to adaptability

**Key insight:** Autonomous systems excel in structured environments but require terrain-specific calibration for real-world deployment.
