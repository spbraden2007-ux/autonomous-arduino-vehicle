# Autonomous Arduino Vehicle — Greedy Best-First Pathfinding with Real-Time Obstacle Avoidance

> **🏆 STEM Fair Robotics Division — 1st Place**

4-wheel Arduino robot navigating a 6×6 grid using greedy best-first pathfinding (A*-inspired, optimized for Arduino's 2KB SRAM). Features ultrasonic obstacle detection with dynamic re-routing and Bluetooth control via custom Android app.

## Demo

[![Demo Video](https://img.shields.io/badge/▶_Watch_Demo-Google_Drive-4285F4?style=for-the-badge&logo=googledrive&logoColor=white)](https://drive.google.com/file/d/1uKarYfRPUSvqBH1gG7XaHbdlwjgqWeYI/view)
[![Demo Video](https://img.shields.io/badge/▶_Watch_Demo-Google_Drive-4285F4?style=for-the-badge&logo=googledrive&logoColor=white)](https://drive.google.com/file/d/1fb5O3GCMhDKX_a49j4kzPB81KhXHyAGl/view?t=2) 
[![Demo Video](https://img.shields.io/badge/▶_Watch_Demo-Google_Drive-4285F4?style=for-the-badge&logo=googledrive&logoColor=white)](https://drive.google.com/file/d/1s98X8F5Gak0GjhBeS7kO9mXCLfXCBJgz/view?t=7)

## Key Results

| Metric | Autonomous | Human-Driven RC | Improvement |
|--------|-----------|-----------------|-------------|
| Total Collisions (sum over 3 trials) | **0** | 6 | 100% reduction |
| Position Error | **~1.0 cm** | ~14.1 cm | 14× more accurate |
| Consistency | High | Variable | — |

*Test conditions: Normal terrain, human operator with limited practice time*

**Note:** Performance degraded on dirt terrain due to wheel slippage — see [experiment results](docs/experiment_results.md) for details.

## Technical Highlights

**Algorithm**
- **Greedy best-first search** (A*-inspired, memory-optimized)
- Selection criterion: F = H only (Manhattan distance to goal)
- Standard A* uses F = G + H, but G omitted to fit 2KB SRAM
- Dynamic re-routing when obstacle detected mid-path

**Hardware**
- Arduino Uno + Adafruit Motor Shield v1
- 4× DC motors (differential drive)
- HC-SR04 ultrasonic sensor (15cm detection threshold)
- HC-06 Bluetooth module (9600 baud)
- AA battery pack

**Software**
- ~600 lines of C++ (Arduino)
- Custom Android app (MIT App Inventor)
- Real-time Bluetooth communication protocol

## How It Works

```
┌─────────────┐     Bluetooth      ┌─────────────┐
│  Android    │ ──────────────────▶│   Arduino   │
│    App      │   Start/Goal pos   │             │
└─────────────┘                    └──────┬──────┘
                                          │
                                          ▼
                                   ┌─────────────┐
                                   │  Greedy     │
                                   │  Best-First │
                                   │  (6×6 grid) │
                                   └──────┬──────┘
                                          │
                    ┌─────────────────────┼─────────────────────┐
                    ▼                     ▼                     ▼
             ┌─────────────┐       ┌─────────────┐       ┌─────────────┐
             │   Execute   │       │  Obstacle?  │       │   Arrive    │
             │    Move     │◀──No──│   (15cm)    │       │   at Goal   │
             └─────────────┘       └──────┬──────┘       └─────────────┘
                                          │ Yes
                                          ▼
                                   ┌─────────────┐
                                   │  Re-route   │
                                   │  (recalc)   │
                                   └─────────────┘
```

## Algorithm Implementation

```cpp
// Heuristic: Manhattan distance to goal
byte H(byte curR, byte curC, byte goalS) {
    byte rowg = goalS / 6;
    byte colg = goalS % 6;
    return abs(curR - rowg) + abs(curC - colg);
}

// Greedy best-first: F = H only
// Standard A* uses F = G + H, but G is omitted here
// to fit within Arduino's 2KB SRAM constraint
byte FV(byte curG, byte curH) {
    return curH;  // Returns heuristic only
}
```

**Why greedy best-first instead of full A*?**
- Arduino Uno has only 2KB SRAM
- Open/closed lists use byte arrays (50 bytes each)
- **Greedy approach prioritizes speed over optimality**
- Trade-off: Faster pathfinding, acceptable path quality on 6×6 grid
- Full A* would guarantee shortest path but requires more memory for G-cost tracking

## Project Structure

```
autonomous-arduino-vehicle/
├── README.md
├── main.ino              # Complete Arduino source (~600 lines)
├── hardware/
│   ├── schematic_breadbord.png
│   └── schematic_circuit.png
│ 
└── docs/
    └── experiment_results.md
```

**Note:** Android app source (.aia) available upon request.

## My Contributions

| Component | Contribution | Details |
|-----------|-------------|---------|
| Algorithm | Co-developed | Pathfinding structure design, heuristic selection |
| Hardware | Co-developed | Circuit design, motor calibration |
| **Android App** | **Solo** | Full design, development, Bluetooth integration |
| **Schematics** | **Solo** | Fritzing diagrams |

*3-person team project*

## Hardware Schematic

![Circuit Schematic](hardware/schematic_circuit.png)

### Pin Map & Components

| Component | Pin/Connection | Notes |
|-----------|---------------|-------|
| **Motor Shield v1** | Stacked on Uno | M1-M4 → 4 DC motors |
| **HC-SR04 Ultrasonic** | Trig=A0, Echo=A1 | 15cm detection threshold |
| **HC-06 Bluetooth** | Arduino RX(A5) ← HC-06 TX<br>Arduino TX(A4) → HC-06 RX | 9600 baud, PIN: 1234 |
| **LED Indicator** | A3 | Goal reached signal |
| **Motors (×4)** | M1, M2, M3, M4 | TT DC Gearmotor |
| **Power** | AA battery pack | Motor power via shield |

⚠️ **Power Note:** Motor power is separate from Arduino logic power. Motor Shield v1 has a power jumper — remove it if using external motor power supply.

⚠️ **Motor Shield Note:** This project uses Adafruit Motor Shield v1 with the AFMotor library. The schematic shown is a simplified representation; actual implementation uses the shield's integrated H-bridges.

### Bluetooth Protocol

| Command | Format | Example | Description |
|---------|--------|---------|-------------|
| Set Start | `a<0-35>` | `a0` | Start at cell 0 (top-left) |
| Set Goal | `b<0-35>` | `b35` | Goal at cell 35 (bottom-right) |
| Go | `g` | `g` | Begin navigation |

Grid is **row-major**: cells 0-5 = row 0, cells 6-11 = row 1, etc.

## Experimental Setup

**Variables:**
- Independent: Vehicle type (Autonomous vs Human-controlled)
- Dependent: Collisions, position accuracy
- Controlled: Terrain, distance, obstacles

**Methodology:**
- 3 trials per condition
- Human operator with limited practice time
- Terrain: Indoor smooth surface (normal) and outdoor uneven surface (dirt)

**Results Summary:**
- ✅ **Normal terrain:** Autonomous significantly outperformed (0 vs 6 collisions)
- ⚠️ **Dirt terrain:** Performance degraded due to wheel slippage — human adapted better

See [full experiment results](docs/experiment_results.md) for detailed analysis.

## Quick Start

```bash
# 1. Flash Arduino
#    Open main.ino in Arduino IDE
#    Install: AFMotor.h (Adafruit Motor Shield v1 library)
#    Upload to Arduino Uno

# 2. Android App
#    Contact for MIT App Inventor source (.aia)
#    Or build from scratch following Bluetooth serial protocol

# 3. Connect & Run
#    Pair HC-06 (PIN: 1234)
#    Send: a0    (start at cell 0)
#    Send: b35   (goal at cell 35)
#    Send: g     (go!)
```

## Limitations

- Grid-based navigation (not continuous pathfinding)
- Single ultrasonic sensor (forward-only detection)
- Wheel slippage on uneven terrain
- Greedy best-first may not find optimal path in complex mazes

## Future Improvements

- [ ] IMU sensor for drift correction
- [ ] PID motor control for smoother turns
- [ ] Multi-sensor obstacle detection (side sensors)
- [ ] Terrain-adaptive calibration
- [ ] Full A* implementation with optimized memory usage

## Built With

- Arduino IDE 1.8.x
- MIT App Inventor
- Fritzing (schematics)
- C++ / Arduino

---

**STEM Fair 2023 · Robotics Division · 1st Place**
