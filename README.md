# FluxProcessor v2027.1.0 — FRC Team 10413 (FLUX Robotics)

Automated post-match and post-practice `.wpilog`/`.hoot`, multi-headset voice (`Driver` / `Operator` / `Coach`), on-robot camera snapshot (`cam-snapshots/*.jpg`), match video (`YouTube` / local `.mp4`), Digital-Twin simulation auto-calibration (`--calibrate-sim`), and JSON-RPC 2.0 Model Context Protocol (`--mcp`) analyzer for **FRC Team 10413 (FLUX Robotics)**.

Built on **Java 17+ compatible bytecode** (`--release 17`, running seamlessly on both WPILib 2026 JDK 17 and WPILib 2027 JDK 25) with bundled **`FluxFig`** (`libs/fluxfig-1.0.0.jar`) and zero external Maven runtime dependencies. Works **out of the box with zero modifications to existing robot code** (`New_Robot/`, `Flux-SystemCore-2027/`, `Flux-RoboRIO-2027/`).

---

## Features (v2027.1.0)

1. **`FluxFig` Snapshot Hydration & Zero-Dependency `.wpilog` Multi-Bus Merger (`merged.wpilog`)**
   - Automatically hydrates `RobotProfile` (`COMP_2027`, `PRACTICE_2027`, `FUEL`, `CORAL`) and all mechanism CAN IDs, current limits, and gravity models directly from `/FluxFig/EffectiveSnapshot` inside `.wpilog` files or from a `FluxFig` CSV directory (`--config-dir <dir>`).
   - Reads RoboRIO (`_rio.wpilog`), Drivetrain CANivore (`_drive.wpilog`), and Mechanism CANivore (`_mech.wpilog`) logs (and converts raw CTRE Phoenix 6 `.hoot` files via `owlet` when present).
   - Writes a unified `merged.wpilog` combining `/Rio/*`, `/DriveBus/*`, `/MechBus/*`, `/VoiceNotes/*`, and `/FluxAudit/*` so you open **1 single file** in AdvantageScope.

2. **Named Battery Internal Resistance ($R_{\text{int}}$) & Brownout Audit (`BatteryAnalyzer`)**
   - Estimates battery internal resistance ($R_{\text{int}} = -\Delta V / \Delta I$ in $\text{m}\Omega$) under acceleration load, tracks minimum battery voltage across both `Pigeon2/SupplyVoltage` and `TalonFX/SupplyVoltage` ($\pm 100\text{ ms}$ windowed min/max buckets), brownout duration ($< 7.0\text{ V}$ warning, $< 6.8\text{ V}$ critical), and mid-match enable dropouts, and updates `batteries-health.csv`.

3. **4-Corner Swerve Symmetry, 2027 Elevator Cycles & $5\text{ m} \times 3\text{ m}$ `ShopStressSuiteCmd` Pass Gates**
   - **`SwerveSymmetryAnalyzer`:** Compares Front-Left (`FL`), Front-Right (`FR`), Back-Left (`BL`), and Back-Right (`BR`) drive and steer motor currents and velocities against the 4-corner median.
   - **`ShopStressAndElevatorAnalyzer`:**
     - Evaluates 2027 **Elevator + Arm (`L1..L4`)** scoring cycles and 2D keep-out interlocks alongside 2026 flywheel/feeder shot detection.
     - Audits the **$5\text{ m} \times 3\text{ m}$ `ShopStressSuiteCmd` & `VirtualGeofence`** pass gates (`0` wall breaches, Figure-8 loop closure drift $\le 4.0\text{ cm}$, camera-to-camera handoff pose step $\le 3.0\text{ cm}$, and 90-Degree Box-Step 4-corner current spread $\le 12.0\%$).
     - Correlates event-triggered on-robot camera snapshots (`cam-snapshots/*.jpg`) captured on pre-match lock, pose jumps, shots, and voice notes.

4. **Digital-Twin Simulation Auto-Calibrator (`--calibrate-sim [<fluxfig-csv-dir>]`)**
   - Extracts measured battery $R_{\text{int}}$ (`mOhm`), effective swerve wheel radius (`wheelRadiusMeters`), steer-to-drive coupling ratio (`swerveCouplingRatio`), and mechanism feedforward constants $(k_S, k_V, k_A, k_G)$ via OLS regression (`DigitalTwinCalibrator`).
   - Atomically validates (`FluxFig.validateStrict()`) and writes calibrated constants back into `batteries.csv`, `parameters.csv`, and `mechanisms.csv`, plus outputs `sim-calibration.json`.

5. **Multimodal Model Context Protocol (`--mcp`) Server (`FluxMcpServer`)**
   - Built-in JSON-RPC 2.0 stdio MCP server exposing 8 multimodal tools for coding agents (Claude Code, Gemini, Cursor):
     - `list_entries`, `find_condition`, `time_correlate`, `compare_runs`, `moi_regression`, `search_voice_notes`, `get_camera_snapshots`, and `get_video_frames`.

6. **Multi-Headset Voice Feedback (6-Second Lookback), Video Jump-Links & 14-Metric SystemCore Watch Audit**
   - Supports both `run-meta.json` (written by `FluxTray.jar`) and `*.meta.json`, accepting `"batteryName"` or `"namedBattery"` (including names with spaces) and embedded `"voiceNotes": [...]` JSON arrays in addition to `voice-notes.txt`.
   - Evaluates all **14 Offline SystemCore & Controller Watch Metrics** from `system-snapshot.txt` (without fabricating synthetic SystemCore CPU/eMMC metrics on `.wpilog`-only roboRIO runs).

---

## Quick Start

### Execution (`dist/flux-processor.jar`)
No external install is needed if **WPILib 2026/2027** (`C:\Users\Public\wpilib\202*\jdk`) or **Java 17+** is on the laptop, and a ready-to-run fat-jar [`dist/flux-processor.jar`](dist/flux-processor.jar) (bundling `FluxFig`) is included in the repository:
1. **Pull & Package Robot Logs via `FluxTray.jar`:** Running `java -jar dist/FluxTray.jar` on the Driver Station laptop automatically downloads `.wpilog`, `.hoot`, `driver-audio.ogg`, `.dslog`/`.dsevents`, and `system-snapshot.txt` and packages both `FLUX-10413-Hub/04_Practice_And_Match_Logs/<runId>/` and `FLUX-10413-Hub/04_Practice_And_Match_Logs/match-logs_<yyyy-MM-dd>_<HHmmss>_<ROBOT>.zip` by default (customizable via `--dir`, `--file`, or `--out`).
2. **Analyze Any Run Folder or `.zip` Archive Directly:**
   ```bash
   ./scripts/build.sh
   java -jar dist/flux-processor.jar <path/to/match-logs-dir-or-zip> --out ./output
   ```
   *(Or to recompile and run all unit tests on Windows: `scripts\build.bat`)*

### 3D Student Autonomous Video & Interactive HTML Renderer (`--render-auto`)
Consumes `student-auto-frames.json` produced by JUnit simulation tests (`StudentAutoSimExporter` in `Flux-SystemCore-2027` or `Flux-RoboRIO-2027`) and renders `student-auto.mp4` and `student-auto.html` (supporting both `EMPTY_FIELD_2027` and `REBUILT_2026_SHOOTING`):
```bash
java -jar dist/flux-processor.jar --render-auto <path/to/student-auto-frames.json> --out ./output
```

### Digital-Twin Sim Auto-Calibration (`--calibrate-sim`)
```bash
java -jar dist/flux-processor.jar <path/to/run-logs> --calibrate-sim ../Flux-SystemCore-2027/src/main/deploy/configs
```

### Multimodal MCP Server (`--mcp`)
```bash
java -jar dist/flux-processor.jar --mcp <path/to/match-logs>
```

---

## Custom Robot Profile (`--config-dir`, `--profile`, or `--robot`) for Any FRC Team
`FluxProcessor` works out of the box with **CTRE Phoenix 6** (`.hoot` / `.wpilog`), **AdvantageKit** (`/Drive/Module0..3/*`, `double[]` `/Drive/DriveCurrentAmps`), **WPILib `DataLogManager` / DogLog / Epilogue**, and **REV URCL** logs.
- **Zero-Config Auto-Discovery (`--robot AUTO`, default):** Automatically hydrates from `/FluxFig/EffectiveSnapshot` in the `.wpilog`, detects Team 10413 `COMP_2027` / `FUEL` / `CORAL` presets, or scans the `.wpilog` entry table to discover any other FRC team's swerve modules, mechanisms, battery voltage, and enable signals.
- **`FluxFig` CSV Directory (`--config-dir <path/to/configs>`):** Loads `parameters.csv` and `mechanisms.csv` directly via `FluxFig`.
- **External JSON Profile (`--profile robot-profile.json` or placed in the log folder):** Any team can define custom CAN IDs, gear ratios, or custom topic paths without recompiling Java:
```json
{
  "robotName": "KRAKEN_BOT",
  "pigeonId": 1,
  "wheelRadiusMeters": 0.0508,
  "driveGearRatio": 5.90,
  "corners": {
    "FL": { "driveId": 10, "steerId": 11, "cancoderId": 12 },
    "FR": { "driveId": 20, "steerId": 21, "cancoderId": 22 },
    "BL": { "driveId": 30, "steerId": 31, "cancoderId": 32 },
    "BR": { "driveId": 40, "steerId": 41, "cancoderId": 42 }
  },
  "mechanisms": [
    { "name": "Elevator", "motorId": 50, "role": "Position Lift", "currentLimitAmps": 60.0, "isShooter": false },
    { "name": "Launcher", "motorId": 51, "role": "Flywheel Leader", "currentLimitAmps": 80.0, "isShooter": true },
    { "name": "Kicker",   "motorId": 52, "role": "Feeder Roller",   "currentLimitAmps": 40.0, "isShooter": false }
  ]
}
```

---

## Optional Run Metadata (`run-meta.json` / `*.meta.json`) — Voice Notes, Video Sync & Named Battery

Drop a `run-meta.json` (written automatically by `FluxTray.jar` during practice or competition) or `*.meta.json` into any run folder to correlate multi-headset voice notes (`Driver`, `Operator`, `Coach`), YouTube/stand video jump links (`[▶ Watch @ m:ss]`), and named battery health (`batteries-health.csv`):

```json
{
  "batteryName": "2027-B03 (Comp Pack)",
  "gitSha": "251dc78",
  "configSha256": "a84f92c1",
  "videoUrl": "https://www.youtube.com/watch?v=...",
  "videoOffsetSec": 14.0,
  "voiceNotes": [
    { "role": "Driver",   "timeSec": 26.5,  "text": "Hard acceleration out of source, felt a brief stutter" },
    { "role": "Operator", "timeSec": 82.0,  "text": "Elevator L4 cycle felt super fast" },
    { "role": "Driver",   "timeSec": 145.0, "text": "Robot keeps browning out and stuttering in endgame!" }
  ]
}
```
