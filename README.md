# FluxProcessor — FRC Team 10413 (FLUX Robotics)

Automated post-match and post-practice log, multi-headset voice (`Driver` / `Operator` / `Coach`), and video (`YouTube` / local `.mp4`) analyzer for **FRC Team 10413 (FLUX Robotics)**.

Works **out of the box with zero modifications to existing robot code** (`New_Robot/`).

---

## Features (v0.1)

1. **Zero-Dependency Pure-Java `.wpilog` Binary Parser & Multi-Bus Merger (`merged.wpilog`)**
   - Reads RoboRIO (`_rio.wpilog`), Drivetrain CANivore (`_drive.wpilog`), and Mechanism CANivore (`_mech.wpilog`) logs (and automatically converts raw CTRE Phoenix 6 `.hoot` files via `owlet` when present).
   - Writes a unified `merged.wpilog` combining `/Rio/*`, `/DriveBus/*`, `/MechBus/*`, `/VoiceNotes/*`, and `/FluxAudit/*` so you open **1 single file** in AdvantageScope.

2. **Named Battery Internal Resistance ($R_{\text{int}}$) & Brownout Audit**
   - Estimates battery internal resistance ($R_{\text{int}} = -\Delta V / \Delta I$ in $\text{m}\Omega$) under acceleration load, tracks minimum battery voltage, brownout duration ($< 7.0\text{ V}$ warning, $< 6.8\text{ V}$ critical), and mid-match enable dropouts, and updates `batteries-health.csv`.

3. **4-Corner Swerve Symmetry & Mechanism Health Check**
   - Compares Front-Left (`FL`), Front-Right (`FR`), Back-Left (`BL`), and Back-Right (`BR`) drive and steer motor currents and velocities against the 4-corner median to catch rubbing brakes, worn bearings, or current-limit mismatches early.
   - Audits all mechanisms (`Intake`, `Feeder`, `Indexer`, `Shooter Left/Right`, `Hood`, `Tilter`) and detects every discrete **Shot / Feed Event**.

4. **Multi-Headset Voice Feedback (6-Second Lookback) & Video Jump-Links**
   - Correlates spoken comments from `Driver`, `Operator`, and `Coach` tracks (`voice-notes.txt`, `audio-driver.txt`, `audio-operator.txt`, `audio-coach.txt`) with the preceding 6-second sensor window ($[t - 6.0\text{s} .. t]$) and links every event to a timestamped YouTube or local `.mp4` video clip.

5. **1-Page Browser Report (`report.html`) & Season Leaderboard (`runs-index.html`)**
   - Styled in official **Pelham FLUX Robotics (`pelhamroboticsteam.org`)** brand colors (`#212020` Charcoal Black, `#254BA0` Pelham Royal Blue, `#2B7BFF` FLUX Electric Blue, `Space Grotesk` font).

---

## Quick Start

### Windows (Double-Click, Drag-and-Drop, or PowerShell)
No external install is needed if **WPILib** (`C:\Users\Public\wpilib\2026\jdk` or `2027\jdk`) or **Java 17+** is on the laptop, and a ready-to-run [`dist/flux-processor.jar`](dist/flux-processor.jar) is included in the repository:
1. **Double-Click [`run.bat`](run.bat):** Opens a native Windows folder-picker dialog, analyzes the selected match-logs folder, and automatically opens `runs-index.html` in your browser.
2. **Drag-and-Drop onto [`run.bat`](run.bat):** Drag any match-logs folder **or** any `match-logs_*.zip` file produced by `pull-logs.bat` directly onto `run.bat` in Windows Explorer.
3. **PowerShell / Command Prompt:**
   ```bat
   run.bat C:\Users\Public\Desktop\match-logs --out .\output
   ```
   *(Or to recompile and run all unit tests on Windows: `scripts\build.bat`)*

### Linux / macOS
```bash
./scripts/build.sh
java -jar dist/flux-processor.jar <path/to/match-logs> --out ./output
```

### Custom Robot Profile (`--profile` or `--robot`) for Any FRC Team
`FluxProcessor` works out of the box with **CTRE Phoenix 6** (`.hoot` / `.wpilog`), **AdvantageKit** (`/Drive/Module0..3/*`, `double[]` `/Drive/DriveCurrentAmps`), **WPILib `DataLogManager` / DogLog / Epilogue**, and **REV URCL** logs.
- **Zero-Config Auto-Discovery (`--robot AUTO`, default):** Automatically detects Team 10413 `FUEL` / `CORAL` presets or scans the `.wpilog` entry table to discover any other FRC team's swerve modules, mechanisms, battery voltage, and enable signals.
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

## Optional Run Metadata (`run.meta.json`) — Voice Notes, Video Sync & Named Battery

Drop a `run.meta.json` (or `*.meta.json`) into any run folder (written automatically by `FluxTray.jar` during practice or competition) to correlate multi-headset voice notes (`Driver`, `Operator`, `Coach`), YouTube/stand video jump links (`[▶ Watch @ m:ss]`), and named battery health (`batteries-health.csv`):

```json
{
  "namedBattery": "BAT-03-COMP",
  "gitSha": "251dc78",
  "configSha": "a84f92c1",
  "videoUrl": "https://www.youtube.com/watch?v=...",
  "videoOffsetSec": 14.0,
  "voiceNotes": [
    { "role": "Driver",   "timeSec": 26.5,  "text": "Hard acceleration out of source, felt a brief stutter" },
    { "role": "Operator", "timeSec": 82.0,  "text": "Shot felt slightly short from mid-wing" },
    { "role": "Driver",   "timeSec": 145.0, "text": "Robot keeps browning out and stuttering in endgame!" }
  ]
}
```

- **Competition "Capture on Field, Sync in Pit" Mode:** Because FMS blocks SFTP (Port 22) on the field and requires unplugging immediately on the green light, `FluxTray.jar` records `driver-audio.ogg` + `.dslog` + `run.meta.json` locally on the Driver Station laptop at the field and joins the robot's `.wpilog`/`.hoot` files later in the pit.
- **CTRE `.hoot` Conversion Resilience (`HootConverter`):** Uses `owlet <hoot> -S -F` (full-file signal scan) with essential signal filtering (`-s`) and omits `-e` (`--enable-only`) so pre-match pit/queue enables never truncate the actual competition match at the end of the `.hoot` file; also accepts `owlet` output at 100% completion even when `owlet` exits with code `1` due to power-off truncation of the final CAN frame.
