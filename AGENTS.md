# AGENTS.md — FluxProcessor Standalone Agent Guide (FRC Team 10413)

## 1. Repository Overview & Scope
`FluxProcessor` (`frc.flux.processor`) is the **standalone zero-dependency Java 17+ (`--release 17`) post-match `.wpilog`/`.hoot`/voice/camera analyzer, 3D `FuelSim` physics replayer, student autonomous 3D video/HTML renderer (`--render-auto`), Digital-Twin simulation auto-calibrator (`--calibrate-sim`), and JSON-RPC 2.0 MCP server (`--mcp`)** for **FRC Team 10413 (FLUX Robotics)**:
- [`README.md`](README.md): CLI flags, features, and report outputs.
- [`libs/fluxfig-1.0.0.jar`](libs/fluxfig-1.0.0.jar): Bundled `FluxFig` library providing CSV/snapshot hydration (`FluxFig`), binary `.wpilog` I/O (`FluxWpiLogCodec`), and 14-metric SystemCore/roboRIO watch evaluation (`FluxSystemSnapshot`).
- [`dist/flux-processor.jar`](dist/flux-processor.jar): Self-contained executable fat-jar committed for zero-install Windows/Linux execution (`java -jar dist/flux-processor.jar <dir-or-zip>`).

## 2. Build, Test & Architectural Rules
1. **Java 17+ Compatible Bytecode (`--release 17`):**
   - Always compile with `javac --release 17 -encoding UTF-8 -Xlint:all,-serial -Werror` so `dist/flux-processor.jar` runs on both WPILib 2026 (JDK 17) and WPILib 2027 (JDK 25) workstations.
2. **Separation of Concerns (`FluxTray` Pulls Logs $\rightarrow$ `FluxProcessor` Analyzes & Renders):**
   - Robot log pulling over SFTP/SSH (`--pull-logs`, `--sync-pending`, and 3s Disabled auto-pull) is handled by `FluxTray.jar`. `FluxProcessor` consumes local run folders, `.zip` archives, or `student-auto-frames.json` files.
3. **Verification Command:**
   ```bash
   ./scripts/build.sh
   ```
