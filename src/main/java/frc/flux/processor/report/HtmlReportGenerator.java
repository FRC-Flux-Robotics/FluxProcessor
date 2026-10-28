package frc.flux.processor.report;

import frc.flux.processor.model.RunAnalysisReport;
import frc.flux.processor.model.RunAnalysisReport.BatteryMetrics;
import frc.flux.processor.model.RunAnalysisReport.CameraSnapshotRef;
import frc.flux.processor.model.RunAnalysisReport.CornerMetrics;
import frc.flux.processor.model.RunAnalysisReport.ElevatorCycleEvent;
import frc.flux.processor.model.RunAnalysisReport.FuelSimMetrics;
import frc.flux.processor.model.RunAnalysisReport.MechanismMetrics;
import frc.flux.processor.model.RunAnalysisReport.ShopStressGateMetrics;
import frc.flux.processor.model.RunAnalysisReport.ShotEvent;
import frc.flux.processor.model.RunAnalysisReport.ShotSimSummary;
import frc.flux.processor.model.RunAnalysisReport.Sim3DFrame;
import frc.flux.processor.model.RunAnalysisReport.SystemHealthMetrics;
import frc.flux.processor.model.RunAnalysisReport.SystemWatchDecision;
import frc.flux.processor.model.RunAnalysisReport.TimelinePoint;
import frc.flux.processor.model.RunAnalysisReport.VoiceCorrelation;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Locale;

/**
 * Generates self-contained 1-page HTML reports ({@code report.html}), season summary index ({@code
 * runs-index.html} and {@code runs-index.csv}), and Named Battery Leaderboard ({@code
 * batteries-health.csv}) styled in FRC Team 10413 (FLUX Robotics) official colors.
 */
public final class HtmlReportGenerator {

  private static final String LOGO_URL =
      "https://images.squarespace-cdn.com/content/v1/678bfe582830c94f0965d1b8/c581de14-67ff-442b-b36b-1959857c428e/Caitlin+B%26W+Logo+JPEG.jpg?format=1500w";

  private HtmlReportGenerator() {}

  /** Writes {@code report.html} for a single analyzed run. */
  public static void writeRunReport(RunAnalysisReport r, Path outputHtml) throws IOException {
    StringBuilder sb = new StringBuilder(32768);
    BatteryMetrics b = r.battery();

    String verdictBg =
        "CRITICAL".equals(r.overallVerdict())
            ? "#DC2626"
            : ("WARN".equals(r.overallVerdict()) ? "#D97706" : "#16A34A");

    sb.append("<!DOCTYPE html>\n<html lang=\"en\">\n<head>\n")
        .append("  <meta charset=\"UTF-8\">\n")
        .append("  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\n")
        .append("  <title>FLUX 10413 Report — ")
        .append(esc(r.runId()))
        .append("</title>\n")
        .append(
            "  <link href=\"https://fonts.googleapis.com/css2?family=Space+Grotesk:wght@400;600;700&display=swap\" rel=\"stylesheet\">\n")
        .append("  <style>\n")
        .append("    :root {\n")
        .append("      --black: #212020;\n")
        .append("      --royal: #254BA0;\n")
        .append("      --flux-blue: #2B7BFF;\n")
        .append("      --bg: #F2F3F5;\n")
        .append("      --card: #FFFFFF;\n")
        .append("      --silver: #E4E7EC;\n")
        .append("      --muted: #5A606E;\n")
        .append("    }\n")
        .append("    * { box-sizing: border-box; margin: 0; padding: 0; }\n")
        .append(
            "    body { font-family: 'Space Grotesk', system-ui, sans-serif; background: var(--bg); color: var(--black); padding: 20px; line-height: 1.45; }\n")
        .append("    .container { max-width: 1240px; margin: 0 auto; display: flex; flex-direction: column; gap: 18px; }\n")
        .append(
            "    .header { background: var(--card); border-top: 6px solid var(--royal); border-radius: 10px; padding: 16px 22px; display: flex; align-items: center; justify-content: space-between; box-shadow: 0 2px 6px rgba(0,0,0,0.06); }\n")
        .append("    .header-left { display: flex; align-items: center; gap: 16px; }\n")
        .append("    .logo { height: 48px; width: auto; }\n")
        .append("    .title { font-size: 20px; font-weight: 700; color: var(--black); }\n")
        .append("    .subtitle { font-size: 12px; color: var(--muted); margin-top: 2px; }\n")
        .append("    .badge { padding: 6px 14px; border-radius: 6px; font-weight: 700; font-size: 13px; color: #FFF; }\n")
        .append("    .kpis { display: grid; grid-template-columns: repeat(auto-fit, minmax(160px, 1fr)); gap: 12px; }\n")
        .append("    .kpi { background: var(--card); border-radius: 8px; padding: 12px 14px; border-left: 4px solid var(--royal); box-shadow: 0 1px 4px rgba(0,0,0,0.05); }\n")
        .append("    .kpi-label { font-size: 11px; text-transform: uppercase; letter-spacing: 0.04em; color: var(--muted); font-weight: 600; }\n")
        .append("    .kpi-val { font-size: 20px; font-weight: 700; margin-top: 4px; color: var(--black); }\n")
        .append("    .kpi-sub { font-size: 11px; color: var(--muted); margin-top: 2px; }\n")
        .append("    .alerts { background: var(--black); color: #FFF; border-radius: 8px; padding: 14px 18px; border-left: 5px solid var(--flux-blue); }\n")
        .append("    .alerts h3 { font-size: 13px; color: var(--flux-blue); text-transform: uppercase; margin-bottom: 6px; }\n")
        .append("    .alerts ul { margin-left: 18px; font-size: 13px; }\n")
        .append("    .alerts li { margin-bottom: 4px; }\n")
        .append("    .card { background: var(--card); border-radius: 10px; padding: 16px 20px; box-shadow: 0 1px 4px rgba(0,0,0,0.05); }\n")
        .append("    .card h2 { font-size: 16px; font-weight: 700; color: var(--black); margin-bottom: 10px; padding-bottom: 6px; border-bottom: 2px solid var(--silver); display: flex; justify-content: space-between; align-items: center; }\n")
        .append("    .grid-2 { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }\n")
        .append("    @media (max-width: 900px) { .grid-2 { grid-template-columns: 1fr; } }\n")
        .append("    table { width: 100%; border-collapse: collapse; font-size: 12px; }\n")
        .append("    th { background: var(--black); color: #FFF; text-align: left; padding: 8px 10px; font-weight: 600; }\n")
        .append("    td { padding: 7px 10px; border-bottom: 1px solid var(--silver); }\n")
        .append("    tr:nth-child(even) td { background: #F8F9FB; }\n")
        .append("    .pill { display: inline-block; padding: 2px 8px; border-radius: 4px; font-weight: 700; font-size: 10px; }\n")
        .append("    .pill-pass { background: #DCFCE7; color: #166534; }\n")
        .append("    .pill-warn { background: #FEF3C7; color: #92400E; }\n")
        .append("    .pill-fail { background: #FEE2E2; color: #991B1B; }\n")
        .append("    .btn-link { display: inline-block; background: var(--royal); color: #FFF; text-decoration: none; padding: 3px 8px; border-radius: 4px; font-size: 11px; font-weight: 600; }\n")
        .append("    .btn-link:hover { background: var(--flux-blue); }\n")
        .append("    canvas { width: 100%; height: 220px; background: #18181B; border-radius: 8px; display: block; }\n")
        .append("  </style>\n")
        .append("  <script src=\"https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.min.js\"></script>\n")
        .append("</head>\n<body>\n<div class=\"container\">\n");

    // Header
    sb.append("  <div class=\"header\">\n")
        .append("    <div class=\"header-left\">\n")
        .append("      <img class=\"logo\" src=\"").append(LOGO_URL).append("\" alt=\"FLUX Robotics\">\n")
        .append("      <div>\n")
        .append("        <div class=\"title\">")
        .append(esc(r.runId()))
        .append(" — Automated Post-Run Audit</div>\n")
        .append("        <div class=\"subtitle\">Robot: <strong>")
        .append(esc(r.robotName()))
        .append("</strong>  •  Mode: ")
        .append(esc(r.matchType()))
        .append(" (")
        .append(esc(r.allianceStation()))
        .append(")  •  Named Battery: <strong>")
        .append(esc(r.namedBattery()))
        .append("</strong>  •  Git: <code>")
        .append(esc(r.gitSha()))
        .append("</code>  •  Config: <code>")
        .append(esc(r.configSha()))
        .append("</code>  •  Merged Log: <code>merged.wpilog</code></div>\n")
        .append("      </div>\n")
        .append("    </div>\n")
        .append("    <div class=\"badge\" style=\"background:")
        .append(verdictBg)
        .append("\">VERDICT: ")
        .append(esc(r.overallVerdict()))
        .append("</div>\n")
        .append("  </div>\n");

    // Key Alerts Banner
    sb.append("  <div class=\"alerts\">\n")
        .append("    <h3>Key Audit Findings</h3>\n")
        .append("    <ul>\n");
    for (String alert : r.keyAlerts()) {
      sb.append("      <li>").append(esc(alert)).append("</li>\n");
    }
    sb.append("    </ul>\n  </div>\n");

    // KPI Cards
    int totalCycles = r.shots().size() + (r.elevatorCycles() != null ? r.elevatorCycles().size() : 0);
    String cyclesLabel =
        (r.elevatorCycles() != null && !r.elevatorCycles().isEmpty() && r.shots().isEmpty())
            ? "Elevator Cycles"
            : "Detected Shots / Cycles";

    sb.append("  <div class=\"kpis\">\n")
        .append(
            kpiCard(
                "Named Battery",
                b.batteryName(),
                String.format(
                    Locale.US, "Est. R_int: %.1f mΩ (%s)", b.estimatedRintMilliOhms(), b.healthGrade())))
        .append(
            kpiCard(
                "Min Battery Voltage",
                String.format(Locale.US, "%.2f V", b.minVoltage()),
                String.format(
                    Locale.US,
                    "Rest: %.2f V | Sag @ %.1fs",
                    b.restingVoltage(),
                    b.minVoltageTimeSec())))
        .append(
            kpiCard(
                "Peak Drive Stator",
                String.format(Locale.US, "%.0f A", b.peakDriveStatorAmps()),
                String.format(Locale.US, "Peak Supply: %.0f A", b.peakSupplyCurrentAmps())))
        .append(
            kpiCard(
                "Brownout / Dropouts",
                b.enableDropouts() + " dropouts",
                String.format(Locale.US, "< 6.8V for %.1fs", b.brownoutCriticalSeconds())))
        .append(
            kpiCard(
                "Distance & Speed",
                String.format(Locale.US, "%.1f m", r.totalDistanceMeters()),
                String.format(Locale.US, "Peak Speed: %.2f m/s", r.peakSpeedMps())))
        .append(
            kpiCard(
                cyclesLabel,
                String.valueOf(totalCycles),
                String.format(
                    Locale.US, "Enabled: %.1fs / %.1fs", r.enabledDurationSec(), r.totalDurationSec())));
    if (r.fuelSim() != null) {
      FuelSimMetrics fs = r.fuelSim();
      sb.append(
          kpiCard(
              "3D FuelSim Hub Score",
              fs.ballsScoredInHub() + " / " + fs.ballsLaunched() + " balls",
              String.format(
                  Locale.US,
                  "%.0f%% accuracy • %d floor intaked",
                  fs.hubScoringAccuracyPct(),
                  fs.floorBallsIntaked())));
    }
    sb.append("  </div>\n");

    // Interactive Timeline Canvas
    sb.append("  <div class=\"card\">\n")
        .append(
            "    <h2><span>Battery Voltage (V), Total Drive Stator Current (A) &amp; Shooter Speed (RPS)</span>")
        .append(
            "<span style=\"font-size:12px;font-weight:400;color:var(--muted)\"> Green = Voltage (V) • Blue = Total Stator Current (A) • Gold = Shot Events • Red Dashed = 6.8V Brownout</span></h2>\n")
        .append("    <canvas id=\"timelineCanvas\" width=\"1180\" height=\"220\"></canvas>\n")
        .append("  </div>\n");

    // Interactive 3D Match Replay & Game-Piece Visualizer Card
    if (r.fuelSim() != null && r.fuelSim().frames() != null && !r.fuelSim().frames().isEmpty()) {
      String mp4File = r.fuelSim().videoFileName();
      sb.append("  <div class=\"card\" id=\"sim3dCard\">\n")
          .append("    <h2><span>3D CAD Match Replay &amp; 2026 REBUILT FuelSim Arena (")
          .append(r.fuelSim().ballsScoredInHub())
          .append(" / ")
          .append(r.fuelSim().ballsLaunched())
          .append(" Scored in Hub)</span><span style=\"display:flex;gap:8px;align-items:center\">");
      if (mp4File != null && !mp4File.isEmpty()) {
        sb.append("<a class=\"btn-link\" style=\"background:#16A34A;padding:5px 10px\" download href=\"")
            .append(esc(mp4File))
            .append("\">⬇ Download ")
            .append(esc(mp4File))
            .append(" (1280x720 HD)</a>");
      }
      sb.append(
              "<button id=\"simRecBtn\" class=\"btn-link\" style=\"border:none;cursor:pointer;padding:5px 10px;background:#7C3AED\">🎬 Record Clip (.webm)</button>")
          .append("</span></h2>\n");
      if (mp4File != null && !mp4File.isEmpty()) {
        sb.append(
                "    <div style=\"margin-bottom:14px;\"><div style=\"font-size:12px;font-weight:700;color:var(--muted);margin-bottom:6px;\">🎬 Hardware-Accelerated MuJoCo 3.15 OpenGL/EGL 14-Mesh CAD Replay Video (4096×4096 Shadows + PiP OV9281 &amp; 3rd-Person Chase Cams):</div>\n")
            .append(
                "    <video controls preload=\"metadata\" poster=\"preview-3d.png\" style=\"width:100%;max-height:560px;border-radius:8px;background:#090D16;box-shadow:0 4px 14px rgba(0,0,0,0.22);\" src=\"")
            .append(esc(mp4File))
            .append("\"></video></div>\n");
      }
      sb.append(
              "    <div style=\"font-size:12px;font-weight:700;color:var(--muted);margin-bottom:6px;\">🕹 Interactive Three.js WebGL PBR 3D Arena &amp; Articulated CAD Robot Explorer (Drag to Orbit, Scroll to Zoom):</div>\n")
          .append(
              "    <div style=\"display:flex;flex-wrap:wrap;gap:10px;align-items:center;margin-bottom:10px;background:#F8F9FB;padding:8px 12px;border-radius:6px;border:1px solid var(--silver);font-size:12px;\">\n")
          .append(
              "      <button id=\"simPlayBtn\" class=\"btn-link\" style=\"border:none;cursor:pointer;padding:5px 12px;font-size:12px;\">▶ Play</button>\n")
          .append(
              "      <input id=\"simScrub\" type=\"range\" min=\"0\" max=\"100\" value=\"0\" style=\"flex:1;min-width:180px;cursor:pointer;\">\n")
          .append(
              "      <span id=\"simTimeLabel\" style=\"font-family:monospace;font-weight:700;min-width:110px;\">0.0s / 0.0s</span>\n")
          .append("      <label>Speed: <select id=\"simSpeed\" style=\"padding:3px 6px;border-radius:4px;\"><option value=\"0.5\">0.5x</option><option value=\"1\" selected>1x</option><option value=\"2\">2x</option><option value=\"4\">4x</option></select></label>\n")
          .append("      <label>Camera: <select id=\"simCamMode\" style=\"padding:3px 6px;border-radius:4px;\"><option value=\"iso\" selected>Cinema Orbit 3D (Drag to Rotate)</option><option value=\"side\">Broadcast Side</option><option value=\"top\">Top-Down Overhead</option><option value=\"follow\">3rd-Person Chase Cam</option></select></label>\n")
          .append("    </div>\n")
          .append(
              "    <div style=\"position:relative;\"><canvas id=\"sim3dCanvas\" width=\"1180\" height=\"500\" style=\"height:500px;background:#0b0f19;cursor:grab;border-radius:8px;\"></canvas>\n")
          .append(
              "    <div id=\"sim3dHudTop\" style=\"position:absolute;top:12px;left:14px;right:14px;display:flex;justify-content:space-between;pointer-events:none;font-family:monospace;font-size:13px;font-weight:700;color:#F8FAFC;background:rgba(10,15,28,0.82);padding:8px 14px;border-radius:8px;border:1px solid rgba(56,189,248,0.35);\"></div>\n")
          .append(
              "    <div id=\"sim3dHudBot\" style=\"position:absolute;bottom:12px;left:14px;right:14px;display:flex;justify-content:space-between;pointer-events:none;font-family:monospace;font-size:12px;font-weight:600;color:#CBD5E1;background:rgba(10,15,28,0.82);padding:7px 14px;border-radius:8px;border:1px solid rgba(148,163,184,0.25);\"></div></div>\n")
          .append("  </div>\n");
    }

    // 5m x 3m ShopStressSuiteCmd & VirtualGeofence Pass Gate Card (when present)
    if (r.shopStressGate() != null && r.shopStressGate().present()) {
      ShopStressGateMetrics sg = r.shopStressGate();
      String sgPill =
          "FAIL".equals(sg.overallGateStatus())
              ? "pill-fail"
              : ("WARN".equals(sg.overallGateStatus()) ? "pill-warn" : "pill-pass");
      sb.append("  <div class=\"card\">\n")
          .append(
              "    <h2><span>5m x 3m ShopStressSuiteCmd &amp; VirtualGeofence Pass Gate</span>")
          .append("<span class=\"pill ")
          .append(sgPill)
          .append("\">GATE: ")
          .append(esc(sg.overallGateStatus()))
          .append("</span></h2>\n")
          .append("    <table>\n")
          .append(
              "      <thead><tr><th>VirtualGeofence Breaches</th><th>Figure-8 Loop Closure Drift</th><th>Max Camera Handoff Step</th><th>Box-Step 4-Corner Spread</th><th>Overall Gate</th></tr></thead>\n")
          .append("      <tbody><tr>\n")
          .append("        <td><strong>")
          .append(sg.geofenceBreachCount())
          .append("</strong> (limit: 0)</td>\n")
          .append("        <td><strong>")
          .append(String.format(Locale.US, "%.2f cm", sg.figure8LoopClosureDriftCm()))
          .append("</strong> (&le; 4.0 cm)</td>\n")
          .append("        <td><strong>")
          .append(String.format(Locale.US, "%.2f cm", sg.maxCameraHandoffStepCm()))
          .append("</strong> (&le; 3.0 cm)</td>\n")
          .append("        <td><strong>")
          .append(String.format(Locale.US, "%.1f%%", sg.cornerCurrentSpreadPct()))
          .append("</strong> (&le; 12.0%)</td>\n")
          .append("        <td><span class=\"pill ")
          .append(sgPill)
          .append("\">")
          .append(esc(sg.overallGateStatus()))
          .append("</span></td>\n")
          .append("      </tr></tbody>\n    </table>\n")
          .append("  </div>\n");
    }

    // 6-Second Lookback Voice-to-Sensor Correlation Table
    sb.append("  <div class=\"card\">\n")
        .append(
            "    <h2><span>Part 3: Voice Feedback &amp; 6-Second Sensor Lookback Correlation</span>")
        .append(
            "<span style=\"font-size:12px;font-weight:400;color:var(--muted)\">Driver / Operator / Coach Tracks + Auto-Telemetry Checkpoints</span></h2>\n")
        .append("    <table>\n")
        .append(
            "      <thead><tr><th>Speaker / Role</th><th>Spoken Time</th><th>Spoken Comment / Trigger</th><th>6-Sec Lookback Event</th><th>Sensor Root-Cause Diagnosis</th><th>Video</th></tr></thead>\n")
        .append("      <tbody>\n");
    for (VoiceCorrelation vc : r.voiceCorrelations()) {
      String pillClass =
          "CRITICAL".equals(vc.severity())
              ? "pill-fail"
              : ("WARN".equals(vc.severity()) ? "pill-warn" : "pill-pass");
      sb.append("        <tr>\n")
          .append("          <td><span class=\"pill ")
          .append(pillClass)
          .append("\">")
          .append(esc(vc.speakerRole()))
          .append("</span></td>\n")
          .append("          <td><strong>")
          .append(String.format(Locale.US, "%.1fs", vc.spokenTimeSec()))
          .append("</strong></td>\n")
          .append("          <td>\"")
          .append(esc(vc.transcript()))
          .append("\"</td>\n")
          .append("          <td>")
          .append(esc(vc.correlatedEventSummary()))
          .append("</td>\n")
          .append("          <td>")
          .append(esc(vc.sensorDiagnosis()))
          .append("</td>\n")
          .append("          <td>")
          .append(
              vc.videoLink() != null && !vc.videoLink().isEmpty()
                  ? "<a class=\"btn-link\" target=\"_blank\" href=\""
                      + esc(vc.videoLink())
                      + "\">▶ Watch</a>"
                  : "—")
          .append("</td>\n")
          .append("        </tr>\n");
    }
    sb.append("      </tbody>\n    </table>\n  </div>\n");

    // 2-Column Grid: 4-Corner Swerve Symmetry + Mechanism Audit
    sb.append("  <div class=\"grid-2\">\n");

    // Left: 4-Corner Swerve Symmetry
    sb.append("    <div class=\"card\">\n")
        .append("      <h2>4-Corner Swerve Symmetry Audit</h2>\n")
        .append("      <table>\n")
        .append(
            "        <thead><tr><th>Corner</th><th>CAN IDs</th><th>Mean Drive (A)</th><th>Peak Drive (A)</th><th>Peak Vel (RPS)</th><th>Mean Steer (A)</th><th>Asymmetry</th><th>Status</th></tr></thead>\n")
        .append("        <tbody>\n");
    for (CornerMetrics c : r.swerveCorners()) {
      String pill =
          "FAIL".equals(c.status())
              ? "pill-fail"
              : ("WARN".equals(c.status()) ? "pill-warn" : "pill-pass");
      sb.append("          <tr>\n")
          .append("            <td><strong>")
          .append(esc(c.corner()))
          .append("</strong></td>\n")
          .append("            <td>D:")
          .append(c.driveId())
          .append(" / S:")
          .append(c.steerId())
          .append("</td>\n")
          .append("            <td>")
          .append(String.format(Locale.US, "%.1f A", c.meanDriveStatorAmps()))
          .append("</td>\n")
          .append("            <td>")
          .append(String.format(Locale.US, "%.0f A", c.peakDriveStatorAmps()))
          .append("</td>\n")
          .append("            <td>")
          .append(String.format(Locale.US, "%.1f", c.peakDriveVelocityRps()))
          .append("</td>\n")
          .append("            <td>")
          .append(String.format(Locale.US, "%.1f A", c.meanSteerStatorAmps()))
          .append("</td>\n")
          .append("            <td>")
          .append(String.format(Locale.US, "%+.1f%%", c.driveAsymmetryPct()))
          .append("</td>\n")
          .append("            <td><span class=\"pill ")
          .append(pill)
          .append("\">")
          .append(esc(c.status()))
          .append("</span></td>\n")
          .append("          </tr>\n");
    }
    sb.append("        </tbody>\n      </table>\n    </div>\n");

    // Right: Mechanism Health Audit
    sb.append("    <div class=\"card\">\n")
        .append("      <h2>Mechanism Health Audit</h2>\n")
        .append("      <table>\n")
        .append(
            "        <thead><tr><th>Mechanism</th><th>ID</th><th>Mean (A)</th><th>Peak (A)</th><th>Peak Vel (RPS)</th><th>Pos Range</th><th>Resets</th><th>Status</th></tr></thead>\n")
        .append("        <tbody>\n");
    for (MechanismMetrics m : r.mechanisms()) {
      String pill = "WARN".equals(m.status()) ? "pill-warn" : "pill-pass";
      sb.append("          <tr>\n")
          .append("            <td><strong>")
          .append(esc(m.name()))
          .append("</strong></td>\n")
          .append("            <td>")
          .append(m.motorId())
          .append("</td>\n")
          .append("            <td>")
          .append(String.format(Locale.US, "%.1f A", m.meanStatorAmps()))
          .append("</td>\n")
          .append("            <td>")
          .append(String.format(Locale.US, "%.0f A", m.peakStatorAmps()))
          .append("</td>\n")
          .append("            <td>")
          .append(String.format(Locale.US, "%.1f", m.peakVelocityRps()))
          .append("</td>\n")
          .append("            <td>")
          .append(
              String.format(Locale.US, "[%.1f .. %.1f]", m.minPositionRot(), m.maxPositionRot()))
          .append("</td>\n")
          .append("            <td>")
          .append(m.resetCount())
          .append("</td>\n")
          .append("            <td><span class=\"pill ")
          .append(pill)
          .append("\">")
          .append(esc(m.status()))
          .append("</span></td>\n")
          .append("          </tr>\n");
    }
    sb.append("        </tbody>\n      </table>\n    </div>\n  </div>\n");

    // 2027 Elevator + Arm Scoring Cycles Table (when present)
    if (r.elevatorCycles() != null && !r.elevatorCycles().isEmpty()) {
      sb.append("  <div class=\"card\">\n")
          .append("    <h2><span>2027 Elevator + Arm Scoring Cycles (")
          .append(r.elevatorCycles().size())
          .append(
              ")</span><span style=\"font-size:12px;font-weight:400;color:var(--muted)\">L1..L4 Extension &amp; Keep-Out Interlock Audit</span></h2>\n")
          .append("    <table>\n")
          .append(
              "      <thead><tr><th>Cycle #</th><th>Level</th><th>Start (s)</th><th>Duration</th><th>Peak Elevator (rot)</th><th>Peak Arm (rot)</th><th>Peak Stator (A)</th><th>Min Battery (V)</th><th>Keep-Out Interlock</th><th>Status</th></tr></thead>\n")
          .append("      <tbody>\n");
      for (ElevatorCycleEvent ec : r.elevatorCycles()) {
        String pill = "NOMINAL".equals(ec.status()) ? "pill-pass" : "pill-warn";
        sb.append("        <tr>\n")
            .append("          <td><strong>#")
            .append(ec.cycleNumber())
            .append("</strong></td>\n")
            .append("          <td><strong>")
            .append(esc(ec.targetLevel()))
            .append("</strong></td>\n")
            .append("          <td>")
            .append(String.format(Locale.US, "%.1fs", ec.startTimeSec()))
            .append("</td>\n")
            .append("          <td>")
            .append(String.format(Locale.US, "%.2fs", ec.durationSec()))
            .append("</td>\n")
            .append("          <td>")
            .append(String.format(Locale.US, "%.1f", ec.peakElevatorPosRot()))
            .append("</td>\n")
            .append("          <td>")
            .append(String.format(Locale.US, "%.2f", ec.peakArmPosRot()))
            .append("</td>\n")
            .append("          <td>")
            .append(String.format(Locale.US, "%.1f A", ec.peakStatorAmps()))
            .append("</td>\n")
            .append("          <td>")
            .append(String.format(Locale.US, "%.2f V", ec.minBatteryVoltage()))
            .append("</td>\n")
            .append("          <td>")
            .append(ec.keepOutInterlockTriggered() ? "TRIGGERED" : "CLEAR")
            .append("</td>\n")
            .append("          <td><span class=\"pill ")
            .append(pill)
            .append("\">")
            .append(esc(ec.status()))
            .append("</span></td>\n")
            .append("        </tr>\n");
      }
      sb.append("      </tbody>\n    </table>\n  </div>\n");
    }

    // Shot-by-Shot Table
    sb.append("  <div class=\"card\">\n")
        .append("    <h2><span>Detected Shot / Feed Events (")
        .append(r.shots().size())
        .append(
            ")</span><span style=\"font-size:12px;font-weight:400;color:var(--muted)\">Triggered when Feeder + Shooter Flywheels are active simultaneously • Replayable in 3D via /FuelSim/Fuels</span></h2>\n")
        .append("    <table>\n")
        .append(
            "      <thead><tr><th>Shot #</th><th>Time (s)</th><th>Duration</th><th>Shooter Left (RPS)</th><th>Shooter Right (RPS)</th><th>L/R Δ (RPS)</th><th>Hood Pos (rot)</th><th>Feeder Peak (A)</th><th>Min Battery (V)</th><th>Status</th><th>3D FuelSim Outcome</th><th>Video</th></tr></thead>\n")
        .append("      <tbody>\n");
    for (ShotEvent s : r.shots()) {
      String pill = "NOMINAL".equals(s.status()) ? "pill-pass" : "pill-warn";
      ShotSimSummary simSum = null;
      if (r.fuelSim() != null && r.fuelSim().shotSummaries() != null) {
        for (ShotSimSummary ss : r.fuelSim().shotSummaries()) {
          if (ss.shotNumber() == s.shotNumber()) {
            simSum = ss;
            break;
          }
        }
      }
      String simPill =
          simSum == null
              ? "pill-warn"
              : (simSum.outcome().startsWith("SCORED")
                  ? "pill-pass"
                  : (simSum.outcome().startsWith("PARTIAL") ? "pill-warn" : "pill-fail"));
      String simText =
          simSum == null
              ? "—"
              : String.format(
                  Locale.US,
                  "<span class=\"pill %s\">%s</span> <span style=\"color:var(--muted);font-size:11px\">(%.2fm, %.1fm/s)</span>",
                  simPill,
                  esc(simSum.outcome()),
                  simSum.launchDistanceM(),
                  simSum.launchSpeedMps());
      sb.append("        <tr>\n")
          .append("          <td><strong>#")
          .append(s.shotNumber())
          .append("</strong></td>\n")
          .append("          <td>")
          .append(String.format(Locale.US, "%.1fs", s.timestampSec()))
          .append("</td>\n")
          .append("          <td>")
          .append(String.format(Locale.US, "%.2fs", s.durationSec()))
          .append("</td>\n")
          .append("          <td>")
          .append(String.format(Locale.US, "%.1f", s.shooterLeftRps()))
          .append("</td>\n")
          .append("          <td>")
          .append(String.format(Locale.US, "%.1f", s.shooterRightRps()))
          .append("</td>\n")
          .append("          <td>")
          .append(String.format(Locale.US, "%.1f", s.shooterAsymmetryRps()))
          .append("</td>\n")
          .append("          <td>")
          .append(String.format(Locale.US, "%.2f", s.hoodPositionRot()))
          .append("</td>\n")
          .append("          <td>")
          .append(String.format(Locale.US, "%.1f A", s.feederPeakStatorAmps()))
          .append("</td>\n")
          .append("          <td>")
          .append(String.format(Locale.US, "%.2f V", s.minBatteryVoltageDuringShot()))
          .append("</td>\n")
          .append("          <td><span class=\"pill ")
          .append(pill)
          .append("\">")
          .append(esc(s.status()))
          .append("</span></td>\n")
          .append("          <td>")
          .append(simText)
          .append("</td>\n")
          .append("          <td style=\"display:flex;gap:5px;align-items:center\">")
          .append(
              String.format(
                  Locale.US,
                  "<button class=\"btn-link\" style=\"border:none;cursor:pointer\" onclick=\"if(window.seekSim3D)window.seekSim3D(%.2f)\">🎯 3D</button>",
                  Math.max(0.0, s.timestampSec() - 0.5)))
          .append(
              s.videoLink() != null && !s.videoLink().isEmpty()
                  ? "<a class=\"btn-link\" target=\"_blank\" href=\""
                      + esc(s.videoLink())
                      + "\">▶ Clip</a>"
                  : "")
          .append("</td>\n")
          .append("        </tr>\n");
    }
    sb.append("      </tbody>\n    </table>\n  </div>\n");

    // Event-Triggered Camera Snapshots Card (when present)
    if (r.cameraSnapshots() != null && !r.cameraSnapshots().isEmpty()) {
      sb.append("  <div class=\"card\">\n")
          .append("    <h2><span>Event-Triggered Camera Snapshots (")
          .append(r.cameraSnapshots().size())
          .append(
              ")</span><span style=\"font-size:12px;font-weight:400;color:var(--muted)\">On-robot cam-snapshots/*.jpg correlated to match timeline</span></h2>\n")
          .append("    <table>\n")
          .append(
              "      <thead><tr><th>Timestamp</th><th>Camera</th><th>Trigger Reason</th><th>File</th><th>Preview</th></tr></thead>\n")
          .append("      <tbody>\n");
      for (CameraSnapshotRef snap : r.cameraSnapshots()) {
        sb.append("        <tr>\n")
            .append("          <td><strong>")
            .append(String.format(Locale.US, "%.2fs", snap.timestampSec()))
            .append("</strong></td>\n")
            .append("          <td>")
            .append(esc(snap.cameraName()))
            .append("</td>\n")
            .append("          <td><span class=\"pill pill-pass\">")
            .append(esc(snap.triggerReason()))
            .append("</span></td>\n")
            .append("          <td><code>")
            .append(esc(snap.fileName()))
            .append("</code></td>\n")
            .append("          <td><a class=\"btn-link\" target=\"_blank\" href=\"")
            .append(esc(snap.relativePath()))
            .append("\">View Frame</a></td>\n")
            .append("        </tr>\n");
      }
      sb.append("      </tbody>\n    </table>\n  </div>\n");
    }

    // Offline SystemCore / Controller Watch Metrics & Actionable Decisions Card
    if (r.systemHealth() != null) {
      SystemHealthMetrics sh = r.systemHealth();
      sb.append("  <div class=\"card\">\n")
          .append("    <h2><span>SystemCore &amp; Controller Offline Watch Audit (")
          .append(esc(sh.controllerType()))
          .append(")</span><span style=\"font-size:12px;font-weight:400;color:var(--muted)\">")
          .append(
              String.format(
                  Locale.US,
                  "Loop p50=%.2fms / p99=%.2fms • CPU=%.1f%% (Java %.1f%% / Vision %.1f%%) • Temp=%.1f°C • CAN Drops=%d • Free=%.0fMB",
                  sh.loopMedianMs(),
                  sh.loopP99Ms(),
                  sh.totalCpuPct(),
                  sh.javaProcessCpuPct(),
                  sh.visionDaemonCpuPct(),
                  sh.cpuTempC(),
                  sh.socketCanDropsAndOverruns(),
                  sh.freeStorageMb()))
          .append("</span></h2>\n");
      if (sh.watchDecisions().isEmpty()) {
        sb.append(
            "    <div style=\"font-size:13px;color:#166534;font-weight:600;\">All offline SystemCore / Controller watch metrics are within nominal thresholds (0 warnings).</div>\n");
      } else {
        sb.append("    <table>\n")
            .append(
                "      <thead><tr><th>Severity</th><th>Category</th><th>Watch Metric</th><th>Measured Value</th><th>Threshold</th><th>Actionable Engineering / Pit Decision</th></tr></thead>\n")
            .append("      <tbody>\n");
        for (SystemWatchDecision wd : sh.watchDecisions()) {
          String pill = "CRITICAL".equals(wd.severity()) ? "pill-fail" : "pill-warn";
          sb.append("        <tr>\n")
              .append("          <td><span class=\"pill ")
              .append(pill)
              .append("\">")
              .append(esc(wd.severity()))
              .append("</span></td>\n")
              .append("          <td>")
              .append(esc(wd.category()))
              .append("</td>\n")
              .append("          <td><strong>")
              .append(esc(wd.metricName()))
              .append("</strong></td>\n")
              .append("          <td><code>")
              .append(esc(wd.measuredValue()))
              .append("</code></td>\n")
              .append("          <td>")
              .append(esc(wd.threshold()))
              .append("</td>\n")
              .append("          <td>")
              .append(esc(wd.actionableDecision()))
              .append("</td>\n")
              .append("        </tr>\n");
        }
        sb.append("      </tbody>\n    </table>\n");
      }
      sb.append("  </div>\n");
    }

    // Inline JS to draw timeline canvas
    sb.append("</div>\n<script>\nconst timeline = [\n");
    for (int i = 0; i < r.timeline().size(); i++) {
      TimelinePoint pt = r.timeline().get(i);
      sb.append(
          String.format(
              Locale.US,
              "[%.2f,%.2f,%.1f,%.1f,%d]%s",
              pt.timeSec(),
              pt.batteryVoltage(),
              pt.totalDriveStatorAmps(),
              pt.shooterLeftRps(),
              pt.enabled() ? 1 : 0,
              (i + 1 < r.timeline().size()) ? "," : ""));
    }
    sb.append("\n];\nconst shots = [");
    for (int i = 0; i < r.shots().size(); i++) {
      sb.append(
          String.format(
              Locale.US,
              "%.2f%s",
              r.shots().get(i).timestampSec(),
              (i + 1 < r.shots().size()) ? "," : ""));
    }
    sb.append("];\n")
        .append("(function(){\n")
        .append("  const c = document.getElementById('timelineCanvas');\n")
        .append("  if (!c || timeline.length < 2) return;\n")
        .append("  const ctx = c.getContext('2d');\n")
        .append("  const W = c.width, H = c.height, padL = 48, padR = 48, padT = 18, padB = 26;\n")
        .append("  const plotW = W - padL - padR, plotH = H - padT - padB;\n")
        .append("  const maxT = timeline[timeline.length - 1][0] || 1;\n")
        .append("  const xOf = t => padL + (t / maxT) * plotW;\n")
        .append("  const yVolt = v => padT + (1 - (Math.max(4, Math.min(14, v)) - 4) / 10) * plotH;\n")
        .append("  const yAmps = a => padT + (1 - Math.min(600, a) / 600) * plotH;\n")
        .append("  // Draw 6.8V brownout line\n")
        .append("  ctx.strokeStyle = '#EF4444'; ctx.setLineDash([5, 5]); ctx.lineWidth = 1;\n")
        .append("  ctx.beginPath(); ctx.moveTo(padL, yVolt(6.8)); ctx.lineTo(W - padR, yVolt(6.8)); ctx.stroke();\n")
        .append("  ctx.setLineDash([]);\n")
        .append("  // Draw Total Drive Stator Amps (Blue)\n")
        .append("  ctx.strokeStyle = '#2B7BFF'; ctx.lineWidth = 1.5; ctx.beginPath();\n")
        .append(
            "  timeline.forEach((p, i) => { const x = xOf(p[0]), y = yAmps(p[2]); if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y); });\n")
        .append("  ctx.stroke();\n")
        .append("  // Draw Battery Voltage (Green)\n")
        .append("  ctx.strokeStyle = '#22C55E'; ctx.lineWidth = 2; ctx.beginPath();\n")
        .append(
            "  timeline.forEach((p, i) => { const x = xOf(p[0]), y = yVolt(p[1]); if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y); });\n")
        .append("  ctx.stroke();\n")
        .append("  // Draw Shot Markers (Gold)\n")
        .append("  ctx.fillStyle = '#F59E0B';\n")
        .append(
            "  shots.forEach(t => { const x = xOf(t); ctx.beginPath(); ctx.arc(x, padT + 8, 4, 0, Math.PI*2); ctx.fill(); });\n")
        .append("  // Axes labels\n")
        .append("  ctx.fillStyle = '#A1A1AA'; ctx.font = '11px sans-serif';\n")
        .append(
            "  ctx.fillText('14V', 12, yVolt(14)+4); ctx.fillText('10V', 12, yVolt(10)+4); ctx.fillText('6.8V', 8, yVolt(6.8)+4);\n")
        .append(
            "  ctx.fillText('600A', W-42, yAmps(600)+4); ctx.fillText('300A', W-42, yAmps(300)+4); ctx.fillText('0A', W-42, yAmps(0)+4);\n")
        .append("})();\n");

    // Inline 3D FuelSim Arena Canvas Player & Browser WebM Video Recorder
    if (r.fuelSim() != null && r.fuelSim().frames() != null && !r.fuelSim().frames().isEmpty()) {
      boolean isBlue =
          r.allianceStation() == null
              || !r.allianceStation().toLowerCase(Locale.ROOT).startsWith("red");
      List<Sim3DFrame> frames = r.fuelSim().frames();
      sb.append("const sim3dFrames = [\n");
      for (int i = 0; i < frames.size(); i++) {
        Sim3DFrame sf = frames.get(i);
        sb.append(
            String.format(
                Locale.US,
                "[%.2f,%.2f,%.2f,%.3f,%.2f,%.0f,%.1f,%.2f,%.2f,%.2f,%d,%d,%d,\"%s\",[",
                sf.timeSec(),
                sf.robotX(),
                sf.robotY(),
                sf.robotHeadingRad(),
                sf.batteryVoltage(),
                sf.driveStatorAmps(),
                sf.shooterRps(),
                sf.hoodRot(),
                sf.elevatorHeightM(),
                sf.armAngleRad(),
                sf.blueScore(),
                sf.redScore(),
                sf.intakedCount(),
                escJs(sf.lastShotBanner())));
        double[] bxyz = sf.ballXyz();
        if (bxyz != null) {
          for (int bIdx = 0; bIdx < bxyz.length; bIdx++) {
            sb.append(
                String.format(
                    Locale.US, "%.2f%s", bxyz[bIdx], (bIdx + 1 < bxyz.length) ? "," : ""));
          }
        }
        sb.append("]]").append((i + 1 < frames.size()) ? ",\n" : "\n");
      }
      sb.append("];\n")
          .append("(function(){\n")
          .append("  const c = document.getElementById('sim3dCanvas');\n")
          .append("  if (!c || sim3dFrames.length === 0) return;\n")
          .append("  const playBtn = document.getElementById('simPlayBtn');\n")
          .append("  const scrub = document.getElementById('simScrub');\n")
          .append("  const timeLbl = document.getElementById('simTimeLabel');\n")
          .append("  const speedSel = document.getElementById('simSpeed');\n")
          .append("  const camSel = document.getElementById('simCamMode');\n")
          .append("  const recBtn = document.getElementById('simRecBtn');\n")
          .append("  const hudTop = document.getElementById('sim3dHudTop');\n")
          .append("  const hudBot = document.getElementById('sim3dHudBot');\n")
          .append("  const isBlue = ").append(isBlue ? "true" : "false").append(";\n")
          .append("  scrub.max = sim3dFrames.length - 1;\n")
          .append("  let idx = 0, playing = false, lastTs = 0, camYaw = isBlue ? -2.25 : -0.89, camPitch = 0.46, camDist = 5.6, dragging = false, dragX = 0, dragY = 0;\n")
          .append("  c.addEventListener('mousedown', e => { dragging = true; dragX = e.clientX; dragY = e.clientY; c.style.cursor='grabbing'; });\n")
          .append("  window.addEventListener('mouseup', () => { dragging = false; c.style.cursor='grab'; });\n")
          .append("  window.addEventListener('mousemove', e => { if(!dragging) return; camYaw -= (e.clientX - dragX)*0.007; camPitch = Math.max(0.14, Math.min(1.48, camPitch + (e.clientY - dragY)*0.007)); dragX = e.clientX; dragY = e.clientY; draw(); });\n")
          .append("  c.addEventListener('wheel', e => { e.preventDefault(); camDist = Math.max(2.2, Math.min(16.0, camDist + e.deltaY * 0.004)); draw(); }, {passive:false});\n")
          .append("  camSel.addEventListener('change', () => { if(camSel.value==='side'){camYaw=-Math.PI/2;camPitch=0.45;camDist=9.5;} else if(camSel.value==='top'){camYaw=-Math.PI/2;camPitch=1.50;camDist=11.5;} else if(camSel.value==='follow'){camDist=3.2;camPitch=0.42;} else {camYaw=isBlue?-2.25:-0.89;camPitch=0.46;camDist=5.6;} draw(); });\n")
          .append("  // Initialize Three.js WebGL PBR + Shadow-Mapped 3D Scene (if THREE loaded)\n")
          .append("  let glRenderer = null, scene = null, camera = null, robotGroup = null, tilterGroup = null, hoodMesh = null, elevGroup = null, armGroup = null, ballMeshes = [], trailMeshes = [];\n")
          .append("  if (window.THREE) {\n")
          .append("    try {\n")
          .append("      const THREE = window.THREE;\n")
          .append("      glRenderer = new THREE.WebGLRenderer({ canvas: c, antialias: true, preserveDrawingBuffer: true });\n")
          .append("      glRenderer.setSize(c.width, c.height, false);\n")
          .append("      glRenderer.shadowMap.enabled = true;\n")
          .append("      glRenderer.shadowMap.type = THREE.PCFSoftShadowMap;\n")
          .append("      glRenderer.toneMapping = THREE.ACESFilmicToneMapping;\n")
          .append("      glRenderer.toneMappingExposure = 1.12;\n")
          .append("      scene = new THREE.Scene();\n")
          .append("      scene.background = new THREE.Color(0x0b0f19);\n")
          .append("      scene.fog = new THREE.FogExp2(0x0b0f19, 0.025);\n")
          .append("      camera = new THREE.PerspectiveCamera(46, c.width / c.height, 0.05, 50);\n")
          .append("      camera.up.set(0, 0, 1);\n")
          .append("      scene.add(new THREE.HemisphereLight(0xdbeafe, 0x1e293b, 0.75));\n")
          .append("      const dirLight = new THREE.DirectionalLight(0xffffff, 1.35);\n")
          .append("      dirLight.position.set(5.5, 2.2, 8.5);\n")
          .append("      dirLight.castShadow = true;\n")
          .append("      dirLight.shadow.mapSize.width = 2048; dirLight.shadow.mapSize.height = 2048;\n")
          .append("      dirLight.shadow.camera.near = 1; dirLight.shadow.camera.far = 22;\n")
          .append("      dirLight.shadow.camera.left = -10; dirLight.shadow.camera.right = 10; dirLight.shadow.camera.top = 6; dirLight.shadow.camera.bottom = -6;\n")
          .append("      dirLight.target.position.set(8.27, 4.035, 0);\n")
          .append("      scene.add(dirLight); scene.add(dirLight.target);\n")
          .append("      // Field Carpet & Subfloor\n")
          .append("      const L = 16.541, W = 8.069;\n")
          .append("      const carpet = new THREE.Mesh(new THREE.BoxGeometry(L, W, 0.02), new THREE.MeshStandardMaterial({color: 0x262b36, roughness: 0.85, metalness: 0.08}));\n")
          .append("      carpet.position.set(L/2, W/2, -0.01); carpet.receiveShadow = true; scene.add(carpet);\n")
          .append("      const lineMatW = new THREE.MeshBasicMaterial({color: 0xf8fafc});\n")
          .append("      const lineMatB = new THREE.MeshBasicMaterial({color: 0x3b82f6});\n")
          .append("      const lineMatR = new THREE.MeshBasicMaterial({color: 0xef4444});\n")
          .append("      [[L/2, lineMatW],[4.626, lineMatB],[11.915, lineMatR],[1.90, lineMatW],[14.64, lineMatW]].forEach(p => {\n")
          .append("        const ln = new THREE.Mesh(new THREE.BoxGeometry(0.05, W, 0.004), p[1]); ln.position.set(p[0], W/2, 0.002); scene.add(ln);\n")
          .append("      });\n")
          .append("      // Polycarbonate Perimeter Guardrails & Driver Station Walls\n")
          .append("      const matSteel = new THREE.MeshStandardMaterial({color: 0x334155, metalness: 0.7, roughness: 0.35});\n")
          .append("      const matAlum = new THREE.MeshStandardMaterial({color: 0xcbd5e1, metalness: 0.8, roughness: 0.25});\n")
          .append("      const matGlass = new THREE.MeshPhysicalMaterial({color: 0xdbeafe, transparent: true, opacity: 0.26, roughness: 0.1, metalness: 0.1});\n")
          .append("      [-0.03, W+0.03].forEach(wy => {\n")
          .append("        const b = new THREE.Mesh(new THREE.BoxGeometry(L, 0.06, 0.12), matSteel); b.position.set(L/2, wy, 0.06); scene.add(b);\n")
          .append("        const g = new THREE.Mesh(new THREE.BoxGeometry(L, 0.02, 0.36), matGlass); g.position.set(L/2, wy, 0.30); scene.add(g);\n")
          .append("      });\n")
          .append("      // Build 3D Octagonal Hubs (Blue & Red) + Trenches + Center Climb Tower\n")
          .append("      function addHub3D(hx, hy, colHex) {\n")
          .append("        const hg = new THREE.Group(); hg.position.set(hx, hy, 0);\n")
          .append("        const matHub = new THREE.MeshStandardMaterial({color: colHex, metalness: 0.4, roughness: 0.4});\n")
          .append("        const matGold = new THREE.MeshStandardMaterial({color: 0xfacc15, metalness: 0.65, roughness: 0.25});\n")
          .append("        const base = new THREE.Mesh(new THREE.CylinderGeometry(0.46, 0.46, 1.05, 8), matHub);\n")
          .append("        base.rotation.x = Math.PI/2; base.position.z = 0.525; base.castShadow = true; base.receiveShadow = true; hg.add(base);\n")
          .append("        const funnel = new THREE.Mesh(new THREE.CylinderGeometry(0.58, 0.46, 0.60, 8, 1, true), new THREE.MeshPhysicalMaterial({color: colHex, transparent: true, opacity: 0.48, side: THREE.DoubleSide, roughness: 0.2}));\n")
          .append("        funnel.rotation.x = Math.PI/2; funnel.position.z = 1.35; hg.add(funnel);\n")
          .append("        const rim = new THREE.Mesh(new THREE.TorusGeometry(0.58, 0.032, 12, 8), matGold);\n")
          .append("        rim.position.z = 1.65; rim.castShadow = true; hg.add(rim);\n")
          .append("        scene.add(hg);\n")
          .append("      }\n")
          .append("      addHub3D(4.626, 4.035, 0x2563eb); addHub3D(11.915, 4.035, 0xdc2626);\n")
          .append("      // Build Articulated 3D CAD Robot Assembly\n")
          .append("      robotGroup = new THREE.Group();\n")
          .append("      const bumperMat = new THREE.MeshStandardMaterial({color: isBlue ? 0x1d4ed8 : 0xb91c1c, roughness: 0.45, metalness: 0.2});\n")
          .append("      const bumpers = new THREE.Mesh(new THREE.BoxGeometry(0.86, 0.86, 0.13), bumperMat);\n")
          .append("      bumpers.position.z = 0.09; bumpers.castShadow = true; robotGroup.add(bumpers);\n")
          .append("      const frame = new THREE.Mesh(new THREE.BoxGeometry(0.68, 0.68, 0.10), matAlum);\n")
          .append("      frame.position.z = 0.11; frame.castShadow = true; robotGroup.add(frame);\n")
          .append("      const shooter = new THREE.Mesh(new THREE.BoxGeometry(0.28, 0.36, 0.26), new THREE.MeshStandardMaterial({color: 0xfbbf24, metalness: 0.7, roughness: 0.25}));\n")
          .append("      shooter.position.set(0.14, 0, 0.29); shooter.castShadow = true; robotGroup.add(shooter);\n")
          .append("      hoodMesh = new THREE.Mesh(new THREE.BoxGeometry(0.18, 0.34, 0.06), new THREE.MeshStandardMaterial({color: 0x9333ea, metalness: 0.5, roughness: 0.3}));\n")
          .append("      hoodMesh.position.set(0.16, 0, 0.44); robotGroup.add(hoodMesh);\n")
          .append("      tilterGroup = new THREE.Group(); tilterGroup.position.set(-0.32, 0, 0.18);\n")
          .append("      const tilterArm = new THREE.Mesh(new THREE.BoxGeometry(0.26, 0.54, 0.06), new THREE.MeshStandardMaterial({color: 0xf97316, metalness: 0.5, roughness: 0.35}));\n")
          .append("      tilterArm.position.set(-0.12, 0, 0); tilterGroup.add(tilterArm);\n")
          .append("      const roller = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.04, 0.52, 16), new THREE.MeshStandardMaterial({color: 0x10b981}));\n")
          .append("      roller.position.set(-0.24, 0, 0); tilterGroup.add(roller); robotGroup.add(tilterGroup);\n")
          .append("      scene.add(robotGroup);\n")
          .append("      // 36 PBR Fuel Spheres + 24 Glowing Trail Spheres\n")
          .append("      const ballGeo = new THREE.SphereGeometry(0.0889, 20, 16);\n")
          .append("      const ballMat = new THREE.MeshStandardMaterial({color: 0xfacc15, roughness: 0.32, metalness: 0.15});\n")
          .append("      for(let i=0;i<36;i++){ const m = new THREE.Mesh(ballGeo, ballMat); m.castShadow = true; m.visible = false; scene.add(m); ballMeshes.push(m); }\n")
          .append("      const trGeo = new THREE.SphereGeometry(0.032, 10, 10);\n")
          .append("      const trMat = new THREE.MeshBasicMaterial({color: 0x38bdf8, transparent: true, opacity: 0.65});\n")
          .append("      for(let i=0;i<24;i++){ const m = new THREE.Mesh(trGeo, trMat); m.visible = false; scene.add(m); trailMeshes.push(m); }\n")
          .append("    } catch (e) { glRenderer = null; }\n")
          .append("  }\n")
          .append("  function draw(){\n")
          .append("    const f = sim3dFrames[idx]; if(!f) return;\n")
          .append("    const maxT = sim3dFrames[sim3dFrames.length-1][0];\n")
          .append("    timeLbl.textContent = f[0].toFixed(1) + 's / ' + maxT.toFixed(1) + 's';\n")
          .append("    scrub.value = idx;\n")
          .append("    if (hudTop) hudTop.innerHTML = '<span><strong style=\"color:#60A5FA\">BLUE HUB: '+f[10]+'</strong> &nbsp;•&nbsp; <span style=\"color:#F87171\">RED HUB: '+f[11]+'</span> &nbsp;•&nbsp; <span style=\"color:#FACC15\">INTAKED: '+f[12]+'</span></span><span>'+(f[13] ? '<span style=\"color:'+(f[13].indexOf('SCORED')>=0?'#4ADE80':'#FCA5A5')+'\">'+f[13]+'</span>' : 'T = '+f[0].toFixed(1)+'s')+'</span>';\n")
          .append("    if (hudBot) hudBot.innerHTML = '<span style=\"color:'+(f[4]<6.8?'#F87171':'#4ADE80')+'\">BATTERY: '+f[4].toFixed(2)+' V'+(f[4]<6.8?' [BROWNOUT!]':'')+'</span><span>DRIVE STATOR: '+f[5]+' A &nbsp;•&nbsp; SHOOTER: '+f[6].toFixed(1)+' RPS &nbsp;•&nbsp; HOOD: '+f[7].toFixed(2)+' rot</span>';\n")
          .append("    if (glRenderer && scene && camera) {\n")
          .append("      const rx = f[1], ry = f[2], yaw = f[3];\n")
          .append("      robotGroup.position.set(rx, ry, 0);\n")
          .append("      robotGroup.rotation.z = yaw;\n")
          .append("      if (tilterGroup) tilterGroup.rotation.y = f[6] > 20 ? -0.25 : 0.35;\n")
          .append("      if (hoodMesh) hoodMesh.rotation.y = -f[7] * 0.04;\n")
          .append("      const balls = f[14];\n")
          .append("      for (let i=0; i<ballMeshes.length; i++) {\n")
          .append("        if (i*3+2 < balls.length) { ballMeshes[i].position.set(balls[i*3], balls[i*3+1], balls[i*3+2]); ballMeshes[i].visible = true; } else { ballMeshes[i].visible = false; }\n")
          .append("      }\n")
          .append("      let tCnt = 0;\n")
          .append("      for (let k=Math.max(0,idx-16); k<=idx && tCnt<trailMeshes.length; k++) {\n")
          .append("        const b = sim3dFrames[k][14];\n")
          .append("        for (let j=0; j+2<b.length && tCnt<trailMeshes.length; j+=3) {\n")
          .append("          if (b[j+2] > 0.15) { trailMeshes[tCnt].position.set(b[j], b[j+1], b[j+2]); trailMeshes[tCnt].visible = true; tCnt++; }\n")
          .append("        }\n")
          .append("      }\n")
          .append("      for (let k=tCnt; k<trailMeshes.length; k++) trailMeshes[k].visible = false;\n")
          .append("      const hubX = isBlue ? 4.626 : 11.915, hubY = 4.035;\n")
          .append("      const tx = camSel.value==='follow' ? rx : (0.55*rx + 0.45*hubX);\n")
          .append("      const ty = camSel.value==='follow' ? ry : (0.55*ry + 0.45*hubY);\n")
          .append("      const tz = 0.62;\n")
          .append("      camera.position.set(tx + camDist*Math.cos(camPitch)*Math.cos(camYaw), ty + camDist*Math.cos(camPitch)*Math.sin(camYaw), tz + camDist*Math.sin(camPitch));\n")
          .append("      camera.lookAt(tx, ty, tz);\n")
          .append("      glRenderer.render(scene, camera);\n")
          .append("    }\n")
          .append("  }\n")
          .append("  function loop(ts){\n")
          .append("    if(!playing) return;\n")
          .append("    const spd = parseFloat(speedSel.value||'1');\n")
          .append("    if(ts - lastTs >= (75 / spd)){\n")
          .append("      lastTs = ts; idx++; if(idx >= sim3dFrames.length){ idx = 0; playing = false; playBtn.textContent = '▶ Play'; }\n")
          .append("      draw();\n")
          .append("    }\n")
          .append("    if(playing) requestAnimationFrame(loop);\n")
          .append("  }\n")
          .append("  playBtn.addEventListener('click', () => { playing = !playing; playBtn.textContent = playing ? '⏸ Pause' : '▶ Play'; if(playing){ lastTs = performance.now(); requestAnimationFrame(loop); } });\n")
          .append("  scrub.addEventListener('input', () => { idx = parseInt(scrub.value,10)||0; draw(); });\n")
          .append("  window.seekSim3D = function(sec){\n")
          .append("    let best = 0; for(let i=0;i<sim3dFrames.length;i++){ if(sim3dFrames[i][0]<=sec) best=i; else break; }\n")
          .append("    idx = best; draw(); document.getElementById('sim3dCard').scrollIntoView({behavior:'smooth'});\n")
          .append("    if(!playing){ playing = true; playBtn.textContent = '⏸ Pause'; lastTs = performance.now(); requestAnimationFrame(loop); }\n")
          .append("  };\n")
          .append("  if(recBtn && c.captureStream && window.MediaRecorder){\n")
          .append("    let rec = null, chunks = [];\n")
          .append("    recBtn.addEventListener('click', () => {\n")
          .append("      if(rec && rec.state === 'recording'){ rec.stop(); return; }\n")
          .append("      chunks = []; const stream = c.captureStream(30);\n")
          .append("      rec = new MediaRecorder(stream, {mimeType: MediaRecorder.isTypeSupported('video/webm;codecs=vp9') ? 'video/webm;codecs=vp9' : 'video/webm'});\n")
          .append("      rec.ondataavailable = e => { if(e.data && e.data.size > 0) chunks.push(e.data); };\n")
          .append("      rec.onstop = () => { const blob = new Blob(chunks, {type:'video/webm'}); const a = document.createElement('a'); a.href = URL.createObjectURL(blob); a.download = 'flux-3d-replay.webm'; a.click(); recBtn.textContent = '🎬 Record Clip (.webm)'; };\n")
          .append("      rec.start(); recBtn.textContent = '⏹ Stop & Save (.webm)';\n")
          .append("      if(!playing) playBtn.click();\n")
          .append("    });\n")
          .append("  }\n")
          .append("  draw();\n")
          .append("})();\n");
    }
    sb.append("</script>\n</body>\n</html>\n");

    Files.writeString(outputHtml, sb.toString(), StandardCharsets.UTF_8);
  }

  /**
   * Writes {@code runs-index.html}, {@code runs-index.csv}, and {@code batteries-health.csv} across
   * all analyzed runs.
   */
  public static void writeBatchSummary(List<RunAnalysisReport> reports, Path outDir)
      throws IOException {
    Files.createDirectories(outDir);

    // 1. runs-index.csv
    StringBuilder csv = new StringBuilder();
    csv.append(
        "RunId,Robot,MatchType,Alliance,NamedBattery,EstRintMilliOhms,MinVoltageV,PeakDriveStatorA,BrownoutCritSec,EnableDropouts,DistanceM,PeakSpeedMps,ShotsCount,OverallVerdict\n");
    for (RunAnalysisReport r : reports) {
      BatteryMetrics b = r.battery();
      csv.append(
          String.format(
              Locale.US,
              "%s,%s,%s,%s,%s,%.1f,%.2f,%.0f,%.2f,%d,%.1f,%.2f,%d,%s\n",
              csvQuote(r.runId()),
              csvQuote(r.robotName()),
              csvQuote(r.matchType()),
              csvQuote(r.allianceStation()),
              csvQuote(b.batteryName()),
              b.estimatedRintMilliOhms(),
              b.minVoltage(),
              b.peakDriveStatorAmps(),
              b.brownoutCriticalSeconds(),
              b.enableDropouts(),
              r.totalDistanceMeters(),
              r.peakSpeedMps(),
              r.shots().size(),
              csvQuote(r.overallVerdict())));
    }
    Files.writeString(outDir.resolve("runs-index.csv"), csv.toString(), StandardCharsets.UTF_8);

    // 2. batteries-health.csv
    StringBuilder bcsv = new StringBuilder();
    bcsv.append(
        "NamedBattery,RunId,RestingVoltageV,MinVoltageUnderLoadV,EstimatedRintMilliOhms,PeakSupplyAmps,EnableDropouts,HealthGrade\n");
    for (RunAnalysisReport r : reports) {
      BatteryMetrics b = r.battery();
      bcsv.append(
          String.format(
              Locale.US,
              "%s,%s,%.2f,%.2f,%.1f,%.0f,%d,%s\n",
              csvQuote(b.batteryName()),
              csvQuote(r.runId()),
              b.restingVoltage(),
              b.minVoltage(),
              b.estimatedRintMilliOhms(),
              b.peakSupplyCurrentAmps(),
              b.enableDropouts(),
              csvQuote(b.healthGrade())));
    }
    Files.writeString(
        outDir.resolve("batteries-health.csv"), bcsv.toString(), StandardCharsets.UTF_8);

    // 3. runs-index.html
    StringBuilder html = new StringBuilder(16384);
    html.append("<!DOCTYPE html>\n<html lang=\"en\">\n<head>\n")
        .append("  <meta charset=\"UTF-8\">\n")
        .append("  <title>FLUX 10413 — Season Log &amp; Battery Leaderboard</title>\n")
        .append(
            "  <link href=\"https://fonts.googleapis.com/css2?family=Space+Grotesk:wght@400;600;700&display=swap\" rel=\"stylesheet\">\n")
        .append("  <style>\n")
        .append(
            "    body { font-family: 'Space Grotesk', system-ui, sans-serif; background: #F2F3F5; color: #212020; padding: 24px; }\n")
        .append(
            "    .container { max-width: 1240px; margin: 0 auto; display: flex; flex-direction: column; gap: 20px; }\n")
        .append(
            "    .header { background: #FFF; border-top: 6px solid #254BA0; border-radius: 10px; padding: 18px 24px; display: flex; align-items: center; justify-content: space-between; box-shadow: 0 2px 6px rgba(0,0,0,0.06); }\n")
        .append("    .logo { height: 50px; }\n")
        .append(
            "    .card { background: #FFF; border-radius: 10px; padding: 18px 22px; box-shadow: 0 1px 4px rgba(0,0,0,0.05); }\n")
        .append(
            "    h2 { font-size: 17px; margin-bottom: 12px; border-bottom: 2px solid #E4E7EC; padding-bottom: 6px; }\n")
        .append("    table { width: 100%; border-collapse: collapse; font-size: 13px; }\n")
        .append("    th { background: #212020; color: #FFF; text-align: left; padding: 9px 12px; }\n")
        .append("    td { padding: 8px 12px; border-bottom: 1px solid #E4E7EC; }\n")
        .append("    tr:nth-child(even) td { background: #F8F9FB; }\n")
        .append("    .pill { padding: 3px 8px; border-radius: 4px; font-weight: 700; font-size: 11px; }\n")
        .append("    .pill-pass { background: #DCFCE7; color: #166534; }\n")
        .append("    .pill-warn { background: #FEF3C7; color: #92400E; }\n")
        .append("    .pill-fail { background: #FEE2E2; color: #991B1B; }\n")
        .append(
            "    a.btn { background: #254BA0; color: #FFF; text-decoration: none; padding: 4px 10px; border-radius: 4px; font-size: 12px; font-weight: 600; }\n")
        .append("  </style>\n</head>\n<body>\n<div class=\"container\">\n")
        .append("  <div class=\"header\">\n")
        .append("    <div style=\"display:flex;align-items:center;gap:16px;\">\n")
        .append("      <img class=\"logo\" src=\"")
        .append(LOGO_URL)
        .append("\" alt=\"FLUX Robotics\">\n")
        .append(
            "      <div><h1 style=\"font-size:21px;\">FRC Team 10413 (FLUX Robotics) — FluxProcessor Index</h1>\n")
        .append(
            "      <div style=\"font-size:12px;color:#5A606E;\">Automated Match &amp; Practice Audit • Battery Internal Resistance Leaderboard • Swerve Symmetry</div></div>\n")
        .append("    </div>\n  </div>\n")
        .append("  <div class=\"card\">\n")
        .append("    <h2>Analyzed Matches &amp; Practice Sessions (")
        .append(reports.size())
        .append(")</h2>\n")
        .append("    <table>\n")
        .append(
            "      <thead><tr><th>Run / Match</th><th>Alliance</th><th>Battery</th><th>Est. R_int</th><th>Min Volt</th><th>Peak Drive Stator</th><th>Dropouts</th><th>Distance</th><th>Shots</th><th>Verdict</th><th>Report</th></tr></thead>\n")
        .append("      <tbody>\n");

    for (RunAnalysisReport r : reports) {
      BatteryMetrics b = r.battery();
      String pill =
          "CRITICAL".equals(r.overallVerdict())
              ? "pill-fail"
              : ("WARN".equals(r.overallVerdict()) ? "pill-warn" : "pill-pass");
      html.append("        <tr>\n")
          .append("          <td><strong>")
          .append(esc(r.runId()))
          .append("</strong></td>\n")
          .append("          <td>")
          .append(esc(r.allianceStation()))
          .append("</td>\n")
          .append("          <td>")
          .append(esc(b.batteryName()))
          .append("</td>\n")
          .append("          <td><strong>")
          .append(String.format(Locale.US, "%.1f mΩ", b.estimatedRintMilliOhms()))
          .append("</strong></td>\n")
          .append("          <td>")
          .append(String.format(Locale.US, "%.2f V", b.minVoltage()))
          .append("</td>\n")
          .append("          <td>")
          .append(String.format(Locale.US, "%.0f A", b.peakDriveStatorAmps()))
          .append("</td>\n")
          .append("          <td>")
          .append(b.enableDropouts())
          .append("</td>\n")
          .append("          <td>")
          .append(String.format(Locale.US, "%.1f m", r.totalDistanceMeters()))
          .append("</td>\n")
          .append("          <td>")
          .append(r.shots().size())
          .append("</td>\n")
          .append("          <td><span class=\"pill ")
          .append(pill)
          .append("\">")
          .append(esc(r.overallVerdict()))
          .append("</span></td>\n")
          .append("          <td style=\"display:flex;gap:6px;\"><a class=\"btn\" href=\"")
          .append(esc(r.runId()))
          .append("/report.html\">Open Report</a>")
          .append(
              r.fuelSim() != null && r.fuelSim().videoFileName() != null
                  ? "<a class=\"btn\" style=\"background:#16A34A\" href=\""
                      + esc(r.runId())
                      + "/"
                      + esc(r.fuelSim().videoFileName())
                      + "\">🎬 3D Video</a>"
                  : "")
          .append("</td>\n")
          .append("        </tr>\n");
    }
    html.append("      </tbody>\n    </table>\n  </div>\n</div>\n</body>\n</html>\n");
    Files.writeString(outDir.resolve("runs-index.html"), html.toString(), StandardCharsets.UTF_8);
  }

  private static String kpiCard(String label, String val, String sub) {
    return "    <div class=\"kpi\"><div class=\"kpi-label\">"
        + esc(label)
        + "</div><div class=\"kpi-val\">"
        + esc(val)
        + "</div><div class=\"kpi-sub\">"
        + esc(sub)
        + "</div></div>\n";
  }

  private static String esc(String s) {
    if (s == null) {
      return "";
    }
    return s.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace("\"", "&quot;");
  }

  private static String escJs(String s) {
    if (s == null) {
      return "";
    }
    return s.replace("\\", "\\\\")
        .replace("\"", "\\\"")
        .replace("</", "<\\/")
        .replace("\n", " ")
        .replace("\r", "");
  }

  private static String csvQuote(String s) {
    if (s == null) {
      return "";
    }
    if (s.contains(",") || s.contains("\"") || s.contains("\n") || s.contains("\r")) {
      return "\"" + s.replace("\"", "\"\"") + "\"";
    }
    return s;
  }
}
