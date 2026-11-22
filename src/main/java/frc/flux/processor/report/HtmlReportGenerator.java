package frc.flux.processor.report;

import frc.flux.processor.analysis.RunAnalyzer;
import frc.flux.processor.model.RunAnalysisReport;
import frc.flux.processor.model.RunAnalysisReport.BatteryMetrics;
import frc.flux.processor.model.RunAnalysisReport.CameraSnapshotRef;
import frc.flux.processor.model.RunAnalysisReport.CandidateTuningReview;
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
import frc.flux.processor.model.RunAnalysisReport.TuningDiffEntry;
import frc.flux.processor.model.RunAnalysisReport.VoiceCorrelation;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Locale;

/**
 * Generates 100% offline-safe, XSS-clean 1-page HTML reports ({@code report.html}), season summary
 * index ({@code runs-index.html} and {@code runs-index.csv}), and Named Battery Leaderboard ({@code
 * batteries-health.csv}) styled in FRC Team 10413 (FLUX Robotics) official colors.
 */
public final class HtmlReportGenerator {

  private static final String FLUX_LOGO_SVG =
      "<svg class=\"logo\" viewBox=\"0 0 120 48\" width=\"120\" height=\"48\" role=\"img\" aria-label=\"FLUX Robotics 10413\">"
          + "<rect x=\"1\" y=\"1\" width=\"118\" height=\"46\" rx=\"8\" fill=\"#212020\" stroke=\"#254BA0\" stroke-width=\"2\"/>"
          + "<polygon points=\"14,34 24,12 36,12 29,24 38,24 20,40 24,28 16,28\" fill=\"#2B7BFF\"/>"
          + "<text x=\"46\" y=\"25\" fill=\"#FFFFFF\" font-family=\"system-ui,-apple-system,sans-serif\" font-size=\"15\" font-weight=\"800\" letter-spacing=\"1\">FLUX</text>"
          + "<text x=\"46\" y=\"39\" fill=\"#93C5FD\" font-family=\"ui-monospace,SFMono-Regular,monospace\" font-size=\"11\" font-weight=\"700\">FRC 10413</text>"
          + "</svg>";

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
            "    body { font-family: 'Segoe UI', system-ui, -apple-system, BlinkMacSystemFont, sans-serif; background: var(--bg); color: var(--black); padding: 20px; line-height: 1.45; }\n")
        .append(
            "    code, pre { font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; }\n")
        .append(
            "    .container { max-width: 1240px; margin: 0 auto; display: flex; flex-direction: column; gap: 18px; }\n")
        .append(
            "    .header { background: var(--card); border-top: 6px solid var(--royal); border-radius: 10px; padding: 16px 22px; display: flex; align-items: center; justify-content: space-between; box-shadow: 0 2px 6px rgba(0,0,0,0.06); }\n")
        .append("    .header-left { display: flex; align-items: center; gap: 16px; }\n")
        .append("    .logo { height: 48px; width: auto; flex-shrink: 0; }\n")
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
        .append("    .card h2 { font-size: 16px; font-weight: 700; color: var(--black); margin-bottom: 10px; padding-bottom: 6px; border-bottom: 2px solid var(--silver); display: flex; justify-content: space-between; align-items: center; gap: 10px; flex-wrap: wrap; }\n")
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
        .append("    .cmd-box { background: #0F172A; color: #E2E8F0; border-radius: 6px; padding: 10px 12px; margin-top: 10px; display: flex; justify-content: space-between; align-items: center; gap: 10px; flex-wrap: wrap; font-size: 12px; }\n")
        .append("    .cmd-box code { white-space: pre-wrap; word-break: break-all; }\n")
        .append("    .tuning-reasons { margin: 8px 0 12px 18px; font-size: 12px; color: var(--black); }\n")
        .append("    .tuning-reasons li { margin-bottom: 4px; }\n")
        .append("    canvas { width: 100%; height: 220px; background: #18181B; border-radius: 8px; display: block; }\n")
        .append("  </style>\n")
        .append("</head>\n<body>\n<div class=\"container\">\n");

    sb.append("  <div class=\"header\">\n")
        .append("    <div class=\"header-left\">\n")
        .append("      ").append(FLUX_LOGO_SVG).append("\n")
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

    sb.append("  <div class=\"alerts\">\n")
        .append("    <h3>Key Audit Findings</h3>\n")
        .append("    <ul>\n");
    for (String alert : r.keyAlerts()) {
      sb.append("      <li>").append(esc(alert)).append("</li>\n");
    }
    sb.append("    </ul>\n  </div>\n");

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
                    Locale.US,
                    "Est. R_int: %s (%s)",
                    fmtFinite(b.estimatedRintMilliOhms(), "%.1f mΩ", "N/A"),
                    b.healthGrade())))
        .append(
            kpiCard(
                "Min Battery Voltage",
                fmtFinite(b.minVoltage(), "%.2f V", "N/A"),
                String.format(
                    Locale.US,
                    "Rest: %s | Sag @ %.1fs",
                    fmtFinite(b.restingVoltage(), "%.2f V", "N/A"),
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

    // Candidate Practice Tuning Diff (Unmerged) Panel
    if (r.candidateTuning() != null && r.candidateTuning().present()) {
      CandidateTuningReview ct = r.candidateTuning();
      String ctPill =
          "DO_NOT_PROMOTE".equals(ct.verdict())
              ? "pill-fail"
              : ("REVIEW_WARNINGS".equals(ct.verdict()) ? "pill-warn" : "pill-pass");
      String borderCol =
          "DO_NOT_PROMOTE".equals(ct.verdict())
              ? "#DC2626"
              : ("REVIEW_WARNINGS".equals(ct.verdict()) ? "#D97706" : "#16A34A");
      sb.append("  <div class=\"card\" id=\"candidateTuningCard\" style=\"border-left: 5px solid ")
          .append(borderCol)
          .append("\">\n")
          .append("    <h2><span>Candidate Practice Tuning Diff (Unmerged)</span>")
          .append("<span class=\"pill ")
          .append(ctPill)
          .append("\">")
          .append(esc(ct.verdict()))
          .append("</span></h2>\n")
          .append("    <div class=\"subtitle\" style=\"margin-bottom:8px;\">Snapshot: <code>")
          .append(esc(ct.sourceFilePath()))
          .append("</code>  •  Profile: <strong>")
          .append(esc(ct.activeProfile()))
          .append("</strong>  •  STABLE Base SHA: <code>")
          .append(esc(ct.baseConfigSha()))
          .append("</code>  •  Overrides: <strong>")
          .append(ct.entries().size())
          .append("</strong></div>\n");

      if (!ct.verdictReasons().isEmpty()) {
        sb.append("    <ul class=\"tuning-reasons\">\n");
        for (String reason : ct.verdictReasons()) {
          sb.append("      <li>").append(esc(reason)).append("</li>\n");
        }
        sb.append("    </ul>\n");
      }

      if (!ct.entries().isEmpty()) {
        sb.append("    <table>\n")
            .append(
                "      <thead><tr><th>Scope</th><th>Parameter / Cell</th><th>STABLE Base</th><th>Candidate Value</th><th>Safety Gate</th></tr></thead>\n")
            .append("      <tbody>\n");
        for (TuningDiffEntry entry : ct.entries()) {
          String gatePill = entry.structural() ? "pill-fail" : "pill-pass";
          String gateLabel = entry.structural() ? "STRUCTURAL (BLOCKED)" : "TUNABLE";
          sb.append("        <tr>\n")
              .append("          <td><code>")
              .append(esc(entry.scope()))
              .append("</code></td>\n")
              .append("          <td><strong><code>")
              .append(esc(entry.key()))
              .append("</code></strong></td>\n")
              .append("          <td><code>")
              .append(esc(entry.baseValue()))
              .append("</code></td>\n")
              .append("          <td><code>")
              .append(esc(entry.candidateValue()))
              .append("</code></td>\n")
              .append("          <td><span class=\"pill ")
              .append(gatePill)
              .append("\">")
              .append(gateLabel)
              .append("</span></td>\n")
              .append("        </tr>\n");
        }
        sb.append("      </tbody>\n    </table>\n");
      }

      String cmdLabel =
          "DO_NOT_PROMOTE".equals(ct.verdict())
              ? "Blocked — Do Not Promote Until Findings Are Resolved (Reference CLI):"
              : "Promote Gate CLI (Run Manually After Review):";
      sb.append("    <div style=\"font-size:12px;font-weight:700;margin-top:12px;color:var(--muted);\">")
          .append(esc(cmdLabel))
          .append("</div>\n")
          .append("    <div class=\"cmd-box\"><code id=\"promoteCmdText\">")
          .append(esc(ct.promoteCommand()))
          .append("</code><button id=\"copyPromoteBtn\" class=\"btn-link\" type=\"button\" style=\"border:none;cursor:pointer;padding:5px 10px;\">Copy Command</button></div>\n")
          .append("  </div>\n");
    }

    // Interactive Timeline Canvas
    sb.append("  <div class=\"card\">\n")
        .append(
            "    <h2><span>Battery Voltage (V), Total Drive Stator Current (A) &amp; Shooter Speed (RPS)</span>")
        .append(
            "<span style=\"font-size:12px;font-weight:400;color:var(--muted)\"> Green = Voltage (V) • Blue = Total Stator Current (A) • Gold = Shot Events • Red Dashed = 6.8V Brownout</span></h2>\n")
        .append("    <canvas id=\"timelineCanvas\" width=\"1180\" height=\"220\"></canvas>\n")
        .append("  </div>\n");

    // Offline Interactive Canvas Field Replay & Game-Piece Visualizer Card
    if (r.fuelSim() != null && r.fuelSim().frames() != null && !r.fuelSim().frames().isEmpty()) {
      String mp4File = sanitizeRelativePath(r.fuelSim().videoFileName());
      sb.append("  <div class=\"card\" id=\"sim3dCard\">\n")
          .append("    <h2><span>Offline Match Replay &amp; 2026 REBUILT FuelSim Arena (")
          .append(r.fuelSim().ballsScoredInHub())
          .append(" / ")
          .append(r.fuelSim().ballsLaunched())
          .append(" Scored in Hub)</span><span style=\"display:flex;gap:8px;align-items:center\">");
      if (!mp4File.isEmpty()) {
        sb.append("<a class=\"btn-link\" style=\"background:#16A34A;padding:5px 10px\" download href=\"")
            .append(esc(mp4File))
            .append("\">Download ")
            .append(esc(mp4File))
            .append(" (HD)</a>");
      }
      sb.append(
              "<button id=\"simRecBtn\" class=\"btn-link\" type=\"button\" style=\"border:none;cursor:pointer;padding:5px 10px;background:#7C3AED\">Record Clip (.webm)</button>")
          .append("</span></h2>\n");
      if (!mp4File.isEmpty()) {
        sb.append(
                "    <div style=\"margin-bottom:14px;\"><div style=\"font-size:12px;font-weight:700;color:var(--muted);margin-bottom:6px;\">Rendered Replay Video:</div>\n")
            .append(
                "    <video controls preload=\"metadata\" poster=\"preview-3d.png\" style=\"width:100%;max-height:560px;border-radius:8px;background:#090D16;box-shadow:0 4px 14px rgba(0,0,0,0.22);\" src=\"")
            .append(esc(mp4File))
            .append("\"></video></div>\n");
      }
      sb.append(
              "    <div style=\"font-size:12px;font-weight:700;color:var(--muted);margin-bottom:6px;\">Interactive Offline Canvas Field &amp; Replay Explorer (Drag to Orbit Isometric View, Scroll to Zoom):</div>\n")
          .append(
              "    <div style=\"display:flex;flex-wrap:wrap;gap:10px;align-items:center;margin-bottom:10px;background:#F8F9FB;padding:8px 12px;border-radius:6px;border:1px solid var(--silver);font-size:12px;\">\n")
          .append(
              "      <button id=\"simPlayBtn\" class=\"btn-link\" type=\"button\" style=\"border:none;cursor:pointer;padding:5px 12px;font-size:12px;\">Play</button>\n")
          .append(
              "      <input id=\"simScrub\" type=\"range\" min=\"0\" max=\"100\" value=\"0\" style=\"flex:1;min-width:180px;cursor:pointer;\">\n")
          .append(
              "      <span id=\"simTimeLabel\" style=\"font-family:monospace;font-weight:700;min-width:110px;\">0.0s / 0.0s</span>\n")
          .append(
              "      <label>Speed: <select id=\"simSpeed\" style=\"padding:3px 6px;border-radius:4px;\"><option value=\"0.5\">0.5x</option><option value=\"1\" selected>1x</option><option value=\"2\">2x</option><option value=\"4\">4x</option></select></label>\n")
          .append(
              "      <label>Camera: <select id=\"simCamMode\" style=\"padding:3px 6px;border-radius:4px;\"><option value=\"iso\" selected>Isometric 2.5D (Drag to Rotate)</option><option value=\"top\">Top-Down 2D Field</option><option value=\"side\">Broadcast Side Elevation</option><option value=\"follow\">Robot Chase View</option></select></label>\n")
          .append("    </div>\n")
          .append(
              "    <div style=\"position:relative;\"><canvas id=\"sim3dCanvas\" width=\"1180\" height=\"500\" style=\"height:500px;background:#0b0f19;cursor:grab;border-radius:8px;\"></canvas>\n")
          .append(
              "    <div id=\"sim3dHudTop\" style=\"position:absolute;top:12px;left:14px;right:14px;display:flex;justify-content:space-between;pointer-events:none;font-family:monospace;font-size:13px;font-weight:700;color:#F8FAFC;background:rgba(10,15,28,0.86);padding:8px 14px;border-radius:8px;border:1px solid rgba(56,189,248,0.35);\"></div>\n")
          .append(
              "    <div id=\"sim3dHudBot\" style=\"position:absolute;bottom:12px;left:14px;right:14px;display:flex;justify-content:space-between;pointer-events:none;font-family:monospace;font-size:12px;font-weight:600;color:#CBD5E1;background:rgba(10,15,28,0.86);padding:7px 14px;border-radius:8px;border:1px solid rgba(148,163,184,0.25);\"></div></div>\n")
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
      String safeVideoLink = sanitizeExternalUrl(vc.videoLink());
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
              !safeVideoLink.isEmpty()
                  ? "<a class=\"btn-link\" target=\"_blank\" rel=\"noopener noreferrer\" href=\""
                      + esc(safeVideoLink)
                      + "\">Watch</a>"
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
            .append(fmtFinite(ec.minBatteryVoltage(), "%.2f V", "N/A"))
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
            ")</span><span style=\"font-size:12px;font-weight:400;color:var(--muted)\">Triggered when Feeder + Shooter Flywheels are active simultaneously • Replayable offline via /FuelSim/Fuels</span></h2>\n")
        .append("    <table>\n")
        .append(
            "      <thead><tr><th>Shot #</th><th>Time (s)</th><th>Duration</th><th>Shooter Left (RPS)</th><th>Shooter Right (RPS)</th><th>L/R Δ (RPS)</th><th>Hood Pos (rot)</th><th>Feeder Peak (A)</th><th>Min Battery (V)</th><th>Status</th><th>FuelSim Outcome</th><th>Video</th></tr></thead>\n")
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
      String safeShotLink = sanitizeExternalUrl(s.videoLink());
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
          .append(fmtFinite(s.minBatteryVoltageDuringShot(), "%.2f V", "N/A"))
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
                  "<button class=\"btn-link\" type=\"button\" style=\"border:none;cursor:pointer\" onclick=\"if(window.seekSim3D)window.seekSim3D(%.2f)\">Replay</button>",
                  Math.max(0.0, s.timestampSec() - 0.5)))
          .append(
              !safeShotLink.isEmpty()
                  ? "<a class=\"btn-link\" target=\"_blank\" rel=\"noopener noreferrer\" href=\""
                      + esc(safeShotLink)
                      + "\">Clip</a>"
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
        String safeSnapHref = sanitizeRelativePath(snap.relativePath());
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
            .append("          <td>")
            .append(
                !safeSnapHref.isEmpty()
                    ? "<a class=\"btn-link\" target=\"_blank\" rel=\"noopener noreferrer\" href=\""
                        + esc(safeSnapHref)
                        + "\">View Frame</a>"
                    : "—")
            .append("</td>\n")
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
                  "Loop p50=%s / p99=%s • CPU=%s (Java %s / Vision %s) • Temp=%s • CAN Drops=%d • Free=%s",
                  fmtFinite(sh.loopMedianMs(), "%.2fms", "NOT MEASURED"),
                  fmtFinite(sh.loopP99Ms(), "%.2fms", "NOT MEASURED"),
                  fmtFinite(sh.totalCpuPct(), "%.1f%%", "NOT MEASURED"),
                  fmtFinite(sh.javaProcessCpuPct(), "%.1f%%", "NOT MEASURED"),
                  fmtFinite(sh.visionDaemonCpuPct(), "%.1f%%", "NOT MEASURED"),
                  fmtFinite(sh.cpuTempC(), "%.1f°C", "NOT MEASURED"),
                  sh.socketCanDropsAndOverruns(),
                  fmtFinite(sh.freeStorageMb(), "%.0fMB", "NOT MEASURED")))
          .append("</span></h2>\n");
      if (sh.watchDecisions().isEmpty()) {
        boolean hasAnyMeasuredMetric =
            Double.isFinite(sh.loopMedianMs())
                || Double.isFinite(sh.loopP99Ms())
                || Double.isFinite(sh.loopMaxMs())
                || Double.isFinite(sh.totalCpuPct())
                || Double.isFinite(sh.javaProcessCpuPct())
                || Double.isFinite(sh.cpuTempC())
                || Double.isFinite(sh.freeStorageMb());
        if (!hasAnyMeasuredMetric) {
          sb.append(
              "    <div style=\"font-size:13px;color:var(--muted);font-weight:600;\">No controller watch metrics were captured in this run (NOT MEASURED).</div>\n");
        } else if (sh.controllerType() != null
            && sh.controllerType().toUpperCase(Locale.ROOT).contains("ROBORIO")) {
          sb.append(
              "    <div style=\"font-size:13px;color:#166534;font-weight:600;\">Measured roboRIO watch metrics are within nominal thresholds (0 warnings; SystemCore-only thermal/PMU fields are N/A).</div>\n");
        } else {
          sb.append(
              "    <div style=\"font-size:13px;color:#166534;font-weight:600;\">All offline SystemCore / Controller watch metrics are within nominal thresholds (0 warnings).</div>\n");
        }
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

    // Inline JS for copy button, timeline canvas, and 100% offline 2.5D Canvas replay viewer
    sb.append("</div>\n<script>\n");
    sb.append("(function(){\n")
        .append("  const copyBtn = document.getElementById('copyPromoteBtn');\n")
        .append("  const cmdEl = document.getElementById('promoteCmdText');\n")
        .append("  if (copyBtn && cmdEl && navigator.clipboard) {\n")
        .append("    copyBtn.addEventListener('click', () => {\n")
        .append("      navigator.clipboard.writeText(cmdEl.textContent || '').then(() => {\n")
        .append("        copyBtn.textContent = 'Copied';\n")
        .append("        setTimeout(() => { copyBtn.textContent = 'Copy Command'; }, 1600);\n")
        .append("      });\n")
        .append("    });\n")
        .append("  }\n")
        .append("})();\n");

    sb.append("const timeline = [\n");
    for (int i = 0; i < r.timeline().size(); i++) {
      TimelinePoint pt = r.timeline().get(i);
      sb.append(
          String.format(
              Locale.US,
              "[%s,%s,%s,%s,%d]%s",
              jsNum(pt.timeSec(), "%.2f"),
              jsNum(pt.batteryVoltage(), "%.2f"),
              jsNum(pt.totalDriveStatorAmps(), "%.1f"),
              jsNum(pt.shooterLeftRps(), "%.1f"),
              pt.enabled() ? 1 : 0,
              (i + 1 < r.timeline().size()) ? "," : ""));
    }
    sb.append("\n];\nconst shots = [");
    for (int i = 0; i < r.shots().size(); i++) {
      sb.append(
          String.format(
              Locale.US,
              "%s%s",
              jsNum(r.shots().get(i).timestampSec(), "%.2f"),
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
        .append("  ctx.strokeStyle = '#EF4444'; ctx.setLineDash([5, 5]); ctx.lineWidth = 1;\n")
        .append("  ctx.beginPath(); ctx.moveTo(padL, yVolt(6.8)); ctx.lineTo(W - padR, yVolt(6.8)); ctx.stroke();\n")
        .append("  ctx.setLineDash([]);\n")
        .append("  ctx.strokeStyle = '#2B7BFF'; ctx.lineWidth = 1.5; ctx.beginPath();\n")
        .append("  let startedA = false;\n")
        .append("  timeline.forEach(p => {\n")
        .append("    if (p[2] === null) { startedA = false; return; }\n")
        .append("    const x = xOf(p[0]), y = yAmps(p[2]);\n")
        .append("    if (!startedA) { ctx.moveTo(x, y); startedA = true; } else { ctx.lineTo(x, y); }\n")
        .append("  });\n")
        .append("  ctx.stroke();\n")
        .append("  ctx.strokeStyle = '#22C55E'; ctx.lineWidth = 2; ctx.beginPath();\n")
        .append("  let startedV = false;\n")
        .append("  timeline.forEach(p => {\n")
        .append("    if (p[1] === null) { startedV = false; return; }\n")
        .append("    const x = xOf(p[0]), y = yVolt(p[1]);\n")
        .append("    if (!startedV) { ctx.moveTo(x, y); startedV = true; } else { ctx.lineTo(x, y); }\n")
        .append("  });\n")
        .append("  ctx.stroke();\n")
        .append("  ctx.fillStyle = '#F59E0B';\n")
        .append("  shots.forEach(t => { if (t === null) return; const x = xOf(t); ctx.beginPath(); ctx.arc(x, padT + 8, 4, 0, Math.PI*2); ctx.fill(); });\n")
        .append("  ctx.fillStyle = '#A1A1AA'; ctx.font = '11px sans-serif';\n")
        .append("  ctx.fillText('14V', 12, yVolt(14)+4); ctx.fillText('10V', 12, yVolt(10)+4); ctx.fillText('6.8V', 8, yVolt(6.8)+4);\n")
        .append("  ctx.fillText('600A', W-42, yAmps(600)+4); ctx.fillText('300A', W-42, yAmps(300)+4); ctx.fillText('0A', W-42, yAmps(0)+4);\n")
        .append("})();\n");

    // 100% Offline Pure Canvas 2D / Isometric 2.5D FuelSim Arena Player (zero external CDNs, zero innerHTML)
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
                "[%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%d,%d,%d,\"%s\",[",
                jsNum(sf.timeSec(), "%.2f"),
                jsNum(sf.robotX(), "%.2f"),
                jsNum(sf.robotY(), "%.2f"),
                jsNum(sf.robotHeadingRad(), "%.3f"),
                jsNum(sf.batteryVoltage(), "%.2f"),
                jsNum(sf.driveStatorAmps(), "%.0f"),
                jsNum(sf.shooterRps(), "%.1f"),
                jsNum(sf.hoodRot(), "%.2f"),
                jsNum(sf.elevatorHeightM(), "%.2f"),
                jsNum(sf.armAngleRad(), "%.2f"),
                sf.blueScore(),
                sf.redScore(),
                sf.intakedCount(),
                escJs(sf.lastShotBanner())));
        double[] bxyz = sf.ballXyz();
        if (bxyz != null) {
          for (int bIdx = 0; bIdx < bxyz.length; bIdx++) {
            sb.append(
                String.format(
                    Locale.US, "%s%s", jsNum(bxyz[bIdx], "%.2f"), (bIdx + 1 < bxyz.length) ? "," : ""));
          }
        }
        sb.append("]]").append((i + 1 < frames.size()) ? ",\n" : "\n");
      }
      sb.append("];\n")
          .append("(function(){\n")
          .append("  const c = document.getElementById('sim3dCanvas');\n")
          .append("  if (!c || sim3dFrames.length === 0) return;\n")
          .append("  const ctx = c.getContext('2d');\n")
          .append("  const playBtn = document.getElementById('simPlayBtn');\n")
          .append("  const scrub = document.getElementById('simScrub');\n")
          .append("  const timeLbl = document.getElementById('simTimeLabel');\n")
          .append("  const speedSel = document.getElementById('simSpeed');\n")
          .append("  const camSel = document.getElementById('simCamMode');\n")
          .append("  const recBtn = document.getElementById('simRecBtn');\n")
          .append("  const hudTop = document.getElementById('sim3dHudTop');\n")
          .append("  const hudBot = document.getElementById('sim3dHudBot');\n")
          .append("  const isBlue = ").append(isBlue ? "true" : "false").append(";\n")
          .append("  const L = 16.541, W = 8.069;\n")
          .append("  scrub.max = sim3dFrames.length - 1;\n")
          .append("  let idx = 0, playing = false, lastTs = 0;\n")
          .append("  let orbitYaw = -0.22, orbitPitch = 0.62, zoom = 1.0, dragging = false, dragX = 0, dragY = 0;\n")
          .append("  function makeSpan(text, color, bold) {\n")
          .append("    const s = document.createElement('span');\n")
          .append("    s.textContent = text;\n")
          .append("    if (color) s.style.color = color;\n")
          .append("    if (bold) s.style.fontWeight = '700';\n")
          .append("    return s;\n")
          .append("  }\n")
          .append("  function proj(fx, fy, fz, mode, rx, ry) {\n")
          .append("    const cw = c.width, ch = c.height;\n")
          .append("    if (mode === 'top') {\n")
          .append("      const sc = Math.min((cw - 90) / L, (ch - 120) / W) * zoom;\n")
          .append("      return [cw * 0.5 + (fx - L * 0.5) * sc, ch * 0.52 - (fy - W * 0.5) * sc];\n")
          .append("    }\n")
          .append("    if (mode === 'side') {\n")
          .append("      const sc = ((cw - 90) / L) * zoom;\n")
          .append("      return [cw * 0.5 + (fx - L * 0.5) * sc, ch * 0.78 - fz * sc * 1.45 - (fy - W * 0.5) * sc * 0.14];\n")
          .append("    }\n")
          .append("    const cx = mode === 'follow' ? rx : L * 0.5;\n")
          .append("    const cy = mode === 'follow' ? ry : W * 0.5;\n")
          .append("    const sc = (mode === 'follow' ? 78 : 54) * zoom;\n")
          .append("    const dx = fx - cx, dy = fy - cy;\n")
          .append("    const ux = dx * Math.cos(orbitYaw) - dy * Math.sin(orbitYaw);\n")
          .append("    const uy = dx * Math.sin(orbitYaw) + dy * Math.cos(orbitYaw);\n")
          .append("    return [cw * 0.5 + ux * sc, ch * 0.56 - (uy * Math.sin(orbitPitch) + fz * Math.cos(orbitPitch) * 1.25) * sc];\n")
          .append("  }\n")
          .append("  function drawPoly(pts, fill, stroke, lw) {\n")
          .append("    ctx.beginPath();\n")
          .append("    pts.forEach((p, i) => i === 0 ? ctx.moveTo(p[0], p[1]) : ctx.lineTo(p[0], p[1]));\n")
          .append("    ctx.closePath();\n")
          .append("    if (fill) { ctx.fillStyle = fill; ctx.fill(); }\n")
          .append("    if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = lw || 1.5; ctx.stroke(); }\n")
          .append("  }\n")
          .append("  function drawHub(hx, hy, fill, rim, mode, rx, ry) {\n")
          .append("    const basePts = [], rimPts = [];\n")
          .append("    for (let i = 0; i < 8; i++) {\n")
          .append("      const a = i * Math.PI / 4;\n")
          .append("      basePts.push(proj(hx + 0.46 * Math.cos(a), hy + 0.46 * Math.sin(a), 0, mode, rx, ry));\n")
          .append("      rimPts.push(proj(hx + 0.58 * Math.cos(a), hy + 0.58 * Math.sin(a), 1.65, mode, rx, ry));\n")
          .append("    }\n")
          .append("    drawPoly(basePts, fill, rim, 1.2);\n")
          .append("    ctx.strokeStyle = fill; ctx.lineWidth = 1.4;\n")
          .append("    for (let i = 0; i < 8; i++) { ctx.beginPath(); ctx.moveTo(basePts[i][0], basePts[i][1]); ctx.lineTo(rimPts[i][0], rimPts[i][1]); ctx.stroke(); }\n")
          .append("    drawPoly(rimPts, fill, rim, 2.2);\n")
          .append("  }\n")
          .append("  function draw() {\n")
          .append("    const f = sim3dFrames[idx]; if (!f) return;\n")
          .append("    const maxT = sim3dFrames[sim3dFrames.length - 1][0] || 0;\n")
          .append("    const tVal = f[0] || 0;\n")
          .append("    timeLbl.textContent = tVal.toFixed(1) + 's / ' + maxT.toFixed(1) + 's';\n")
          .append("    scrub.value = idx;\n")
          .append("    if (hudTop) {\n")
          .append("      const left = document.createElement('span');\n")
          .append("      left.append(makeSpan('BLUE HUB: ' + f[10], '#60A5FA', true), makeSpan('  •  ', '#94A3B8', false), makeSpan('RED HUB: ' + f[11], '#F87171', false), makeSpan('  •  ', '#94A3B8', false), makeSpan('INTAKED: ' + f[12], '#FACC15', false));\n")
          .append("      const banner = f[13] || '';\n")
          .append("      const right = banner ? makeSpan(banner, banner.indexOf('SCORED') >= 0 ? '#4ADE80' : '#FCA5A5', true) : makeSpan('T = ' + tVal.toFixed(1) + 's', '#E2E8F0', true);\n")
          .append("      hudTop.replaceChildren(left, right);\n")
          .append("    }\n")
          .append("    if (hudBot) {\n")
          .append("      const v = f[4];\n")
          .append("      const vText = v === null ? 'BATTERY: N/A' : ('BATTERY: ' + v.toFixed(2) + ' V' + (v < 6.8 ? ' [BROWNOUT!]' : ''));\n")
          .append("      const vCol = v === null ? '#94A3B8' : (v < 6.8 ? '#F87171' : '#4ADE80');\n")
          .append("      const stats = 'DRIVE STATOR: ' + (f[5] || 0) + ' A  •  SHOOTER: ' + (f[6] || 0).toFixed(1) + ' RPS  •  HOOD: ' + (f[7] || 0).toFixed(2) + ' rot';\n")
          .append("      hudBot.replaceChildren(makeSpan(vText, vCol, true), makeSpan(stats, '#CBD5E1', false));\n")
          .append("    }\n")
          .append("    const mode = camSel ? camSel.value : 'iso';\n")
          .append("    const rx = f[1] || 0, ry = f[2] || 0, yaw = f[3] || 0;\n")
          .append("    ctx.fillStyle = '#0b0f19'; ctx.fillRect(0, 0, c.width, c.height);\n")
          .append("    drawPoly([proj(0,0,0,mode,rx,ry), proj(4.626,0,0,mode,rx,ry), proj(4.626,W,0,mode,rx,ry), proj(0,W,0,mode,rx,ry)], '#172554', '#1E3A8A', 1);\n")
          .append("    drawPoly([proj(4.626,0,0,mode,rx,ry), proj(11.915,0,0,mode,rx,ry), proj(11.915,W,0,mode,rx,ry), proj(4.626,W,0,mode,rx,ry)], '#1E293B', '#334155', 1);\n")
          .append("    drawPoly([proj(11.915,0,0,mode,rx,ry), proj(L,0,0,mode,rx,ry), proj(L,W,0,mode,rx,ry), proj(11.915,W,0,mode,rx,ry)], '#450A0A', '#7F1D1D', 1);\n")
          .append("    ctx.strokeStyle = 'rgba(255,255,255,0.14)'; ctx.lineWidth = 1;\n")
          .append("    for (let gx = 2; gx < L; gx += 2) { const a = proj(gx,0,0,mode,rx,ry), b = proj(gx,W,0,mode,rx,ry); ctx.beginPath(); ctx.moveTo(a[0],a[1]); ctx.lineTo(b[0],b[1]); ctx.stroke(); }\n")
          .append("    for (let gy = 2; gy < W; gy += 2) { const a = proj(0,gy,0,mode,rx,ry), b = proj(L,gy,0,mode,rx,ry); ctx.beginPath(); ctx.moveTo(a[0],a[1]); ctx.lineTo(b[0],b[1]); ctx.stroke(); }\n")
          .append("    const cA = proj(L*0.5,0,0,mode,rx,ry), cB = proj(L*0.5,W,0,mode,rx,ry);\n")
          .append("    ctx.strokeStyle = 'rgba(248,250,252,0.45)'; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(cA[0],cA[1]); ctx.lineTo(cB[0],cB[1]); ctx.stroke();\n")
          .append("    drawHub(4.626, 4.035, 'rgba(37,99,235,0.45)', '#60A5FA', mode, rx, ry);\n")
          .append("    drawHub(11.915, 4.035, 'rgba(220,38,38,0.45)', '#F87171', mode, rx, ry);\n")
          .append("    ctx.strokeStyle = 'rgba(56,189,248,0.55)'; ctx.lineWidth = 2;\n")
          .append("    ctx.beginPath();\n")
          .append("    for (let k = Math.max(0, idx - 60); k <= idx; k++) {\n")
          .append("      const p = proj(sim3dFrames[k][1] || 0, sim3dFrames[k][2] || 0, 0.04, mode, rx, ry);\n")
          .append("      if (k === Math.max(0, idx - 60)) ctx.moveTo(p[0], p[1]); else ctx.lineTo(p[0], p[1]);\n")
          .append("    }\n")
          .append("    ctx.stroke();\n")
          .append("    const half = 0.43, cos = Math.cos(yaw), sin = Math.sin(yaw);\n")
          .append("    const corners = [[-half,-half],[half,-half],[half,half],[-half,half]].map(pt => proj(rx + pt[0]*cos - pt[1]*sin, ry + pt[0]*sin + pt[1]*cos, 0.16, mode, rx, ry));\n")
          .append("    drawPoly(corners, isBlue ? '#1D4ED8' : '#B91C1C', '#F8FAFC', 2.2);\n")
          .append("    const rc = proj(rx, ry, 0.22, mode, rx, ry), rh = proj(rx + 0.60*cos, ry + 0.60*sin, 0.22, mode, rx, ry);\n")
          .append("    ctx.strokeStyle = '#FACC15'; ctx.lineWidth = 3; ctx.beginPath(); ctx.moveTo(rc[0], rc[1]); ctx.lineTo(rh[0], rh[1]); ctx.stroke();\n")
          .append("    if ((f[8] || 0) > 0.05) {\n")
          .append("      const ez = 0.22 + f[8], ep = proj(rx, ry, ez, mode, rx, ry);\n")
          .append("      ctx.strokeStyle = '#38BDF8'; ctx.lineWidth = 3.5; ctx.beginPath(); ctx.moveTo(rc[0], rc[1]); ctx.lineTo(ep[0], ep[1]); ctx.stroke();\n")
          .append("      const ap = proj(rx + 0.5*Math.cos(f[9]||0)*cos, ry + 0.5*Math.cos(f[9]||0)*sin, ez + 0.5*Math.sin(f[9]||0), mode, rx, ry);\n")
          .append("      ctx.strokeStyle = '#FACC15'; ctx.lineWidth = 3; ctx.beginPath(); ctx.moveTo(ep[0], ep[1]); ctx.lineTo(ap[0], ap[1]); ctx.stroke();\n")
          .append("    }\n")
          .append("    for (let k = Math.max(0, idx - 14); k <= idx; k++) {\n")
          .append("      const tb = sim3dFrames[k][14] || [];\n")
          .append("      for (let j = 0; j + 2 < tb.length; j += 3) {\n")
          .append("        if (tb[j+2] > 0.16) {\n")
          .append("          const tp = proj(tb[j], tb[j+1], tb[j+2], mode, rx, ry);\n")
          .append("          ctx.fillStyle = 'rgba(56,189,248,0.45)'; ctx.beginPath(); ctx.arc(tp[0], tp[1], 3, 0, Math.PI*2); ctx.fill();\n")
          .append("        }\n")
          .append("      }\n")
          .append("    }\n")
          .append("    const balls = f[14] || [];\n")
          .append("    for (let i = 0; i + 2 < balls.length; i += 3) {\n")
          .append("      const sh = proj(balls[i], balls[i+1], 0, mode, rx, ry);\n")
          .append("      ctx.fillStyle = 'rgba(0,0,0,0.45)'; ctx.beginPath(); ctx.ellipse(sh[0], sh[1], 4.5, 2.5, 0, 0, Math.PI*2); ctx.fill();\n")
          .append("      const bp = proj(balls[i], balls[i+1], balls[i+2], mode, rx, ry);\n")
          .append("      ctx.fillStyle = balls[i+2] > 0.16 ? '#FACC15' : '#EAB308'; ctx.beginPath(); ctx.arc(bp[0], bp[1], balls[i+2] > 0.16 ? 5.5 : 4.2, 0, Math.PI*2); ctx.fill();\n")
          .append("    }\n")
          .append("  }\n")
          .append("  c.addEventListener('mousedown', e => { dragging = true; dragX = e.clientX; dragY = e.clientY; c.style.cursor = 'grabbing'; });\n")
          .append("  window.addEventListener('mouseup', () => { dragging = false; c.style.cursor = 'grab'; });\n")
          .append("  window.addEventListener('mousemove', e => { if (!dragging) return; orbitYaw += (e.clientX - dragX) * 0.008; orbitPitch = Math.max(0.2, Math.min(1.45, orbitPitch + (e.clientY - dragY) * 0.006)); dragX = e.clientX; dragY = e.clientY; draw(); });\n")
          .append("  c.addEventListener('wheel', e => { e.preventDefault(); zoom = Math.max(0.55, Math.min(2.4, zoom * (e.deltaY > 0 ? 0.92 : 1.08))); draw(); }, { passive: false });\n")
          .append("  if (camSel) camSel.addEventListener('change', draw);\n")
          .append("  function loop(ts) {\n")
          .append("    if (!playing) return;\n")
          .append("    const spd = parseFloat(speedSel.value || '1');\n")
          .append("    if (ts - lastTs >= (75 / spd)) {\n")
          .append("      lastTs = ts; idx++; if (idx >= sim3dFrames.length) { idx = 0; playing = false; playBtn.textContent = 'Play'; }\n")
          .append("      draw();\n")
          .append("    }\n")
          .append("    if (playing) requestAnimationFrame(loop);\n")
          .append("  }\n")
          .append("  playBtn.addEventListener('click', () => { playing = !playing; playBtn.textContent = playing ? 'Pause' : 'Play'; if (playing) { lastTs = performance.now(); requestAnimationFrame(loop); } });\n")
          .append("  scrub.addEventListener('input', () => { idx = parseInt(scrub.value, 10) || 0; draw(); });\n")
          .append("  window.seekSim3D = function(sec) {\n")
          .append("    let best = 0; for (let i = 0; i < sim3dFrames.length; i++) { if ((sim3dFrames[i][0] || 0) <= sec) best = i; else break; }\n")
          .append("    idx = best; draw(); document.getElementById('sim3dCard').scrollIntoView({behavior:'smooth'});\n")
          .append("    if (!playing) { playing = true; playBtn.textContent = 'Pause'; lastTs = performance.now(); requestAnimationFrame(loop); }\n")
          .append("  };\n")
          .append("  if (recBtn && c.captureStream && window.MediaRecorder) {\n")
          .append("    let rec = null, chunks = [];\n")
          .append("    recBtn.addEventListener('click', () => {\n")
          .append("      if (rec && rec.state === 'recording') { rec.stop(); return; }\n")
          .append("      chunks = []; const stream = c.captureStream(30);\n")
          .append("      rec = new MediaRecorder(stream, {mimeType: MediaRecorder.isTypeSupported('video/webm;codecs=vp9') ? 'video/webm;codecs=vp9' : 'video/webm'});\n")
          .append("      rec.ondataavailable = e => { if (e.data && e.data.size > 0) chunks.push(e.data); };\n")
          .append("      rec.onstop = () => { const blob = new Blob(chunks, {type:'video/webm'}); const a = document.createElement('a'); a.href = URL.createObjectURL(blob); a.download = 'flux-replay.webm'; a.click(); recBtn.textContent = 'Record Clip (.webm)'; };\n")
          .append("      rec.start(); recBtn.textContent = 'Stop & Save (.webm)';\n")
          .append("      if (!playing) playBtn.click();\n")
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

    StringBuilder csv = new StringBuilder();
    csv.append(
        "RunId,Robot,MatchType,Alliance,NamedBattery,EstRintMilliOhms,MinVoltageV,PeakDriveStatorA,BrownoutCritSec,EnableDropouts,DistanceM,PeakSpeedMps,ShotsCount,OverallVerdict\n");
    for (RunAnalysisReport r : reports) {
      BatteryMetrics b = r.battery();
      csv.append(
          String.format(
              Locale.US,
              "%s,%s,%s,%s,%s,%s,%s,%.0f,%.2f,%d,%.1f,%.2f,%d,%s\n",
              csvQuote(r.runId()),
              csvQuote(r.robotName()),
              csvQuote(r.matchType()),
              csvQuote(r.allianceStation()),
              csvQuote(b.batteryName()),
              fmtFinite(b.estimatedRintMilliOhms(), "%.1f", "NO_DATA"),
              fmtFinite(b.minVoltage(), "%.2f", "NO_DATA"),
              b.peakDriveStatorAmps(),
              b.brownoutCriticalSeconds(),
              b.enableDropouts(),
              r.totalDistanceMeters(),
              r.peakSpeedMps(),
              r.shots().size(),
              csvQuote(r.overallVerdict())));
    }
    Files.writeString(outDir.resolve("runs-index.csv"), csv.toString(), StandardCharsets.UTF_8);

    StringBuilder bcsv = new StringBuilder();
    bcsv.append(
        "NamedBattery,RunId,RestingVoltageV,MinVoltageUnderLoadV,EstimatedRintMilliOhms,PeakSupplyAmps,EnableDropouts,HealthGrade\n");
    for (RunAnalysisReport r : reports) {
      BatteryMetrics b = r.battery();
      bcsv.append(
          String.format(
              Locale.US,
              "%s,%s,%s,%s,%s,%.0f,%d,%s\n",
              csvQuote(b.batteryName()),
              csvQuote(r.runId()),
              fmtFinite(b.restingVoltage(), "%.2f", "NO_DATA"),
              fmtFinite(b.minVoltage(), "%.2f", "NO_DATA"),
              fmtFinite(b.estimatedRintMilliOhms(), "%.1f", "NO_DATA"),
              b.peakSupplyCurrentAmps(),
              b.enableDropouts(),
              csvQuote(b.healthGrade())));
    }
    Files.writeString(
        outDir.resolve("batteries-health.csv"), bcsv.toString(), StandardCharsets.UTF_8);

    StringBuilder html = new StringBuilder(16384);
    html.append("<!DOCTYPE html>\n<html lang=\"en\">\n<head>\n")
        .append("  <meta charset=\"UTF-8\">\n")
        .append("  <title>FLUX 10413 — Season Log &amp; Battery Leaderboard</title>\n")
        .append("  <style>\n")
        .append(
            "    body { font-family: 'Segoe UI', system-ui, -apple-system, BlinkMacSystemFont, sans-serif; background: #F2F3F5; color: #212020; padding: 24px; }\n")
        .append(
            "    .container { max-width: 1240px; margin: 0 auto; display: flex; flex-direction: column; gap: 20px; }\n")
        .append(
            "    .header { background: #FFF; border-top: 6px solid #254BA0; border-radius: 10px; padding: 18px 24px; display: flex; align-items: center; justify-content: space-between; box-shadow: 0 2px 6px rgba(0,0,0,0.06); }\n")
        .append("    .logo { height: 50px; width: auto; flex-shrink: 0; }\n")
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
        .append("      ").append(FLUX_LOGO_SVG).append("\n")
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
      String safeRunDir = sanitizeRelativePath(r.runId());
      String safeVideoFile =
          r.fuelSim() != null ? sanitizeRelativePath(r.fuelSim().videoFileName()) : "";
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
          .append(fmtFinite(b.estimatedRintMilliOhms(), "%.1f mΩ", "NO DATA"))
          .append("</strong></td>\n")
          .append("          <td>")
          .append(fmtFinite(b.minVoltage(), "%.2f V", "N/A"))
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
          .append("          <td style=\"display:flex;gap:6px;\">")
          .append(
              !safeRunDir.isEmpty()
                  ? "<a class=\"btn\" href=\"" + esc(safeRunDir) + "/report.html\">Open Report</a>"
                  : "—")
          .append(
              !safeRunDir.isEmpty() && !safeVideoFile.isEmpty()
                  ? "<a class=\"btn\" style=\"background:#16A34A\" href=\""
                      + esc(safeRunDir)
                      + "/"
                      + esc(safeVideoFile)
                      + "\">Replay Video</a>"
                  : "")
          .append("</td>\n")
          .append("        </tr>\n");
    }
    html.append("      </tbody>\n    </table>\n  </div>\n</div>\n</body>\n</html>\n");
    Files.writeString(outDir.resolve("runs-index.html"), html.toString(), StandardCharsets.UTF_8);
  }

  /**
   * Sanitizes external URLs for HTML {@code href} attributes, permitting only {@code http://} and
   * {@code https://} schemes and rejecting {@code javascript:}, {@code data:}, and injection chars.
   */
  public static String sanitizeExternalUrl(String rawUrl) {
    return RunAnalyzer.isSafeHttpUrl(rawUrl) ? rawUrl.trim() : "";
  }

  /** Sanitizes local relative paths for {@code href}/{@code src} attributes. */
  public static String sanitizeRelativePath(String rawPath) {
    if (rawPath == null) {
      return "";
    }
    String clean = rawPath.trim().replace('\\', '/');
    if (clean.isEmpty()
        || clean.startsWith("/")
        || clean.contains("..")
        || clean.contains(":")) {
      return "";
    }
    for (int i = 0; i < clean.length(); i++) {
      char ch = clean.charAt(i);
      if (ch <= 0x1F || ch == '"' || ch == '\'' || ch == '<' || ch == '>') {
        return "";
      }
    }
    return clean;
  }

  private static String fmtFinite(double val, String format, String fallback) {
    return Double.isFinite(val) ? String.format(Locale.US, format, val) : fallback;
  }

  private static String jsNum(double val, String format) {
    return Double.isFinite(val) ? String.format(Locale.US, format, val) : "null";
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
        .replace("\"", "&quot;")
        .replace("'", "&#39;");
  }

  private static String escJs(String s) {
    if (s == null) {
      return "";
    }
    return s.replace("\\", "\\\\")
        .replace("\"", "\\\"")
        .replace("'", "\\'")
        .replace("</", "<\\/")
        .replace("<", "\\u003c")
        .replace(">", "\\u003e")
        .replace("\u2028", "\\u2028")
        .replace("\u2029", "\\u2029")
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
