package frc.flux.processor.report;

import frc.flux.processor.model.RunAnalysisReport;
import frc.flux.processor.model.RunAnalysisReport.BatteryMetrics;
import frc.flux.processor.model.RunAnalysisReport.CornerMetrics;
import frc.flux.processor.model.RunAnalysisReport.MechanismMetrics;
import frc.flux.processor.model.RunAnalysisReport.ShotEvent;
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
        .append("  </style>\n</head>\n<body>\n<div class=\"container\">\n");

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
        .append("</strong>  •  Merged Log: <code>merged.wpilog</code></div>\n")
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
    sb.append("  <div class=\"kpis\">\n")
        .append(
            kpiCard(
                "Named Battery",
                b.batteryName(),
                String.format(Locale.US, "Est. R_int: %.1f mΩ (%s)", b.estimatedRintMilliOhms(), b.healthGrade())))
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
                "Detected Shots",
                String.valueOf(r.shots().size()),
                String.format(Locale.US, "Enabled: %.1fs / %.1fs", r.enabledDurationSec(), r.totalDurationSec())))
        .append("  </div>\n");

    // Interactive Timeline Canvas
    sb.append("  <div class=\"card\">\n")
        .append("    <h2><span>Battery Voltage (V), Total Drive Stator Current (A) &amp; Shooter Speed (RPS)</span>")
        .append("<span style=\"font-size:12px;font-weight:400;color:var(--muted)\"> Green = Voltage (V) • Blue = Total Stator Current (A) • Gold = Shot Events • Red Dashed = 6.8V Brownout</span></h2>\n")
        .append("    <canvas id=\"timelineCanvas\" width=\"1180\" height=\"220\"></canvas>\n")
        .append("  </div>\n");

    // 6-Second Lookback Voice-to-Sensor Correlation Table
    sb.append("  <div class=\"card\">\n")
        .append("    <h2><span>Part 3: Voice Feedback &amp; 6-Second Sensor Lookback Correlation</span>")
        .append("<span style=\"font-size:12px;font-weight:400;color:var(--muted)\">Driver / Operator / Coach Tracks + Auto-Telemetry Checkpoints</span></h2>\n")
        .append("    <table>\n")
        .append("      <thead><tr><th>Speaker / Role</th><th>Spoken Time</th><th>Spoken Comment / Trigger</th><th>6-Sec Lookback Event</th><th>Sensor Root-Cause Diagnosis</th><th>Video</th></tr></thead>\n")
        .append("      <tbody>\n");
    for (VoiceCorrelation vc : r.voiceCorrelations()) {
      String pillClass =
          "CRITICAL".equals(vc.severity())
              ? "pill-fail"
              : ("WARN".equals(vc.severity()) ? "pill-warn" : "pill-pass");
      sb.append("        <tr>\n")
          .append("          <td><span class=\"pill ").append(pillClass).append("\">")
          .append(esc(vc.speakerRole())).append("</span></td>\n")
          .append("          <td><strong>").append(String.format(Locale.US, "%.1fs", vc.spokenTimeSec())).append("</strong></td>\n")
          .append("          <td>\"").append(esc(vc.transcript())).append("\"</td>\n")
          .append("          <td>").append(esc(vc.correlatedEventSummary())).append("</td>\n")
          .append("          <td>").append(esc(vc.sensorDiagnosis())).append("</td>\n")
          .append("          <td>")
          .append(
              vc.videoLink() != null && !vc.videoLink().isEmpty()
                  ? "<a class=\"btn-link\" target=\"_blank\" href=\"" + esc(vc.videoLink()) + "\">▶ Watch</a>"
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
        .append("        <thead><tr><th>Corner</th><th>CAN IDs</th><th>Mean Drive (A)</th><th>Peak Drive (A)</th><th>Peak Vel (RPS)</th><th>Mean Steer (A)</th><th>Asymmetry</th><th>Status</th></tr></thead>\n")
        .append("        <tbody>\n");
    for (CornerMetrics c : r.swerveCorners()) {
      String pill =
          "FAIL".equals(c.status())
              ? "pill-fail"
              : ("WARN".equals(c.status()) ? "pill-warn" : "pill-pass");
      sb.append("          <tr>\n")
          .append("            <td><strong>").append(esc(c.corner())).append("</strong></td>\n")
          .append("            <td>D:").append(c.driveId()).append(" / S:").append(c.steerId()).append("</td>\n")
          .append("            <td>").append(String.format(Locale.US, "%.1f A", c.meanDriveStatorAmps())).append("</td>\n")
          .append("            <td>").append(String.format(Locale.US, "%.0f A", c.peakDriveStatorAmps())).append("</td>\n")
          .append("            <td>").append(String.format(Locale.US, "%.1f", c.peakDriveVelocityRps())).append("</td>\n")
          .append("            <td>").append(String.format(Locale.US, "%.1f A", c.meanSteerStatorAmps())).append("</td>\n")
          .append("            <td>").append(String.format(Locale.US, "%+.1f%%", c.driveAsymmetryPct())).append("</td>\n")
          .append("            <td><span class=\"pill ").append(pill).append("\">").append(esc(c.status())).append("</span></td>\n")
          .append("          </tr>\n");
    }
    sb.append("        </tbody>\n      </table>\n    </div>\n");

    // Right: Mechanism Health Audit
    sb.append("    <div class=\"card\">\n")
        .append("      <h2>Mechanism Health Audit (Mech CANivore)</h2>\n")
        .append("      <table>\n")
        .append("        <thead><tr><th>Mechanism</th><th>ID</th><th>Mean (A)</th><th>Peak (A)</th><th>Peak Vel (RPS)</th><th>Pos Range</th><th>Resets</th><th>Status</th></tr></thead>\n")
        .append("        <tbody>\n");
    for (MechanismMetrics m : r.mechanisms()) {
      String pill = "WARN".equals(m.status()) ? "pill-warn" : "pill-pass";
      sb.append("          <tr>\n")
          .append("            <td><strong>").append(esc(m.name())).append("</strong></td>\n")
          .append("            <td>").append(m.motorId()).append("</td>\n")
          .append("            <td>").append(String.format(Locale.US, "%.1f A", m.meanStatorAmps())).append("</td>\n")
          .append("            <td>").append(String.format(Locale.US, "%.0f A", m.peakStatorAmps())).append("</td>\n")
          .append("            <td>").append(String.format(Locale.US, "%.1f", m.peakVelocityRps())).append("</td>\n")
          .append("            <td>").append(String.format(Locale.US, "[%.1f .. %.1f]", m.minPositionRot(), m.maxPositionRot())).append("</td>\n")
          .append("            <td>").append(m.resetCount()).append("</td>\n")
          .append("            <td><span class=\"pill ").append(pill).append("\">").append(esc(m.status())).append("</span></td>\n")
          .append("          </tr>\n");
    }
    sb.append("        </tbody>\n      </table>\n    </div>\n  </div>\n");

    // Shot-by-Shot Table
    sb.append("  <div class=\"card\">\n")
        .append("    <h2><span>Detected Shot / Feed Events (")
        .append(r.shots().size())
        .append(")</span><span style=\"font-size:12px;font-weight:400;color:var(--muted)\">Triggered when Feeder + Shooter Flywheels are active simultaneously</span></h2>\n")
        .append("    <table>\n")
        .append("      <thead><tr><th>Shot #</th><th>Time (s)</th><th>Duration</th><th>Shooter Left (RPS)</th><th>Shooter Right (RPS)</th><th>L/R Δ (RPS)</th><th>Hood Pos (rot)</th><th>Feeder Peak (A)</th><th>Min Battery (V)</th><th>Status</th><th>Video</th></tr></thead>\n")
        .append("      <tbody>\n");
    for (ShotEvent s : r.shots()) {
      String pill = "NOMINAL".equals(s.status()) ? "pill-pass" : "pill-warn";
      sb.append("        <tr>\n")
          .append("          <td><strong>#").append(s.shotNumber()).append("</strong></td>\n")
          .append("          <td>").append(String.format(Locale.US, "%.1fs", s.timestampSec())).append("</td>\n")
          .append("          <td>").append(String.format(Locale.US, "%.2fs", s.durationSec())).append("</td>\n")
          .append("          <td>").append(String.format(Locale.US, "%.1f", s.shooterLeftRps())).append("</td>\n")
          .append("          <td>").append(String.format(Locale.US, "%.1f", s.shooterRightRps())).append("</td>\n")
          .append("          <td>").append(String.format(Locale.US, "%.1f", s.shooterAsymmetryRps())).append("</td>\n")
          .append("          <td>").append(String.format(Locale.US, "%.2f", s.hoodPositionRot())).append("</td>\n")
          .append("          <td>").append(String.format(Locale.US, "%.1f A", s.feederPeakStatorAmps())).append("</td>\n")
          .append("          <td>").append(String.format(Locale.US, "%.2f V", s.minBatteryVoltageDuringShot())).append("</td>\n")
          .append("          <td><span class=\"pill ").append(pill).append("\">").append(esc(s.status())).append("</span></td>\n")
          .append("          <td>")
          .append(
              s.videoLink() != null && !s.videoLink().isEmpty()
                  ? "<a class=\"btn-link\" target=\"_blank\" href=\"" + esc(s.videoLink()) + "\">▶ Clip</a>"
                  : "—")
          .append("</td>\n")
          .append("        </tr>\n");
    }
    sb.append("      </tbody>\n    </table>\n  </div>\n");

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
      sb.append(String.format(Locale.US, "%.2f%s", r.shots().get(i).timestampSec(), (i + 1 < r.shots().size()) ? "," : ""));
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
        .append("  timeline.forEach((p, i) => { const x = xOf(p[0]), y = yAmps(p[2]); if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y); });\n")
        .append("  ctx.stroke();\n")
        .append("  // Draw Battery Voltage (Green)\n")
        .append("  ctx.strokeStyle = '#22C55E'; ctx.lineWidth = 2; ctx.beginPath();\n")
        .append("  timeline.forEach((p, i) => { const x = xOf(p[0]), y = yVolt(p[1]); if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y); });\n")
        .append("  ctx.stroke();\n")
        .append("  // Draw Shot Markers (Gold)\n")
        .append("  ctx.fillStyle = '#F59E0B';\n")
        .append("  shots.forEach(t => { const x = xOf(t); ctx.beginPath(); ctx.arc(x, padT + 8, 4, 0, Math.PI*2); ctx.fill(); });\n")
        .append("  // Axes labels\n")
        .append("  ctx.fillStyle = '#A1A1AA'; ctx.font = '11px sans-serif';\n")
        .append("  ctx.fillText('14V', 12, yVolt(14)+4); ctx.fillText('10V', 12, yVolt(10)+4); ctx.fillText('6.8V', 8, yVolt(6.8)+4);\n")
        .append("  ctx.fillText('600A', W-42, yAmps(600)+4); ctx.fillText('300A', W-42, yAmps(300)+4); ctx.fillText('0A', W-42, yAmps(0)+4);\n")
        .append("})();\n</script>\n</body>\n</html>\n");

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
              r.runId(),
              r.robotName(),
              r.matchType(),
              r.allianceStation(),
              b.batteryName(),
              b.estimatedRintMilliOhms(),
              b.minVoltage(),
              b.peakDriveStatorAmps(),
              b.brownoutCriticalSeconds(),
              b.enableDropouts(),
              r.totalDistanceMeters(),
              r.peakSpeedMps(),
              r.shots().size(),
              r.overallVerdict()));
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
              "%s,%s,%.2f,%.2f,%.1f,%.0f,%d,\"%s\"\n",
              b.batteryName(),
              r.runId(),
              b.restingVoltage(),
              b.minVoltage(),
              b.estimatedRintMilliOhms(),
              b.peakSupplyCurrentAmps(),
              b.enableDropouts(),
              b.healthGrade()));
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
        .append("    body { font-family: 'Space Grotesk', system-ui, sans-serif; background: #F2F3F5; color: #212020; padding: 24px; }\n")
        .append("    .container { max-width: 1240px; margin: 0 auto; display: flex; flex-direction: column; gap: 20px; }\n")
        .append("    .header { background: #FFF; border-top: 6px solid #254BA0; border-radius: 10px; padding: 18px 24px; display: flex; align-items: center; justify-content: space-between; box-shadow: 0 2px 6px rgba(0,0,0,0.06); }\n")
        .append("    .logo { height: 50px; }\n")
        .append("    .card { background: #FFF; border-radius: 10px; padding: 18px 22px; box-shadow: 0 1px 4px rgba(0,0,0,0.05); }\n")
        .append("    h2 { font-size: 17px; margin-bottom: 12px; border-bottom: 2px solid #E4E7EC; padding-bottom: 6px; }\n")
        .append("    table { width: 100%; border-collapse: collapse; font-size: 13px; }\n")
        .append("    th { background: #212020; color: #FFF; text-align: left; padding: 9px 12px; }\n")
        .append("    td { padding: 8px 12px; border-bottom: 1px solid #E4E7EC; }\n")
        .append("    tr:nth-child(even) td { background: #F8F9FB; }\n")
        .append("    .pill { padding: 3px 8px; border-radius: 4px; font-weight: 700; font-size: 11px; }\n")
        .append("    .pill-pass { background: #DCFCE7; color: #166534; }\n")
        .append("    .pill-warn { background: #FEF3C7; color: #92400E; }\n")
        .append("    .pill-fail { background: #FEE2E2; color: #991B1B; }\n")
        .append("    a.btn { background: #254BA0; color: #FFF; text-decoration: none; padding: 4px 10px; border-radius: 4px; font-size: 12px; font-weight: 600; }\n")
        .append("  </style>\n</head>\n<body>\n<div class=\"container\">\n")
        .append("  <div class=\"header\">\n")
        .append("    <div style=\"display:flex;align-items:center;gap:16px;\">\n")
        .append("      <img class=\"logo\" src=\"").append(LOGO_URL).append("\" alt=\"FLUX Robotics\">\n")
        .append("      <div><h1 style=\"font-size:21px;\">FRC Team 10413 (FLUX Robotics) — FluxProcessor Index</h1>\n")
        .append("      <div style=\"font-size:12px;color:#5A606E;\">Automated Match &amp; Practice Audit • Battery Internal Resistance Leaderboard • Swerve Symmetry</div></div>\n")
        .append("    </div>\n  </div>\n")
        .append("  <div class=\"card\">\n")
        .append("    <h2>Analyzed Matches &amp; Practice Sessions (").append(reports.size()).append(")</h2>\n")
        .append("    <table>\n")
        .append("      <thead><tr><th>Run / Match</th><th>Alliance</th><th>Battery</th><th>Est. R_int</th><th>Min Volt</th><th>Peak Drive Stator</th><th>Dropouts</th><th>Distance</th><th>Shots</th><th>Verdict</th><th>Report</th></tr></thead>\n")
        .append("      <tbody>\n");

    for (RunAnalysisReport r : reports) {
      BatteryMetrics b = r.battery();
      String pill =
          "CRITICAL".equals(r.overallVerdict())
              ? "pill-fail"
              : ("WARN".equals(r.overallVerdict()) ? "pill-warn" : "pill-pass");
      html.append("        <tr>\n")
          .append("          <td><strong>").append(esc(r.runId())).append("</strong></td>\n")
          .append("          <td>").append(esc(r.allianceStation())).append("</td>\n")
          .append("          <td>").append(esc(b.batteryName())).append("</td>\n")
          .append("          <td><strong>").append(String.format(Locale.US, "%.1f mΩ", b.estimatedRintMilliOhms())).append("</strong></td>\n")
          .append("          <td>").append(String.format(Locale.US, "%.2f V", b.minVoltage())).append("</td>\n")
          .append("          <td>").append(String.format(Locale.US, "%.0f A", b.peakDriveStatorAmps())).append("</td>\n")
          .append("          <td>").append(b.enableDropouts()).append("</td>\n")
          .append("          <td>").append(String.format(Locale.US, "%.1f m", r.totalDistanceMeters())).append("</td>\n")
          .append("          <td>").append(r.shots().size()).append("</td>\n")
          .append("          <td><span class=\"pill ").append(pill).append("\">").append(esc(r.overallVerdict())).append("</span></td>\n")
          .append("          <td><a class=\"btn\" href=\"").append(esc(r.runId())).append("/report.html\">Open Report</a></td>\n")
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
}
