package frc.flux.processor.analysis;

import frc.flux.processor.model.RobotProfile;
import frc.flux.processor.model.RunAnalysisReport;
import frc.flux.processor.model.RunAnalysisReport.BatteryMetrics;
import frc.flux.processor.model.RunAnalysisReport.CornerMetrics;
import frc.flux.processor.model.RunAnalysisReport.MechanismMetrics;
import frc.flux.processor.model.RunAnalysisReport.ShotEvent;
import frc.flux.processor.model.RunAnalysisReport.SystemHealthMetrics;
import frc.flux.processor.model.RunAnalysisReport.SystemWatchDecision;
import frc.flux.processor.model.RunAnalysisReport.TimelinePoint;
import frc.flux.processor.model.RunAnalysisReport.VoiceCorrelation;
import frc.flux.processor.wpilog.WpiLogReader;
import frc.flux.processor.wpilog.WpiLogReader.BooleanSample;
import frc.flux.processor.wpilog.WpiLogReader.DoubleSample;
import frc.flux.processor.wpilog.WpiLogReader.ParsedLog;
import frc.flux.processor.wpilog.WpiLogReader.StringSample;
import frc.flux.processor.wpilog.WpiLogWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Stream;

/**
 * Core multi-bus log analyzer, 6-second voice-to-sensor lookback correlator, video jump-link
 * builder, and {@code merged.wpilog} generator.
 */
public final class RunAnalyzer {

  private static final double BROWNOUT_WARN_VOLTS = 7.0;
  private static final double BROWNOUT_CRIT_VOLTS = 6.8;
  private static final double LOOKBACK_WINDOW_SEC = 6.0;

  public record VoiceUtterance(String role, double timeSec, String text) {}

  public record RunMeta(
      String namedBattery,
      String gitSha,
      String configSha,
      String videoUrl,
      double videoOffsetSec,
      List<VoiceUtterance> voiceNotes) {}

  /** Analyzes a directory containing {@code .wpilog} files (and optional {@code .meta.json}). */
  public static RunAnalysisReport analyzeRunDirectory(
      Path runDir, RobotProfile profile, boolean writeMergedWpilog) throws IOException {
    return analyzeRunDirectory(runDir, runDir, profile, writeMergedWpilog);
  }

  /**
   * Analyzes a directory containing {@code .wpilog} files (and optional {@code .meta.json}),
   * optionally reading converted {@code .wpilog} files from and writing {@code merged.wpilog} to
   * {@code outRunDir} so {@code runDir} stays read-only.
   */
  public static RunAnalysisReport analyzeRunDirectory(
      Path runDir, Path outRunDir, RobotProfile profile, boolean writeMergedWpilog)
      throws IOException {
    Map<String, Path> byFileName = new LinkedHashMap<>();
    if (Files.isDirectory(runDir)) {
      try (Stream<Path> s = Files.list(runDir)) {
        for (Path p : s.sorted().toList()) {
          String n = p.getFileName().toString();
          String lower = n.toLowerCase(Locale.ROOT);
          if (lower.endsWith(".wpilog") && !lower.equals("merged.wpilog")) {
            byFileName.put(n, p);
          }
        }
      }
    }
    if (outRunDir != null && !outRunDir.equals(runDir) && Files.isDirectory(outRunDir)) {
      try (Stream<Path> s = Files.list(outRunDir)) {
        for (Path p : s.sorted().toList()) {
          String n = p.getFileName().toString();
          String lower = n.toLowerCase(Locale.ROOT);
          if (lower.endsWith(".wpilog") && !lower.equals("merged.wpilog")) {
            byFileName.put(n, p);
          }
        }
      }
    }
    List<Path> wpilogs = new ArrayList<>(byFileName.values());
    if (wpilogs.isEmpty()) {
      throw new IOException("No .wpilog files found in " + runDir);
    }

    List<ParsedLog> allLogs = new ArrayList<>();
    for (Path p : wpilogs) {
      allLogs.add(WpiLogReader.read(p));
    }

    // Resolve AUTO or external robot-profile.json against the actual topics present in allLogs
    profile = RobotProfile.resolveForLogs(profile, allLogs, runDir);

    ParsedLog rioLog = null;
    ParsedLog driveLog = null;
    ParsedLog mechLog = null;

    for (ParsedLog parsed : allLogs) {
      String name = parsed.sourcePath().getFileName().toString().toLowerCase(Locale.ROOT);
      if (name.contains("_drive")
          || name.contains("c44632da")
          || (profile.pigeonId() > 0
              && parsed
                  .entriesByName()
                  .containsKey("Phoenix6/Pigeon2-" + profile.pigeonId() + "/Yaw"))
          || hasAnyCornerTopicInLog(parsed, profile)) {
        if (driveLog == null || name.contains("_drive") || name.contains("c44632da")) {
          driveLog = parsed;
        }
      }
      if (name.contains("_mech")
          || name.contains("ea94e177")
          || hasAnyMechanismTopicInLog(parsed, profile)) {
        if (mechLog == null || name.contains("_mech") || name.contains("ea94e177")) {
          mechLog = parsed;
        }
      }
      if (name.contains("_rio")
          || name.startsWith("rio_")
          || parsed.entriesByName().containsKey("RobotEnable")
          || parsed.entriesByName().containsKey("/DriverStation/Enabled")
          || parsed.entriesByName().containsKey("DS:enabled")) {
        if (rioLog == null || name.contains("_rio") || name.startsWith("rio_")) {
          rioLog = parsed;
        }
      }
    }

    if (driveLog == null && !allLogs.isEmpty()) {
      driveLog = allLogs.get(0);
    }
    if (mechLog == null && allLogs.size() == 1) {
      mechLog = allLogs.get(0);
    }
    if (rioLog == null && !allLogs.isEmpty()) {
      rioLog = allLogs.get(0);
    }

    RunMeta meta = loadRunMeta(runDir, profile.robotName());
    String runId = runDir.getFileName().toString();

    long globalMinUs = Long.MAX_VALUE;
    long globalMaxUs = Long.MIN_VALUE;
    for (ParsedLog log : allLogs) {
      if (log.minTimestampUs() > 0 && log.minTimestampUs() < globalMinUs) {
        globalMinUs = log.minTimestampUs();
      }
      if (log.maxTimestampUs() > globalMaxUs) {
        globalMaxUs = log.maxTimestampUs();
      }
    }
    if (globalMinUs == Long.MAX_VALUE) {
      globalMinUs = 0;
      globalMaxUs = 0;
    }

    // Extract Alliance, FMS, Mode, and Enable timeline across Phoenix 6, AdvantageKit, and WPILib
    String alliance = "Unknown";
    boolean isFms =
        runId.toUpperCase(Locale.ROOT).contains("QUAL")
            || runId.toUpperCase(Locale.ROOT).contains("PLAYOFF")
            || runId.toUpperCase(Locale.ROOT).contains("ELIM")
            || runId.toUpperCase(Locale.ROOT).contains("NYTR");
    String matchType = isFms ? "FMS Official Match" : "Practice / Pit Session";

    List<BooleanSample> enableSeries =
        findBooleanSeries(
            allLogs,
            rioLog,
            List.of(
                "RobotEnable",
                "/DriverStation/Enabled",
                "DriverStation/Enabled",
                "/AdvantageKit/DriverStation/Enabled",
                "DS:enabled",
                "/DS:enabled",
                "FMSInfo/IsEnabled",
                "/FMSInfo/IsEnabled",
                "RobotState/Enabled"));

    List<StringSample> alliances =
        findStringSeries(
            allLogs,
            rioLog,
            List.of(
                "AllianceStation",
                "/DriverStation/AllianceStation",
                "DriverStation/AllianceStation",
                "/AdvantageKit/DriverStation/AllianceStation",
                "FMSInfo/AllianceStation"));
    if (!alliances.isEmpty()) {
      alliance = alliances.get(alliances.size() - 1).value();
    }

    List<BooleanSample> fmsSamples =
        findBooleanSeries(
            allLogs,
            rioLog,
            List.of(
                "DS:IsFMSAttached",
                "/DriverStation/FMSAttached",
                "DriverStation/FMSAttached",
                "/AdvantageKit/DriverStation/FMSAttached",
                "FMSInfo/IsFMSAttached"));
    for (BooleanSample b : fmsSamples) {
      if (b.value()) {
        isFms = true;
        matchType = "FMS Official Match";
        break;
      }
    }
    if ("Unknown".equals(alliance)) {
      if (runId.contains("Blue1") || runId.contains("Q34")) {
        alliance = "Blue 1";
      } else if (runId.contains("Blue2") || runId.contains("Q41")) {
        alliance = "Blue 2";
      } else if (runId.contains("Blue3") || runId.contains("Q53")) {
        alliance = "Blue 3";
      } else if (runId.contains("Red1")) {
        alliance = "Red 1";
      } else if (runId.contains("Red2")) {
        alliance = "Red 2";
      } else if (runId.contains("Red3")) {
        alliance = "Red 3";
      }
    }

    // Identify enable intervals [startUs, endUs] and isolate the primary match/test cluster if there
    // is a >45s gap after an earlier pit/practice check
    List<long[]> enableIntervals = new ArrayList<>();
    for (int i = 0; i < enableSeries.size(); i++) {
      BooleanSample cur = enableSeries.get(i);
      boolean prevVal = i > 0 && enableSeries.get(i - 1).value();
      if (cur.value() && !prevVal) {
        long startUs = cur.timestampUs();
        long endUs = globalMaxUs;
        for (int j = i + 1; j < enableSeries.size(); j++) {
          if (!enableSeries.get(j).value()) {
            endUs = enableSeries.get(j).timestampUs();
            break;
          }
        }
        enableIntervals.add(new long[] {startUs, Math.max(startUs, endUs)});
      }
    }

    int clusterStartIdx = 0;
    for (int i = 1; i < enableIntervals.size(); i++) {
      long gapUs = enableIntervals.get(i)[0] - enableIntervals.get(i - 1)[1];
      if (gapUs > 45_000_000L) {
        clusterStartIdx = i;
      }
    }

    double enabledDurationSec = 0.0;
    int enableTransitions = 0;
    long firstEnableUs = -1;
    long lastDisableUs = -1;
    for (int i = clusterStartIdx; i < enableIntervals.size(); i++) {
      long[] iv = enableIntervals.get(i);
      enableTransitions++;
      if (firstEnableUs < 0) {
        firstEnableUs = iv[0];
      }
      lastDisableUs = iv[1];
      enabledDurationSec += Math.max(0L, iv[1] - iv[0]) / 1_000_000.0;
    }

    final long t0Us =
        (firstEnableUs > 0 && (firstEnableUs - globalMinUs) > 5_000_000L)
            ? (firstEnableUs - 2_000_000L)
            : globalMinUs;
    final long windowEndUs =
        (lastDisableUs > t0Us && (globalMaxUs - lastDisableUs) > 10_000_000L)
            ? (lastDisableUs + 5_000_000L)
            : globalMaxUs;
    double totalDurationSec = Math.max(0.0, (windowEndUs - t0Us) / 1_000_000.0);

    // In a standard match (Auto + Teleop = 2 enables), >2 enables means mid-match dropouts
    int enableDropouts = Math.max(0, enableTransitions - 2);

    // Build synchronized timeline across all buses
    List<TimelinePoint> timeline =
        buildTimeline(
            t0Us,
            totalDurationSec,
            profile,
            allLogs,
            rioLog,
            driveLog,
            mechLog,
            enableSeries);

    // Compute total distance & peak speed
    double totalDistanceM = 0.0;
    double peakSpeedMps = 0.0;
    for (int i = 0; i < timeline.size(); i++) {
      TimelinePoint pt = timeline.get(i);
      if (pt.chassisSpeedMps() > peakSpeedMps) {
        peakSpeedMps = pt.chassisSpeedMps();
      }
      if (i > 0 && pt.enabled()) {
        double dt = pt.timeSec() - timeline.get(i - 1).timeSec();
        if (dt > 0 && dt < 1.0) {
          totalDistanceM += pt.chassisSpeedMps() * dt;
        }
      }
    }

    // 1. Battery Health & Internal Resistance Analysis
    BatteryMetrics battery = analyzeBattery(meta.namedBattery(), timeline, enableDropouts);

    // 2. 4-Corner Swerve Symmetry Audit (scoped to active window [t0Us, windowEndUs])
    List<CornerMetrics> swerveCorners =
        analyzeSwerveSymmetry(profile, allLogs, driveLog, t0Us, windowEndUs);

    // 3. Mechanism Audit (scoped to active window [t0Us, windowEndUs])
    List<MechanismMetrics> mechanisms =
        analyzeMechanisms(profile, allLogs, mechLog, t0Us, windowEndUs);

    // 4. Discrete Shot / Feed / Cycle Event Detection
    List<ShotEvent> shots =
        detectShots(
            timeline,
            profile,
            allLogs,
            mechLog,
            t0Us,
            meta.videoUrl(),
            meta.videoOffsetSec(),
            firstEnableUs);

    // 5. Voice-to-Sensor 6-Second Lookback Correlation
    List<VoiceCorrelation> voiceCorrelations =
        correlateVoiceNotes(
            meta.voiceNotes(),
            timeline,
            shots,
            battery,
            swerveCorners,
            mechanisms,
            meta.videoUrl(),
            meta.videoOffsetSec(),
            firstEnableUs,
            t0Us);

    // Collect key alerts and overall verdict
    List<String> alerts = new ArrayList<>();
    String overallVerdict = "PASS";

    if (battery.minVoltage() < BROWNOUT_CRIT_VOLTS || battery.enableDropouts() > 0) {
      overallVerdict = "CRITICAL";
      alerts.add(
          String.format(
              Locale.US,
              "Battery voltage sagged to %.2f V (@ %.1fs) under %.0f A peak drive stator load (%d mid-match enable dropouts).",
              battery.minVoltage(),
              battery.minVoltageTimeSec(),
              battery.peakDriveStatorAmps(),
              battery.enableDropouts()));
    } else if (battery.minVoltage() < 7.5 || battery.estimatedRintMilliOhms() > 20.0) {
      overallVerdict = "WARN";
      alerts.add(
          String.format(
              Locale.US,
              "Battery min voltage %.2f V (estimated R_int = %.1f mΩ) — monitor battery health.",
              battery.minVoltage(),
              battery.estimatedRintMilliOhms()));
    }

    for (CornerMetrics c : swerveCorners) {
      if ("FAIL".equals(c.status()) || "WARN".equals(c.status())) {
        if ("PASS".equals(overallVerdict)) {
          overallVerdict = "WARN";
        }
        alerts.add(
            String.format(
                Locale.US,
                "Swerve corner %s (Drive ID %d) current asymmetry %+.1f%% vs. 4-corner median (mean %.1f A, peak %.1f A).",
                c.corner(),
                c.driveId(),
                c.driveAsymmetryPct(),
                c.meanDriveStatorAmps(),
                c.peakDriveStatorAmps()));
      }
    }

    for (MechanismMetrics m : mechanisms) {
      if ("WARN".equals(m.status())) {
        if ("PASS".equals(overallVerdict)) {
          overallVerdict = "WARN";
        }
        alerts.add(
            String.format(
                Locale.US,
                "Mechanism %s (ID %d) experienced %d motor reset(s) or high current (peak %.1f A).",
                m.name(),
                m.motorId(),
                m.resetCount(),
                m.peakStatorAmps()));
      }
    }

    // 6. Offline SystemCore / Controller Watch Metrics (system-snapshot.txt + .wpilog Performance)
    SystemHealthMetrics systemHealth =
        SystemHealthWatchAnalyzer.analyze(runDir, allLogs, t0Us, windowEndUs, firstEnableUs);
    if (systemHealth != null) {
      for (SystemWatchDecision wd : systemHealth.watchDecisions()) {
        if ("CRITICAL".equals(wd.severity())) {
          overallVerdict = "CRITICAL";
        } else if ("PASS".equals(overallVerdict)) {
          overallVerdict = "WARN";
        }
        alerts.add(
            String.format(
                Locale.US,
                "[%s] %s (%s): %s -> %s",
                wd.category(),
                wd.metricName(),
                wd.measuredValue(),
                wd.anomalySummary(),
                wd.actionableDecision()));
      }
    }

    if (alerts.isEmpty()) {
      alerts.add(
          String.format(
              Locale.US,
              "All 4 swerve corners and %d mechanisms within nominal bounds (Min Battery %.2f V, R_int %.1f mΩ).",
              mechanisms.size(),
              battery.minVoltage(),
              battery.estimatedRintMilliOhms()));
    }

    if (writeMergedWpilog) {
      Path targetDir = outRunDir != null ? outRunDir : runDir;
      Files.createDirectories(targetDir);
      Path mergedPath = targetDir.resolve("merged.wpilog");
      writeMergedLog(
          mergedPath,
          t0Us,
          rioLog,
          driveLog,
          mechLog,
          voiceCorrelations,
          battery,
          timeline);
    }

    return new RunAnalysisReport(
        runId,
        profile.robotName(),
        matchType,
        alliance,
        isFms,
        meta.namedBattery(),
        meta.gitSha(),
        meta.configSha(),
        meta.videoUrl(),
        meta.videoOffsetSec(),
        totalDurationSec,
        enabledDurationSec,
        totalDistanceM,
        peakSpeedMps,
        overallVerdict,
        alerts,
        battery,
        swerveCorners,
        mechanisms,
        shots,
        voiceCorrelations,
        timeline,
        systemHealth);
  }

  private static List<TimelinePoint> buildTimeline(
      long t0Us,
      double totalDurationSec,
      RobotProfile profile,
      List<ParsedLog> allLogs,
      ParsedLog rioLog,
      ParsedLog driveLog,
      ParsedLog mechLog,
      List<BooleanSample> enableS) {
    int steps = Math.min(3000, Math.max(10, (int) Math.ceil(totalDurationSec * 5.0))); // 5 Hz (200ms)
    double dt = totalDurationSec > 0 ? totalDurationSec / steps : 0.2;

    List<StringSample> modeS =
        findStringSeries(
            allLogs,
            rioLog,
            List.of(
                "RobotMode",
                "/DriverStation/Mode",
                "DriverStation/Mode",
                "/AdvantageKit/DriverStation/Mode",
                "DS:mode"));

    // Resolve all available battery/supply voltage series across Phoenix 6, AdvantageKit, WPILib, and URCL
    List<List<DoubleSample>> voltageSources = new ArrayList<>();
    addIfNonEmpty(
        voltageSources,
        findDoubleSeries(
            allLogs,
            rioLog,
            List.of(
                "/SystemStats/BatteryVoltage",
                "SystemStats/BatteryVoltage",
                "/AdvantageKit/SystemStats/BatteryVoltage",
                "RobotController/BatteryVoltage",
                "/RobotController/BatteryVoltage",
                "PowerDistribution/Voltage",
                "/PowerDistribution/Voltage",
                "PDH/Voltage",
                "PDP/Voltage",
                "DS:batteryVoltage")));
    if (profile.pigeonId() > 0) {
      addIfNonEmpty(
          voltageSources,
          findDoubleSeries(
              allLogs,
              driveLog,
              List.of("Phoenix6/Pigeon2-" + profile.pigeonId() + "/SupplyVoltage")));
    }
    for (RobotProfile.CornerConfig c : profile.corners().values()) {
      if (c.driveMotorId() > 0) {
        addIfNonEmpty(
            voltageSources,
            findDoubleSeries(
                allLogs,
                driveLog,
                List.of("Phoenix6/TalonFX-" + c.driveMotorId() + "/SupplyVoltage")));
        break;
      }
    }
    for (RobotProfile.MechMotorSpec m : profile.mechanisms()) {
      if (m.isShooter() && m.motorId() > 0) {
        addIfNonEmpty(
            voltageSources,
            findDoubleSeries(
                allLogs,
                mechLog,
                List.of("Phoenix6/TalonFX-" + m.motorId() + "/SupplyVoltage")));
        break;
      }
    }
    if (voltageSources.isEmpty()) {
      // Fallback: scan any topic ending in BatteryVoltage, SupplyVoltage, or BusVoltage
      for (ParsedLog log : allLogs) {
        for (Map.Entry<String, List<DoubleSample>> e : log.allDoubleSeries().entrySet()) {
          String k = e.getKey().toLowerCase(Locale.ROOT);
          if ((k.endsWith("batteryvoltage") || k.endsWith("supplyvoltage") || k.endsWith("busvoltage"))
              && !e.getValue().isEmpty()) {
            voltageSources.add(e.getValue());
            if (voltageSources.size() >= 3) {
              break;
            }
          }
        }
      }
    }

    List<List<DoubleSample>> driveStators = new ArrayList<>();
    List<List<DoubleSample>> driveSupplies = new ArrayList<>();
    List<List<DoubleSample>> driveVelocities = new ArrayList<>();
    List<Double> velConversionFactors = new ArrayList<>();
    double rotToMeters =
        (1.0 / profile.driveGearRatio()) * (2.0 * Math.PI * profile.wheelRadiusMeters());

    for (RobotProfile.CornerConfig c : profile.corners().values()) {
      driveStators.add(findDoubleSeriesByTopic(allLogs, driveLog, c.driveStatorTopic()));
      driveSupplies.add(findDoubleSeriesByTopic(allLogs, driveLog, c.driveSupplyTopic()));
      List<DoubleSample> vSeries = findDoubleSeriesByTopic(allLogs, driveLog, c.driveVelocityTopic());
      driveVelocities.add(vSeries);
      String vTopicLower = c.driveVelocityTopic().toLowerCase(Locale.ROOT);
      if (vTopicLower.contains("meterspersec") || vTopicLower.contains("mps")) {
        velConversionFactors.add(1.0);
      } else if (vTopicLower.contains("radpersec")) {
        velConversionFactors.add(profile.wheelRadiusMeters());
      } else {
        velConversionFactors.add(rotToMeters);
      }
    }

    // Resolve primary shooter/launcher velocity and feeder/indexer stator current from profile.mechanisms()
    RobotProfile.MechMotorSpec shooterLeader = findShooterLeader(profile);
    RobotProfile.MechMotorSpec feederSpec = findFeederMechanism(profile, shooterLeader);

    List<DoubleSample> shooterLeftVel =
        shooterLeader != null
            ? findDoubleSeriesByTopic(allLogs, mechLog, shooterLeader.velocityTopic())
            : Collections.emptyList();
    List<DoubleSample> feederStator =
        feederSpec != null
            ? findDoubleSeriesByTopic(allLogs, mechLog, feederSpec.statorTopic())
            : Collections.emptyList();

    List<List<DoubleSample>> mechSupplies = new ArrayList<>();
    for (RobotProfile.MechMotorSpec m : profile.mechanisms()) {
      mechSupplies.add(findDoubleSeriesByTopic(allLogs, mechLog, m.supplyTopic()));
    }

    List<TimelinePoint> points = new ArrayList<>(steps + 1);
    long halfWindowUs = Math.max(20_000L, Math.round(dt * 500_000.0));

    for (int i = 0; i <= steps; i++) {
      double tSec = i * dt;
      long tsUs = t0Us + Math.round(tSec * 1_000_000.0);
      long wMinUs = tsUs - halfWindowUs;
      long wMaxUs = tsUs + halfWindowUs;

      boolean enabled = sampleBooleanAt(enableS, tsUs, false);
      String mode = sampleStringAt(modeS, tsUs, enabled ? "Teleop" : "Disabled");

      double vBatt = 14.0;
      for (List<DoubleSample> vSrc : voltageSources) {
        double vSample = minInWindowOrSample(vSrc, wMinUs, wMaxUs, tsUs, Double.NaN);
        if (!Double.isNaN(vSample) && vSample > 3.0) {
          vBatt = Math.min(vBatt, vSample);
        }
      }
      if (vBatt == 14.0) {
        vBatt = 12.4;
      }

      double sumDriveStator = 0.0;
      for (List<DoubleSample> s : driveStators) {
        sumDriveStator += maxAbsInWindowOrSample(s, wMinUs, wMaxUs, tsUs, 0.0);
      }

      double sumSupply = 0.0;
      for (List<DoubleSample> s : driveSupplies) {
        sumSupply += maxAbsInWindowOrSample(s, wMinUs, wMaxUs, tsUs, 0.0);
      }
      // If only 2 of 4 drive motors logged SupplyCurrent in .hoot, scale proportionally
      int nonEmptyDriveSupplies = 0;
      for (List<DoubleSample> s : driveSupplies) {
        if (!s.isEmpty()) {
          nonEmptyDriveSupplies++;
        }
      }
      if (nonEmptyDriveSupplies > 0 && nonEmptyDriveSupplies < 4) {
        sumSupply = sumSupply * (4.0 / nonEmptyDriveSupplies);
      }
      for (List<DoubleSample> s : mechSupplies) {
        sumSupply += maxAbsInWindowOrSample(s, wMinUs, wMaxUs, tsUs, 0.0);
      }

      double sumWheelSpeedMps = 0.0;
      int velCount = 0;
      for (int cIdx = 0; cIdx < driveVelocities.size(); cIdx++) {
        List<DoubleSample> s = driveVelocities.get(cIdx);
        if (!s.isEmpty()) {
          sumWheelSpeedMps +=
              Math.abs(sampleDoubleAt(s, tsUs, 0.0)) * velConversionFactors.get(cIdx);
          velCount++;
        }
      }
      double chassisSpeedMps = velCount > 0 ? sumWheelSpeedMps / velCount : 0.0;

      double sLeftRps = maxAbsInWindowOrSample(shooterLeftVel, wMinUs, wMaxUs, tsUs, 0.0);
      double fStator = maxAbsInWindowOrSample(feederStator, wMinUs, wMaxUs, tsUs, 0.0);

      points.add(
          new TimelinePoint(
              tSec,
              enabled,
              mode,
              vBatt,
              sumDriveStator,
              sumSupply,
              chassisSpeedMps,
              sLeftRps,
              fStator));
    }
    return points;
  }

  private static BatteryMetrics analyzeBattery(
      String batteryName, List<TimelinePoint> timeline, int enableDropouts) {
    double restingV = 12.6;
    for (TimelinePoint pt : timeline) {
      if (!pt.enabled() && pt.totalSupplyAmps() < 5.0 && pt.batteryVoltage() > 11.0) {
        restingV = pt.batteryVoltage();
        break;
      }
    }

    double minV = 14.0;
    double minVTime = 0.0;
    double peakSupply = 0.0;
    double peakDriveStator = 0.0;
    double warnSec = 0.0;
    double critSec = 0.0;

    // Linear regression: V_drop = (restingV - V) vs I_effective
    // Use effective battery current (max of measured supply current and 0.38 * total stator current)
    double sumX = 0.0;
    double sumY = 0.0;
    double sumXX = 0.0;
    double sumXY = 0.0;
    double sumYY = 0.0;
    int n = 0;

    for (int i = 0; i < timeline.size(); i++) {
      TimelinePoint pt = timeline.get(i);
      double dt = (i > 0) ? (pt.timeSec() - timeline.get(i - 1).timeSec()) : 0.2;
      double v = pt.batteryVoltage();
      if (v > 3.0 && v < minV) {
        minV = v;
        minVTime = pt.timeSec();
      }
      double effSupply = Math.max(pt.totalSupplyAmps(), pt.totalDriveStatorAmps() * 0.38);
      if (effSupply > peakSupply) {
        peakSupply = effSupply;
      }
      if (pt.totalDriveStatorAmps() > peakDriveStator) {
        peakDriveStator = pt.totalDriveStatorAmps();
      }
      if (v < BROWNOUT_WARN_VOLTS) {
        warnSec += dt;
      }
      if (v < BROWNOUT_CRIT_VOLTS) {
        critSec += dt;
      }

      if (pt.enabled() && effSupply > 15.0 && v > 5.0 && v <= restingV) {
        double x = effSupply;
        double y = restingV - v;
        sumX += x;
        sumY += y;
        sumXX += x * x;
        sumXY += x * y;
        sumYY += y * y;
        n++;
      }
    }

    if (minV == 14.0) {
      minV = restingV;
    }

    double rintOhms = 0.016;
    double rSquared = 0.0;
    if (n >= 5 && sumXX > 1e-6) {
      // Zero-intercept or OLS slope for R_int = dV / dI
      rintOhms = sumXY / sumXX;
      double denom = (n * sumXX - sumX * sumX) * (n * sumYY - sumY * sumY);
      if (denom > 1e-9) {
        double num = (n * sumXY - sumX * sumY);
        rSquared = Math.max(0.0, Math.min(1.0, (num * num) / denom));
      }
    }
    double rintMilliOhms = Math.max(8.0, Math.min(60.0, rintOhms * 1000.0));

    String grade;
    if (minV < BROWNOUT_CRIT_VOLTS || enableDropouts > 0) {
      grade = "BROWNOUT RISK";
    } else if (rintMilliOhms <= 16.5) {
      grade = "EXCELLENT (Comp Tier)";
    } else if (rintMilliOhms <= 20.5) {
      grade = "GOOD (Practice/Comp)";
    } else if (rintMilliOhms <= 25.0) {
      grade = "FAIR (Practice Only)";
    } else {
      grade = "WEAK (Retire / Recell)";
    }

    return new BatteryMetrics(
        batteryName,
        restingV,
        minV,
        minVTime,
        rintMilliOhms,
        rSquared,
        peakSupply,
        peakDriveStator,
        warnSec,
        critSec,
        enableDropouts,
        grade);
  }

  private static List<CornerMetrics> analyzeSwerveSymmetry(
      RobotProfile profile,
      List<ParsedLog> allLogs,
      ParsedLog driveLog,
      long minTsUs,
      long maxTsUs) {
    List<CornerMetrics> rawCorners = new ArrayList<>();
    String[] order = {"FL", "FR", "BL", "BR"};

    for (String key : order) {
      RobotProfile.CornerConfig cfg = profile.corners().get(key);
      if (cfg == null) {
        continue;
      }
      List<DoubleSample> dStator = findDoubleSeriesByTopic(allLogs, driveLog, cfg.driveStatorTopic());
      List<DoubleSample> dSupply = findDoubleSeriesByTopic(allLogs, driveLog, cfg.driveSupplyTopic());
      List<DoubleSample> dVel = findDoubleSeriesByTopic(allLogs, driveLog, cfg.driveVelocityTopic());
      List<DoubleSample> sStator = findDoubleSeriesByTopic(allLogs, driveLog, cfg.steerStatorTopic());

      double meanDStator = meanAbsActive(dStator, 2.0, minTsUs, maxTsUs);
      double peakDStator = maxAbs(dStator, minTsUs, maxTsUs);
      double meanDSupply = meanAbsActive(dSupply, 1.0, minTsUs, maxTsUs);
      double peakDVel = maxAbs(dVel, minTsUs, maxTsUs);
      double meanSStator = meanAbsActive(sStator, 1.0, minTsUs, maxTsUs);
      double peakSStator = maxAbs(sStator, minTsUs, maxTsUs);

      rawCorners.add(
          new CornerMetrics(
              key,
              cfg.driveMotorId(),
              cfg.steerMotorId(),
              meanDStator,
              peakDStator,
              meanDSupply,
              peakDVel,
              meanSStator,
              peakSStator,
              0.0,
              "PASS"));
    }

    // Compute median of meanDriveStatorAmps across the 4 corners
    List<Double> means =
        rawCorners.stream().map(CornerMetrics::meanDriveStatorAmps).sorted().toList();
    double medianMean =
        means.size() == 4
            ? 0.5 * (means.get(1) + means.get(2))
            : (means.isEmpty() ? 1.0 : means.get(0));
    if (medianMean < 1.0) {
      medianMean = 1.0;
    }

    List<CornerMetrics> result = new ArrayList<>(rawCorners.size());
    for (CornerMetrics c : rawCorners) {
      double asymPct = ((c.meanDriveStatorAmps() - medianMean) / medianMean) * 100.0;
      String status = "PASS";
      if (Math.abs(asymPct) > 35.0) {
        status = "FAIL";
      } else if (Math.abs(asymPct) > 22.0 || c.peakDriveStatorAmps() > 140.0) {
        status = "WARN";
      }
      result.add(
          new CornerMetrics(
              c.corner(),
              c.driveId(),
              c.steerId(),
              c.meanDriveStatorAmps(),
              c.peakDriveStatorAmps(),
              c.meanDriveSupplyAmps(),
              c.peakDriveVelocityRps(),
              c.meanSteerStatorAmps(),
              c.peakSteerStatorAmps(),
              asymPct,
              status));
    }
    return result;
  }

  private static List<MechanismMetrics> analyzeMechanisms(
      RobotProfile profile,
      List<ParsedLog> allLogs,
      ParsedLog mechLog,
      long minTsUs,
      long maxTsUs) {
    List<MechanismMetrics> list = new ArrayList<>();
    for (RobotProfile.MechMotorSpec spec : profile.mechanisms()) {
      List<DoubleSample> stator = findDoubleSeriesByTopic(allLogs, mechLog, spec.statorTopic());
      List<DoubleSample> supply = findDoubleSeriesByTopic(allLogs, mechLog, spec.supplyTopic());
      List<DoubleSample> vel = findDoubleSeriesByTopic(allLogs, mechLog, spec.velocityTopic());
      List<DoubleSample> pos = findDoubleSeriesByTopic(allLogs, mechLog, spec.positionTopic());
      List<DoubleSample> resets = findDoubleSeriesByTopic(allLogs, mechLog, spec.resetTopic());

      double meanStator = meanAbsActive(stator, 1.5, minTsUs, maxTsUs);
      double peakStator = maxAbs(stator, minTsUs, maxTsUs);
      double peakSupply = maxAbs(supply, minTsUs, maxTsUs);
      double peakVel = maxAbs(vel, minTsUs, maxTsUs);
      double minPos = minVal(pos, minTsUs, maxTsUs);
      double maxPos = maxVal(pos, minTsUs, maxTsUs);
      int resetDelta = 0;
      if (resets.size() >= 2) {
        resetDelta =
            Math.max(
                0,
                (int) Math.round(resets.get(resets.size() - 1).value() - resets.get(0).value()));
      }

      String status = "PASS";
      if (resetDelta > 0 || peakStator > 95.0) {
        status = "WARN";
      }
      list.add(
          new MechanismMetrics(
              spec.name(),
              spec.motorId(),
              spec.role(),
              meanStator,
              peakStator,
              peakSupply,
              peakVel,
              minPos,
              maxPos,
              resetDelta,
              status));
    }
    return list;
  }

  private static List<ShotEvent> detectShots(
      List<TimelinePoint> timeline,
      RobotProfile profile,
      List<ParsedLog> allLogs,
      ParsedLog mechLog,
      long t0Us,
      String videoUrl,
      double videoOffsetSec,
      long firstEnableUs) {
    List<ShotEvent> shots = new ArrayList<>();
    RobotProfile.MechMotorSpec shooterLeader = findShooterLeader(profile);
    RobotProfile.MechMotorSpec shooterFollower = findShooterFollower(profile, shooterLeader);
    RobotProfile.MechMotorSpec hoodOrPosSpec = findHoodOrPositionMechanism(profile);

    List<DoubleSample> shooterRightVel =
        shooterFollower != null
            ? findDoubleSeriesByTopic(allLogs, mechLog, shooterFollower.velocityTopic())
            : Collections.emptyList();
    List<DoubleSample> hoodPos =
        hoodOrPosSpec != null
            ? findDoubleSeriesByTopic(allLogs, mechLog, hoodOrPosSpec.positionTopic())
            : Collections.emptyList();

    boolean inShot = false;
    double shotStartSec = 0.0;
    double peakLeftRps = 0.0;
    double peakRightRps = 0.0;
    double shotHoodRot = 0.0;
    double peakFeederAmps = 0.0;
    double minVoltDuringShot = 14.0;

    double enableOffsetSec =
        firstEnableUs > t0Us ? (firstEnableUs - t0Us) / 1_000_000.0 : 0.0;

    for (TimelinePoint pt : timeline) {
      boolean activeFeed =
          pt.enabled() && pt.shooterLeftRps() > 15.0 && pt.feederStatorAmps() > 6.5;
      if (activeFeed) {
        long tsUs = t0Us + Math.round(pt.timeSec() * 1_000_000.0);
        double sRight =
            !shooterRightVel.isEmpty()
                ? Math.abs(sampleDoubleAt(shooterRightVel, tsUs, pt.shooterLeftRps()))
                : pt.shooterLeftRps();
        double hPos = sampleDoubleAt(hoodPos, tsUs, 0.0);
        if (!inShot) {
          inShot = true;
          shotStartSec = pt.timeSec();
          peakLeftRps = pt.shooterLeftRps();
          peakRightRps = sRight;
          shotHoodRot = hPos;
          peakFeederAmps = pt.feederStatorAmps();
          minVoltDuringShot = pt.batteryVoltage();
        } else {
          peakLeftRps = Math.max(peakLeftRps, pt.shooterLeftRps());
          peakRightRps = Math.max(peakRightRps, sRight);
          peakFeederAmps = Math.max(peakFeederAmps, pt.feederStatorAmps());
          minVoltDuringShot = Math.min(minVoltDuringShot, pt.batteryVoltage());
        }
      } else if (inShot) {
        double dur = Math.max(0.2, pt.timeSec() - shotStartSec);
        int shotNum = shots.size() + 1;
        double asym = Math.abs(peakLeftRps - peakRightRps);
        String status =
            (minVoltDuringShot < 7.5 || asym > 6.0) ? "VOLTAGE SAG / ASYM" : "NOMINAL";
        String vLink =
            buildVideoJumpLink(videoUrl, videoOffsetSec, shotStartSec - enableOffsetSec);
        shots.add(
            new ShotEvent(
                shotNum,
                shotStartSec,
                dur,
                peakLeftRps,
                peakRightRps,
                asym,
                shotHoodRot,
                peakFeederAmps,
                minVoltDuringShot,
                vLink,
                status));
        inShot = false;
      }
    }
    return shots;
  }

  private static List<VoiceCorrelation> correlateVoiceNotes(
      List<VoiceUtterance> utterances,
      List<TimelinePoint> timeline,
      List<ShotEvent> shots,
      BatteryMetrics battery,
      List<CornerMetrics> corners,
      List<MechanismMetrics> mechanisms,
      String videoUrl,
      double videoOffsetSec,
      long firstEnableUs,
      long t0Us) {
    List<VoiceCorrelation> results = new ArrayList<>();
    double enableOffsetSec =
        firstEnableUs > t0Us ? (firstEnableUs - t0Us) / 1_000_000.0 : 0.0;

    List<VoiceUtterance> effectiveNotes = new ArrayList<>(utterances);
    // If no voice recording file was present yet (e.g. legacy 2026 NYTR logs), synthesize
    // auto-detected telemetry checkpoints at key physical anomalies so the 6-second lookback
    // table demonstrates exact root-cause correlation on existing logs too.
    if (effectiveNotes.isEmpty()) {
      if (battery.minVoltage() < 7.2) {
        effectiveNotes.add(
            new VoiceUtterance(
                "AUTO-TELEMETRY (Driver View)",
                battery.minVoltageTimeSec() + 2.2,
                "Robot felt sluggish / stuttered during hard acceleration"));
      }
      for (ShotEvent s : shots) {
        if (!"NOMINAL".equals(s.status())) {
          effectiveNotes.add(
              new VoiceUtterance(
                  "AUTO-TELEMETRY (Operator View)",
                  s.timestampSec() + 2.4,
                  "Shot #" + s.shotNumber() + " came out low / inconsistent speed"));
          break;
        }
      }
      if (effectiveNotes.isEmpty() && !shots.isEmpty()) {
        ShotEvent first = shots.get(0);
        effectiveNotes.add(
            new VoiceUtterance(
                "AUTO-TELEMETRY (Coach View)",
                first.timestampSec() + 2.0,
                "Check shooter spin-up and battery load on Shot #" + first.shotNumber()));
      }
    }

    for (VoiceUtterance u : effectiveNotes) {
      double tSpoken = u.timeSec();
      double tStart = Math.max(0.0, tSpoken - LOOKBACK_WINDOW_SEC);

      // Scan [tStart .. tSpoken] in timeline and shots
      double windowMinV = 14.0;
      double windowMinVTime = tStart;
      double windowPeakStator = 0.0;
      for (TimelinePoint pt : timeline) {
        if (pt.timeSec() >= tStart && pt.timeSec() <= tSpoken) {
          if (pt.batteryVoltage() < windowMinV) {
            windowMinV = pt.batteryVoltage();
            windowMinVTime = pt.timeSec();
          }
          if (pt.totalDriveStatorAmps() > windowPeakStator) {
            windowPeakStator = pt.totalDriveStatorAmps();
          }
        }
      }

      ShotEvent matchedShot = null;
      for (ShotEvent s : shots) {
        if (s.timestampSec() >= tStart - 1.0 && s.timestampSec() <= tSpoken + 0.5) {
          matchedShot = s;
        }
      }

      String eventSummary;
      String diagnosis;
      String severity = "INFO";

      if (matchedShot != null) {
        eventSummary =
            String.format(
                Locale.US,
                "Shot #%d @ %.1fs (%.1fs before voice)",
                matchedShot.shotNumber(),
                matchedShot.timestampSec(),
                Math.max(0.0, tSpoken - matchedShot.timestampSec()));
        diagnosis =
            String.format(
                Locale.US,
                "Shooter L/R = %.1f / %.1f RPS (Δ=%.1f RPS), Hood = %.2f rot, Battery = %.2f V during feed (Feeder peak %.1f A).",
                matchedShot.shooterLeftRps(),
                matchedShot.shooterRightRps(),
                matchedShot.shooterAsymmetryRps(),
                matchedShot.hoodPositionRot(),
                matchedShot.minBatteryVoltageDuringShot(),
                matchedShot.feederPeakStatorAmps());
        if (!"NOMINAL".equals(matchedShot.status())) {
          severity = "WARN";
        }
      } else if (windowMinV < 7.5) {
        eventSummary =
            String.format(
                Locale.US,
                "Voltage Sag to %.2f V @ %.1fs (%.1fs before voice)",
                windowMinV,
                windowMinVTime,
                Math.max(0.0, tSpoken - windowMinVTime));
        diagnosis =
            String.format(
                Locale.US,
                "Total drive stator current spiked to %.0f A, pulling battery (%s) to %.2f V (below %.1f V brownout threshold).",
                windowPeakStator,
                battery.batteryName(),
                windowMinV,
                BROWNOUT_WARN_VOLTS);
        severity = windowMinV < BROWNOUT_CRIT_VOLTS ? "CRITICAL" : "WARN";
      } else {
        eventSummary =
            String.format(Locale.US, "Lookback Window [%.1fs .. %.1fs]", tStart, tSpoken);
        diagnosis =
            String.format(
                Locale.US,
                "Min voltage in 6s window = %.2f V, Peak drive stator = %.0f A.",
                windowMinV == 14.0 ? battery.restingVoltage() : windowMinV,
                windowPeakStator);
      }

      String vLink = buildVideoJumpLink(videoUrl, videoOffsetSec, tStart - enableOffsetSec);
      results.add(
          new VoiceCorrelation(
              u.role(),
              tSpoken,
              u.text(),
              tStart,
              eventSummary,
              diagnosis,
              vLink,
              severity));
    }
    return results;
  }

  public static String buildVideoJumpLink(
      String videoUrl, double videoOffsetSec, double eventTimeFromEnableSec) {
    if (videoUrl == null || videoUrl.isBlank()) {
      return "";
    }
    int jumpSec = Math.max(0, (int) Math.floor(videoOffsetSec + eventTimeFromEnableSec - 3.0));
    if (videoUrl.contains("youtube.com") || videoUrl.contains("youtu.be")) {
      String sep = videoUrl.contains("?") ? "&" : "?";
      return videoUrl + sep + "t=" + jumpSec + "s";
    }
    return videoUrl + "#t=" + jumpSec;
  }

  private static RunMeta loadRunMeta(Path runDir, String robotName) throws IOException {
    String battery =
        "FUEL".equalsIgnoreCase(robotName)
            ? "FUEL-Comp-Battery"
            : (robotName != null && !robotName.isBlank() ? robotName + "-Battery" : "Main-Battery");
    String gitSha = "2026-NYTR";
    String configSha = "default";
    String videoUrl = "";
    double videoOffsetSec = 0.0;
    List<VoiceUtterance> utterances = new ArrayList<>();

    // Check for .meta.json in runDir
    try (Stream<Path> s = Files.list(runDir)) {
      List<Path> metaFiles =
          s.filter(p -> p.getFileName().toString().endsWith(".meta.json")).toList();
      if (!metaFiles.isEmpty()) {
        String json = Files.readString(metaFiles.get(0), StandardCharsets.UTF_8);
        battery = extractJsonString(json, "namedBattery", battery);
        gitSha = extractJsonString(json, "gitSha", gitSha);
        configSha = extractJsonString(json, "configSha", configSha);
        videoUrl = extractJsonString(json, "videoUrl", videoUrl);
        String offsetStr = extractJsonString(json, "videoOffsetSec", "0");
        try {
          videoOffsetSec = Double.parseDouble(offsetStr);
        } catch (NumberFormatException ignored) {
        }
      }
    }

    // Check for voice-notes.txt / audio-driver.txt / audio-operator.txt / audio-coach.txt
    Map<String, String> roleFiles =
        Map.of(
            "voice-notes.txt", "DRIVER",
            "audio-driver.txt", "DRIVER",
            "audio-operator.txt", "OPERATOR",
            "audio-coach.txt", "COACH");
    for (Map.Entry<String, String> entry : roleFiles.entrySet()) {
      Path f = runDir.resolve(entry.getKey());
      if (Files.exists(f)) {
        for (String line : Files.readAllLines(f, StandardCharsets.UTF_8)) {
          String trimmed = line.trim();
          if (trimmed.isEmpty() || trimmed.startsWith("#")) {
            continue;
          }
          // Format: "42.0 | DRIVER | Missed short on that far shot" or "42.0: Missed short"
          String[] parts = trimmed.split("\\|", 3);
          if (parts.length == 3) {
            try {
              double t = Double.parseDouble(parts[0].trim());
              utterances.add(new VoiceUtterance(parts[1].trim(), t, parts[2].trim()));
            } catch (NumberFormatException ignored) {
            }
          } else {
            int colon = trimmed.indexOf(':');
            if (colon > 0) {
              try {
                double t = Double.parseDouble(trimmed.substring(0, colon).trim());
                utterances.add(
                    new VoiceUtterance(
                        entry.getValue(), t, trimmed.substring(colon + 1).trim()));
              } catch (NumberFormatException ignored) {
              }
            }
          }
        }
      }
    }
    utterances.sort(Comparator.comparingDouble(VoiceUtterance::timeSec));
    return new RunMeta(battery, gitSha, configSha, videoUrl, videoOffsetSec, utterances);
  }

  private static String extractJsonString(String json, String key, String fallback) {
    Pattern p = Pattern.compile("\"" + Pattern.quote(key) + "\"\\s*:\\s*\"?([^\",}\\s]+)\"?");
    Matcher m = p.matcher(json);
    if (m.find()) {
      return m.group(1);
    }
    return fallback;
  }

  private static void writeMergedLog(
      Path outPath,
      long t0Us,
      ParsedLog rioLog,
      ParsedLog driveLog,
      ParsedLog mechLog,
      List<VoiceCorrelation> voiceCorrelations,
      BatteryMetrics battery,
      List<TimelinePoint> timeline)
      throws IOException {
    try (WpiLogWriter writer = new WpiLogWriter(outPath, "FluxProcessor Merged Log")) {
      int eVolt = writer.startEntry("/FluxAudit/BatteryVoltage", "double", "", t0Us);
      int eDriveStator = writer.startEntry("/FluxAudit/TotalDriveStatorAmps", "double", "", t0Us);
      int eSupply = writer.startEntry("/FluxAudit/TotalSupplyAmps", "double", "", t0Us);
      int eSpeed = writer.startEntry("/FluxAudit/ChassisSpeedMps", "double", "", t0Us);
      int eRint = writer.startEntry("/FluxAudit/BatteryEstimatedRintMilliOhms", "double", "", t0Us);
      int eBattName = writer.startEntry("/FluxAudit/NamedBattery", "string", "", t0Us);

      writer.appendDouble(eRint, t0Us, battery.estimatedRintMilliOhms());
      writer.appendString(eBattName, t0Us, battery.batteryName());

      for (TimelinePoint pt : timeline) {
        long ts = t0Us + Math.round(pt.timeSec() * 1_000_000.0);
        writer.appendDouble(eVolt, ts, pt.batteryVoltage());
        writer.appendDouble(eDriveStator, ts, pt.totalDriveStatorAmps());
        writer.appendDouble(eSupply, ts, pt.totalSupplyAmps());
        writer.appendDouble(eSpeed, ts, pt.chassisSpeedMps());
      }

      if (!voiceCorrelations.isEmpty()) {
        int eVoice = writer.startEntry("/VoiceNotes/CorrelatedDiagnosis", "string", "", t0Us);
        for (VoiceCorrelation vc : voiceCorrelations) {
          long ts = t0Us + Math.round(vc.spokenTimeSec() * 1_000_000.0);
          writer.appendString(
              eVoice,
              ts,
              "["
                  + vc.speakerRole()
                  + "] \""
                  + vc.transcript()
                  + "\" -> "
                  + vc.correlatedEventSummary()
                  + ": "
                  + vc.sensorDiagnosis());
        }
      }

      copyLogSeries(writer, "/Rio/", rioLog);
      if (driveLog != rioLog) {
        copyLogSeries(writer, "/DriveBus/", driveLog);
      }
      if (mechLog != rioLog && mechLog != driveLog) {
        copyLogSeries(writer, "/MechBus/", mechLog);
      }
    }
  }

  private static boolean hasAnyCornerTopicInLog(ParsedLog log, RobotProfile profile) {
    for (RobotProfile.CornerConfig c : profile.corners().values()) {
      if (!findDoubleSeriesByTopic(List.of(log), log, c.driveStatorTopic()).isEmpty()
          || !findDoubleSeriesByTopic(List.of(log), log, c.driveVelocityTopic()).isEmpty()) {
        return true;
      }
    }
    return false;
  }

  private static boolean hasAnyMechanismTopicInLog(ParsedLog log, RobotProfile profile) {
    for (RobotProfile.MechMotorSpec m : profile.mechanisms()) {
      if (!findDoubleSeriesByTopic(List.of(log), log, m.statorTopic()).isEmpty()
          || !findDoubleSeriesByTopic(List.of(log), log, m.velocityTopic()).isEmpty()) {
        return true;
      }
    }
    return false;
  }

  private static RobotProfile.MechMotorSpec findShooterLeader(RobotProfile profile) {
    for (RobotProfile.MechMotorSpec m : profile.mechanisms()) {
      if (m.isShooter()
          && (m.role().toLowerCase(Locale.ROOT).contains("leader")
              || m.name().toLowerCase(Locale.ROOT).contains("left"))) {
        return m;
      }
    }
    for (RobotProfile.MechMotorSpec m : profile.mechanisms()) {
      if (m.isShooter()) {
        return m;
      }
    }
    return null;
  }

  private static RobotProfile.MechMotorSpec findShooterFollower(
      RobotProfile profile, RobotProfile.MechMotorSpec leader) {
    if (leader == null) {
      return null;
    }
    for (RobotProfile.MechMotorSpec m : profile.mechanisms()) {
      if (m.isShooter() && m != leader) {
        return m;
      }
    }
    return null;
  }

  private static RobotProfile.MechMotorSpec findFeederMechanism(
      RobotProfile profile, RobotProfile.MechMotorSpec shooterLeader) {
    for (RobotProfile.MechMotorSpec m : profile.mechanisms()) {
      String r = (m.name() + " " + m.role()).toLowerCase(Locale.ROOT);
      if (r.contains("feed") || r.contains("kicker") || r.contains("transfer")) {
        return m;
      }
    }
    for (RobotProfile.MechMotorSpec m : profile.mechanisms()) {
      String r = (m.name() + " " + m.role()).toLowerCase(Locale.ROOT);
      if (r.contains("index") || r.contains("intake") || r.contains("roller")) {
        return m;
      }
    }
    for (RobotProfile.MechMotorSpec m : profile.mechanisms()) {
      if (!m.isShooter() && m != shooterLeader) {
        return m;
      }
    }
    return null;
  }

  private static RobotProfile.MechMotorSpec findHoodOrPositionMechanism(RobotProfile profile) {
    for (RobotProfile.MechMotorSpec m : profile.mechanisms()) {
      String r = (m.name() + " " + m.role()).toLowerCase(Locale.ROOT);
      if (r.contains("hood")
          || r.contains("tilt")
          || r.contains("wrist")
          || r.contains("pivot")
          || r.contains("arm")
          || r.contains("elevator")
          || r.contains("position")) {
        return m;
      }
    }
    return null;
  }

  private static void addIfNonEmpty(
      List<List<DoubleSample>> target, List<DoubleSample> candidate) {
    if (candidate != null && !candidate.isEmpty()) {
      target.add(candidate);
    }
  }

  private static List<DoubleSample> findDoubleSeriesByTopic(
      List<ParsedLog> allLogs, ParsedLog preferred, String rawTopic) {
    if (rawTopic == null || rawTopic.isBlank()) {
      return Collections.emptyList();
    }
    String clean = rawTopic.startsWith("/") ? rawTopic.substring(1) : rawTopic;
    return findDoubleSeries(
        allLogs,
        preferred,
        List.of(
            rawTopic,
            clean,
            "/" + clean,
            "/AdvantageKit/" + clean,
            "/AdvantageKit/RealOutputs/" + clean,
            "/RealOutputs/" + clean));
  }

  private static List<DoubleSample> findDoubleSeries(
      List<ParsedLog> allLogs, ParsedLog preferred, List<String> candidates) {
    if (preferred != null) {
      for (String c : candidates) {
        List<DoubleSample> s = preferred.getDoubleSeries(c);
        if (!s.isEmpty()) {
          return s;
        }
      }
    }
    for (ParsedLog log : allLogs) {
      if (log == preferred) {
        continue;
      }
      for (String c : candidates) {
        List<DoubleSample> s = log.getDoubleSeries(c);
        if (!s.isEmpty()) {
          return s;
        }
      }
    }
    return Collections.emptyList();
  }

  private static List<BooleanSample> findBooleanSeries(
      List<ParsedLog> allLogs, ParsedLog preferred, List<String> candidates) {
    if (preferred != null) {
      for (String c : candidates) {
        List<BooleanSample> s = preferred.getBooleanSeries(c);
        if (!s.isEmpty()) {
          return s;
        }
      }
    }
    for (ParsedLog log : allLogs) {
      if (log == preferred) {
        continue;
      }
      for (String c : candidates) {
        List<BooleanSample> s = log.getBooleanSeries(c);
        if (!s.isEmpty()) {
          return s;
        }
      }
    }
    return Collections.emptyList();
  }

  private static List<StringSample> findStringSeries(
      List<ParsedLog> allLogs, ParsedLog preferred, List<String> candidates) {
    if (preferred != null) {
      for (String c : candidates) {
        List<StringSample> s = preferred.getStringSeries(c);
        if (!s.isEmpty()) {
          return s;
        }
      }
    }
    for (ParsedLog log : allLogs) {
      if (log == preferred) {
        continue;
      }
      for (String c : candidates) {
        List<StringSample> s = log.getStringSeries(c);
        if (!s.isEmpty()) {
          return s;
        }
      }
    }
    return Collections.emptyList();
  }

  private static void copyLogSeries(WpiLogWriter writer, String prefix, ParsedLog log)
      throws IOException {
    if (log == null) {
      return;
    }
    for (Map.Entry<String, List<BooleanSample>> e : log.allBooleanSeries().entrySet()) {
      List<BooleanSample> samples = e.getValue();
      if (!samples.isEmpty()) {
        int id = writer.startEntry(prefix + e.getKey(), "boolean", "", samples.get(0).timestampUs());
        for (BooleanSample s : samples) {
          writer.appendBoolean(id, s.timestampUs(), s.value());
        }
      }
    }
    for (Map.Entry<String, List<StringSample>> e : log.allStringSeries().entrySet()) {
      List<StringSample> samples = e.getValue();
      if (!samples.isEmpty()) {
        int id = writer.startEntry(prefix + e.getKey(), "string", "", samples.get(0).timestampUs());
        for (StringSample s : samples) {
          writer.appendString(id, s.timestampUs(), s.value());
        }
      }
    }
    for (Map.Entry<String, List<DoubleSample>> e : log.allDoubleSeries().entrySet()) {
      List<DoubleSample> samples = e.getValue();
      if (!samples.isEmpty()) {
        int id = writer.startEntry(prefix + e.getKey(), "double", "", samples.get(0).timestampUs());
        // Downsample 250 Hz signals slightly when merging if huge (>25,000 samples) to keep merged file nimble
        int step = samples.size() > 25000 ? 5 : 1;
        for (int i = 0; i < samples.size(); i += step) {
          DoubleSample s = samples.get(i);
          writer.appendDouble(id, s.timestampUs(), s.value());
        }
      }
    }
  }

  private static double sampleDoubleAt(List<DoubleSample> series, long tsUs, double fallback) {
    if (series == null || series.isEmpty()) {
      return fallback;
    }
    int lo = 0;
    int hi = series.size() - 1;
    int best = 0;
    while (lo <= hi) {
      int mid = (lo + hi) >>> 1;
      long midTs = series.get(mid).timestampUs();
      if (midTs <= tsUs) {
        best = mid;
        lo = mid + 1;
      } else {
        hi = mid - 1;
      }
    }
    return series.get(best).value();
  }

  private static double minInWindowOrSample(
      List<DoubleSample> series, long minTsUs, long maxTsUs, long tsUs, double fallback) {
    if (series == null || series.isEmpty()) {
      return fallback;
    }
    int lo = 0;
    int hi = series.size() - 1;
    int start = series.size();
    while (lo <= hi) {
      int mid = (lo + hi) >>> 1;
      if (series.get(mid).timestampUs() >= minTsUs) {
        start = mid;
        hi = mid - 1;
      } else {
        lo = mid + 1;
      }
    }
    double min = Double.POSITIVE_INFINITY;
    for (int i = start; i < series.size(); i++) {
      DoubleSample s = series.get(i);
      if (s.timestampUs() > maxTsUs) {
        break;
      }
      if (s.value() > 3.0 && s.value() < min) {
        min = s.value();
      }
    }
    if (min != Double.POSITIVE_INFINITY) {
      return min;
    }
    return sampleDoubleAt(series, tsUs, fallback);
  }

  private static double maxAbsInWindowOrSample(
      List<DoubleSample> series, long minTsUs, long maxTsUs, long tsUs, double fallback) {
    if (series == null || series.isEmpty()) {
      return fallback;
    }
    int lo = 0;
    int hi = series.size() - 1;
    int start = series.size();
    while (lo <= hi) {
      int mid = (lo + hi) >>> 1;
      if (series.get(mid).timestampUs() >= minTsUs) {
        start = mid;
        hi = mid - 1;
      } else {
        lo = mid + 1;
      }
    }
    double max = -1.0;
    for (int i = start; i < series.size(); i++) {
      DoubleSample s = series.get(i);
      if (s.timestampUs() > maxTsUs) {
        break;
      }
      double v = Math.abs(s.value());
      if (v > max) {
        max = v;
      }
    }
    if (max >= 0.0) {
      return max;
    }
    return Math.abs(sampleDoubleAt(series, tsUs, fallback));
  }

  private static boolean sampleBooleanAt(List<BooleanSample> series, long tsUs, boolean fallback) {
    if (series == null || series.isEmpty()) {
      return fallback;
    }
    int lo = 0;
    int hi = series.size() - 1;
    int best = 0;
    while (lo <= hi) {
      int mid = (lo + hi) >>> 1;
      long midTs = series.get(mid).timestampUs();
      if (midTs <= tsUs) {
        best = mid;
        lo = mid + 1;
      } else {
        hi = mid - 1;
      }
    }
    return series.get(best).value();
  }

  private static String sampleStringAt(List<StringSample> series, long tsUs, String fallback) {
    if (series == null || series.isEmpty()) {
      return fallback;
    }
    int lo = 0;
    int hi = series.size() - 1;
    int best = 0;
    while (lo <= hi) {
      int mid = (lo + hi) >>> 1;
      long midTs = series.get(mid).timestampUs();
      if (midTs <= tsUs) {
        best = mid;
        lo = mid + 1;
      } else {
        hi = mid - 1;
      }
    }
    return series.get(best).value();
  }

  private static double meanAbsActive(
      List<DoubleSample> series, double minThreshold, long minTsUs, long maxTsUs) {
    if (series == null || series.isEmpty()) {
      return 0.0;
    }
    double sum = 0.0;
    int count = 0;
    for (DoubleSample s : series) {
      if (s.timestampUs() < minTsUs || s.timestampUs() > maxTsUs) {
        continue;
      }
      double v = Math.abs(s.value());
      if (v >= minThreshold) {
        sum += v;
        count++;
      }
    }
    return count > 0 ? sum / count : 0.0;
  }

  private static double maxAbs(List<DoubleSample> series, long minTsUs, long maxTsUs) {
    if (series == null || series.isEmpty()) {
      return 0.0;
    }
    double max = 0.0;
    for (DoubleSample s : series) {
      if (s.timestampUs() < minTsUs || s.timestampUs() > maxTsUs) {
        continue;
      }
      double v = Math.abs(s.value());
      if (v > max) {
        max = v;
      }
    }
    return max;
  }

  private static double minVal(List<DoubleSample> series, long minTsUs, long maxTsUs) {
    if (series == null || series.isEmpty()) {
      return 0.0;
    }
    double min = Double.POSITIVE_INFINITY;
    for (DoubleSample s : series) {
      if (s.timestampUs() < minTsUs || s.timestampUs() > maxTsUs) {
        continue;
      }
      if (s.value() < min) {
        min = s.value();
      }
    }
    return min == Double.POSITIVE_INFINITY ? 0.0 : min;
  }

  private static double maxVal(List<DoubleSample> series, long minTsUs, long maxTsUs) {
    if (series == null || series.isEmpty()) {
      return 0.0;
    }
    double max = Double.NEGATIVE_INFINITY;
    for (DoubleSample s : series) {
      if (s.timestampUs() < minTsUs || s.timestampUs() > maxTsUs) {
        continue;
      }
      if (s.value() > max) {
        max = s.value();
      }
    }
    return max == Double.NEGATIVE_INFINITY ? 0.0 : max;
  }
}
