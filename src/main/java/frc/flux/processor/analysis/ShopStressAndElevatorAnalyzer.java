package frc.flux.processor.analysis;

import frc.flux.processor.model.RobotProfile;
import frc.flux.processor.model.RunAnalysisReport.CameraSnapshotRef;
import frc.flux.processor.model.RunAnalysisReport.CornerMetrics;
import frc.flux.processor.model.RunAnalysisReport.ElevatorCycleEvent;
import frc.flux.processor.model.RunAnalysisReport.ShopStressGateMetrics;
import frc.flux.processor.model.RunAnalysisReport.TimelinePoint;
import frc.flux.processor.wpilog.WpiLogReader.BooleanSample;
import frc.flux.processor.wpilog.WpiLogReader.DoubleSample;
import frc.flux.processor.wpilog.WpiLogReader.ParsedLog;
import frc.flux.processor.wpilog.WpiLogReader.StringSample;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Stream;

/**
 * Evaluates 2027 {@code ShopStressSuiteCmd} &amp; {@code VirtualGeofence} 5m x 3m arena pass gates,
 * 2027 Elevator + Arm ({@code L1..L4}) scoring cycles &amp; keep-out interlocks, and event-triggered
 * camera snapshots ({@code cam-snapshots/*.jpg}).
 */
public final class ShopStressAndElevatorAnalyzer {

  private static final Pattern SNAPSHOT_TIME_PATTERN =
      Pattern.compile("(?i)(?:t|ts|time)[_-]?([0-9]+(?:\\.[0-9]+)?)s?");

  private ShopStressAndElevatorAnalyzer() {}

  public static ShopStressGateMetrics analyzeShopStressGate(
      List<ParsedLog> allLogs,
      List<CornerMetrics> swerveCorners,
      long minTsUs,
      long maxTsUs) {
    List<DoubleSample> breachSeries =
        RunAnalyzer.findDoubleSeries(
            allLogs,
            null,
            List.of(
                "VirtualGeofence/WallBreachCount",
                "/VirtualGeofence/WallBreachCount",
                "/AdvantageKit/RealOutputs/VirtualGeofence/WallBreachCount",
                "/RealOutputs/VirtualGeofence/WallBreachCount",
                "ShopStress/GeofenceBreachCount",
                "/ShopStress/GeofenceBreachCount"));
    List<BooleanSample> breachBoolSeries =
        RunAnalyzer.findBooleanSeries(
            allLogs,
            null,
            List.of(
                "VirtualGeofence/Breached",
                "/VirtualGeofence/Breached",
                "/AdvantageKit/RealOutputs/VirtualGeofence/Breached",
                "/RealOutputs/VirtualGeofence/Breached"));
    List<DoubleSample> driftCmSeries =
        RunAnalyzer.findDoubleSeries(
            allLogs,
            null,
            List.of(
                "ShopStress/Figure8LoopClosureDriftCm",
                "/ShopStress/Figure8LoopClosureDriftCm",
                "/AdvantageKit/RealOutputs/ShopStress/Figure8LoopClosureDriftCm",
                "/RealOutputs/ShopStress/Figure8LoopClosureDriftCm"));
    List<DoubleSample> driftMetersSeries =
        RunAnalyzer.findDoubleSeries(
            allLogs,
            null,
            List.of(
                "ShopStress/Figure8LoopClosureDriftMeters",
                "/ShopStress/Figure8LoopClosureDriftMeters",
                "/AdvantageKit/RealOutputs/ShopStress/Figure8LoopClosureDriftMeters",
                "/RealOutputs/ShopStress/Figure8LoopClosureDriftMeters"));
    List<DoubleSample> handoffCmSeries =
        RunAnalyzer.findDoubleSeries(
            allLogs,
            null,
            List.of(
                "ShopStress/MaxCameraHandoffStepCm",
                "/ShopStress/MaxCameraHandoffStepCm",
                "/AdvantageKit/RealOutputs/ShopStress/MaxCameraHandoffStepCm",
                "/RealOutputs/ShopStress/MaxCameraHandoffStepCm",
                "Vision/MaxCameraHandoffStepCm",
                "/Vision/MaxCameraHandoffStepCm"));
    List<DoubleSample> handoffMetersSeries =
        RunAnalyzer.findDoubleSeries(
            allLogs,
            null,
            List.of(
                "ShopStress/MaxCameraHandoffStepMeters",
                "/ShopStress/MaxCameraHandoffStepMeters",
                "Vision/MaxCameraHandoffStepMeters",
                "/Vision/MaxCameraHandoffStepMeters",
                "/AdvantageKit/RealOutputs/Vision/MaxCameraHandoffStepMeters"));
    List<DoubleSample> boxSpreadSeries =
        RunAnalyzer.findDoubleSeries(
            allLogs,
            null,
            List.of(
                "ShopStress/BoxStepCornerSpreadPct",
                "/ShopStress/BoxStepCornerSpreadPct",
                "/AdvantageKit/RealOutputs/ShopStress/BoxStepCornerSpreadPct",
                "/RealOutputs/ShopStress/BoxStepCornerSpreadPct"));
    List<StringSample> phaseSeries =
        RunAnalyzer.findStringSeries(
            allLogs,
            null,
            List.of(
                "ShopStress/Phase",
                "/ShopStress/Phase",
                "/AdvantageKit/RealOutputs/ShopStress/Phase",
                "/RealOutputs/ShopStress/Phase"));

    boolean present =
        !breachSeries.isEmpty()
            || !breachBoolSeries.isEmpty()
            || !driftCmSeries.isEmpty()
            || !driftMetersSeries.isEmpty()
            || !handoffCmSeries.isEmpty()
            || !handoffMetersSeries.isEmpty()
            || !boxSpreadSeries.isEmpty()
            || !phaseSeries.isEmpty();

    if (!present) {
      return null;
    }

    int breaches = (int) Math.round(RunAnalyzer.maxAbs(breachSeries, minTsUs, maxTsUs));
    if (breaches == 0 && !breachBoolSeries.isEmpty()) {
      for (int i = 0; i < breachBoolSeries.size(); i++) {
        BooleanSample cur = breachBoolSeries.get(i);
        boolean prev = i > 0 && breachBoolSeries.get(i - 1).value();
        if (cur.value() && !prev) {
          breaches++;
        }
      }
    }

    double driftCm =
        !driftCmSeries.isEmpty()
            ? RunAnalyzer.maxAbs(driftCmSeries, minTsUs, maxTsUs)
            : RunAnalyzer.maxAbs(driftMetersSeries, minTsUs, maxTsUs) * 100.0;
    double handoffCm =
        !handoffCmSeries.isEmpty()
            ? RunAnalyzer.maxAbs(handoffCmSeries, minTsUs, maxTsUs)
            : RunAnalyzer.maxAbs(handoffMetersSeries, minTsUs, maxTsUs) * 100.0;

    double cornerSpreadPct =
        !boxSpreadSeries.isEmpty() ? RunAnalyzer.maxAbs(boxSpreadSeries, minTsUs, maxTsUs) : 0.0;
    if (cornerSpreadPct <= 0.0 && swerveCorners != null) {
      for (CornerMetrics c : swerveCorners) {
        cornerSpreadPct = Math.max(cornerSpreadPct, Math.abs(c.driveAsymmetryPct()));
      }
    }

    List<String> checks = new ArrayList<>();
    String status = "PASS";

    // Gate 1: VirtualGeofence Wall Breaches (0 allowed)
    if (breaches > 0) {
      status = "FAIL";
      checks.add(
          String.format(
              Locale.US,
              "[FAIL] VirtualGeofence Wall Breaches: %d breach(es) detected (gate requires 0).",
              breaches));
    } else {
      checks.add("[PASS] VirtualGeofence Wall Breaches: 0 breaches inside 5m x 3m boundary.");
    }

    // Gate 2: Beyblade Figure-8 Odometry Loop Closure Drift (<= 4.0 cm PASS, <= 8.0 cm WARN)
    if (driftCm > 8.0) {
      status = "FAIL";
      checks.add(
          String.format(
              Locale.US,
              "[FAIL] Figure-8 Loop Closure Drift: %.2f cm (gate requires <= 4.0 cm).",
              driftCm));
    } else if (driftCm > 4.0) {
      if ("PASS".equals(status)) {
        status = "WARN";
      }
      checks.add(
          String.format(
              Locale.US,
              "[WARN] Figure-8 Loop Closure Drift: %.2f cm (target <= 4.0 cm).",
              driftCm));
    } else {
      checks.add(
          String.format(
              Locale.US, "[PASS] Figure-8 Loop Closure Drift: %.2f cm (<= 4.0 cm).", driftCm));
    }

    // Gate 3: Camera-to-Camera Handoff Step (<= 3.0 cm PASS, <= 6.0 cm WARN)
    if (handoffCm > 6.0) {
      status = "FAIL";
      checks.add(
          String.format(
              Locale.US,
              "[FAIL] Camera Handoff Pose Step: %.2f cm (gate requires <= 3.0 cm — re-run ZeroPriorBootstrapCmd).",
              handoffCm));
    } else if (handoffCm > 3.0) {
      if ("PASS".equals(status)) {
        status = "WARN";
      }
      checks.add(
          String.format(
              Locale.US,
              "[WARN] Camera Handoff Pose Step: %.2f cm (target <= 3.0 cm).",
              handoffCm));
    } else {
      checks.add(
          String.format(
              Locale.US, "[PASS] Camera Handoff Pose Step: %.2f cm (<= 3.0 cm).", handoffCm));
    }

    // Gate 4: 90-Degree Box-Step 4-Corner Symmetry (<= 12% PASS, <= 22% WARN)
    if (cornerSpreadPct > 22.0) {
      status = "FAIL";
      checks.add(
          String.format(
              Locale.US,
              "[FAIL] Box-Step 4-Corner Current Spread: %.1f%% (gate requires <= 12.0%%).",
              cornerSpreadPct));
    } else if (cornerSpreadPct > 12.0) {
      if ("PASS".equals(status)) {
        status = "WARN";
      }
      checks.add(
          String.format(
              Locale.US,
              "[WARN] Box-Step 4-Corner Current Spread: %.1f%% (target <= 12.0%%).",
              cornerSpreadPct));
    } else {
      checks.add(
          String.format(
              Locale.US,
              "[PASS] Box-Step 4-Corner Current Spread: %.1f%% (<= 12.0%%).",
              cornerSpreadPct));
    }

    return new ShopStressGateMetrics(
        true, breaches, driftCm, handoffCm, cornerSpreadPct, status, List.copyOf(checks));
  }

  public static List<ElevatorCycleEvent> detectElevatorCycles(
      RobotProfile profile,
      List<ParsedLog> allLogs,
      ParsedLog mechLog,
      List<TimelinePoint> timeline,
      long t0Us,
      long windowEndUs) {
    RobotProfile.MechMotorSpec elevSpec = null;
    RobotProfile.MechMotorSpec armSpec = null;
    for (RobotProfile.MechMotorSpec m : profile.mechanisms()) {
      String lower = (m.name() + " " + m.role()).toLowerCase(Locale.ROOT);
      if (elevSpec == null && (lower.contains("elevator") || lower.contains("lift"))) {
        elevSpec = m;
      } else if (armSpec == null
          && (lower.contains("arm") || lower.contains("wrist") || lower.contains("pivot"))) {
        armSpec = m;
      }
    }

    List<DoubleSample> elevPos =
        elevSpec != null
            ? RunAnalyzer.findDoubleSeriesByTopic(allLogs, mechLog, elevSpec.positionTopic())
            : RunAnalyzer.findDoubleSeries(
                allLogs,
                mechLog,
                List.of(
                    "Mechanisms/Elevator/PositionRot",
                    "/Mechanisms/Elevator/PositionRot",
                    "/AdvantageKit/RealOutputs/Mechanisms/Elevator/PositionRot",
                    "/RealOutputs/Mechanisms/Elevator/PositionRot",
                    "Elevator/PositionRot"));
    List<DoubleSample> elevStator =
        elevSpec != null
            ? RunAnalyzer.findDoubleSeriesByTopic(allLogs, mechLog, elevSpec.statorTopic())
            : RunAnalyzer.findDoubleSeries(
                allLogs,
                mechLog,
                List.of(
                    "Mechanisms/Elevator/LeaderStatorAmps",
                    "/Mechanisms/Elevator/LeaderStatorAmps",
                    "/AdvantageKit/RealOutputs/Mechanisms/Elevator/LeaderStatorAmps",
                    "/RealOutputs/Mechanisms/Elevator/LeaderStatorAmps",
                    "Elevator/LeaderStatorAmps"));
    List<DoubleSample> armPos =
        armSpec != null
            ? RunAnalyzer.findDoubleSeriesByTopic(allLogs, mechLog, armSpec.positionTopic())
            : RunAnalyzer.findDoubleSeries(
                allLogs,
                mechLog,
                List.of(
                    "Mechanisms/Arm/PositionRot",
                    "/Mechanisms/Arm/PositionRot",
                    "/AdvantageKit/RealOutputs/Mechanisms/Arm/PositionRot",
                    "/RealOutputs/Mechanisms/Arm/PositionRot",
                    "Arm/PositionRot"));
    List<StringSample> stateSeries =
        RunAnalyzer.findStringSeries(
            allLogs,
            mechLog,
            List.of(
                "Superstructure/State",
                "/Superstructure/State",
                "/AdvantageKit/RealOutputs/Superstructure/State",
                "/RealOutputs/Superstructure/State",
                "Superstructure/TargetState",
                "/Superstructure/TargetState"));
    List<BooleanSample> keepOutSeries =
        RunAnalyzer.findBooleanSeries(
            allLogs,
            mechLog,
            List.of(
                "Superstructure/KeepOutInterlockActive",
                "/Superstructure/KeepOutInterlockActive",
                "/AdvantageKit/RealOutputs/Superstructure/KeepOutInterlockActive",
                "ElevatorArmKinematics/KeepOutActive",
                "/ElevatorArmKinematics/KeepOutActive",
                "/AdvantageKit/RealOutputs/ElevatorArmKinematics/KeepOutActive"));

    if (elevPos.isEmpty() && stateSeries.isEmpty()) {
      return List.of();
    }

    List<ElevatorCycleEvent> cycles = new ArrayList<>();
    boolean inCycle = false;
    double startSec = 0.0;
    String cycleLevel = "L2";
    double peakElevRot = 0.0;
    double peakArmRot = 0.0;
    double peakStator = 0.0;
    double minBattV = 14.0;
    boolean keepOutTriggered = false;

    for (TimelinePoint pt : timeline) {
      long tsUs = t0Us + Math.round(pt.timeSec() * 1_000_000.0);
      if (tsUs > windowEndUs) {
        break;
      }
      double eRot = RunAnalyzer.sampleDoubleAt(elevPos, tsUs, 0.0);
      double aRot = RunAnalyzer.sampleDoubleAt(armPos, tsUs, 0.0);
      double eAmps = Math.abs(RunAnalyzer.sampleDoubleAt(elevStator, tsUs, 0.0));
      String state = RunAnalyzer.sampleStringAt(stateSeries, tsUs, "");
      boolean keepOut = RunAnalyzer.sampleBooleanAt(keepOutSeries, tsUs, false);

      boolean elevated =
          pt.enabled()
              && (eRot >= 4.0
                  || state.contains("L1")
                  || state.contains("L2")
                  || state.contains("L3")
                  || state.contains("L4")
                  || state.contains("SCORE"));

      if (elevated) {
        String level = inferLevel(state, eRot);
        if (!inCycle) {
          inCycle = true;
          startSec = pt.timeSec();
          cycleLevel = level;
          peakElevRot = eRot;
          peakArmRot = aRot;
          peakStator = eAmps;
          minBattV = pt.batteryVoltage();
          keepOutTriggered = keepOut;
        } else {
          if (eRot > peakElevRot) {
            peakElevRot = eRot;
            cycleLevel = inferLevel(state, peakElevRot);
          }
          if (Math.abs(aRot) > Math.abs(peakArmRot)) {
            peakArmRot = aRot;
          }
          peakStator = Math.max(peakStator, eAmps);
          minBattV = Math.min(minBattV, pt.batteryVoltage());
          keepOutTriggered = keepOutTriggered || keepOut;
        }
      } else if (inCycle) {
        double dur = Math.max(0.2, pt.timeSec() - startSec);
        int num = cycles.size() + 1;
        String status =
            (minBattV < 7.5 || peakStator > 65.0) ? "HIGH LOAD / SAG" : "NOMINAL";
        cycles.add(
            new ElevatorCycleEvent(
                num,
                cycleLevel,
                startSec,
                dur,
                peakElevRot,
                peakArmRot,
                peakStator,
                minBattV,
                keepOutTriggered,
                status));
        inCycle = false;
      }
    }

    if (inCycle && !timeline.isEmpty()) {
      double endSec = timeline.get(timeline.size() - 1).timeSec();
      double dur = Math.max(0.2, endSec - startSec);
      int num = cycles.size() + 1;
      String status = (minBattV < 7.5 || peakStator > 65.0) ? "HIGH LOAD / SAG" : "NOMINAL";
      cycles.add(
          new ElevatorCycleEvent(
              num,
              cycleLevel,
              startSec,
              dur,
              peakElevRot,
              peakArmRot,
              peakStator,
              minBattV,
              keepOutTriggered,
              status));
    }

    return List.copyOf(cycles);
  }

  private static String inferLevel(String state, double elevRot) {
    if (state != null) {
      String u = state.toUpperCase(Locale.ROOT);
      if (u.contains("L4")) {
        return "L4";
      }
      if (u.contains("L3")) {
        return "L3";
      }
      if (u.contains("L2")) {
        return "L2";
      }
      if (u.contains("L1")) {
        return "L1";
      }
    }
    if (elevRot >= 38.0) {
      return "L4";
    }
    if (elevRot >= 24.0) {
      return "L3";
    }
    if (elevRot >= 12.0) {
      return "L2";
    }
    return "L1";
  }

  /**
   * Scans {@code runDir/cam-snapshots} and {@code runDir/snapshots} for event-triggered {@code .jpg}
   * / {@code .png} images captured during pre-match calibration, shots, pose jumps, or voice notes.
   */
  public static List<CameraSnapshotRef> discoverCameraSnapshots(Path runDir) {
    if (runDir == null || !Files.isDirectory(runDir)) {
      return List.of();
    }
    List<CameraSnapshotRef> refs = new ArrayList<>();
    String[] subDirs = {"cam-snapshots", "snapshots"};
    for (String sub : subDirs) {
      Path dir = runDir.resolve(sub);
      if (!Files.isDirectory(dir)) {
        continue;
      }
      try (Stream<Path> s = Files.list(dir)) {
        for (Path p : s.sorted(Comparator.comparing(x -> x.getFileName().toString())).toList()) {
          String fn = p.getFileName().toString();
          String lower = fn.toLowerCase(Locale.ROOT);
          if (!lower.endsWith(".jpg") && !lower.endsWith(".jpeg") && !lower.endsWith(".png")) {
            continue;
          }
          String cam = inferCameraName(fn);
          double ts = inferSnapshotTimestamp(fn);
          String reason = inferTriggerReason(fn);
          refs.add(new CameraSnapshotRef(fn, cam, ts, reason, sub + "/" + fn));
        }
      } catch (IOException ignored) {
      }
    }
    refs.sort(Comparator.comparingDouble(CameraSnapshotRef::timestampSec));
    return List.copyOf(refs);
  }

  private static String inferCameraName(String fileName) {
    String lower = fileName.toLowerCase(Locale.ROOT);
    for (int i = 0; i <= 3; i++) {
      if (lower.contains("limelightsc" + i)) {
        return "limelightsc" + i;
      }
      if (lower.contains("usb" + i)) {
        return "USB-" + i;
      }
      if (lower.contains("ov9281-" + (i + 1)) || lower.contains("ov9281_" + (i + 1))) {
        return "OV9281-" + (i + 1);
      }
    }
    int us = fileName.indexOf('_');
    return us > 0 ? fileName.substring(0, us) : "Camera-0";
  }

  private static double inferSnapshotTimestamp(String fileName) {
    Matcher m = SNAPSHOT_TIME_PATTERN.matcher(fileName);
    if (m.find()) {
      try {
        return Double.parseDouble(m.group(1));
      } catch (NumberFormatException ignored) {
      }
    }
    Pattern numP = Pattern.compile("([0-9]+\\.[0-9]+)");
    Matcher nm = numP.matcher(fileName);
    if (nm.find()) {
      try {
        return Double.parseDouble(nm.group(1));
      } catch (NumberFormatException ignored) {
      }
    }
    return 0.0;
  }

  private static String inferTriggerReason(String fileName) {
    String lower = fileName.toLowerCase(Locale.ROOT);
    if (lower.contains("prematch") || lower.contains("lock") || lower.contains("seed")) {
      return "PRE_MATCH_LOCK";
    }
    if (lower.contains("jump") || lower.contains("handoff") || lower.contains("reject")) {
      return "POSE_JUMP";
    }
    if (lower.contains("shot") || lower.contains("score") || lower.contains("cycle")) {
      return "SHOT_OR_SCORE";
    }
    if (lower.contains("voice") || lower.contains("audio")) {
      return "VOICE_NOTE";
    }
    return "EVENT_BURST";
  }
}
