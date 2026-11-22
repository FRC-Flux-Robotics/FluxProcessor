package frc.flux.processor.analysis;

import frc.flux.processor.model.RobotProfile;
import frc.flux.processor.model.RunAnalysisReport;
import frc.lib.fluxfig.FluxFig;
import frc.lib.fluxfig.FluxWpiLogCodec;
import frc.lib.fluxfig.FluxWpiLogCodec.DoubleSample;
import frc.lib.fluxfig.FluxWpiLogCodec.ParsedLog;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.LocalDate;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.stream.Stream;

/**
 * Digital-Twin Simulation Auto-Calibrator ({@code --calibrate-sim <config-dir>}, PLAN-2027 Section
 * 6.H.3).
 *
 * <p>Extracts empirical plant constants from real robot {@code .wpilog} runs under strict R^2 and
 * excitation gates:
 *
 * <ul>
 *   <li>Battery internal resistance {@code R_int (mOhm)} (gated on {@code R^2 >= 0.35}) &rarr;
 *       written to {@code batteries.csv}.
 *   <li>Effective swerve wheel radius {@code wheelRadiusMeters} (from vision ground-distance or
 *       gyro spin-in-place calibration) and azimuth coupling ratio {@code swerveCouplingRatio}
 *       &rarr; written to {@code parameters.csv} only when calibrated.
 *   <li>Mechanism feedforward plant constants {@code (kS, kV, kA, kG)} via gated OLS regression
 *       ({@code R^2 >= 0.50}) &rarr; written strictly to {@code sim-calibration.json} without ever
 *       overwriting production PID gains ({@code kP, kI, kD}) in {@code mechanisms.csv}.
 * </ul>
 */
public final class DigitalTwinCalibrator {

  public static final double MIN_MECHANISM_R_SQUARED = 0.50;
  public static final int MIN_MECHANISM_SAMPLES = 10;
  public static final double MIN_VELOCITY_SPREAD_RPS = 3.0;
  public static final double MIN_BATTERY_R_SQUARED = BatteryAnalyzer.MIN_R_SQUARED;
  private static final double MIN_SPIN_YAW_RAD = Math.PI;
  private static final double MIN_SPIN_WHEEL_RAD = 2.0;

  public record MechanismFeedforwardFit(
      String mechanism,
      int sampleCount,
      double kS,
      double kV,
      double kA,
      double kG,
      double rSquared) {}

  public record SimCalibrationSummary(
      String targetProfile,
      double calibratedWheelRadiusMeters,
      double calibratedCouplingRatio,
      Map<String, RunAnalysisReport.BatteryMetrics> calibratedBatteries,
      List<MechanismFeedforwardFit> calibratedMechanisms,
      String updatedConfigSha256,
      Path summaryJsonPath) {}

  private record ScalarCalibration(double value, boolean calibrated, String source) {}

  private DigitalTwinCalibrator() {}

  public static SimCalibrationSummary calibrateAndWrite(
      List<RunAnalysisReport> reports,
      List<Path> runDirs,
      Path configDir,
      Path outputDir,
      String requestedProfile)
      throws IOException {
    List<ParsedLog> allLogs = new ArrayList<>();
    for (Path dir : runDirs) {
      if (Files.isDirectory(dir)) {
        try (Stream<Path> s = Files.walk(dir, 4)) {
          for (Path p : s.filter(Files::isRegularFile).sorted().toList()) {
            String n = p.getFileName().toString().toLowerCase(Locale.ROOT);
            if (n.endsWith(".wpilog") && !n.equals("merged.wpilog")) {
              allLogs.add(FluxWpiLogCodec.read(p));
            }
          }
        }
      }
    }

    String targetProfile =
        (requestedProfile != null
                && !requestedProfile.isBlank()
                && !"AUTO".equalsIgnoreCase(requestedProfile))
            ? requestedProfile
            : (!reports.isEmpty() ? reports.get(0).robotName() : "COMP_2027");

    Map<String, RunAnalysisReport.BatteryMetrics> batteryMap = new LinkedHashMap<>();
    for (RunAnalysisReport r : reports) {
      if (r.battery() != null && r.battery().batteryName() != null) {
        batteryMap.put(r.battery().batteryName(), r.battery());
      }
    }

    double nominalWheelRadius = 0.0508;
    double nominalCoupling = 3.5714;
    double driveBaseRadius = defaultDriveBaseRadius(targetProfile);
    RobotProfile resolvedProfile = RobotProfile.fromPresetName(targetProfile);
    FluxFig fig = null;
    if (configDir != null && Files.isDirectory(configDir)) {
      fig = FluxFig.load(configDir, targetProfile);
      nominalWheelRadius = RobotProfile.getDoubleOrDefault(fig, "wheelRadiusMeters", 0.0508);
      nominalCoupling = RobotProfile.getDoubleOrDefault(fig, "swerveCouplingRatio", 3.5714);
      driveBaseRadius = resolveDriveBaseRadius(fig, driveBaseRadius);
      resolvedProfile = RobotProfile.fromFluxFigDirectory(configDir, targetProfile);
    }

    ScalarCalibration wheelCal =
        estimateWheelRadius(allLogs, resolvedProfile, driveBaseRadius, nominalWheelRadius);
    ScalarCalibration couplingCal = estimateCouplingRatio(allLogs, nominalCoupling);

    List<MechanismFeedforwardFit> mechFits = new ArrayList<>();
    if (fig != null && fig.getTableNames().contains("mechanisms")) {
      List<FluxFig.MechanismSpec> existingSpecs =
          fig.bindTable("mechanisms", targetProfile, FluxFig.MechanismSpec.class);
      for (FluxFig.MechanismSpec spec : existingSpecs) {
        if (spec.mechanism().toLowerCase(Locale.ROOT).startsWith("swervedrive")) {
          continue;
        }
        MechanismFeedforwardFit fit = fitMechanismFeedforward(allLogs, spec);
        if (fit != null) {
          mechFits.add(fit);
        }
      }
    } else {
      mechFits.addAll(fitDiscoveredMechanisms(allLogs));
    }

    String updatedSha = "unmodified";
    if (fig != null && configDir != null) {
      if (Files.exists(configDir.resolve("batteries.csv")) && !batteryMap.isEmpty()) {
        List<FluxFig.BatterySpec> currentBatteries =
            new ArrayList<>(fig.bindTable("batteries", FluxFig.BatterySpec.class));
        int currentYear = LocalDate.now().getYear();
        boolean anyBatteryUpdated = false;

        for (RunAnalysisReport.BatteryMetrics bm : batteryMap.values()) {
          if (!Double.isFinite(bm.estimatedRintMilliOhms())
              || !Double.isFinite(bm.rSquared())
              || bm.rSquared() < MIN_BATTERY_R_SQUARED) {
            continue;
          }
          double clampedNominal =
              Math.max(8.0, Math.min(35.0, round1(bm.estimatedRintMilliOhms())));
          int existingIdx = -1;
          for (int i = 0; i < currentBatteries.size(); i++) {
            if (currentBatteries.get(i).batteryName().equalsIgnoreCase(bm.batteryName())
                || currentBatteries.get(i).qrCode().equalsIgnoreCase(bm.batteryName())) {
              existingIdx = i;
              break;
            }
          }
          if (existingIdx >= 0) {
            FluxFig.BatterySpec prev = currentBatteries.get(existingIdx);
            double maxAllowed =
                Math.min(45.0, Math.max(prev.maxAllowedRintMilliOhms(), clampedNominal + 2.0));
            currentBatteries.set(
                existingIdx,
                new FluxFig.BatterySpec(
                    prev.batteryName(),
                    prev.qrCode(),
                    prev.purchaseYear(),
                    Math.min(clampedNominal, maxAllowed),
                    maxAllowed));
          } else {
            double maxAllowed = Math.min(45.0, Math.max(24.0, clampedNominal + 4.0));
            currentBatteries.add(
                new FluxFig.BatterySpec(
                    bm.batteryName(),
                    "QR-" + bm.batteryName(),
                    currentYear,
                    Math.min(clampedNominal, maxAllowed),
                    maxAllowed));
          }
          anyBatteryUpdated = true;
        }
        if (anyBatteryUpdated) {
          fig.saveBatterySpecs(currentBatteries);
        }
      }

      if (Files.exists(configDir.resolve("parameters.csv"))
          && (wheelCal.calibrated() || couplingCal.calibrated())) {
        FluxFig tunableFig = FluxFig.load(configDir, targetProfile);
        tunableFig.setMode(FluxFig.Mode.TUNING);
        if (wheelCal.calibrated()
            && !Double.isNaN(
                RobotProfile.getDoubleOrDefault(tunableFig, "wheelRadiusMeters", Double.NaN))) {
          tunableFig.setOverride(
              targetProfile,
              "wheelRadiusMeters",
              String.format(Locale.US, "%.5f", wheelCal.value()));
        }
        if (couplingCal.calibrated()
            && !Double.isNaN(
                RobotProfile.getDoubleOrDefault(tunableFig, "swerveCouplingRatio", Double.NaN))) {
          tunableFig.setOverride(
              targetProfile,
              "swerveCouplingRatio",
              String.format(Locale.US, "%.4f", couplingCal.value()));
        }
        if (tunableFig.isModified()) {
          tunableFig.persistOverridesToCsv();
        }
      }

      FluxFig reloaded = FluxFig.load(configDir, targetProfile);
      updatedSha = reloaded.getConfigSha256();
    }

    Path outFolder = outputDir != null ? outputDir : (configDir != null ? configDir : Path.of("."));
    Files.createDirectories(outFolder);
    Path jsonPath = outFolder.resolve("sim-calibration.json");
    writeSummaryJson(
        jsonPath,
        targetProfile,
        wheelCal,
        couplingCal,
        batteryMap,
        mechFits,
        updatedSha);

    return new SimCalibrationSummary(
        targetProfile,
        wheelCal.value(),
        couplingCal.value(),
        batteryMap,
        mechFits,
        updatedSha,
        jsonPath);
  }

  private static double defaultDriveBaseRadius(String profileName) {
    String upper = profileName != null ? profileName.toUpperCase(Locale.ROOT) : "";
    if ("CORAL".equals(upper) || "CHASSIS_3".equals(upper)) {
      return Math.hypot(0.5969 * 0.5, 0.5969 * 0.5);
    }
    return Math.hypot(0.56515 * 0.5, 0.56515 * 0.5);
  }

  private static double resolveDriveBaseRadius(FluxFig fig, double fallback) {
    double direct = RobotProfile.getDoubleOrDefault(fig, "driveBaseRadiusMeters", Double.NaN);
    if (Double.isFinite(direct) && direct > 0.15 && direct < 1.0) {
      return direct;
    }
    double trackW = RobotProfile.getDoubleOrDefault(fig, "trackWidthMeters", Double.NaN);
    double wheelB = RobotProfile.getDoubleOrDefault(fig, "wheelBaseMeters", Double.NaN);
    if (Double.isFinite(trackW) && Double.isFinite(wheelB) && trackW > 0.2 && wheelB > 0.2) {
      return Math.hypot(trackW * 0.5, wheelB * 0.5);
    }
    return fallback;
  }

  private static ScalarCalibration estimateWheelRadius(
      List<ParsedLog> allLogs,
      RobotProfile profile,
      double driveBaseRadiusMeters,
      double nominal) {
    List<DoubleSample> wheelTravelRot =
        RunAnalyzer.findDoubleSeries(
            allLogs,
            null,
            List.of(
                "SimCal/WheelRotationsDelta",
                "/SimCal/WheelRotationsDelta",
                "Drive/WheelRadiusCalRotations",
                "/AdvantageKit/RealOutputs/Drive/WheelRadiusCalRotations"));
    List<DoubleSample> groundDistMeters =
        RunAnalyzer.findDoubleSeries(
            allLogs,
            null,
            List.of(
                "SimCal/VisionGroundDistanceMeters",
                "/SimCal/VisionGroundDistanceMeters",
                "Drive/WheelRadiusCalGroundMeters",
                "/AdvantageKit/RealOutputs/Drive/WheelRadiusCalGroundMeters"));
    if (!wheelTravelRot.isEmpty() && !groundDistMeters.isEmpty()) {
      double lastRot = wheelTravelRot.get(wheelTravelRot.size() - 1).value();
      double firstRot = wheelTravelRot.get(0).value();
      double rot =
          (wheelTravelRot.size() > 1 && Math.abs(lastRot - firstRot) > 1.0)
              ? Math.abs(lastRot - firstRot)
              : Math.abs(lastRot);
      double lastDist = groundDistMeters.get(groundDistMeters.size() - 1).value();
      double firstDist = groundDistMeters.get(0).value();
      double dist =
          (groundDistMeters.size() > 1 && Math.abs(lastDist - firstDist) > 0.3)
              ? Math.abs(lastDist - firstDist)
              : Math.abs(lastDist);
      if (rot > 1.0 && dist > 0.3) {
        double est = dist / (2.0 * Math.PI * rot);
        if (est >= 0.044 && est <= 0.056) {
          return new ScalarCalibration(est, true, "VISION_GROUND_DISTANCE");
        }
      }
    }

    ScalarCalibration spinCal =
        estimateWheelRadiusFromGyroSpin(allLogs, profile, driveBaseRadiusMeters);
    if (spinCal != null && spinCal.calibrated()) {
      return spinCal;
    }
    return new ScalarCalibration(nominal, false, "NOMINAL_UNMEASURED");
  }

  /**
   * Solves effective swerve {@code wheelRadiusMeters} from a gyro spin-in-place maneuver using
   * unwrapped gyro yaw displacement and 4-corner wheel angular displacement: {@code R_wheel =
   * |deltaTheta_gyro| * driveBaseRadius / avgWheelRadDelta}.
   */
  private static ScalarCalibration estimateWheelRadiusFromGyroSpin(
      List<ParsedLog> allLogs, RobotProfile profile, double driveBaseRadiusMeters) {
    if (allLogs.isEmpty() || driveBaseRadiusMeters <= 0.15) {
      return null;
    }

    int pigeonId = profile != null && profile.pigeonId() > 0 ? profile.pigeonId() : 20;
    List<DoubleSample> yawRadSeries =
        RunAnalyzer.findDoubleSeries(
            allLogs,
            null,
            List.of(
                "/Drive/Gyro/YawPositionRad",
                "Drive/Gyro/YawPositionRad",
                "/AdvantageKit/Drive/Gyro/YawPositionRad"));
    List<DoubleSample> yawDegSeries =
        RunAnalyzer.findDoubleSeries(
            allLogs,
            null,
            List.of(
                "Phoenix6/Pigeon2-" + pigeonId + "/Yaw",
                "/Phoenix6/Pigeon2-" + pigeonId + "/Yaw",
                "/Drive/GyroYawDeg",
                "Drive/GyroYawDeg"));
    boolean yawIsRadians = !yawRadSeries.isEmpty();
    List<DoubleSample> yawSeries = yawIsRadians ? yawRadSeries : yawDegSeries;
    if (yawSeries.size() < 10) {
      return null;
    }

    boolean wheelScaleP6 = RunAnalyzer.isPhoenix6DriveWheelScale(profile, allLogs);
    double gearRatio =
        profile != null && profile.driveGearRatio() > 0.0 ? profile.driveGearRatio() : 6.3947;
    List<List<DoubleSample>> wheelSeries = new ArrayList<>();
    List<Double> toWheelRadScale = new ArrayList<>();

    for (int i = 0; i < 4; i++) {
      List<DoubleSample> akitPos =
          RunAnalyzer.findDoubleSeries(
              allLogs,
              null,
              List.of(
                  "/Drive/Module" + i + "/DrivePositionRad",
                  "Drive/Module" + i + "/DrivePositionRad",
                  "/AdvantageKit/Drive/Module" + i + "/DrivePositionRad"));
      if (akitPos.size() >= 10) {
        wheelSeries.add(akitPos);
        toWheelRadScale.add(1.0);
      }
    }
    if (wheelSeries.size() < 2 && profile != null && !profile.corners().isEmpty()) {
      wheelSeries.clear();
      toWheelRadScale.clear();
      for (RobotProfile.CornerConfig c : profile.corners().values()) {
        if (c.driveMotorId() > 0) {
          List<DoubleSample> p6Pos =
              RunAnalyzer.findDoubleSeries(
                  allLogs,
                  null,
                  List.of(
                      "Phoenix6/TalonFX-" + c.driveMotorId() + "/Position",
                      "/Phoenix6/TalonFX-" + c.driveMotorId() + "/Position"));
          if (p6Pos.size() >= 10) {
            wheelSeries.add(p6Pos);
            toWheelRadScale.add(wheelScaleP6 ? (2.0 * Math.PI) : (2.0 * Math.PI / gearRatio));
          }
        }
      }
    }
    if (wheelSeries.size() < 2) {
      return null;
    }

    List<DoubleSample> vxSeries =
        RunAnalyzer.findDoubleSeries(
            allLogs,
            null,
            List.of(
                "/RealOutputs/Drive/ChassisSpeeds.vx",
                "RealOutputs/Drive/ChassisSpeeds.vx",
                "/AdvantageKit/RealOutputs/Drive/ChassisSpeeds.vx"));
    List<DoubleSample> vySeries =
        RunAnalyzer.findDoubleSeries(
            allLogs,
            null,
            List.of(
                "/RealOutputs/Drive/ChassisSpeeds.vy",
                "RealOutputs/Drive/ChassisSpeeds.vy",
                "/AdvantageKit/RealOutputs/Drive/ChassisSpeeds.vy"));
    boolean hasChassisSpeeds = !vxSeries.isEmpty() && !vySeries.isEmpty();

    double spinYawRad = 0.0;
    double spinWheelRad = 0.0;
    int spinSamples = 0;

    double prevRawYawRad =
        yawIsRadians ? yawSeries.get(0).value() : Math.toRadians(yawSeries.get(0).value());
    long prevTs = yawSeries.get(0).timestampUs();
    double[] prevWheelRad = new double[wheelSeries.size()];
    for (int m = 0; m < wheelSeries.size(); m++) {
      prevWheelRad[m] =
          RunAnalyzer.sampleDoubleAt(wheelSeries.get(m), prevTs, 0.0) * toWheelRadScale.get(m);
    }

    for (int i = 1; i < yawSeries.size(); i++) {
      DoubleSample cur = yawSeries.get(i);
      long ts = cur.timestampUs();
      double dt = (ts - prevTs) / 1_000_000.0;
      double curRawYawRad = yawIsRadians ? cur.value() : Math.toRadians(cur.value());
      double dYaw = Math.atan2(Math.sin(curRawYawRad - prevRawYawRad), Math.cos(curRawYawRad - prevRawYawRad));

      double sumAbsDWheel = 0.0;
      for (int m = 0; m < wheelSeries.size(); m++) {
        double curW =
            RunAnalyzer.sampleDoubleAt(wheelSeries.get(m), ts, prevWheelRad[m] / toWheelRadScale.get(m))
                * toWheelRadScale.get(m);
        sumAbsDWheel += Math.abs(curW - prevWheelRad[m]);
        prevWheelRad[m] = curW;
      }
      double avgDWheel = sumAbsDWheel / wheelSeries.size();
      prevRawYawRad = curRawYawRad;
      prevTs = ts;

      if (dt <= 0.002 || dt > 0.5) {
        continue;
      }
      double yawRate = Math.abs(dYaw) / dt;
      if (yawRate < 0.20) {
        continue;
      }
      if (hasChassisSpeeds) {
        double vx = RunAnalyzer.sampleDoubleAt(vxSeries, ts, 0.0);
        double vy = RunAnalyzer.sampleDoubleAt(vySeries, ts, 0.0);
        if (Math.hypot(vx, vy) > 0.35) {
          continue;
        }
      }
      if (avgDWheel > 1e-4) {
        spinYawRad += Math.abs(dYaw);
        spinWheelRad += avgDWheel;
        spinSamples++;
      }
    }

    if (spinSamples >= 10 && spinYawRad >= MIN_SPIN_YAW_RAD && spinWheelRad >= MIN_SPIN_WHEEL_RAD) {
      double est = (spinYawRad * driveBaseRadiusMeters) / spinWheelRad;
      if (est >= 0.044 && est <= 0.056) {
        return new ScalarCalibration(est, true, "GYRO_SPIN_IN_PLACE");
      }
    }
    return null;
  }

  private static ScalarCalibration estimateCouplingRatio(List<ParsedLog> allLogs, double nominal) {
    List<DoubleSample> couplingSeries =
        RunAnalyzer.findDoubleSeries(
            allLogs,
            null,
            List.of(
                "SimCal/MeasuredCouplingRatio",
                "/SimCal/MeasuredCouplingRatio",
                "Drive/MeasuredCouplingRatio"));
    if (!couplingSeries.isEmpty()) {
      double v = couplingSeries.get(couplingSeries.size() - 1).value();
      if (Double.isFinite(v) && v >= 0.0 && v <= 6.0) {
        return new ScalarCalibration(v, true, "MEASURED_COUPLING");
      }
    }
    return new ScalarCalibration(nominal, false, "NOMINAL_UNMEASURED");
  }

  private static MechanismFeedforwardFit fitMechanismFeedforward(
      List<ParsedLog> allLogs, FluxFig.MechanismSpec spec) {
    String name = spec.mechanism();
    int id = spec.motorId();
    List<DoubleSample> volts =
        RunAnalyzer.findDoubleSeries(
            allLogs,
            null,
            List.of(
                "/" + name + "/AppliedVoltage",
                name + "/AppliedVoltage",
                "/AdvantageKit/" + name + "/AppliedVoltage",
                "Mechanisms/" + name + "/LeaderAppliedVolts",
                "/Mechanisms/" + name + "/LeaderAppliedVolts",
                "/AdvantageKit/RealOutputs/Mechanisms/" + name + "/LeaderAppliedVolts",
                "Phoenix6/TalonFX-" + id + "/MotorVoltage",
                "/Phoenix6/TalonFX-" + id + "/MotorVoltage",
                name + "/AppliedVolts"));
    List<DoubleSample> vel =
        RunAnalyzer.findDoubleSeries(
            allLogs,
            null,
            List.of(
                "/" + name + "/VelocityRPS",
                name + "/VelocityRPS",
                "/AdvantageKit/" + name + "/VelocityRPS",
                "Mechanisms/" + name + "/VelocityRps",
                "/Mechanisms/" + name + "/VelocityRps",
                "/AdvantageKit/RealOutputs/Mechanisms/" + name + "/VelocityRps",
                "Phoenix6/TalonFX-" + id + "/Velocity",
                "/Phoenix6/TalonFX-" + id + "/Velocity",
                name + "/VelocityRps"));
    List<DoubleSample> pos =
        RunAnalyzer.findDoubleSeries(
            allLogs,
            null,
            List.of(
                "/" + name + "/PositionRotations",
                name + "/PositionRotations",
                "/AdvantageKit/" + name + "/PositionRotations",
                "Mechanisms/" + name + "/PositionRot",
                "/Mechanisms/" + name + "/PositionRot",
                "/AdvantageKit/RealOutputs/Mechanisms/" + name + "/PositionRot",
                "Phoenix6/TalonFX-" + id + "/Position",
                "/Phoenix6/TalonFX-" + id + "/Position",
                name + "/PositionRot"));

    return fitFromSeries(
        name,
        volts,
        vel,
        pos,
        spec.gravityType(),
        spec.gearRatio(),
        spec.kS(),
        spec.kV(),
        spec.kA(),
        spec.kG());
  }

  private static List<MechanismFeedforwardFit> fitDiscoveredMechanisms(List<ParsedLog> allLogs) {
    List<MechanismFeedforwardFit> fits = new ArrayList<>();
    String[] candidates = {
      "Elevator", "ElevatorLeader", "Arm", "ArmPivot", "Shooter", "Flywheel", "Hood", "Tilter"
    };
    for (String name : candidates) {
      List<DoubleSample> volts =
          RunAnalyzer.findDoubleSeries(
              allLogs,
              null,
              List.of(
                  "/" + name + "/AppliedVoltage",
                  name + "/AppliedVoltage",
                  "/AdvantageKit/" + name + "/AppliedVoltage",
                  "Mechanisms/" + name + "/LeaderAppliedVolts",
                  "/Mechanisms/" + name + "/LeaderAppliedVolts",
                  "/AdvantageKit/RealOutputs/Mechanisms/" + name + "/LeaderAppliedVolts",
                  name + "/AppliedVolts"));
      List<DoubleSample> vel =
          RunAnalyzer.findDoubleSeries(
              allLogs,
              null,
              List.of(
                  "/" + name + "/VelocityRPS",
                  name + "/VelocityRPS",
                  "/AdvantageKit/" + name + "/VelocityRPS",
                  "Mechanisms/" + name + "/VelocityRps",
                  "/Mechanisms/" + name + "/VelocityRps",
                  "/AdvantageKit/RealOutputs/Mechanisms/" + name + "/VelocityRps",
                  name + "/VelocityRps"));
      List<DoubleSample> pos =
          RunAnalyzer.findDoubleSeries(
              allLogs,
              null,
              List.of(
                  "/" + name + "/PositionRotations",
                  name + "/PositionRotations",
                  "/AdvantageKit/" + name + "/PositionRotations",
                  "Mechanisms/" + name + "/PositionRot",
                  "/Mechanisms/" + name + "/PositionRot",
                  "/AdvantageKit/RealOutputs/Mechanisms/" + name + "/PositionRot",
                  name + "/PositionRot"));
      if (!volts.isEmpty() && !vel.isEmpty()) {
        String lower = name.toLowerCase(Locale.ROOT);
        String grav =
            lower.contains("elevator")
                ? "Elevator_Static"
                : (lower.contains("arm") ? "Arm_Cosine" : "None");
        MechanismFeedforwardFit f =
            fitFromSeries(name, volts, vel, pos, grav, 1.0, 0.15, 0.12, 0.01, 0.0);
        if (f != null) {
          fits.add(f);
        }
      }
    }
    return fits;
  }

  private static MechanismFeedforwardFit fitFromSeries(
      String name,
      List<DoubleSample> volts,
      List<DoubleSample> vel,
      List<DoubleSample> pos,
      String gravityType,
      double gearRatio,
      double defaultKs,
      double defaultKv,
      double defaultKa,
      double defaultKg) {
    if (volts.size() < MIN_MECHANISM_SAMPLES || vel.size() < MIN_MECHANISM_SAMPLES) {
      return null;
    }

    boolean isElevator = "Elevator_Static".equalsIgnoreCase(gravityType);
    boolean isArm = "Arm_Cosine".equalsIgnoreCase(gravityType);
    double effGearRatio = (Double.isFinite(gearRatio) && gearRatio > 0.0) ? gearRatio : 1.0;

    // Estimate kG in motor Volts from near-stationary holding samples (|v| < 0.25 RPS, |V| > 0.05 V)
    double sumHoldG = 0.0;
    int holdCount = 0;
    if (isElevator || isArm) {
      for (DoubleSample vs : volts) {
        long ts = vs.timestampUs();
        double v = RunAnalyzer.sampleDoubleAt(vel, ts, 0.0);
        if (Math.abs(v) < 0.25 && Math.abs(vs.value()) > 0.05 && Math.abs(vs.value()) < 3.5) {
          if (isElevator) {
            sumHoldG += vs.value();
            holdCount++;
          } else {
            double pRot = RunAnalyzer.sampleDoubleAt(pos, ts, 0.0);
            double jointAngleRad = (pRot / effGearRatio) * 2.0 * Math.PI;
            double cosVal = Math.cos(jointAngleRad);
            if (Math.abs(cosVal) > 0.3) {
              sumHoldG += vs.value() / cosVal;
              holdCount++;
            }
          }
        }
      }
    }
    double kG =
        holdCount >= 3
            ? Math.max(0.0, Math.min(2.5, sumHoldG / holdCount))
            : ((isElevator || isArm) ? defaultKg : 0.0);

    // Collect moving samples for joint [sign(v), v, a] regression
    List<double[]> rows = new ArrayList<>();
    double minAbsV = Double.POSITIVE_INFINITY;
    double maxAbsV = Double.NEGATIVE_INFINITY;
    for (int i = 1; i < vel.size(); i++) {
      DoubleSample curVel = vel.get(i);
      DoubleSample prevVel = vel.get(i - 1);
      double dt = (curVel.timestampUs() - prevVel.timestampUs()) / 1_000_000.0;
      if (dt <= 0.001 || dt > 0.5) {
        continue;
      }
      double v = curVel.value();
      if (!Double.isFinite(v) || Math.abs(v) < 1.0) {
        continue;
      }
      double appliedV = RunAnalyzer.sampleDoubleAt(volts, curVel.timestampUs(), Double.NaN);
      if (!Double.isFinite(appliedV) || Math.abs(appliedV) < 0.3) {
        continue;
      }
      double gravTerm = 0.0;
      if (isElevator) {
        gravTerm = kG;
      } else if (isArm) {
        double pRot = RunAnalyzer.sampleDoubleAt(pos, curVel.timestampUs(), 0.0);
        double jointAngleRad = (pRot / effGearRatio) * 2.0 * Math.PI;
        gravTerm = kG * Math.cos(jointAngleRad);
      }
      double netV = appliedV - gravTerm;
      double sign = Math.signum(v);
      if (netV * sign <= 0.0) {
        continue;
      }
      double a = (v - prevVel.value()) / dt;
      double absV = Math.abs(v);
      minAbsV = Math.min(minAbsV, absV);
      maxAbsV = Math.max(maxAbsV, absV);
      rows.add(new double[] {sign, v, a, netV});
    }

    int n = rows.size();
    if (n < MIN_MECHANISM_SAMPLES || (maxAbsV - minAbsV) < MIN_VELOCITY_SPREAD_RPS) {
      return null;
    }

    // Attempt 3-parameter OLS: netV = kS*sign(v) + kV*v + kA*a
    double[] ols3 = solve3x3Ols(rows);
    double kS;
    double kV;
    double kA;
    double rSquared;

    if (ols3 != null
        && Double.isFinite(ols3[0])
        && Double.isFinite(ols3[1])
        && Double.isFinite(ols3[2])
        && ols3[0] >= 0.0
        && ols3[1] > 0.0
        && ols3[2] >= 0.0
        && ols3[2] <= 0.5) {
      kS = ols3[0];
      kV = ols3[1];
      kA = ols3[2];
      rSquared = computeRSquared(rows, kS, kV, kA);
    } else {
      // Fallback 2-parameter OLS on |v| when acceleration is constant/collinear with sign(v)
      double sumV = 0.0;
      double sumY = 0.0;
      double sumVV = 0.0;
      double sumVY = 0.0;
      double sumYY = 0.0;
      for (double[] r : rows) {
        double x = Math.abs(r[1]);
        double y = r[3] * r[0];
        sumV += x;
        sumY += y;
        sumVV += x * x;
        sumVY += x * y;
        sumYY += y * y;
      }
      double denom = n * sumVV - sumV * sumV;
      double varY = n * sumYY - sumY * sumY;
      if (denom <= 1e-6 || varY <= 1e-9) {
        return null;
      }
      kV = (n * sumVY - sumV * sumY) / denom;
      kS = (sumY - kV * sumV) / n;
      double rNum = n * sumVY - sumV * sumY;
      rSquared = (rNum * rNum) / (denom * varY);

      double sumAccelRatio = 0.0;
      int accelCount = 0;
      for (double[] r : rows) {
        if (Math.abs(r[2]) >= 8.0) {
          double residual = r[3] - kS * r[0] - kV * r[1];
          double kaSample = residual / r[2];
          if (Double.isFinite(kaSample) && kaSample > 0.001 && kaSample < 0.25) {
            sumAccelRatio += kaSample;
            accelCount++;
          }
        }
      }
      kA = accelCount >= 3 ? (sumAccelRatio / accelCount) : defaultKa;
    }

    // Strict R^2 and physical sign gate before clamping; reject low-confidence fits
    if (!Double.isFinite(rSquared)
        || rSquared < MIN_MECHANISM_R_SQUARED
        || !Double.isFinite(kV)
        || kV <= 0.0
        || !Double.isFinite(kS)
        || kS < -0.1) {
      return null;
    }

    kS = Math.max(0.01, Math.min(1.5, kS));
    kV = Math.max(0.01, Math.min(2.5, kV));
    kA = Math.max(0.001, Math.min(0.25, kA));
    rSquared = Math.max(0.0, Math.min(1.0, rSquared));

    return new MechanismFeedforwardFit(
        name, n, round4(kS), round4(kV), round4(kA), round4(kG), rSquared);
  }

  private static double[] solve3x3Ols(List<double[]> rows) {
    double[][] a = new double[3][3];
    double[] b = new double[3];
    for (double[] r : rows) {
      double[] x = {r[0], r[1], r[2]};
      double y = r[3];
      for (int i = 0; i < 3; i++) {
        b[i] += x[i] * y;
        for (int j = 0; j < 3; j++) {
          a[i][j] += x[i] * x[j];
        }
      }
    }
    double det =
        a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
            - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
            + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    if (!Double.isFinite(det) || Math.abs(det) < 1e-5) {
      return null;
    }
    double[][] aug = new double[3][4];
    for (int i = 0; i < 3; i++) {
      System.arraycopy(a[i], 0, aug[i], 0, 3);
      aug[i][3] = b[i];
    }
    for (int col = 0; col < 3; col++) {
      int pivot = col;
      for (int row = col + 1; row < 3; row++) {
        if (Math.abs(aug[row][col]) > Math.abs(aug[pivot][col])) {
          pivot = row;
        }
      }
      if (Math.abs(aug[pivot][col]) < 1e-9) {
        return null;
      }
      double[] tmp = aug[col];
      aug[col] = aug[pivot];
      aug[pivot] = tmp;
      double div = aug[col][col];
      for (int j = col; j < 4; j++) {
        aug[col][j] /= div;
      }
      for (int row = 0; row < 3; row++) {
        if (row != col) {
          double factor = aug[row][col];
          for (int j = col; j < 4; j++) {
            aug[row][j] -= factor * aug[col][j];
          }
        }
      }
    }
    return new double[] {aug[0][3], aug[1][3], aug[2][3]};
  }

  private static double computeRSquared(List<double[]> rows, double kS, double kV, double kA) {
    double sumY = 0.0;
    for (double[] r : rows) {
      sumY += r[3];
    }
    double meanY = sumY / rows.size();
    double ssTot = 0.0;
    double ssRes = 0.0;
    for (double[] r : rows) {
      double pred = kS * r[0] + kV * r[1] + kA * r[2];
      double err = r[3] - pred;
      double dev = r[3] - meanY;
      ssRes += err * err;
      ssTot += dev * dev;
    }
    if (ssTot <= 1e-9) {
      return 0.0;
    }
    return 1.0 - (ssRes / ssTot);
  }

  private static void writeSummaryJson(
      Path jsonPath,
      String profile,
      ScalarCalibration wheelCal,
      ScalarCalibration couplingCal,
      Map<String, RunAnalysisReport.BatteryMetrics> batteries,
      List<MechanismFeedforwardFit> mechs,
      String configSha)
      throws IOException {
    StringBuilder sb = new StringBuilder();
    sb.append("{\n");
    sb.append("  \"targetProfile\": \"").append(esc(profile)).append("\",\n");
    sb.append("  \"updatedConfigSha256\": \"").append(esc(configSha)).append("\",\n");
    sb.append("  \"mechanismsCsvModified\": false,\n");
    sb.append(String.format(Locale.US, "  \"wheelRadiusMeters\": %.5f,\n", wheelCal.value()));
    sb.append("  \"wheelRadiusCalibrated\": ").append(wheelCal.calibrated()).append(",\n");
    sb.append("  \"wheelRadiusSource\": \"").append(esc(wheelCal.source())).append("\",\n");
    sb.append(
        String.format(Locale.US, "  \"swerveCouplingRatio\": %.4f,\n", couplingCal.value()));
    sb.append("  \"couplingCalibrated\": ").append(couplingCal.calibrated()).append(",\n");
    sb.append("  \"batteries\": [\n");
    int bIdx = 0;
    for (RunAnalysisReport.BatteryMetrics b : batteries.values()) {
      sb.append(
          String.format(
              Locale.US,
              "    { \"batteryName\": \"%s\", \"estimatedRintMilliOhms\": %s, \"restingVoltage\": %s, \"minVoltage\": %s, \"rSquared\": %s }%s\n",
              esc(b.batteryName()),
              jsonNum(b.estimatedRintMilliOhms(), "%.1f"),
              jsonNum(b.restingVoltage(), "%.2f"),
              jsonNum(b.minVoltage(), "%.2f"),
              jsonNum(b.rSquared(), "%.3f"),
              (++bIdx < batteries.size()) ? "," : ""));
    }
    sb.append("  ],\n");
    sb.append("  \"mechanisms\": [\n");
    for (int i = 0; i < mechs.size(); i++) {
      MechanismFeedforwardFit m = mechs.get(i);
      sb.append(
          String.format(
              Locale.US,
              "    { \"mechanism\": \"%s\", \"samples\": %d, \"kS\": %.4f, \"kV\": %.4f, \"kA\": %.4f, \"kG\": %.4f, \"rSquared\": %.3f }%s\n",
              esc(m.mechanism()),
              m.sampleCount(),
              m.kS(),
              m.kV(),
              m.kA(),
              m.kG(),
              m.rSquared(),
              (i + 1 < mechs.size()) ? "," : ""));
    }
    sb.append("  ]\n}\n");
    Files.writeString(jsonPath, sb.toString(), StandardCharsets.UTF_8);
  }

  private static String jsonNum(double val, String fmt) {
    return Double.isFinite(val) ? String.format(Locale.US, fmt, val) : "null";
  }

  private static double round1(double v) {
    return Math.round(v * 10.0) / 10.0;
  }

  private static double round4(double v) {
    return Math.round(v * 10000.0) / 10000.0;
  }

  private static String esc(String s) {
    return s == null ? "" : s.replace("\\", "\\\\").replace("\"", "\\\"");
  }
}
