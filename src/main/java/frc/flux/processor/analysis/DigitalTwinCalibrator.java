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
 * <p>Extracts empirical physical constants from real robot {@code .wpilog} runs:
 *
 * <ul>
 *   <li>Battery internal resistance {@code R_int (mOhm)} &rarr; written to {@code batteries.csv}.
 *   <li>Effective swerve wheel radius {@code wheelRadiusMeters} and azimuth coupling ratio {@code
 *       swerveCouplingRatio} &rarr; written to {@code parameters.csv}.
 *   <li>Mechanism feedforward constants {@code (kS, kV, kA, kG)} via OLS regression on {@code V =
 *       kS*sign(v) + kV*v + kA*a + kG*g(pos)} &rarr; written to {@code mechanisms.csv}.
 * </ul>
 */
public final class DigitalTwinCalibrator {

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
        try (Stream<Path> s = Files.list(dir)) {
          for (Path p : s.sorted().toList()) {
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
    FluxFig fig = null;
    if (configDir != null && Files.isDirectory(configDir)) {
      fig = FluxFig.load(configDir, targetProfile);
      nominalWheelRadius = RobotProfile.getDoubleOrDefault(fig, "wheelRadiusMeters", 0.0508);
      nominalCoupling = RobotProfile.getDoubleOrDefault(fig, "swerveCouplingRatio", 3.5714);
    }

    double calibratedWheelRadius = estimateWheelRadius(allLogs, nominalWheelRadius);
    double calibratedCoupling = estimateCouplingRatio(allLogs, nominalCoupling);

    List<MechanismFeedforwardFit> mechFits = new ArrayList<>();
    if (fig != null && fig.getTableNames().contains("mechanisms")) {
      List<FluxFig.MechanismSpec> existingSpecs =
          fig.bindTable("mechanisms", targetProfile, FluxFig.MechanismSpec.class);
      List<FluxFig.MechanismSpec> updatedSpecs = new ArrayList<>(existingSpecs.size());
      boolean anyMechUpdated = false;

      for (FluxFig.MechanismSpec spec : existingSpecs) {
        MechanismFeedforwardFit fit = fitMechanismFeedforward(allLogs, spec);
        if (fit != null && fit.sampleCount() >= 8) {
          mechFits.add(fit);
          updatedSpecs.add(
              new FluxFig.MechanismSpec(
                  spec.robot(),
                  spec.mechanism(),
                  spec.motorId(),
                  spec.canBus(),
                  spec.statorLimitAmps(),
                  spec.supplyLimitAmps(),
                  spec.slipCurrentAmps(),
                  spec.gravityType(),
                  spec.kP(),
                  spec.kI(),
                  spec.kD(),
                  round4(fit.kS()),
                  round4(fit.kV()),
                  round4(fit.kA()),
                  round4(fit.kG()),
                  spec.minPosRot(),
                  spec.maxPosRot(),
                  spec.followerMotorId(),
                  spec.invertFollower(),
                  spec.motorInverted(),
                  spec.gearRatio()));
          anyMechUpdated = true;
        } else {
          mechFits.add(
              new MechanismFeedforwardFit(
                  spec.mechanism(), 0, spec.kS(), spec.kV(), spec.kA(), spec.kG(), 0.0));
          updatedSpecs.add(spec);
        }
      }
      if (anyMechUpdated && Files.exists(configDir.resolve("mechanisms.csv"))) {
        fig.saveMechanismSpecs(targetProfile, updatedSpecs);
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

        for (RunAnalysisReport.BatteryMetrics bm : batteryMap.values()) {
          double clampedNominal = Math.max(8.0, Math.min(35.0, round1(bm.estimatedRintMilliOhms())));
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
        }
        fig.saveBatterySpecs(currentBatteries);
      }

      if (Files.exists(configDir.resolve("parameters.csv"))) {
        FluxFig tunableFig = FluxFig.load(configDir, targetProfile);
        tunableFig.setMode(FluxFig.Mode.TUNING);
        if (!Double.isNaN(
            RobotProfile.getDoubleOrDefault(tunableFig, "wheelRadiusMeters", Double.NaN))) {
          tunableFig.setOverride(
              targetProfile,
              "wheelRadiusMeters",
              String.format(Locale.US, "%.5f", calibratedWheelRadius));
        }
        if (!Double.isNaN(
            RobotProfile.getDoubleOrDefault(tunableFig, "swerveCouplingRatio", Double.NaN))) {
          tunableFig.setOverride(
              targetProfile,
              "swerveCouplingRatio",
              String.format(Locale.US, "%.4f", calibratedCoupling));
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
        calibratedWheelRadius,
        calibratedCoupling,
        batteryMap,
        mechFits,
        updatedSha);

    return new SimCalibrationSummary(
        targetProfile,
        calibratedWheelRadius,
        calibratedCoupling,
        batteryMap,
        mechFits,
        updatedSha,
        jsonPath);
  }

  private static double estimateWheelRadius(List<ParsedLog> allLogs, double nominal) {
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
          return est;
        }
      }
    }
    return nominal;
  }

  private static double estimateCouplingRatio(List<ParsedLog> allLogs, double nominal) {
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
      if (v >= 0.0 && v <= 6.0) {
        return v;
      }
    }
    return nominal;
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
        spec.kS(),
        spec.kV(),
        spec.kA(),
        spec.kG());
  }

  private static List<MechanismFeedforwardFit> fitDiscoveredMechanisms(List<ParsedLog> allLogs) {
    List<MechanismFeedforwardFit> fits = new ArrayList<>();
    String[] candidates = {"Elevator", "Arm", "Shooter", "Flywheel", "Hood"};
    for (String name : candidates) {
      List<DoubleSample> volts =
          RunAnalyzer.findDoubleSeries(
              allLogs,
              null,
              List.of(
                  "Mechanisms/" + name + "/LeaderAppliedVolts",
                  "/Mechanisms/" + name + "/LeaderAppliedVolts",
                  "/AdvantageKit/RealOutputs/Mechanisms/" + name + "/LeaderAppliedVolts",
                  name + "/AppliedVolts"));
      List<DoubleSample> vel =
          RunAnalyzer.findDoubleSeries(
              allLogs,
              null,
              List.of(
                  "Mechanisms/" + name + "/VelocityRps",
                  "/Mechanisms/" + name + "/VelocityRps",
                  "/AdvantageKit/RealOutputs/Mechanisms/" + name + "/VelocityRps",
                  name + "/VelocityRps"));
      List<DoubleSample> pos =
          RunAnalyzer.findDoubleSeries(
              allLogs,
              null,
              List.of(
                  "Mechanisms/" + name + "/PositionRot",
                  "/Mechanisms/" + name + "/PositionRot",
                  "/AdvantageKit/RealOutputs/Mechanisms/" + name + "/PositionRot",
                  name + "/PositionRot"));
      if (!volts.isEmpty() && !vel.isEmpty()) {
        String grav =
            "Elevator".equalsIgnoreCase(name)
                ? "Elevator_Static"
                : ("Arm".equalsIgnoreCase(name) ? "Arm_Cosine" : "None");
        MechanismFeedforwardFit f =
            fitFromSeries(name, volts, vel, pos, grav, 0.15, 0.12, 0.01, 0.0);
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
      double defaultKs,
      double defaultKv,
      double defaultKa,
      double defaultKg) {
    if (volts.size() < 8 || vel.size() < 8) {
      return null;
    }

    boolean isElevator = "Elevator_Static".equalsIgnoreCase(gravityType);
    boolean isArm = "Arm_Cosine".equalsIgnoreCase(gravityType);

    // Estimate kG from near-stationary holding samples (|v| < 0.25 RPS, |V| > 0.05 V)
    double sumHoldG = 0.0;
    int holdCount = 0;
    if (isElevator || isArm) {
      for (DoubleSample vs : volts) {
        long ts = vs.timestampUs();
        double v = RunAnalyzer.sampleDoubleAt(vel, ts, 0.0);
        if (Math.abs(v) < 0.25 && Math.abs(vs.value()) > 0.05 && Math.abs(vs.value()) < 3.0) {
          if (isElevator) {
            sumHoldG += vs.value();
            holdCount++;
          } else {
            double pRot = RunAnalyzer.sampleDoubleAt(pos, ts, 0.0);
            double cosVal = Math.cos(pRot * 2.0 * Math.PI);
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

    // OLS regression on moving samples: (V - kG*g) = kS * sign(v) + kV * v + kA * a
    double sumV = 0.0;
    double sumY = 0.0;
    double sumVV = 0.0;
    double sumVY = 0.0;
    double sumYY = 0.0;
    double sumAccelRatio = 0.0;
    int accelCount = 0;
    int n = 0;

    for (int i = 1; i < vel.size(); i++) {
      DoubleSample curVel = vel.get(i);
      DoubleSample prevVel = vel.get(i - 1);
      double dt = (curVel.timestampUs() - prevVel.timestampUs()) / 1_000_000.0;
      if (dt <= 0.001 || dt > 0.5) {
        continue;
      }
      double v = curVel.value();
      if (Math.abs(v) < 1.0) {
        continue;
      }
      double appliedV = RunAnalyzer.sampleDoubleAt(volts, curVel.timestampUs(), Double.NaN);
      if (Double.isNaN(appliedV) || Math.abs(appliedV) < 0.3) {
        continue;
      }
      double gravTerm = 0.0;
      if (isElevator) {
        gravTerm = kG;
      } else if (isArm) {
        double pRot = RunAnalyzer.sampleDoubleAt(pos, curVel.timestampUs(), 0.0);
        gravTerm = kG * Math.cos(pRot * 2.0 * Math.PI);
      }
      double netV = appliedV - gravTerm;
      double absV = Math.abs(v);
      double signAdjustedNetV = netV * Math.signum(v);
      if (signAdjustedNetV <= 0.0) {
        continue;
      }

      sumV += absV;
      sumY += signAdjustedNetV;
      sumVV += absV * absV;
      sumVY += absV * signAdjustedNetV;
      sumYY += signAdjustedNetV * signAdjustedNetV;
      n++;
    }

    if (n < 5) {
      return null;
    }

    double denom = n * sumVV - sumV * sumV;
    double kV = defaultKv;
    double kS = defaultKs;
    double rSquared = 0.0;

    if (Math.abs(denom) > 1e-6) {
      kV = (n * sumVY - sumV * sumY) / denom;
      kS = (sumY - kV * sumV) / n;
      double rDenom = denom * (n * sumYY - sumY * sumY);
      if (rDenom > 1e-9) {
        double rNum = n * sumVY - sumV * sumY;
        rSquared = Math.max(0.0, Math.min(1.0, (rNum * rNum) / rDenom));
      }
    }
    kS = Math.max(0.02, Math.min(1.5, kS));
    kV = Math.max(0.02, Math.min(2.5, kV));

    // Estimate kA from high-acceleration samples
    for (int i = 1; i < vel.size(); i++) {
      DoubleSample curVel = vel.get(i);
      DoubleSample prevVel = vel.get(i - 1);
      double dt = (curVel.timestampUs() - prevVel.timestampUs()) / 1_000_000.0;
      if (dt <= 0.001 || dt > 0.25) {
        continue;
      }
      double a = (curVel.value() - prevVel.value()) / dt;
      if (Math.abs(a) < 8.0) {
        continue;
      }
      double appliedV = RunAnalyzer.sampleDoubleAt(volts, curVel.timestampUs(), Double.NaN);
      if (Double.isNaN(appliedV)) {
        continue;
      }
      double gravTerm = 0.0;
      if (isElevator) {
        gravTerm = kG;
      } else if (isArm) {
        double pRot = RunAnalyzer.sampleDoubleAt(pos, curVel.timestampUs(), 0.0);
        gravTerm = kG * Math.cos(pRot * 2.0 * Math.PI);
      }
      double residual =
          appliedV - gravTerm - kS * Math.signum(curVel.value()) - kV * curVel.value();
      double kaSample = residual / a;
      if (kaSample > 0.001 && kaSample < 0.25) {
        sumAccelRatio += kaSample;
        accelCount++;
      }
    }
    double kA = accelCount >= 3 ? (sumAccelRatio / accelCount) : defaultKa;

    return new MechanismFeedforwardFit(name, n, kS, kV, kA, kG, rSquared);
  }

  private static void writeSummaryJson(
      Path jsonPath,
      String profile,
      double wheelRadius,
      double couplingRatio,
      Map<String, RunAnalysisReport.BatteryMetrics> batteries,
      List<MechanismFeedforwardFit> mechs,
      String configSha)
      throws IOException {
    StringBuilder sb = new StringBuilder();
    sb.append("{\n");
    sb.append("  \"targetProfile\": \"").append(esc(profile)).append("\",\n");
    sb.append("  \"updatedConfigSha256\": \"").append(esc(configSha)).append("\",\n");
    sb.append(
        String.format(Locale.US, "  \"wheelRadiusMeters\": %.5f,\n", wheelRadius));
    sb.append(
        String.format(Locale.US, "  \"swerveCouplingRatio\": %.4f,\n", couplingRatio));
    sb.append("  \"batteries\": [\n");
    int bIdx = 0;
    for (RunAnalysisReport.BatteryMetrics b : batteries.values()) {
      sb.append(
          String.format(
              Locale.US,
              "    { \"batteryName\": \"%s\", \"estimatedRintMilliOhms\": %.1f, \"restingVoltage\": %.2f, \"minVoltage\": %.2f, \"rSquared\": %.3f }%s\n",
              esc(b.batteryName()),
              b.estimatedRintMilliOhms(),
              b.restingVoltage(),
              b.minVoltage(),
              b.rSquared(),
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
