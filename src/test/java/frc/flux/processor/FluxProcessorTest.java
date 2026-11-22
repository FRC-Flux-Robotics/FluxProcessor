package frc.flux.processor;

import frc.flux.processor.analysis.BatteryAnalyzer;
import frc.flux.processor.analysis.DigitalTwinCalibrator;
import frc.flux.processor.analysis.RunAnalyzer;
import frc.flux.processor.hoot.HootConverter;
import frc.flux.processor.mcp.FluxMcpServer;
import frc.flux.processor.model.RobotProfile;
import frc.flux.processor.model.RunAnalysisReport;
import frc.flux.processor.report.AutoSimVideoRenderer;
import frc.flux.processor.report.HtmlReportGenerator;
import frc.lib.fluxfig.FluxFig;
import frc.lib.fluxfig.FluxWpiLogCodec;
import frc.lib.fluxfig.FluxWpiLogCodec.Writer;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Stream;

/**
 * Self-contained verification tests for {@code FluxProcessor} v2027.1.0:
 *
 * <ul>
 *   <li>Binary {@code .wpilog} read/write round-trip
 *   <li>Battery internal resistance regression, 4-corner swerve symmetry, shot detection, 6-second
 *       multi-headset voice lookback, and video jump links
 *   <li>Multi-team AdvantageKit &amp; {@code double[]} array auto-discovery
 *   <li>Custom {@code robot-profile.json} &amp; custom Phoenix 6 CAN IDs
 *   <li>Offline SystemCore 14-metric {@code system-snapshot.txt} watch audit
 *   <li>{@code FluxFig} {@code /FluxFig/EffectiveSnapshot} hydration &amp; {@code FluxTray} {@code
 *       run-meta.json} contract ({@code batteryName} with spaces + JSON {@code voiceNotes} array)
 *   <li>5m x 3m {@code ShopStressSuiteCmd} pass gates, 2027 Elevator {@code L1..L4} cycles, {@code
 *       cam-snapshots/*.jpg}, and {@code .wpilog}-only watch metric guard
 *   <li>Digital-Twin Sim Auto-Calibration ({@code --calibrate-sim}) CSV write-back via {@code
 *       FluxFig}
 *   <li>JSON-RPC 2.0 {@code --mcp} Multimodal Server across all 8 tools
 * </ul>
 */
public final class FluxProcessorTest {

  private static final List<Path> TEMP_DIRS = new ArrayList<>();

  private static Path createTempDir(String prefix) throws IOException {
    Path dir = Files.createTempDirectory(prefix);
    TEMP_DIRS.add(dir);
    return dir;
  }

  private static void cleanupTempDirs() {
    for (Path root : TEMP_DIRS) {
      if (!Files.exists(root)) {
        continue;
      }
      try (Stream<Path> walk = Files.walk(root)) {
        walk.sorted(Comparator.reverseOrder())
            .forEach(
                p -> {
                  try {
                    Files.deleteIfExists(p);
                  } catch (IOException ignored) {
                  }
                });
      } catch (IOException ignored) {
      }
    }
  }

  public static void main(String[] args) throws Exception {
    try {
      testWpiLogReadWriteRoundTrip();
      testEndToEndSyntheticRunWithVoiceAndVideo();
      testOtherTeamAdvantageKitAndDoubleArrayAutoDiscovery();
      testOtherTeamCustomJsonProfileAndCustomPhoenix6Ids();
      testOfflineSystemCoreWatchMetricsAndReportCard();
      testFluxFigSnapshotAndFluxTrayRunMetaContract();
      testShopStressGateElevatorCyclesAndWpilogOnlyWatchGuard();
      testDigitalTwinCalibratorCsvWriteBack();
      testMcpServerMultimodalTools();
      testStudentAutoVideoRendererCli();
      testBinaryAudioEmptyBatteryNoDataAndRecursiveHootDiscovery();
      testRoboRioTotalLoopAndTraceTopicAlignment();
      testTempDirZipCleanupAndCliParsing();
      testReal2027AdvantageKitTopicsAndSwerveModulesCsv();
      testCandidatePracticeTuningDiffAndPromoteGate();
      testTelemetryTruthfulnessHootScaleNoSyntheticVoltageMultiEnableAndBatteryGates();
      testDigitalTwinCalibratorGyroSpinArmCosineR2GateAndPidProtection();
      testHtmlReportOfflineSafeAndXssClean();
      System.out.println("ALL FLUX PROCESSOR TESTS PASSED (18/18)");
    } finally {
      cleanupTempDirs();
    }
  }

  public static void testWpiLogReadWriteRoundTrip() throws Exception {
    Path tmpDir = createTempDir("flux-wpilog-test");
    Path logFile = tmpDir.resolve("test_rio.wpilog");
    try (Writer w = new Writer(logFile, "TestHeader")) {
      int eEnable = w.startEntry("RobotEnable", "boolean", "", 1_000_000L);
      int eVolt = w.startEntry("Voltage", "double", "", 1_000_000L);
      int eMode = w.startEntry("RobotMode", "string", "", 1_000_000L);

      w.appendBoolean(eEnable, 1_000_000L, true);
      w.appendDouble(eVolt, 1_000_000L, 12.5);
      w.appendString(eMode, 1_000_000L, "Teleop");
      w.appendDouble(eVolt, 2_000_000L, 11.8);
    }

    FluxWpiLogCodec.ParsedLog parsed = FluxWpiLogCodec.read(logFile);
    assertTrue("TestHeader".equals(parsed.extraHeader()), "Expected TestHeader");
    assertTrue(parsed.getBooleanSeries("RobotEnable").size() == 1, "Expected 1 boolean sample");
    assertTrue(parsed.getDoubleSeries("Voltage").size() == 2, "Expected 2 double samples");
    assertTrue(
        Math.abs(parsed.getDoubleSeries("Voltage").get(1).value() - 11.8) < 1e-6,
        "Expected 11.8V");
  }

  public static void testEndToEndSyntheticRunWithVoiceAndVideo() throws Exception {
    Path runDir = createTempDir("flux-run-test");

    Files.writeString(
        runDir.resolve("run-meta.json"),
        """
        {
          "robotId": "FUEL",
          "profile": "FUEL",
          "batteryName": "Thor",
          "gitSha": "abc1234",
          "configSha256": "cfg9876",
          "loopPeriodMs": 20,
          "bootTimestampIso": "2026-04-17T14:22:00Z",
          "videoUrl": "https://youtu.be/dQw4w9WgXcQ",
          "videoOffsetSec": 12.0
        }
        """,
        StandardCharsets.UTF_8);

    Files.writeString(
        runDir.resolve("voice-notes.txt"),
        """
        6.2 | DRIVER | Felt a hard battery sag on acceleration
        12.0 | OPERATOR | Missed short on that shot
        """,
        StandardCharsets.UTF_8);

    try (Writer rio = new Writer(runDir.resolve("test_rio.wpilog"), "Rio");
        Writer drive = new Writer(runDir.resolve("test_drive.wpilog"), "Drive");
        Writer mech = new Writer(runDir.resolve("test_mech.wpilog"), "Mech")) {

      int eEnable = rio.startEntry("RobotEnable", "boolean", "", 0L);
      int eMode = rio.startEntry("RobotMode", "string", "", 0L);
      int eAlliance = rio.startEntry("AllianceStation", "string", "", 0L);

      int eVolt = drive.startEntry("Phoenix6/Pigeon2-20/SupplyVoltage", "double", "", 0L);
      int eFlStator = drive.startEntry("Phoenix6/TalonFX-2/StatorCurrent", "double", "", 0L);
      int eFrStator = drive.startEntry("Phoenix6/TalonFX-4/StatorCurrent", "double", "", 0L);
      int eBlStator = drive.startEntry("Phoenix6/TalonFX-6/StatorCurrent", "double", "", 0L);
      int eBrStator = drive.startEntry("Phoenix6/TalonFX-8/StatorCurrent", "double", "", 0L);

      int eShootL = mech.startEntry("Phoenix6/TalonFX-11/Velocity", "double", "", 0L);
      int eShootR = mech.startEntry("Phoenix6/TalonFX-10/Velocity", "double", "", 0L);
      int eFeeder = mech.startEntry("Phoenix6/TalonFX-3/StatorCurrent", "double", "", 0L);
      int eHood = mech.startEntry("Phoenix6/TalonFX-12/Position", "double", "", 0L);

      rio.appendString(eAlliance, 0L, "Blue 1");
      rio.appendBoolean(eEnable, 0L, false);
      drive.appendDouble(eVolt, 0L, 12.6);

      for (int step = 1; step <= 75; step++) {
        double tSec = step * 0.2;
        long tsUs = Math.round(tSec * 1_000_000.0);
        rio.appendBoolean(eEnable, tsUs, true);
        rio.appendString(eMode, tsUs, "Teleop");

        // Heavy acceleration spike around t = 4.0s .. 5.0s
        double statorPerWheel = (tSec >= 4.0 && tSec <= 5.0) ? 95.0 : 25.0;
        double totalLoad = statorPerWheel * 4 * 0.38;
        double vBatt = (tSec >= 4.0 && tSec <= 5.0) ? 6.65 : (12.6 - totalLoad * 0.015);

        drive.appendDouble(eVolt, tsUs, vBatt);
        drive.appendDouble(eFlStator, tsUs, statorPerWheel);
        drive.appendDouble(eFrStator, tsUs, statorPerWheel);
        drive.appendDouble(eBlStator, tsUs, statorPerWheel);
        // Simulate rubbing gearbox on BR (+45% higher current)
        drive.appendDouble(eBrStator, tsUs, statorPerWheel * 1.45);

        // Simulate a shot at t = 9.6s .. 10.2s
        boolean shooting = (tSec >= 9.6 && tSec <= 10.2);
        mech.appendDouble(eShootL, tsUs, shooting ? 42.0 : 0.0);
        mech.appendDouble(eShootR, tsUs, shooting ? 41.5 : 0.0);
        mech.appendDouble(eFeeder, tsUs, shooting ? 18.0 : 0.0);
        mech.appendDouble(eHood, tsUs, 8.5);
      }
    }

    RunAnalysisReport report =
        RunAnalyzer.analyzeRunDirectory(runDir, RobotProfile.FUEL, true);

    assertTrue("Thor".equals(report.namedBattery()), "Expected Named Battery Thor");
    assertTrue(
        "CRITICAL".equals(report.overallVerdict()),
        "Expected CRITICAL verdict due to 6.65V sag");
    assertTrue(
        report.shots().size() == 1, "Expected 1 detected shot, got " + report.shots().size());
    assertTrue(
        report.voiceCorrelations().size() == 2,
        "Expected 2 correlated voice notes, got " + report.voiceCorrelations().size());
    assertTrue(
        report.voiceCorrelations().get(1).correlatedEventSummary().contains("Shot #1"),
        "Operator voice note at 12.0s should correlate with Shot #1 at 9.6s");
    assertTrue(
        report.voiceCorrelations().get(1).videoLink().contains("youtu.be/dQw4w9WgXcQ?t="),
        "Expected YouTube jump link on voice correlation");
    assertTrue(
        Files.exists(runDir.resolve("merged.wpilog")), "Expected merged.wpilog to be written");
    FluxWpiLogCodec.ParsedLog mergedLog = FluxWpiLogCodec.read(runDir.resolve("merged.wpilog"));
    assertTrue(
        !mergedLog.getDoubleSeries("/Odometry/Robot.x").isEmpty(),
        "Expected /Odometry/Robot struct:Pose2d in merged.wpilog");
    assertTrue(
        !mergedLog.getDoubleSeries("/FuelSim/Fuels.count").isEmpty(),
        "Expected /FuelSim/Fuels struct:Translation3d[] in merged.wpilog");
    assertTrue(
        mergedLog.entriesByName().containsKey("/.schema/struct:Pose2d")
            && mergedLog.entriesByName().containsKey("/.schema/struct:Translation3d"),
        "Expected WPILib structschema definitions in merged.wpilog for AdvantageScope 3D Field");

    Path reportHtml = runDir.resolve("report.html");
    HtmlReportGenerator.writeRunReport(report, reportHtml);
    HtmlReportGenerator.writeBatchSummary(List.of(report), runDir);
    assertTrue(Files.exists(reportHtml), "Expected report.html to be generated");
    assertTrue(
        report.fuelSim() != null && !report.fuelSim().frames().isEmpty(),
        "Expected 3D FuelSim frames in RunAnalysisReport");
    String htmlContent = Files.readString(reportHtml, StandardCharsets.UTF_8);
    assertTrue(
        htmlContent.contains("sim3dCanvas") && htmlContent.contains("sim3dFrames"),
        "Expected interactive 3D FuelSim canvas player in report.html");
    assertTrue(Files.exists(runDir.resolve("runs-index.html")), "Expected runs-index.html");
    assertTrue(
        Files.exists(runDir.resolve("batteries-health.csv")), "Expected batteries-health.csv");
  }

  public static void testOtherTeamAdvantageKitAndDoubleArrayAutoDiscovery() throws Exception {
    Path runDir = createTempDir("other-team-akit-test");
    try (Writer akit =
        new Writer(runDir.resolve("akit_match.wpilog"), "AdvantageKit")) {
      int eRobot = akit.startEntry("/AdvantageKit/Robot", "string", "", 0L);
      int eEnable = akit.startEntry("/DriverStation/Enabled", "boolean", "", 0L);
      int eVolt = akit.startEntry("/SystemStats/BatteryVoltage", "double", "", 0L);
      int eDriveAmpsArr = akit.startEntry("/Drive/DriveCurrentAmps", "double[]", "", 0L);
      int eMod0Vel =
          akit.startEntry("/Drive/Module0/DriveVelocityMetersPerSec", "double", "", 0L);
      int eMod1Vel =
          akit.startEntry("/Drive/Module1/DriveVelocityMetersPerSec", "double", "", 0L);
      int eMod2Vel =
          akit.startEntry("/Drive/Module2/DriveVelocityMetersPerSec", "double", "", 0L);
      int eMod3Vel =
          akit.startEntry("/Drive/Module3/DriveVelocityMetersPerSec", "double", "", 0L);
      int eFlywheelVel = akit.startEntry("/Flywheel/VelocityRps", "double", "", 0L);
      int eFlywheelAmps = akit.startEntry("/Flywheel/CurrentAmps", "double", "", 0L);
      int eFeederAmps = akit.startEntry("/Feeder/CurrentAmps", "double", "", 0L);

      akit.appendString(eRobot, 0L, "ALPHA_6328");
      akit.appendBoolean(eEnable, 0L, false);
      akit.appendDouble(eVolt, 0L, 12.7);

      for (int step = 1; step <= 40; step++) {
        double tSec = step * 0.2;
        long tsUs = Math.round(tSec * 1_000_000.0);
        akit.appendBoolean(eEnable, tsUs, true);
        akit.appendDouble(eVolt, tsUs, 11.9);
        akit.appendDoubleArray(eDriveAmpsArr, tsUs, new double[] {32.0, 31.5, 33.0, 48.0});
        akit.appendDouble(eMod0Vel, tsUs, 3.8);
        akit.appendDouble(eMod1Vel, tsUs, 3.8);
        akit.appendDouble(eMod2Vel, tsUs, 3.8);
        akit.appendDouble(eMod3Vel, tsUs, 3.8);

        boolean shooting = (tSec >= 3.0 && tSec <= 3.8);
        akit.appendDouble(eFlywheelVel, tsUs, shooting ? 50.0 : 0.0);
        akit.appendDouble(eFlywheelAmps, tsUs, shooting ? 35.0 : 1.0);
        akit.appendDouble(eFeederAmps, tsUs, shooting ? 15.0 : 0.5);
      }
    }

    RunAnalysisReport report =
        RunAnalyzer.analyzeRunDirectory(runDir, RobotProfile.AUTO, true);

    assertTrue(
        "ALPHA_6328".equals(report.robotName()),
        "Expected auto-discovered robot name ALPHA_6328, got " + report.robotName());
    assertTrue(
        report.swerveCorners().size() == 4,
        "Expected 4 auto-discovered AdvantageKit swerve corners");
    assertTrue(
        Math.abs(report.peakSpeedMps() - 3.8) < 0.1,
        "Expected 3.8 m/s peak speed from AdvantageKit m/s topics, got "
            + report.peakSpeedMps());
    assertTrue(
        report.shots().size() == 1,
        "Expected 1 detected shot from generic /Flywheel + /Feeder subsystems, got "
            + report.shots().size());
  }

  public static void testOtherTeamCustomJsonProfileAndCustomPhoenix6Ids() throws Exception {
    Path runDir = createTempDir("other-team-json-test");
    Files.writeString(
        runDir.resolve("robot-profile.json"),
        """
        {
          "robotName": "KRAKEN_254",
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
            { "name": "Kicker", "motorId": 52, "role": "Feeder Roller", "currentLimitAmps": 40.0, "isShooter": false }
          ]
        }
        """,
        StandardCharsets.UTF_8);

    try (Writer log = new Writer(runDir.resolve("custom_match.wpilog"), "Custom")) {
      int eEnable = log.startEntry("DS:enabled", "boolean", "", 0L);
      int eVolt = log.startEntry("RobotController/BatteryVoltage", "double", "", 0L);
      int eFl = log.startEntry("Phoenix6/TalonFX-10/StatorCurrent", "double", "", 0L);
      int eFr = log.startEntry("Phoenix6/TalonFX-20/StatorCurrent", "double", "", 0L);
      int eBl = log.startEntry("Phoenix6/TalonFX-30/StatorCurrent", "double", "", 0L);
      int eBr = log.startEntry("Phoenix6/TalonFX-40/StatorCurrent", "double", "", 0L);
      int eElevPos = log.startEntry("Phoenix6/TalonFX-50/Position", "double", "", 0L);
      int eElevAmps = log.startEntry("Phoenix6/TalonFX-50/StatorCurrent", "double", "", 0L);
      int eLaunchVel = log.startEntry("Phoenix6/TalonFX-51/Velocity", "double", "", 0L);
      int eKickerAmps = log.startEntry("Phoenix6/TalonFX-52/StatorCurrent", "double", "", 0L);

      for (int step = 1; step <= 30; step++) {
        double tSec = step * 0.2;
        long tsUs = Math.round(tSec * 1_000_000.0);
        log.appendBoolean(eEnable, tsUs, true);
        log.appendDouble(eVolt, tsUs, 12.1);
        log.appendDouble(eFl, tsUs, 40.0);
        log.appendDouble(eFr, tsUs, 40.0);
        log.appendDouble(eBl, tsUs, 40.0);
        log.appendDouble(eBr, tsUs, 40.0);
        log.appendDouble(eElevPos, tsUs, 14.5);
        log.appendDouble(eElevAmps, tsUs, 22.0);
        boolean launch = (tSec >= 2.0 && tSec <= 2.6);
        log.appendDouble(eLaunchVel, tsUs, launch ? 65.0 : 0.0);
        log.appendDouble(eKickerAmps, tsUs, launch ? 20.0 : 0.0);
      }
    }

    RunAnalysisReport report =
        RunAnalyzer.analyzeRunDirectory(runDir, RobotProfile.AUTO, false);

    assertTrue(
        "KRAKEN_254".equals(report.robotName()),
        "Expected KRAKEN_254 from robot-profile.json");
    assertTrue(report.swerveCorners().get(0).driveId() == 10, "Expected FL driveId 10");
    assertTrue(
        report.mechanisms().size() == 3, "Expected 3 custom mechanisms from JSON profile");
    assertTrue(
        report.shots().size() == 1,
        "Expected 1 shot detected on custom Launcher 51 + Kicker 52");
    assertTrue(
        Math.abs(report.shots().get(0).hoodPositionRot() - 14.5) < 1e-6,
        "Expected Elevator position 14.5 captured during shot");
  }

  public static void testOfflineSystemCoreWatchMetricsAndReportCard() throws Exception {
    Path runDir = createTempDir("systemcore-watch-test");
    Files.writeString(
        runDir.resolve("system-snapshot.txt"),
        """
        [PLATFORM]
        controllerType=SYSTEMCORE
        hostname=SystemCore-10413
        [LOOP_AND_TIMING]
        loopPeriodMs=5.0
        loopMedianMs=1.6
        loopP99Ms=7.9
        loopMaxMs=15.2
        firstEnablePeakMs=16.8
        odometryJitterMaxMs=2.1
        driveTraceMaxMs=0.8
        visionTraceMaxMs=3.2
        [CPU_AND_IRQ]
        totalCpuPct=86.0
        javaProcessCpuPct=14.0
        visionDaemonCpuPct=66.0
        softIrqPct=29.0
        ioWaitPct=6.5
        [THERMAL_AND_POWER]
        cpuTempC=83.0
        cpuFreqMHz=1800
        throttledHex=0x50005
        [SOCKETCAN]
        can.can_s0=350000,90000,12,0,4,0,0,ERROR-ACTIVE
        can.can_s1=500,100,0,0,0,0,0,ERROR-ACTIVE
        can.can_s2=140000,40000,0,0,0,8,1,ERROR-PASSIVE
        [JVM_AND_STORAGE]
        heapAllocRateMbPerSec=3.6
        gcCorrelatedLoopSpikes=2
        freeStorageMb=250.0
        [HARDWARE_PMU]
        ipc=0.72
        llcMissPct=18.5
        involuntaryContextSwitches=620
        """,
        StandardCharsets.UTF_8);

    try (Writer log = new Writer(runDir.resolve("akit_sc.wpilog"), "AdvantageKit")) {
      int eEnable = log.startEntry("/DriverStation/Enabled", "boolean", "", 0L);
      int eVolt = log.startEntry("/SystemStats/BatteryVoltage", "double", "", 0L);
      for (int step = 1; step <= 20; step++) {
        long tsUs = step * 200_000L;
        log.appendBoolean(eEnable, tsUs, true);
        log.appendDouble(eVolt, tsUs, 12.2);
      }
    }

    RunAnalysisReport report =
        RunAnalyzer.analyzeRunDirectory(runDir, RobotProfile.AUTO, false);
    assertTrue(
        report.systemHealth() != null,
        "Expected systemHealth to be populated from system-snapshot.txt");
    assertTrue(
        report.systemHealth().watchDecisions().size() >= 14,
        "Expected all 14 offline SystemCore watch decisions to trigger, got "
            + report.systemHealth().watchDecisions().size());
    assertTrue(
        "CRITICAL".equals(report.overallVerdict()),
        "Expected CRITICAL verdict from odd CAN port / thermal");

    Path htmlPath = runDir.resolve("report.html");
    HtmlReportGenerator.writeRunReport(report, htmlPath);
    String html = Files.readString(htmlPath, StandardCharsets.UTF_8);
    assertTrue(
        html.contains("SystemCore &amp; Controller Offline Watch Audit"),
        "Expected SystemCore watch audit card in report.html");
  }

  public static void testFluxFigSnapshotAndFluxTrayRunMetaContract() throws Exception {
    Path cfgDir = createSampleFluxFigDirectory();
    FluxFig fig = FluxFig.load(cfgDir, "COMP_2027");
    String snapshot = fig.getEffectiveSnapshot();

    Path runDir = createTempDir("fluxfig-snapshot-test");
    // Write unified robot/FluxTray run-meta.json format with spaces in batteryName and JSON voiceNotes array
    Files.writeString(
        runDir.resolve("run-meta.json"),
        """
        {
          "runId": "run_2027_q1",
          "robotId": "COMP_2027",
          "profile": "COMP_2027",
          "batteryName": "2027-B04 (Alpha Pack)",
          "gitSha": "9f8e7d6",
          "configSha256": "deadbeef1234",
          "loopPeriodMs": 5,
          "bootTimestampIso": "2027-02-18T16:08:00Z",
          "voiceNotes": [
            { "role": "DRIVER", "timeSec": 4.2, "text": "Elevator L4 extension felt super smooth" }
          ]
        }
        """,
        StandardCharsets.UTF_8);

    try (Writer log = new Writer(runDir.resolve("sc_2027.wpilog"), "AdvantageKit")) {
      int eSnap = log.startEntry("/FluxFig/EffectiveSnapshot", "string", "", 0L);
      int eActive = log.startEntry("/FluxFig/ActiveProfile", "string", "", 0L);
      int eEnable = log.startEntry("/DriverStation/Enabled", "boolean", "", 0L);
      int eVolt = log.startEntry("/SystemStats/BatteryVoltage", "double", "", 0L);
      int eElevPos = log.startEntry("/Mechanisms/Elevator/PositionRot", "double", "", 0L);
      int eElevAmps = log.startEntry("/Mechanisms/Elevator/LeaderStatorAmps", "double", "", 0L);
      int eArmPos = log.startEntry("/Mechanisms/Arm/PositionRot", "double", "", 0L);
      int eArmAmps = log.startEntry("/Mechanisms/Arm/LeaderStatorAmps", "double", "", 0L);

      log.appendString(eSnap, 0L, snapshot);
      log.appendString(eActive, 0L, "COMP_2027");
      for (int step = 1; step <= 30; step++) {
        double tSec = step * 0.2;
        long tsUs = Math.round(tSec * 1_000_000.0);
        log.appendBoolean(eEnable, tsUs, true);
        log.appendDouble(eVolt, tsUs, 12.3);
        boolean scoring = (tSec >= 2.0 && tSec <= 3.6);
        log.appendDouble(eElevPos, tsUs, scoring ? 41.0 : 0.5);
        log.appendDouble(eElevAmps, tsUs, scoring ? 38.0 : 2.0);
        log.appendDouble(eArmPos, tsUs, scoring ? 0.22 : 0.0);
        log.appendDouble(eArmAmps, tsUs, scoring ? 14.0 : 1.0);
      }
    }

    RunAnalysisReport report =
        RunAnalyzer.analyzeRunDirectory(runDir, RobotProfile.AUTO, false);

    assertTrue(
        "COMP_2027".equals(report.robotName()),
        "Expected COMP_2027 hydrated from /FluxFig/EffectiveSnapshot, got " + report.robotName());
    assertTrue(
        "2027-B04 (Alpha Pack)".equals(report.namedBattery()),
        "Expected full batteryName with spaces from run-meta.json, got " + report.namedBattery());
    assertTrue(
        report.mechanisms().size() >= 2,
        "Expected Elevator and Arm mechanisms hydrated from FluxFig snapshot");
    assertTrue(
        report.elevatorCycles().size() == 1 && "L4".equals(report.elevatorCycles().get(0).targetLevel()),
        "Expected 1 L4 ElevatorCycleEvent");
    assertTrue(
        !report.voiceCorrelations().isEmpty()
            && report
                .voiceCorrelations()
                .get(0)
                .transcript()
                .contains("Elevator L4 extension felt super smooth"),
        "Expected JSON voiceNotes array from run-meta.json to be parsed and correlated");
  }

  public static void testShopStressGateElevatorCyclesAndWpilogOnlyWatchGuard() throws Exception {
    Path runDir = createTempDir("shop-stress-gate-test");
    Path snapDir = runDir.resolve("cam-snapshots");
    Files.createDirectories(snapDir);
    Files.writeString(
        snapDir.resolve("limelightsc0_t2.40s_shot.jpg"), "FAKE_JPEG_BYTES", StandardCharsets.UTF_8);

    try (Writer log = new Writer(runDir.resolve("shop_stress.wpilog"), "ShopStress")) {
      int eEnable = log.startEntry("/DriverStation/Enabled", "boolean", "", 0L);
      int eVolt = log.startEntry("/SystemStats/BatteryVoltage", "double", "", 0L);
      int eBreach = log.startEntry("/VirtualGeofence/WallBreachCount", "double", "", 0L);
      int eDrift = log.startEntry("/ShopStress/Figure8LoopClosureDriftCm", "double", "", 0L);
      int eHandoff = log.startEntry("/ShopStress/MaxCameraHandoffStepCm", "double", "", 0L);
      int eSpread = log.startEntry("/ShopStress/BoxStepCornerSpreadPct", "double", "", 0L);
      int eDriveMs = log.startEntry("/Performance/DriveMs", "double", "", 0L);
      int eCpuTemp = log.startEntry("/SystemStats/CPUTemp", "double", "", 0L);

      for (int step = 1; step <= 25; step++) {
        long tsUs = step * 200_000L;
        log.appendBoolean(eEnable, tsUs, true);
        log.appendDouble(eVolt, tsUs, 12.2);
        log.appendDouble(eBreach, tsUs, 0.0);
        log.appendDouble(eDrift, tsUs, 2.75);
        log.appendDouble(eHandoff, tsUs, 1.85);
        log.appendDouble(eSpread, tsUs, 6.4);
        log.appendDouble(eDriveMs, tsUs, 1.1);
        log.appendDouble(eCpuTemp, tsUs, 48.0);
      }
    }

    RunAnalysisReport report =
        RunAnalyzer.analyzeRunDirectory(runDir, RobotProfile.AUTO, false);

    assertTrue(
        report.shopStressGate() != null && report.shopStressGate().present(),
        "Expected ShopStressGateMetrics to be present");
    assertTrue(
        "PASS".equals(report.shopStressGate().overallGateStatus()),
        "Expected ShopStressGateMetrics PASS, got " + report.shopStressGate().overallGateStatus());
    assertTrue(
        report.cameraSnapshots().size() == 1
            && "SHOT_OR_SCORE".equals(report.cameraSnapshots().get(0).triggerReason()),
        "Expected 1 discovered camera snapshot with trigger SHOT_OR_SCORE");
    assertTrue(
        report.systemHealth() != null
            && report.systemHealth().controllerType().contains("WPILOG-ONLY")
            && Double.isNaN(report.systemHealth().totalCpuPct())
            && Double.isNaN(report.systemHealth().freeStorageMb()),
        "Expected .wpilog-only run without system-snapshot.txt not to fabricate fake SystemCore CPU/storage stats");
  }

  public static void testDigitalTwinCalibratorCsvWriteBack() throws Exception {
    Path cfgDir = createSampleFluxFigDirectory();
    Path runDir = createTempDir("sim-cal-run");

    Files.writeString(
        runDir.resolve("run-meta.json"),
        """
        { "batteryName": "2027-B01", "gitSha": "cal123" }
        """,
        StandardCharsets.UTF_8);

    try (Writer log = new Writer(runDir.resolve("cal.wpilog"), "Cal")) {
      int eSnap = log.startEntry("/FluxFig/EffectiveSnapshot", "string", "", 0L);
      int eEnable = log.startEntry("/DriverStation/Enabled", "boolean", "", 0L);
      int eVolt = log.startEntry("/SystemStats/BatteryVoltage", "double", "", 0L);
      int eSupply = log.startEntry("Phoenix6/TalonFX-2/SupplyCurrent", "double", "", 0L);
      int eRot = log.startEntry("/SimCal/WheelRotationsDelta", "double", "", 0L);
      int eDist = log.startEntry("/SimCal/VisionGroundDistanceMeters", "double", "", 0L);
      int eCoupling = log.startEntry("/SimCal/MeasuredCouplingRatio", "double", "", 0L);
      int eElevVolts = log.startEntry("/Mechanisms/Elevator/LeaderAppliedVolts", "double", "", 0L);
      int eElevVel = log.startEntry("/Mechanisms/Elevator/VelocityRps", "double", "", 0L);
      int eElevPos = log.startEntry("/Mechanisms/Elevator/PositionRot", "double", "", 0L);

      log.appendString(eSnap, 0L, FluxFig.load(cfgDir, "COMP_2027").getEffectiveSnapshot());
      log.appendBoolean(eEnable, 0L, false);
      log.appendDouble(eVolt, 0L, 12.70);
      log.appendDouble(eSupply, 0L, 1.0);

      for (int step = 1; step <= 40; step++) {
        double tSec = step * 0.2;
        long tsUs = Math.round(tSec * 1_000_000.0);
        log.appendBoolean(eEnable, tsUs, true);
        double iPerWheel = 20.0 + (step % 10) * 3.0;
        double iTotal = iPerWheel * 4.0;
        // True R_int = 19.0 mOhm (0.019 Ohm)
        log.appendDouble(eVolt, tsUs, 12.70 - iTotal * 0.019);
        log.appendDouble(eSupply, tsUs, iPerWheel);

        // True Elevator kG = 0.35V, kS = 0.18V, kV = 0.12 V/RPS, kA = 0.01 V/RPS^2
        double vel = (step <= 5) ? 0.0 : (10.0 + step * 0.8);
        double accel = (step <= 5) ? 0.0 : 4.0;
        double appliedV = (step <= 5) ? 0.35 : (0.35 + 0.18 + 0.12 * vel + 0.01 * accel);
        log.appendDouble(eElevVel, tsUs, vel);
        log.appendDouble(eElevVolts, tsUs, appliedV);
        log.appendDouble(eElevPos, tsUs, step * 0.5);
      }
      // 10.0 rotations -> 3.129 m => wheel radius = 0.04980 m
      log.appendDouble(eRot, 8_000_000L, 10.0);
      log.appendDouble(eDist, 8_000_000L, 2.0 * Math.PI * 10.0 * 0.04980);
      log.appendDouble(eCoupling, 8_000_000L, 3.4500);
    }

    RunAnalysisReport report =
        RunAnalyzer.analyzeRunDirectory(runDir, RobotProfile.AUTO, false);
    DigitalTwinCalibrator.SimCalibrationSummary summary =
        DigitalTwinCalibrator.calibrateAndWrite(
            List.of(report), List.of(runDir), cfgDir, runDir, "COMP_2027");

    assertTrue(
        Math.abs(summary.calibratedWheelRadiusMeters() - 0.04980) < 1e-4,
        "Expected calibrated wheelRadiusMeters ~0.04980, got "
            + summary.calibratedWheelRadiusMeters());
    assertTrue(
        Math.abs(summary.calibratedCouplingRatio() - 3.4500) < 1e-3,
        "Expected calibrated swerveCouplingRatio ~3.4500");
    assertTrue(
        Files.exists(summary.summaryJsonPath()), "Expected sim-calibration.json to be written");

    // Reload FluxFig from cfgDir and verify CSVs were updated and pass strict validation
    FluxFig reloaded = FluxFig.load(cfgDir, "COMP_2027");
    assertTrue(
        Math.abs(reloaded.getDouble("wheelRadiusMeters") - 0.04980) < 1e-4,
        "Expected parameters.csv wheelRadiusMeters updated to 0.04980");
    List<FluxFig.BatterySpec> batts = reloaded.bindTable("batteries", FluxFig.BatterySpec.class);
    assertTrue(
        !batts.isEmpty() && Math.abs(batts.get(0).nominalRintMilliOhms() - 19.0) < 2.0,
        "Expected batteries.csv 2027-B01 nominalRintMilliOhms updated near 19.0 mOhm");
  }

  public static void testMcpServerMultimodalTools() throws Exception {
    Path wsDir = createTempDir("flux-mcp-test");
    Path runA = wsDir.resolve("run_A");
    Path runB = wsDir.resolve("run_B");
    Files.createDirectories(runA.resolve("cam-snapshots"));
    Files.createDirectories(runB);

    Files.writeString(
        runA.resolve("cam-snapshots").resolve("limelightsc1_t3.50s_jump.jpg"),
        "FAKE_JPG",
        StandardCharsets.UTF_8);
    Files.writeString(
        runA.resolve("run-meta.json"),
        """
        {
          "batteryName": "Thor",
          "videoUrl": "https://youtu.be/abc123xyz",
          "videoOffsetSec": 5.0,
          "voiceNotes": [
            { "role": "DRIVER", "timeSec": 4.0, "text": "Pose jumped near center truss" }
          ]
        }
        """,
        StandardCharsets.UTF_8);

    for (Path dir : List.of(runA, runB)) {
      try (Writer w = new Writer(dir.resolve("match.wpilog"), "MCP")) {
        int eEnable = w.startEntry("/DriverStation/Enabled", "boolean", "", 0L);
        int eVolt = w.startEntry("/SystemStats/BatteryVoltage", "double", "", 0L);
        for (int i = 1; i <= 25; i++) {
          long ts = i * 200_000L;
          w.appendBoolean(eEnable, ts, true);
          double v = (dir.equals(runA) && i == 15) ? 6.60 : 12.1;
          w.appendDouble(eVolt, ts, v);
        }
      }
    }

    FluxMcpServer mcp = new FluxMcpServer(wsDir, RobotProfile.AUTO);

    String initResp = mcp.handleRequest("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"initialize\"}");
    assertTrue(
        initResp.contains("\"flux-processor-mcp\""), "Expected MCP serverInfo in initialize");

    String listResp = mcp.handleRequest("{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"tools/list\"}");
    for (String tool :
        List.of(
            "list_entries",
            "find_condition",
            "time_correlate",
            "compare_runs",
            "moi_regression",
            "search_voice_notes",
            "get_camera_snapshots",
            "get_video_frames")) {
      assertTrue(listResp.contains("\"" + tool + "\""), "Expected tool " + tool + " in tools/list");
    }

    String findResp =
        mcp.handleRequest(
            """
            {"jsonrpc":"2.0","id":3,"method":"tools/call","params":{"name":"find_condition","arguments":{"runId":"run_A","topic":"/SystemStats/BatteryVoltage","operator":"<","threshold":7.0}}}""");
    assertTrue(
        findResp.contains("\\\"matchingSamples\\\":1") && findResp.contains("\"isError\":false"),
        "Expected 1 sample < 7.0V in find_condition, got: " + findResp);

    String voiceResp =
        mcp.handleRequest(
            """
            {"jsonrpc":"2.0","id":4,"method":"tools/call","params":{"name":"search_voice_notes","arguments":{"query":"truss"}}}""");
    assertTrue(
        voiceResp.contains("Pose jumped near center truss"),
        "Expected voice note match in search_voice_notes");

    String snapResp =
        mcp.handleRequest(
            """
            {"jsonrpc":"2.0","id":5,"method":"tools/call","params":{"name":"get_camera_snapshots","arguments":{"runId":"run_A"}}}""");
    assertTrue(
        snapResp.contains("limelightsc1_t3.50s_jump.jpg") && snapResp.contains("POSE_JUMP"),
        "Expected camera snapshot in get_camera_snapshots");

    String cmpResp =
        mcp.handleRequest(
            """
            {"jsonrpc":"2.0","id":6,"method":"tools/call","params":{"name":"compare_runs","arguments":{"runA":"run_A","runB":"run_B"}}}""");
    assertTrue(
        cmpResp.contains("deltaMinVoltage"), "Expected deltaMinVoltage in compare_runs");

    String vidResp =
        mcp.handleRequest(
            """
            {"jsonrpc":"2.0","id":7,"method":"tools/call","params":{"name":"get_video_frames","arguments":{"runId":"run_A","timestampSec":4.0}}}""");
    assertTrue(
        vidResp.contains("youtu.be/abc123xyz?t="), "Expected YouTube jump link in get_video_frames");
  }

  public static void testStudentAutoVideoRendererCli() throws Exception {
    Path autoDir = createTempDir("flux-auto-render-test");
    Path jsonFile = autoDir.resolve("student-auto-frames.json");

    List<AutoSimVideoRenderer.AutoFrame> frames =
        List.of(
            new AutoSimVideoRenderer.AutoFrame(
                0.00, 1.90, 4.10, 0.02, 0.0, 0.0, 0.0, 3.2, 0.0, 0.0, 12.6, 5.0, 45.0, "SHOOTING"),
            new AutoSimVideoRenderer.AutoFrame(
                0.04, 1.95, 4.05, 0.05, 1.2, -1.2, 0.4, 3.2, 0.0, 30.0, 12.4, 35.0, 46.0, "INTAKING"),
            new AutoSimVideoRenderer.AutoFrame(
                0.08, 2.05, 3.95, 0.10, 1.5, -1.5, 0.5, 3.5, 0.1, 30.0, 12.3, 42.0, 47.0, "SHOOTING"));
    List<AutoSimVideoRenderer.ShotArc> shots =
        List.of(
            new AutoSimVideoRenderer.ShotArc(
                1, 0.00, 0.65, 1.90, 4.10, 0.52, 4.63, 4.21, 1.83, 2.45, true));
    List<AutoSimVideoRenderer.FloorFuelBall> balls =
        List.of(new AutoSimVideoRenderer.FloorFuelBall(2.90, 2.60, 2.20));

    AutoSimVideoRenderer.writeFramesJson(
        jsonFile,
        "STUDENT: 2026 Square, Circle & Hub Shooting",
        "FUEL",
        frames,
        shots,
        balls,
        true,
        "Start pose error 1.25 m exceeds limit 0.50 m");

    FluxProcessorMain.main(
        new String[] {
          "--render-auto", jsonFile.toString(), "--out", autoDir.toString()
        });

    Path htmlPath = autoDir.resolve("student-auto.html");
    assertTrue(
        Files.exists(htmlPath) && Files.size(htmlPath) > 3000,
        "Expected student-auto.html to be rendered via FluxProcessor --render-auto");
    String htmlText = Files.readString(htmlPath, StandardCharsets.UTF_8);
    assertTrue(
        htmlText.contains("<link rel=\"icon\"")
            && htmlText.contains("FLOOR_BALLS")
            && htmlText.contains("2026 BLUE HUB (Z=1.83m)")
            && htmlText.contains("BLUE DEPOT")
            && htmlText.contains("HUB SCORES:")
            && htmlText.contains("Start pose error 1.25 m"),
        "Expected student-auto.html to include favicon, 3D Blue Hub, Blue Depot, floor balls, Hub score HUD, and SafeAuto warning banner");

    Path starterDir = createTempDir("flux-auto-starter-test");
    Path starterJson = starterDir.resolve("student-auto-frames.json");
    List<AutoSimVideoRenderer.AutoFrame> starterFrames =
        List.of(
            new AutoSimVideoRenderer.AutoFrame(
                0.00, 3.10, 4.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 12.6, 0.0, 0.0, "IDLE"),
            new AutoSimVideoRenderer.AutoFrame(
                15.00, 3.10, 4.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 12.6, 0.0, 0.0, "IDLE"));
    AutoSimVideoRenderer.writeFramesJson(
        starterJson,
        "STUDENT: Default 2026 Autonomous",
        "FUEL",
        starterFrames,
        List.of(),
        balls,
        true);
    FluxProcessorMain.main(
        new String[] {"--render-auto", starterJson.toString(), "--out", starterDir.toString()});
    String starterHtml =
        Files.readString(starterDir.resolve("student-auto.html"), StandardCharsets.UTF_8);
    assertTrue(
        starterHtml.contains("STUDENT NOTE: Empty starter auto"),
        "Expected unedited empty STUDENT routine to display STUDENT NOTE starter callout");
  }

  public static void testBinaryAudioEmptyBatteryNoDataAndRecursiveHootDiscovery() throws Exception {
    RunAnalysisReport.BatteryMetrics emptyBatt = BatteryAnalyzer.analyze("2026-B01", List.of(), 0);
    assertTrue(
        "NO DATA".equals(emptyBatt.healthGrade()),
        "Empty battery series must report NO DATA, got " + emptyBatt.healthGrade());
    assertTrue(
        Double.isNaN(emptyBatt.estimatedRintMilliOhms()),
        "Empty battery series must not fabricate 16.0 mOhm R_int");

    Path runDir = createTempDir("flux-binary-audio-hoot");
    Files.write(
        runDir.resolve("driver-audio.ogg"),
        new byte[] {0x4F, 0x67, 0x67, 0x53, 0x00, (byte) 0xFF, (byte) 0xFE, (byte) 0x80, 0x01});
    Files.write(
        runDir.resolve("mentor-audio.wav"),
        new byte[] {'R', 'I', 'F', 'F', 36, 0, 0, 0, 'W', 'A', 'V', 'E', (byte) 0xFF, (byte) 0x80});
    Files.writeString(
        runDir.resolve("audio-sync.json"),
        "{\"audioFile\":\"mentor-audio.wav\",\"enableEpochMs\":1700000000000,\"disableEpochMs\":1700000015000}\n",
        StandardCharsets.UTF_8);

    try (Writer w = new Writer(runDir.resolve("fuel_run.wpilog"), "FUEL")) {
      int eEnable = w.startEntry("/DriverStation/Enabled", "boolean", "", 0L);
      int eVolt = w.startEntry("/SystemStats/BatteryVoltage", "double", "", 0L);
      for (int i = 1; i <= 15; i++) {
        long ts = i * 200_000L;
        w.appendBoolean(eEnable, ts, true);
        w.appendDouble(eVolt, ts, 12.4);
      }
    }

    RunAnalysisReport report =
        RunAnalyzer.analyzeRunDirectory(runDir, RobotProfile.AUTO, false);
    assertTrue(
        "NO DATA".equals(report.battery().healthGrade())
            && Double.isNaN(report.battery().estimatedRintMilliOhms()),
        "Unloaded run without current draw must grade battery NO DATA, got "
            + report.battery().healthGrade());
    HtmlReportGenerator.writeRunReport(report, runDir.resolve("report.html"));
    String html = Files.readString(runDir.resolve("report.html"), StandardCharsets.UTF_8);
    assertTrue(
        html.contains("NO DATA") && !html.contains("NaN mΩ"),
        "HTML report must render NO DATA / N/A rather than NaN mΩ");

    // Verify recursive Files.walk discovery of nested .hoot files.
    Path nestedDir = runDir.resolve("usb_drive/Drivetrain");
    Files.createDirectories(nestedDir);
    Files.writeString(nestedDir.resolve("rio_2026-11-20.hoot"), "FAKE_HOOT", StandardCharsets.UTF_8);
    Path mockOwlet = runDir.resolve("mock-owlet.sh");
    Files.writeString(
        mockOwlet,
        "#!/usr/bin/env bash\n"
            + "if [ \"$2\" = \"-S\" ]; then echo 'RobotEnable : 1'; exit 0; fi\n"
            + "OUT=\"$2\"\n"
            + "python3 -c 'import pathlib,sys; pathlib.Path(sys.argv[1]).write_bytes(b\"WPILOG\" + b\"\\x00\"*1200)' \"$OUT\"\n",
        StandardCharsets.UTF_8);
    mockOwlet.toFile().setExecutable(true);
    Path convOut = createTempDir("flux-hoot-out");
    List<Path> converted = HootConverter.convertDirectoryIfNeeded(runDir, convOut, mockOwlet);
    assertTrue(
        converted.size() == 1
            && converted
                .get(0)
                .getFileName()
                .toString()
                .equals("usb_drive_Drivetrain_rio_2026-11-20.wpilog"),
        "Expected recursive HootConverter to convert nested .hoot file, got " + converted);

    // End-to-end ZIP ingestion check matching FluxTray M1.8/M1.9 output layout.
    Path zipBundle = createTempDir("flux-zip-ingest").resolve("match-logs_2026-11-20_180000_FUEL.zip");
    try (java.util.zip.ZipOutputStream zos =
        new java.util.zip.ZipOutputStream(Files.newOutputStream(zipBundle))) {
      for (String fname :
          List.of("fuel_run.wpilog", "mentor-audio.wav", "audio-sync.json")) {
        zos.putNextEntry(new java.util.zip.ZipEntry("run_ct_01/" + fname));
        Files.copy(runDir.resolve(fname), zos);
        zos.closeEntry();
      }
      zos.putNextEntry(new java.util.zip.ZipEntry("run_ct_01/system-snapshot.txt"));
      zos.write(
          "[PLATFORM]\ncontrollerType=ROBORIO\nhostname=roboRIO-10413-FRC\n[CPU_AND_IRQ]\ntotalCpuPct=22.0\n[STORAGE]\nfreeStorageMb=512.0\n"
              .getBytes(StandardCharsets.UTF_8));
      zos.closeEntry();
    }
    Path zipOut = createTempDir("flux-zip-out");
    FluxProcessorMain.main(
        new String[] {zipBundle.toString(), "--out", zipOut.toString(), "--no-merge"});
    Path zipReport = zipOut.resolve("run_ct_01").resolve("report.html");
    assertTrue(Files.isRegularFile(zipReport), "FluxProcessor CLI ingested FluxTray ZIP cleanly");
    String zipHtml = Files.readString(zipReport, StandardCharsets.UTF_8);
    assertTrue(
        zipHtml.contains("ROBORIO") && zipHtml.contains("NOT MEASURED"),
        "FluxProcessor report preserves truthful ROBORIO snapshot with NOT MEASURED absent fields");
  }

  public static void testRoboRioTotalLoopAndTraceTopicAlignment() throws Exception {
    Path runDir = createTempDir("flux-roborio-timing");
    Files.writeString(
        runDir.resolve("system-snapshot.txt"),
        """
        [PLATFORM]
        controllerType=ROBORIO
        hostname=roboRIO-10413-FRC
        [CPU_AND_IRQ]
        totalCpuPct=44.0
        javaProcessCpuPct=32.0
        [STORAGE]
        internalFreeStorageMb=130.0
        usbFreeStorageMb=12000.0
        freeStorageMb=130.0
        """,
        StandardCharsets.UTF_8);

    try (Writer w = new Writer(runDir.resolve("roborio_fuel.wpilog"), "AdvantageKit")) {
      int eEnable = w.startEntry("/DriverStation/Enabled", "boolean", "", 0L);
      int eVolt = w.startEntry("/SystemStats/BatteryVoltage", "double", "", 0L);
      int eTotal = w.startEntry("/RealOutputs/Performance/TotalLoopMs", "double", "", 0L);
      int ePhoenix = w.startEntry("/RealOutputs/Performance/PhoenixSignalsMs", "double", "", 0L);
      int eSched = w.startEntry("/RealOutputs/Performance/CommandSchedulerMs", "double", "", 0L);
      int eContainer = w.startEntry("/RealOutputs/Performance/RobotContainerMs", "double", "", 0L);

      for (int i = 1; i <= 50; i++) {
        long tsUs = i * 20_000L;
        w.appendBoolean(eEnable, tsUs, true);
        w.appendDouble(eVolt, tsUs, 12.2);
        double totalMs = (i == 49 || i == 50) ? 26.5 : 8.0;
        w.appendDouble(eTotal, tsUs, totalMs);
        w.appendDouble(ePhoenix, tsUs, (i == 49) ? 9.5 : 1.4);
        w.appendDouble(eSched, tsUs, 3.2);
        w.appendDouble(eContainer, tsUs, 0.9);
      }
    }

    RunAnalysisReport report = RunAnalyzer.analyzeRunDirectory(runDir, RobotProfile.AUTO, false);
    assertTrue(report.systemHealth() != null, "Expected roboRIO systemHealth populated");
    assertTrue(
        Math.abs(report.systemHealth().loopMedianMs() - 8.0) < 0.2,
        "Expected loopMedianMs ~8.0ms from Performance/TotalLoopMs, got "
            + report.systemHealth().loopMedianMs());
    assertTrue(
        report.systemHealth().loopP99Ms() > 20.0 && report.systemHealth().loopMaxMs() >= 26.5,
        "Expected Performance/TotalLoopMs overrun in p99/max: p99="
            + report.systemHealth().loopP99Ms()
            + ", max="
            + report.systemHealth().loopMaxMs());
    assertTrue(
        report.systemHealth().watchDecisions().stream()
            .anyMatch(d -> d.metricName().contains("Loop Period Jitter")),
        "Expected roboRIO Loop Period Jitter alert: " + report.systemHealth().watchDecisions());
    assertTrue(
        report.systemHealth().watchDecisions().stream()
            .anyMatch(d -> d.metricName().contains("PhoenixSignalsMs")),
        "Expected roboRIO PhoenixSignalsMs alert: " + report.systemHealth().watchDecisions());
    assertTrue(
        report.systemHealth().watchDecisions().stream()
            .anyMatch(d -> d.category().contains("Storage") && "CRITICAL".equals(d.severity())),
        "Expected CRITICAL /home/lvuser low storage alert: "
            + report.systemHealth().watchDecisions());
  }

  public static void testTempDirZipCleanupAndCliParsing() throws Exception {
    Path zipParent = createTempDir("flux-drive-sync-parent");
    Path zipFile = zipParent.resolve("practice_2027_run.zip");
    Path scratchLog = createTempDir("flux-zip-src").resolve("akit_2027.wpilog");
    try (Writer w = new Writer(scratchLog, "AdvantageKit")) {
      int eEnable = w.startEntry("/AdvantageKit/DriverStation/Enabled", "boolean", "", 0L);
      int eVolt = w.startEntry("/AdvantageKit/PowerDistribution/Voltage", "double", "", 0L);
      for (int i = 1; i <= 15; i++) {
        long ts = i * 200_000L;
        w.appendBoolean(eEnable, ts, true);
        w.appendDouble(eVolt, ts, 12.2);
      }
    }
    try (java.util.zip.ZipOutputStream zos =
        new java.util.zip.ZipOutputStream(Files.newOutputStream(zipFile))) {
      zos.putNextEntry(new java.util.zip.ZipEntry("akit_2027.wpilog"));
      Files.copy(scratchLog, zos);
      zos.closeEntry();
    }

    Path explicitOut = createTempDir("flux-zip-explicit-out");
    int codeWithOut =
        FluxProcessorMain.runCli(
            new String[] {
              zipFile.toString(), "--out", explicitOut.toString(), "--robot", "COMP_2027", "--no-merge"
            });
    assertTrue(codeWithOut == 0, "Expected exit code 0 for ZIP with --out");
    assertTrue(
        Files.isRegularFile(explicitOut.resolve("practice_2027_run").resolve("report.html")),
        "Expected report.html written under explicit --out/<zipBaseName>");

    List<Path> parentAfterWithOut;
    try (Stream<Path> s = Files.list(zipParent)) {
      parentAfterWithOut = s.toList();
    }
    assertTrue(
        parentAfterWithOut.size() == 1 && parentAfterWithOut.get(0).equals(zipFile),
        "ZIP parent folder must remain untouched (no *_extracted dir), found: " + parentAfterWithOut);

    int codeNoOut =
        FluxProcessorMain.runCli(
            new String[] {zipFile.toString(), "--robot", "CHASSIS_3", "--no-merge"});
    assertTrue(codeNoOut == 0, "Expected exit code 0 for ZIP without --out");

    List<Path> parentAfterNoOut;
    try (Stream<Path> s = Files.list(zipParent)) {
      parentAfterNoOut = s.toList();
    }
    assertTrue(
        parentAfterNoOut.size() == 1 && parentAfterNoOut.get(0).equals(zipFile),
        "ZIP without --out must not pollute ZIP parent folder, found: " + parentAfterNoOut);

    assertTrue(
        RobotProfile.fromPresetName("COMP_2027").corners().size() == 4
            && RobotProfile.fromPresetName("PRACTICE_2027").corners().size() == 4
            && RobotProfile.fromPresetName("CHASSIS_3").corners().size() == 4,
        "Built-in 2027 presets must resolve 4 swerve corners without falling into an empty profile");

    Path calRunDir = createTempDir("flux-cli-calsim-run");
    Files.copy(scratchLog, calRunDir.resolve("akit_2027.wpilog"));
    Path calOut = createTempDir("flux-cli-calsim-out");
    int calCode =
        FluxProcessorMain.runCli(
            new String[] {
              "--calibrate-sim", calRunDir.toString(), "--out", calOut.toString(), "--no-merge"
            });
    assertTrue(calCode == 0, "Expected --calibrate-sim <run-dir> CLI invocation to succeed");
    assertTrue(
        Files.isRegularFile(calOut.resolve("sim-calibration.json")),
        "Expected sim-calibration.json from --calibrate-sim <run-dir>");
  }

  public static void testReal2027AdvantageKitTopicsAndSwerveModulesCsv() throws Exception {
    Path cfgDir = createSampleFluxFigDirectoryWithSwerveModules();
    RobotProfile fromDir = RobotProfile.fromFluxFigDirectory(cfgDir, "COMP_2027");
    assertTrue(
        fromDir.corners().get("FL").driveMotorId() == 11
            && fromDir.corners().get("FL").steerMotorId() == 12
            && fromDir.corners().get("FL").cancoderId() == 31
            && fromDir.corners().get("BR").driveMotorId() == 17,
        "Expected swerve_modules.csv CAN IDs loaded in RobotProfile.fromFluxFigDirectory, got "
            + fromDir.corners());

    String snapshot = FluxFig.load(cfgDir, "COMP_2027").getEffectiveSnapshot();
    Path runDir = createTempDir("flux-2027-akit-topics");
    try (Writer w = new Writer(runDir.resolve("akit_2027_real.wpilog"), "AdvantageKit")) {
      int eEnable = w.startEntry("/AdvantageKit/DriverStation/Enabled", "boolean", "", 0L);
      int eSnap = w.startEntry("/AdvantageKit/RealOutputs/FluxFig/EffectiveSnapshot", "string", "", 0L);
      int eCfgSha = w.startEntry("/RealMetadata/ConfigSHA256", "string", "", 0L);
      int ePdhVolt = w.startEntry("/AdvantageKit/PowerDistribution/Voltage", "double", "", 0L);
      int ePdhAmps = w.startEntry("/AdvantageKit/PowerDistribution/TotalCurrent", "double", "", 0L);
      int eBattVolt = w.startEntry("/AdvantageKit/Battery/VoltageVolts", "double", "", 0L);
      int eBattAmps = w.startEntry("/AdvantageKit/Battery/CurrentAmps", "double", "", 0L);
      int eLoopMs = w.startEntry("/AdvantageKit/RealOutputs/Performance/TotalLoopMs", "double", "", 0L);
      int eDriveSupplyTotal =
          w.startEntry("/AdvantageKit/RealOutputs/Drive/TotalSupplyCurrentAmps", "double", "", 0L);
      int ePose = w.startEntry("/AdvantageKit/RealOutputs/Drive/Pose", "struct:Pose2d", "", 0L);
      int eSpeeds =
          w.startEntry("/AdvantageKit/RealOutputs/Drive/ChassisSpeeds", "struct:ChassisSpeeds", "", 0L);

      int[] eModStator = new int[4];
      int[] eModSupply = new int[4];
      int[] eModVel = new int[4];
      int[] eModPos = new int[4];
      int[] eTurnStator = new int[4];
      for (int m = 0; m < 4; m++) {
        eModStator[m] =
            w.startEntry("/AdvantageKit/Drive/Module" + m + "/DriveStatorCurrentAmps", "double", "", 0L);
        eModSupply[m] =
            w.startEntry("/AdvantageKit/Drive/Module" + m + "/DriveSupplyCurrentAmps", "double", "", 0L);
        eModVel[m] =
            w.startEntry("/AdvantageKit/Drive/Module" + m + "/DriveVelocityRadPerSec", "double", "", 0L);
        eModPos[m] =
            w.startEntry("/AdvantageKit/Drive/Module" + m + "/DrivePositionRad", "double", "", 0L);
        eTurnStator[m] =
            w.startEntry("/AdvantageKit/Drive/Module" + m + "/TurnStatorCurrentAmps", "double", "", 0L);
      }

      int eElevPos = w.startEntry("/AdvantageKit/Elevator/PositionRotations", "double", "", 0L);
      int eElevVel = w.startEntry("/AdvantageKit/Elevator/VelocityRPS", "double", "", 0L);
      int eElevVolt = w.startEntry("/AdvantageKit/Elevator/AppliedVoltage", "double", "", 0L);
      int eElevStator = w.startEntry("/AdvantageKit/Elevator/StatorCurrentAmps", "double", "", 0L);
      int eElevSupply = w.startEntry("/AdvantageKit/Elevator/SupplyCurrentAmps", "double", "", 0L);

      w.appendString(eSnap, 0L, snapshot);
      w.appendString(eCfgSha, 0L, "sha2027deadbeef");
      w.appendBoolean(eEnable, 0L, false);
      w.appendDouble(ePdhVolt, 0L, 12.70);
      w.appendDouble(eBattVolt, 0L, 12.70);

      for (int i = 1; i <= 35; i++) {
        long ts = i * 200_000L;
        w.appendBoolean(eEnable, ts, true);
        double iPerWheel = 10.0 + (i % 8) * 3.5;
        double iTotal = iPerWheel * 4.0 + 12.0;
        double vBatt = 12.70 - iTotal * 0.017;
        w.appendDouble(ePdhVolt, ts, vBatt);
        w.appendDouble(eBattVolt, ts, vBatt);
        w.appendDouble(ePdhAmps, ts, iTotal);
        w.appendDouble(eBattAmps, ts, iTotal);
        w.appendDouble(eDriveSupplyTotal, ts, iPerWheel * 4.0);
        w.appendDouble(eLoopMs, ts, 9.5);
        w.appendPose2d(ePose, ts, 2.0 + i * 0.1, 4.0, 0.0);
        w.appendChassisSpeeds(eSpeeds, ts, 3.0, 0.0, 0.0);
        for (int m = 0; m < 4; m++) {
          w.appendDouble(eModStator[m], ts, 42.0);
          w.appendDouble(eModSupply[m], ts, iPerWheel);
          w.appendDouble(eModVel[m], ts, 60.0); // 60 rad/s * 0.0508 m = 3.048 m/s
          w.appendDouble(eModPos[m], ts, i * 12.0);
          w.appendDouble(eTurnStator[m], ts, 8.0);
        }
        w.appendDouble(eElevPos, ts, (i >= 10 && i <= 18) ? 28.0 : 0.5);
        w.appendDouble(eElevVel, ts, 12.0);
        w.appendDouble(eElevVolt, ts, 4.2);
        w.appendDouble(eElevStator, ts, 25.0);
        w.appendDouble(eElevSupply, ts, 12.0);
      }
    }

    RunAnalysisReport report = RunAnalyzer.analyzeRunDirectory(runDir, RobotProfile.AUTO, false);
    assertTrue("COMP_2027".equals(report.robotName()), "Expected COMP_2027 hydrated from snapshot");
    assertTrue(
        "sha2027deadbeef".equals(report.configSha()),
        "Expected ConfigSHA256 from /RealMetadata/ConfigSHA256, got " + report.configSha());
    assertTrue(
        report.swerveCorners().size() == 4 && report.swerveCorners().get(0).driveId() == 11,
        "Expected swerve_modules.csv FL driveId=11 hydrated from EffectiveSnapshot");
    assertTrue(
        Math.abs(report.peakSpeedMps() - (60.0 * 0.0508)) < 0.1,
        "Expected peakSpeedMps ~3.048 m/s from DriveVelocityRadPerSec, got " + report.peakSpeedMps());
    assertTrue(
        Double.isFinite(report.battery().estimatedRintMilliOhms())
            && Math.abs(report.battery().estimatedRintMilliOhms() - 17.0) < 2.0,
        "Expected Battery R_int ~17.0 mOhm from 2027 PDH/Battery topics, got "
            + report.battery().estimatedRintMilliOhms());
    assertTrue(
        report.systemHealth() != null && Math.abs(report.systemHealth().loopMedianMs() - 9.5) < 0.2,
        "Expected loopMedianMs ~9.5 ms from /AdvantageKit/RealOutputs/Performance/TotalLoopMs");
    assertTrue(
        report.mechanisms().stream()
            .anyMatch(m -> "Elevator".equals(m.name()) && Math.abs(m.peakStatorAmps() - 25.0) < 0.5),
        "Expected Elevator mechanism metrics from /AdvantageKit/Elevator/* topics");
  }

  public static void testCandidatePracticeTuningDiffAndPromoteGate() throws Exception {
    Path cfgDir = createSampleFluxFigDirectoryWithSwerveModules();
    FluxFig tuningFig = FluxFig.load(cfgDir, "COMP_2027");
    tuningFig.setMode(FluxFig.Mode.TUNING);
    tuningFig.setOverride("COMP_2027", "maxSpeedMps", "4.20");
    tuningFig.setTableCellOverride("mechanisms", "COMP_2027", "Elevator", "kP", "3.25");
    String cleanSnapshot = tuningFig.getEffectiveSnapshot();

    Path cleanRunDir = createTempDir("flux-tuning-ready");
    Files.writeString(cleanRunDir.resolve("tuned-snapshot.txt"), cleanSnapshot, StandardCharsets.UTF_8);
    try (Writer w = new Writer(cleanRunDir.resolve("practice.wpilog"), "AdvantageKit")) {
      int eEnable = w.startEntry("/DriverStation/Enabled", "boolean", "", 0L);
      int eVolt = w.startEntry("/PowerDistribution/Voltage", "double", "", 0L);
      int eLoop = w.startEntry("/RealOutputs/Performance/TotalLoopMs", "double", "", 0L);
      int[] eStator = new int[4];
      for (int m = 0; m < 4; m++) {
        eStator[m] = w.startEntry("/Drive/Module" + m + "/DriveStatorCurrentAmps", "double", "", 0L);
      }
      for (int i = 1; i <= 25; i++) {
        long ts = i * 200_000L;
        w.appendBoolean(eEnable, ts, true);
        w.appendDouble(eVolt, ts, 12.1);
        w.appendDouble(eLoop, ts, 10.0);
        for (int m = 0; m < 4; m++) {
          w.appendDouble(eStator[m], ts, 52.0);
        }
      }
    }

    Path cleanOut = createTempDir("flux-tuning-ready-out");
    RunAnalysisReport cleanReport =
        RunAnalyzer.analyzeRunDirectory(
            cleanRunDir, cleanOut, cleanRunDir, cfgDir, RobotProfile.COMP_2027, false);
    assertTrue(
        cleanReport.candidateTuning() != null && cleanReport.candidateTuning().present(),
        "Expected CandidateTuningReview when tuned-snapshot.txt is present");
    assertTrue(
        "READY_TO_PROMOTE".equals(cleanReport.candidateTuning().verdict()),
        "Expected READY_TO_PROMOTE for clean practice tuning run, got "
            + cleanReport.candidateTuning().verdict()
            + " reasons="
            + cleanReport.candidateTuning().verdictReasons());
    assertTrue(
        cleanReport.candidateTuning().entries().size() == 2,
        "Expected 2 diff entries (parameter + mechanism table cell), got "
            + cleanReport.candidateTuning().entries());
    assertTrue(
        cleanReport.candidateTuning().promoteCommand().contains("fluxfig.jar promote")
            && cleanReport.candidateTuning().promoteCommand().contains("tuned-snapshot.txt"),
        "Expected explicit fluxfig.jar promote CLI command, got "
            + cleanReport.candidateTuning().promoteCommand());

    Path cleanHtml = cleanOut.resolve("report.html");
    HtmlReportGenerator.writeRunReport(cleanReport, cleanHtml);
    String htmlText = Files.readString(cleanHtml, StandardCharsets.UTF_8);
    assertTrue(
        htmlText.contains("Candidate Practice Tuning Diff (Unmerged)")
            && htmlText.contains("READY_TO_PROMOTE")
            && htmlText.contains("maxSpeedMps")
            && htmlText.contains("mechanisms[COMP_2027:Elevator].kP")
            && htmlText.contains("fluxfig.jar promote"),
        "Expected report.html to render Candidate Practice Tuning Diff panel and promote CLI");

    // Verify the copied snapshot promotes cleanly via FluxFig
    Path promotedCfgCopy = createSampleFluxFigDirectoryWithSwerveModules();
    String persistedSnapshot =
        Files.readString(
            Path.of(cleanReport.candidateTuning().sourceFilePath()), StandardCharsets.UTF_8);
    FluxFig.promoteSnapshotToDirectory(
        persistedSnapshot, promotedCfgCopy, "mentor-test", "M2.3 promote test");
    FluxFig promotedFig = FluxFig.load(promotedCfgCopy, "COMP_2027");
    assertTrue(
        Math.abs(promotedFig.getDouble("maxSpeedMps") - 4.20) < 1e-6,
        "Expected FluxFig promotion to apply tuned maxSpeedMps");

    // Blocked run: structural key modified + peak drive stator > 60A + brownout
    Path blockedRunDir = createTempDir("flux-tuning-blocked");
    Files.writeString(
        blockedRunDir.resolve("tuned-snapshot.txt"),
        """
        # FluxFig Effective Snapshot
        # ActiveProfile: COMP_2027
        # RuntimeOverrides:
        #   COMP_2027.driveStatorLimitAmps: 60.0 -> 90.0
        #   COMP_2027.wheelRadiusMeters: 0.05080 -> 0.05010
        """,
        StandardCharsets.UTF_8);
    try (Writer w = new Writer(blockedRunDir.resolve("bad_practice.wpilog"), "AdvantageKit")) {
      int eEnable = w.startEntry("/DriverStation/Enabled", "boolean", "", 0L);
      int eVolt = w.startEntry("/PowerDistribution/Voltage", "double", "", 0L);
      int[] eStator = new int[4];
      for (int m = 0; m < 4; m++) {
        eStator[m] = w.startEntry("/Drive/Module" + m + "/DriveStatorCurrentAmps", "double", "", 0L);
      }
      for (int i = 1; i <= 20; i++) {
        long ts = i * 200_000L;
        w.appendBoolean(eEnable, ts, true);
        w.appendDouble(eVolt, ts, i == 10 ? 6.40 : 12.0);
        for (int m = 0; m < 4; m++) {
          w.appendDouble(eStator[m], ts, i == 10 ? 74.0 : 40.0);
        }
      }
    }

    RunAnalysisReport blockedReport =
        RunAnalyzer.analyzeRunDirectory(
            blockedRunDir, blockedRunDir, blockedRunDir, cfgDir, RobotProfile.COMP_2027, false);
    assertTrue(
        blockedReport.candidateTuning() != null
            && "DO_NOT_PROMOTE".equals(blockedReport.candidateTuning().verdict()),
        "Expected DO_NOT_PROMOTE on structural key + brownout + >60A drive stator, got "
            + blockedReport.candidateTuning());
    assertTrue(
        blockedReport.candidateTuning().entries().stream()
            .anyMatch(e -> e.key().contains("driveStatorLimitAmps") && e.structural()),
        "Expected driveStatorLimitAmps flagged as structural");
  }

  public static void testTelemetryTruthfulnessHootScaleNoSyntheticVoltageMultiEnableAndBatteryGates()
      throws Exception {
    // Phoenix 6 drive wheel-scale (2027) vs legacy rotor-scale (2026 FUEL)
    Path hootDir = createTempDir("flux-hoot-scale-test");
    try (Writer w = new Writer(hootDir.resolve("drive_scale.wpilog"), "Phoenix6")) {
      int eEnable = w.startEntry("RobotEnable", "boolean", "", 0L);
      int eVolt = w.startEntry("Phoenix6/TalonFX-8/SupplyVoltage", "double", "", 0L);
      int[] ids = {2, 4, 6, 8};
      int[] eVel = new int[4];
      int[] eStator = new int[4];
      for (int m = 0; m < 4; m++) {
        eVel[m] = w.startEntry("Phoenix6/TalonFX-" + ids[m] + "/Velocity", "double", "", 0L);
        eStator[m] = w.startEntry("Phoenix6/TalonFX-" + ids[m] + "/StatorCurrent", "double", "", 0L);
      }
      for (int i = 1; i <= 25; i++) {
        long ts = i * 200_000L;
        w.appendBoolean(eEnable, ts, true);
        w.appendDouble(eVolt, ts, i == 12 ? 6.50 : 12.1);
        for (int m = 0; m < 4; m++) {
          w.appendDouble(eVel[m], ts, 10.0);
          w.appendDouble(eStator[m], ts, 65.0);
        }
      }
    }

    RunAnalysisReport rep2027 =
        RunAnalyzer.analyzeRunDirectory(hootDir, RobotProfile.COMP_2027, false);
    RunAnalysisReport repLegacy =
        RunAnalyzer.analyzeRunDirectory(hootDir, RobotProfile.FUEL, false);
    double expectedWheelMps = 10.0 * 2.0 * Math.PI * 0.0508;
    double expectedRotorMps = (10.0 / RobotProfile.FUEL.driveGearRatio()) * 2.0 * Math.PI * 0.0508;
    assertTrue(
        Math.abs(rep2027.peakSpeedMps() - expectedWheelMps) < 0.05,
        "2027 profile must treat Phoenix6 Velocity as wheel RPS (~3.19 m/s), got "
            + rep2027.peakSpeedMps());
    assertTrue(
        Math.abs(repLegacy.peakSpeedMps() - expectedRotorMps) < 0.05,
        "Legacy 2026 FUEL profile must divide Phoenix6 Velocity by driveGearRatio (~0.50 m/s), got "
            + repLegacy.peakSpeedMps());
    assertTrue(
        Math.abs(rep2027.battery().minVoltage() - 6.50) < 0.05,
        "All-motor supply voltage scanning must detect 6.50 V sag on BR motor ID 8, got "
            + rep2027.battery().minVoltage());
    assertTrue(
        rep2027.swerveCorners().stream().allMatch(c -> "WARN".equals(c.status())),
        "65 A peak drive stator must trigger WARN under 60 A limit: " + rep2027.swerveCorners());

    // No-voltage run must emit Double.NaN, never synthetic 12.4 V
    Path noVoltDir = createTempDir("flux-no-volt-test");
    try (Writer w = new Writer(noVoltDir.resolve("no_volt.wpilog"), "AdvantageKit")) {
      int eEnable = w.startEntry("/DriverStation/Enabled", "boolean", "", 0L);
      int eStator = w.startEntry("/Drive/Module0/DriveStatorCurrentAmps", "double", "", 0L);
      for (int i = 1; i <= 20; i++) {
        long ts = i * 200_000L;
        w.appendBoolean(eEnable, ts, true);
        w.appendDouble(eStator, ts, 35.0);
      }
    }
    RunAnalysisReport noVoltReport =
        RunAnalyzer.analyzeRunDirectory(noVoltDir, RobotProfile.COMP_2027, false);
    assertTrue(
        Double.isNaN(noVoltReport.battery().minVoltage())
            && Double.isNaN(noVoltReport.battery().restingVoltage())
            && noVoltReport.timeline().stream().allMatch(pt -> Double.isNaN(pt.batteryVoltage())),
        "Missing voltage topics must yield Double.NaN (never synthetic 12.4 V), got min="
            + noVoltReport.battery().minVoltage());

    // Multi-enable practice session at healthy 12.2 V must NOT trigger false brownout dropouts
    Path multiEnableDir = createTempDir("flux-multi-enable-practice");
    try (Writer w = new Writer(multiEnableDir.resolve("practice_cycles.wpilog"), "AdvantageKit")) {
      int eEnable = w.startEntry("/DriverStation/Enabled", "boolean", "", 0L);
      int eVolt = w.startEntry("/PowerDistribution/Voltage", "double", "", 0L);
      int[] eStator = new int[4];
      for (int m = 0; m < 4; m++) {
        eStator[m] = w.startEntry("/Drive/Module" + m + "/DriveStatorCurrentAmps", "double", "", 0L);
      }
      for (int i = 0; i <= 100; i++) {
        double tSec = i * 0.2;
        long ts = Math.round(tSec * 1_000_000.0);
        // 4 intentional practice enable bursts separated by 2.0 s disabled gaps
        boolean enabled =
            (tSec >= 1.0 && tSec <= 4.0)
                || (tSec >= 6.0 && tSec <= 9.0)
                || (tSec >= 11.0 && tSec <= 14.0)
                || (tSec >= 16.0 && tSec <= 19.0);
        w.appendBoolean(eEnable, ts, enabled);
        w.appendDouble(eVolt, ts, 12.2);
        for (int m = 0; m < 4; m++) {
          w.appendDouble(eStator[m], ts, enabled ? 58.0 : 0.0);
        }
      }
    }
    RunAnalysisReport multiReport =
        RunAnalyzer.analyzeRunDirectory(multiEnableDir, RobotProfile.COMP_2027, false);
    assertTrue(
        multiReport.battery().enableDropouts() == 0 && "PASS".equals(multiReport.overallVerdict()),
        "Healthy multi-enable practice session at 12.2V and 58A must have 0 dropouts and PASS, got dropouts="
            + multiReport.battery().enableDropouts()
            + " verdict="
            + multiReport.overallVerdict());

    // BatteryAnalyzer strict regression gates: reject positive slope, low spread, low R^2, stator-only
    List<RunAnalysisReport.TimelinePoint> posSlope = new ArrayList<>();
    List<RunAnalysisReport.TimelinePoint> lowSpread = new ArrayList<>();
    List<RunAnalysisReport.TimelinePoint> noisyLowR2 = new ArrayList<>();
    List<RunAnalysisReport.TimelinePoint> statorOnly = new ArrayList<>();
    for (int i = 0; i < 20; i++) {
      double t = i * 0.2;
      double cur = 20.0 + i * 5.0;
      posSlope.add(
          new RunAnalysisReport.TimelinePoint(t, true, "Teleop", 11.5 + cur * 0.010, 80.0, cur, 1.0, 0.0, 0.0));
      lowSpread.add(
          new RunAnalysisReport.TimelinePoint(t, true, "Teleop", 12.4 - (20.0 + i * 0.4) * 0.02, 80.0, 20.0 + i * 0.4, 1.0, 0.0, 0.0));
      double noiseV = (i % 2 == 0) ? 12.1 : 11.4;
      noisyLowR2.add(
          new RunAnalysisReport.TimelinePoint(t, true, "Teleop", noiseV, 80.0, cur, 1.0, 0.0, 0.0));
      statorOnly.add(
          new RunAnalysisReport.TimelinePoint(t, true, "Teleop", 12.6 - cur * 0.015, cur * 2.5, 0.0, 1.0, 0.0, 0.0));
    }
    assertTrue(
        Double.isNaN(BatteryAnalyzer.analyze("PosBatt", posSlope, 0).estimatedRintMilliOhms())
            && "NO DATA".equals(BatteryAnalyzer.analyze("PosBatt", posSlope, 0).healthGrade()),
        "Positive dV/dI slope must return NaN / NO DATA rather than 8.0 mOhm EXCELLENT");
    assertTrue(
        Double.isNaN(BatteryAnalyzer.analyze("NarrowBatt", lowSpread, 0).estimatedRintMilliOhms()),
        "Current spread < 15 A must return NaN / NO DATA");
    assertTrue(
        Double.isNaN(BatteryAnalyzer.analyze("NoisyBatt", noisyLowR2, 0).estimatedRintMilliOhms()),
        "Poor R^2 (< 0.35) must return NaN / NO DATA");
    assertTrue(
        Double.isNaN(BatteryAnalyzer.analyze("StatorOnlyBatt", statorOnly, 0).estimatedRintMilliOhms()),
        "Zero supply current must not fabricate supply from stator current");
  }

  public static void testDigitalTwinCalibratorGyroSpinArmCosineR2GateAndPidProtection()
      throws Exception {
    Path cfgDir = createSampleFluxFigDirectoryWithSwerveModules();
    Path mechCsv = cfgDir.resolve("mechanisms.csv");
    String originalMechanismsCsv = Files.readString(mechCsv, StandardCharsets.UTF_8);

    Path runDir = createTempDir("flux-simcal-gyro-arm");
    double trueWheelRadius = 0.04910;
    double driveBaseRadius = Math.hypot(0.56515 * 0.5, 0.56515 * 0.5);
    double armGearRatio = 48.0;

    try (Writer w = new Writer(runDir.resolve("spin_and_arm.wpilog"), "AdvantageKit")) {
      int eEnable = w.startEntry("/DriverStation/Enabled", "boolean", "", 0L);
      int eVolt = w.startEntry("/PowerDistribution/Voltage", "double", "", 0L);
      int eYaw = w.startEntry("/Drive/Gyro/YawPositionRad", "double", "", 0L);
      int[] eModPos = new int[4];
      for (int m = 0; m < 4; m++) {
        eModPos[m] = w.startEntry("/Drive/Module" + m + "/DrivePositionRad", "double", "", 0L);
      }

      int eArmPos = w.startEntry("/Arm/PositionRotations", "double", "", 0L);
      int eArmVel = w.startEntry("/Arm/VelocityRPS", "double", "", 0L);
      int eArmVolt = w.startEntry("/Arm/AppliedVoltage", "double", "", 0L);

      int eElevPos = w.startEntry("/Elevator/PositionRotations", "double", "", 0L);
      int eElevVel = w.startEntry("/Elevator/VelocityRPS", "double", "", 0L);
      int eElevVolt = w.startEntry("/Elevator/AppliedVoltage", "double", "", 0L);

      double unwrappedYaw = 0.0;
      double wheelRad = 0.0;
      double dt = 0.05;
      double prevArmVel = 0.0;
      double armRotorPos = 0.0;
      for (int i = 0; i <= 120; i++) {
        long ts = Math.round(i * dt * 1_000_000.0);
        w.appendBoolean(eEnable, ts, true);
        w.appendDouble(eVolt, ts, 12.2);

        // Spin in place at 1.6 rad/s across +/- pi wrapping boundary
        if (i > 0) {
          double dYaw = 1.6 * dt;
          unwrappedYaw += dYaw;
          wheelRad += (dYaw * driveBaseRadius) / trueWheelRadius;
        }
        double wrappedYaw = Math.atan2(Math.sin(unwrappedYaw), Math.cos(unwrappedYaw));
        w.appendDouble(eYaw, ts, wrappedYaw);
        for (int m = 0; m < 4; m++) {
          w.appendDouble(eModPos[m], ts, wheelRad);
        }

        // ARM_COSINE holding & ramp with rotor positions scaled by gearRatio=48.0
        // True kG=0.42 V, kS=0.16 V, kV=0.14 V/RPS, kA=0.015 V/RPS^2
        double armVel;
        double armAccel;
        if (i < 12) {
          armRotorPos = (i % 4) * 2.0; // 0, 2, 4, 6 rotor rot => 0, 15, 30, 45 deg
          armVel = 0.0;
          armAccel = 0.0;
        } else {
          armVel = 2.0 + (i - 12) * 0.25 + Math.sin(i * 0.3) * 1.5;
          armAccel = (armVel - prevArmVel) / dt;
          armRotorPos = Math.min(10.0, 2.0 + (i - 12) * 0.06);
        }
        prevArmVel = armVel;
        double jointRad = (armRotorPos / armGearRatio) * 2.0 * Math.PI;
        double armAppliedV =
            0.42 * Math.cos(jointRad)
                + (Math.abs(armVel) > 0.25 ? (0.16 + 0.14 * armVel + 0.015 * armAccel) : 0.0);
        w.appendDouble(eArmPos, ts, armRotorPos);
        w.appendDouble(eArmVel, ts, armVel);
        w.appendDouble(eArmVolt, ts, armAppliedV);

        // Noisy uncorrelated Elevator samples (R^2 < 0.50 -> must be rejected!)
        double noisyElevVel = 5.0 + (i % 15) * 1.2;
        double noisyElevVolt = (i % 2 == 0) ? 2.5 : 8.5;
        w.appendDouble(eElevPos, ts, i * 0.2);
        w.appendDouble(eElevVel, ts, noisyElevVel);
        w.appendDouble(eElevVolt, ts, noisyElevVolt);
      }
    }

    RunAnalysisReport report =
        RunAnalyzer.analyzeRunDirectory(runDir, RobotProfile.COMP_2027, false);
    DigitalTwinCalibrator.SimCalibrationSummary summary =
        DigitalTwinCalibrator.calibrateAndWrite(
            List.of(report), List.of(runDir), cfgDir, runDir, "COMP_2027");

    assertTrue(
        Math.abs(summary.calibratedWheelRadiusMeters() - trueWheelRadius) < 5e-4,
        "Expected gyro spin-in-place wheelRadiusMeters ~0.04910 m, got "
            + summary.calibratedWheelRadiusMeters());

    DigitalTwinCalibrator.MechanismFeedforwardFit armFit =
        summary.calibratedMechanisms().stream()
            .filter(f -> "Arm".equals(f.mechanism()))
            .findFirst()
            .orElse(null);
    assertTrue(armFit != null, "Expected high-R^2 Arm fit in sim-calibration summary");
    assertTrue(
        Math.abs(armFit.kG() - 0.42) < 0.03
            && Math.abs(armFit.kV() - 0.14) < 0.02
            && armFit.rSquared() >= 0.90,
        "Expected ARM_COSINE fit with pos/gearRatio scaling to recover kG~0.42, kV~0.14, R^2>=0.90, got "
            + armFit);

    boolean elevatorAccepted =
        summary.calibratedMechanisms().stream().anyMatch(f -> "Elevator".equals(f.mechanism()));
    assertTrue(
        !elevatorAccepted,
        "Noisy low-R^2 Elevator fit must be rejected by R^2 >= 0.50 gate");

    String afterMechanismsCsv = Files.readString(mechCsv, StandardCharsets.UTF_8);
    assertTrue(
        originalMechanismsCsv.equals(afterMechanismsCsv),
        "DigitalTwinCalibrator must NEVER overwrite production PID gains in mechanisms.csv");

    String calJson = Files.readString(summary.summaryJsonPath(), StandardCharsets.UTF_8);
    assertTrue(
        calJson.contains("\"wheelRadiusSource\": \"GYRO_SPIN_IN_PLACE\"")
            && calJson.contains("\"mechanismsCsvModified\": false")
            && !calJson.contains(": NaN"),
        "Expected valid sim-calibration.json with GYRO_SPIN_IN_PLACE and mechanismsCsvModified=false");
  }

  public static void testHtmlReportOfflineSafeAndXssClean() throws Exception {
    Path outDir = createTempDir("flux-html-xss-offline");
    assertTrue(
        RunAnalyzer.buildVideoJumpLink("javascript:alert(1)", 0.0, 5.0).isEmpty()
            && RunAnalyzer.buildVideoJumpLink("data:text/html,<script>alert(1)</script>", 0.0, 5.0)
                .isEmpty()
            && RunAnalyzer.buildVideoJumpLink("https://youtu.be/safe123", 2.0, 10.0)
                .startsWith("https://youtu.be/safe123"),
        "buildVideoJumpLink must allow only http:// and https:// URLs");

    RunAnalysisReport.BatteryMetrics batt =
        new RunAnalysisReport.BatteryMetrics(
            "Batt-<script>alert(0)</script>", Double.NaN, Double.NaN, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0, "NO DATA");
    RunAnalysisReport.ShotEvent xssShot =
        new RunAnalysisReport.ShotEvent(
            1, 4.0, 0.5, 42.0, 41.5, 0.5, 5.0, 15.0, Double.NaN, "javascript:alert(1)", "NOMINAL");
    RunAnalysisReport.VoiceCorrelation xssVoice =
        new RunAnalysisReport.VoiceCorrelation(
            "DRIVER",
            5.0,
            "<img src=x onerror=alert(1)>'\"</script><script>alert(2)</script>",
            0.0,
            "<b>Summary</b>",
            "Diagnosis",
            "data:text/html,<script>alert(9)</script>",
            "WARN");
    RunAnalysisReport.Sim3DFrame xssFrame =
        new RunAnalysisReport.Sim3DFrame(
            1.0,
            3.0,
            4.0,
            0.2,
            Double.NaN,
            45.0,
            40.0,
            3.0,
            0.2,
            0.1,
            1,
            0,
            2,
            "<svg onload=alert(3)></script>",
            new double[] {4.0, 4.0, 0.2});
    RunAnalysisReport.FuelSimMetrics fuelSim =
        new RunAnalysisReport.FuelSimMetrics(
            1, 1, 0, 100.0, 2, 1, 0, null, List.of(), List.of(xssFrame));
    RunAnalysisReport report =
        new RunAnalysisReport(
            "run_<script>alert(4)</script>",
            "COMP_2027",
            "Practice / Pit Session",
            "Blue 1",
            false,
            batt.batteryName(),
            "abc123",
            "cfg123",
            "javascript:alert(5)",
            0.0,
            10.0,
            8.0,
            12.0,
            3.2,
            "WARN",
            List.of("Alert <script>alert(6)</script>"),
            batt,
            List.of(),
            List.of(),
            List.of(xssShot),
            List.of(xssVoice),
            List.of(
                new RunAnalysisReport.TimelinePoint(0.0, true, "Teleop", Double.NaN, 10.0, 0.0, 1.0, 0.0, 0.0),
                new RunAnalysisReport.TimelinePoint(1.0, true, "Teleop", Double.NaN, 20.0, 0.0, 2.0, 0.0, 0.0)),
            null,
            null,
            List.of(),
            List.of(),
            fuelSim,
            null);

    Path htmlPath = outDir.resolve("report.html");
    HtmlReportGenerator.writeRunReport(report, htmlPath);
    HtmlReportGenerator.writeBatchSummary(List.of(report), outDir);

    String html = Files.readString(htmlPath, StandardCharsets.UTF_8);
    String idxHtml = Files.readString(outDir.resolve("runs-index.html"), StandardCharsets.UTF_8);

    assertTrue(
        !html.contains("https://")
            && !html.contains("cdn.jsdelivr.net")
            && !html.contains("fonts.googleapis.com")
            && !html.contains("squarespace-cdn.com")
            && !idxHtml.contains("https://"),
        "Generated HTML reports must have zero external HTTPS CDN dependencies");
    assertTrue(
        !html.contains(".innerHTML"),
        "Generated report.html must not use unsafe .innerHTML mutations");
    assertTrue(
        !html.contains("javascript:alert")
            && !html.contains("data:text/html")
            && !html.contains("<script>alert")
            && !html.contains("<img src=x")
            && !html.contains("<svg onload=alert"),
        "Generated report.html must block javascript:/data: links and escape all dynamic HTML/JS XSS payloads");
    assertTrue(
        html.contains("sim3dCanvas") && html.contains("sim3dFrames"),
        "Offline Canvas replay viewer must remain embedded in report.html");
  }

  private static Path createSampleFluxFigDirectoryWithSwerveModules() throws Exception {
    Path dir = createSampleFluxFigDirectory();
    String baseParams = Files.readString(dir.resolve("parameters.csv"), StandardCharsets.UTF_8);
    Files.writeString(
        dir.resolve("parameters.csv"),
        baseParams + "maxSpeedMps,double,m/s,4.80,4.80,4.80,Maximum chassis linear speed\n",
        StandardCharsets.UTF_8);
    Files.writeString(
        dir.resolve("swerve_modules.csv"),
        """
        robot,corner,driveMotorId,turnMotorId,cancoderId,encoderOffsetRot,invertDrive,invertTurn
        COMP_2027,FL,11,12,31,0.123,false,false
        COMP_2027,FR,13,14,32,0.234,true,false
        COMP_2027,BL,15,16,33,0.345,false,false
        COMP_2027,BR,17,18,34,0.456,true,false
        """,
        StandardCharsets.UTF_8);
    return dir;
  }

  private static Path createSampleFluxFigDirectory() throws Exception {
    Path dir = createTempDir("sample-fluxfig");
    Files.writeString(
        dir.resolve("parameters.csv"),
        """
        Key,Type,Unit,@DEFAULT,@COMP_2027,@FUEL,Description
        pigeonId,int,id,20,20,20,Pigeon 2 IMU CAN ID
        wheelRadiusMeters,double,m,0.05080,0.05080,0.05080,Effective swerve wheel radius
        driveGearRatio,double,ratio,6.3947,6.3947,6.3947,Swerve drive gear ratio
        swerveCouplingRatio,double,ratio,3.5714,3.5714,3.5714,Steer to drive coupling ratio
        flDriveId,int,id,2,2,2,FL Drive ID
        flSteerId,int,id,1,1,1,FL Steer ID
        flCancoderId,int,id,21,21,21,FL CANcoder ID
        frDriveId,int,id,4,4,4,FR Drive ID
        frSteerId,int,id,3,3,3,FR Steer ID
        frCancoderId,int,id,22,22,22,FR CANcoder ID
        blDriveId,int,id,6,6,6,BL Drive ID
        blSteerId,int,id,5,5,5,BL Steer ID
        blCancoderId,int,id,23,23,23,BL CANcoder ID
        brDriveId,int,id,8,8,8,BR Drive ID
        brSteerId,int,id,7,7,7,BR Steer ID
        brCancoderId,int,id,24,24,24,BR CANcoder ID
        """,
        StandardCharsets.UTF_8);

    Files.writeString(
        dir.resolve("mechanisms.csv"),
        """
        robot,mechanism,motorId,canBus,statorLimitAmps,supplyLimitAmps,slipCurrentAmps,gravityType,kP,kI,kD,kS,kV,kA,kG,minPosRot,maxPosRot,followerMotorId,invertFollower,motorInverted,gearRatio
        COMP_2027,Elevator,30,can_s2,60.0,40.0,60.0,Elevator_Static,2.5,0.0,0.1,0.15,0.11,0.01,0.30,0.0,45.0,31,true,false,12.0
        COMP_2027,Arm,32,can_s2,40.0,30.0,40.0,Arm_Cosine,4.0,0.0,0.2,0.12,0.80,0.02,0.25,-0.25,0.50,-1,false,false,48.0
        """,
        StandardCharsets.UTF_8);

    Files.writeString(
        dir.resolve("batteries.csv"),
        """
        batteryName,qrCode,purchaseYear,nominalRintMilliOhms,maxAllowedRintMilliOhms
        2027-B01,QR-2027-B01,2026,15.5,24.0
        """,
        StandardCharsets.UTF_8);
    return dir;
  }

  private static void assertTrue(boolean cond, String msg) {
    if (!cond) {
      throw new AssertionError("Assertion failed: " + msg);
    }
  }
}
