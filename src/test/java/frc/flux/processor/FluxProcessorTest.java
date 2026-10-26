package frc.flux.processor;

import frc.flux.processor.analysis.DigitalTwinCalibrator;
import frc.flux.processor.analysis.RunAnalyzer;
import frc.flux.processor.mcp.FluxMcpServer;
import frc.flux.processor.model.RobotProfile;
import frc.flux.processor.model.RunAnalysisReport;
import frc.flux.processor.report.HtmlReportGenerator;
import frc.flux.processor.wpilog.WpiLogReader;
import frc.flux.processor.wpilog.WpiLogWriter;
import frc.lib.fluxfig.FluxFig;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

/**
 * Self-contained verification tests for {@code FluxProcessor} v2.0:
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

  public static void main(String[] args) throws Exception {
    testWpiLogReadWriteRoundTrip();
    testEndToEndSyntheticRunWithVoiceAndVideo();
    testOtherTeamAdvantageKitAndDoubleArrayAutoDiscovery();
    testOtherTeamCustomJsonProfileAndCustomPhoenix6Ids();
    testOfflineSystemCoreWatchMetricsAndReportCard();
    testFluxFigSnapshotAndFluxTrayRunMetaContract();
    testShopStressGateElevatorCyclesAndWpilogOnlyWatchGuard();
    testDigitalTwinCalibratorCsvWriteBack();
    testMcpServerMultimodalTools();
    System.out.println("ALL FLUX PROCESSOR TESTS PASSED (9/9)");
  }

  public static void testWpiLogReadWriteRoundTrip() throws Exception {
    Path tmpDir = Files.createTempDirectory("flux-wpilog-test");
    Path logFile = tmpDir.resolve("test_rio.wpilog");
    try (WpiLogWriter w = new WpiLogWriter(logFile, "TestHeader")) {
      int eEnable = w.startEntry("RobotEnable", "boolean", "", 1_000_000L);
      int eVolt = w.startEntry("Voltage", "double", "", 1_000_000L);
      int eMode = w.startEntry("RobotMode", "string", "", 1_000_000L);

      w.appendBoolean(eEnable, 1_000_000L, true);
      w.appendDouble(eVolt, 1_000_000L, 12.5);
      w.appendString(eMode, 1_000_000L, "Teleop");
      w.appendDouble(eVolt, 2_000_000L, 11.8);
    }

    WpiLogReader.ParsedLog parsed = WpiLogReader.read(logFile);
    assertTrue("TestHeader".equals(parsed.extraHeader()), "Expected TestHeader");
    assertTrue(parsed.getBooleanSeries("RobotEnable").size() == 1, "Expected 1 boolean sample");
    assertTrue(parsed.getDoubleSeries("Voltage").size() == 2, "Expected 2 double samples");
    assertTrue(
        Math.abs(parsed.getDoubleSeries("Voltage").get(1).value() - 11.8) < 1e-6,
        "Expected 11.8V");
  }

  public static void testEndToEndSyntheticRunWithVoiceAndVideo() throws Exception {
    Path runDir = Files.createTempDirectory("flux-run-test");

    // 1. Write .meta.json with Named Battery "Thor" and YouTube URL
    Files.writeString(
        runDir.resolve("run.meta.json"),
        """
        {
          "namedBattery": "Thor",
          "gitSha": "abc1234",
          "configSha": "cfg9876",
          "videoUrl": "https://youtu.be/dQw4w9WgXcQ",
          "videoOffsetSec": 12.0
        }
        """,
        StandardCharsets.UTF_8);

    // 2. Write multi-headset voice transcript (Driver + Operator)
    Files.writeString(
        runDir.resolve("voice-notes.txt"),
        """
        6.2 | DRIVER | Felt a hard battery sag on acceleration
        12.0 | OPERATOR | Missed short on that shot
        """,
        StandardCharsets.UTF_8);

    // 3. Create synthetic _rio.wpilog, _drive.wpilog, _mech.wpilog over 15 seconds
    try (WpiLogWriter rio = new WpiLogWriter(runDir.resolve("test_rio.wpilog"), "Rio");
        WpiLogWriter drive = new WpiLogWriter(runDir.resolve("test_drive.wpilog"), "Drive");
        WpiLogWriter mech = new WpiLogWriter(runDir.resolve("test_mech.wpilog"), "Mech")) {

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
    WpiLogReader.ParsedLog mergedLog = WpiLogReader.read(runDir.resolve("merged.wpilog"));
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
    Path runDir = Files.createTempDirectory("other-team-akit-test");
    try (WpiLogWriter akit =
        new WpiLogWriter(runDir.resolve("akit_match.wpilog"), "AdvantageKit")) {
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
    Path runDir = Files.createTempDirectory("other-team-json-test");
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

    try (WpiLogWriter log = new WpiLogWriter(runDir.resolve("custom_match.wpilog"), "Custom")) {
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
    Path runDir = Files.createTempDirectory("systemcore-watch-test");
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

    try (WpiLogWriter log = new WpiLogWriter(runDir.resolve("akit_sc.wpilog"), "AdvantageKit")) {
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

    Path runDir = Files.createTempDirectory("fluxfig-snapshot-test");
    // Write FluxTray's exact run-meta.json format with spaces in batteryName and JSON voiceNotes array
    Files.writeString(
        runDir.resolve("run-meta.json"),
        """
        {
          "runId": "run_2027_q1",
          "batteryName": "2027-B04 (Alpha Pack)",
          "gitSha": "9f8e7d6",
          "configSha256": "deadbeef1234",
          "voiceNotes": [
            { "role": "DRIVER", "timeSec": 4.2, "text": "Elevator L4 extension felt super smooth" }
          ]
        }
        """,
        StandardCharsets.UTF_8);

    try (WpiLogWriter log = new WpiLogWriter(runDir.resolve("sc_2027.wpilog"), "AdvantageKit")) {
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
    Path runDir = Files.createTempDirectory("shop-stress-gate-test");
    Path snapDir = runDir.resolve("cam-snapshots");
    Files.createDirectories(snapDir);
    Files.writeString(
        snapDir.resolve("limelightsc0_t2.40s_shot.jpg"), "FAKE_JPEG_BYTES", StandardCharsets.UTF_8);

    try (WpiLogWriter log = new WpiLogWriter(runDir.resolve("shop_stress.wpilog"), "ShopStress")) {
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
            && report.systemHealth().totalCpuPct() == 0.0
            && report.systemHealth().freeStorageMb() == 0.0,
        "Expected .wpilog-only run without system-snapshot.txt not to fabricate fake SystemCore CPU/storage stats");
  }

  public static void testDigitalTwinCalibratorCsvWriteBack() throws Exception {
    Path cfgDir = createSampleFluxFigDirectory();
    Path runDir = Files.createTempDirectory("sim-cal-run");

    Files.writeString(
        runDir.resolve("run-meta.json"),
        """
        { "batteryName": "2027-B01", "gitSha": "cal123" }
        """,
        StandardCharsets.UTF_8);

    try (WpiLogWriter log = new WpiLogWriter(runDir.resolve("cal.wpilog"), "Cal")) {
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
    Path wsDir = Files.createTempDirectory("flux-mcp-test");
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
      try (WpiLogWriter w = new WpiLogWriter(dir.resolve("match.wpilog"), "MCP")) {
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

  private static Path createSampleFluxFigDirectory() throws Exception {
    Path dir = Files.createTempDirectory("sample-fluxfig");
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
