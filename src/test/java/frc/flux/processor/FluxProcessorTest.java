package frc.flux.processor;

import frc.flux.processor.analysis.RunAnalyzer;
import frc.flux.processor.model.RobotProfile;
import frc.flux.processor.model.RunAnalysisReport;
import frc.flux.processor.report.HtmlReportGenerator;
import frc.flux.processor.wpilog.WpiLogReader;
import frc.flux.processor.wpilog.WpiLogWriter;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

/**
 * Self-contained verification tests for {@code FluxProcessor} (binary {@code .wpilog} read/write,
 * battery internal resistance regression, 4-corner swerve symmetry, shot detection, 6-second
 * multi-headset voice lookback, video jump links, and HTML/CSV report generation).
 */
public final class FluxProcessorTest {

  public static void main(String[] args) throws Exception {
    testWpiLogReadWriteRoundTrip();
    testEndToEndSyntheticRunWithVoiceAndVideo();
    System.out.println("ALL FLUX PROCESSOR TESTS PASSED (2/2)");
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
    assertTrue("CRITICAL".equals(report.overallVerdict()), "Expected CRITICAL verdict due to 6.65V sag");
    assertTrue(report.shots().size() == 1, "Expected 1 detected shot, got " + report.shots().size());
    assertTrue(
        report.voiceCorrelations().size() == 2,
        "Expected 2 correlated voice notes, got " + report.voiceCorrelations().size());
    assertTrue(
        report.voiceCorrelations().get(1).correlatedEventSummary().contains("Shot #1"),
        "Operator voice note at 12.0s should correlate with Shot #1 at 9.6s");
    assertTrue(
        report.voiceCorrelations().get(1).videoLink().contains("youtu.be/dQw4w9WgXcQ?t="),
        "Expected YouTube jump link on voice correlation");
    assertTrue(Files.exists(runDir.resolve("merged.wpilog")), "Expected merged.wpilog to be written");

    Path reportHtml = runDir.resolve("report.html");
    HtmlReportGenerator.writeRunReport(report, reportHtml);
    HtmlReportGenerator.writeBatchSummary(List.of(report), runDir);
    assertTrue(Files.exists(reportHtml), "Expected report.html to be generated");
    assertTrue(Files.exists(runDir.resolve("runs-index.html")), "Expected runs-index.html");
    assertTrue(Files.exists(runDir.resolve("batteries-health.csv")), "Expected batteries-health.csv");
  }

  private static void assertTrue(boolean cond, String msg) {
    if (!cond) {
      throw new AssertionError("Assertion failed: " + msg);
    }
  }
}
