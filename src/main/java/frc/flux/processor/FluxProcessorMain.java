package frc.flux.processor;

import frc.flux.processor.analysis.DigitalTwinCalibrator;
import frc.flux.processor.analysis.RunAnalyzer;
import frc.flux.processor.hoot.HootConverter;
import frc.flux.processor.mcp.FluxMcpServer;
import frc.flux.processor.model.RobotProfile;
import frc.flux.processor.model.RunAnalysisReport;
import frc.flux.processor.report.HtmlReportGenerator;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.stream.Stream;

/**
 * CLI &amp; MCP server entry point for {@code FluxProcessor} v2.0.
 *
 * <pre>
 * Usage:
 *   java -jar flux-processor.jar &lt;input-dir&gt; [--out &lt;output-dir&gt;] [--robot AUTO|COMP_2027|FUEL|CORAL|&lt;NAME&gt;]
 *        [--config-dir &lt;fluxfig-csv-dir&gt;] [--profile &lt;robot-profile.json&gt;]
 *        [--calibrate-sim [&lt;fluxfig-csv-dir&gt;]] [--owlet &lt;path&gt;] [--no-merge]
 *   java -jar flux-processor.jar --mcp [&lt;input-dir&gt;] [--robot AUTO|COMP_2027|FUEL|CORAL]
 * </pre>
 */
public final class FluxProcessorMain {

  public static void main(String[] args) throws Exception {
    if (args.length == 0 || "--help".equals(args[0]) || "-h".equals(args[0])) {
      printUsage();
      return;
    }

    Path inputDir = null;
    Path outputDir = null;
    Path owletPath = null;
    Path configDir = null;
    Path calibrateSimDir = null;
    boolean calibrateSimRequested = false;
    boolean mcpMode = false;
    String requestedRobotName = "AUTO";
    RobotProfile profile = RobotProfile.AUTO;
    boolean writeMerged = true;

    for (int i = 0; i < args.length; i++) {
      String arg = args[i];
      switch (arg) {
        case "--mcp" -> {
          mcpMode = true;
          if (i + 1 < args.length && !args[i + 1].startsWith("-") && inputDir == null) {
            inputDir = Paths.get(args[++i]);
          }
        }
        case "--out", "-o" -> {
          if (i + 1 < args.length) {
            outputDir = Paths.get(args[++i]);
          }
        }
        case "--owlet" -> {
          if (i + 1 < args.length) {
            owletPath = Paths.get(args[++i]);
          }
        }
        case "--config-dir", "--fluxfig" -> {
          if (i + 1 < args.length) {
            configDir = Paths.get(args[++i]);
          }
        }
        case "--calibrate-sim" -> {
          calibrateSimRequested = true;
          if (i + 1 < args.length && !args[i + 1].startsWith("-")) {
            calibrateSimDir = Paths.get(args[++i]);
          }
        }
        case "--profile", "-p" -> {
          if (i + 1 < args.length) {
            profile = RobotProfile.fromJson(Paths.get(args[++i]));
            requestedRobotName = profile.robotName();
          }
        }
        case "--robot", "-r" -> {
          if (i + 1 < args.length) {
            String raw = args[++i].trim();
            requestedRobotName = raw;
            String upper = raw.toUpperCase(Locale.ROOT);
            if ("FUEL".equals(upper)) {
              profile = RobotProfile.FUEL;
            } else if ("CORAL".equals(upper)) {
              profile = RobotProfile.CORAL;
            } else if ("AUTO".equals(upper)) {
              profile = RobotProfile.AUTO;
            } else {
              profile =
                  new RobotProfile(
                      raw,
                      RobotProfile.AUTO.pigeonId(),
                      RobotProfile.AUTO.wheelRadiusMeters(),
                      RobotProfile.AUTO.driveGearRatio(),
                      Map.of(),
                      List.of());
            }
          }
        }
        case "--no-merge" -> writeMerged = false;
        default -> {
          if (!arg.startsWith("-") && inputDir == null) {
            inputDir = Paths.get(arg);
          }
        }
      }
    }

    if (configDir != null && Files.isDirectory(configDir)) {
      profile = RobotProfile.fromFluxFigDirectory(configDir, requestedRobotName);
    }

    if (mcpMode) {
      Path mcpRoot = inputDir != null ? inputDir : Paths.get(".");
      FluxMcpServer server = new FluxMcpServer(mcpRoot, profile);
      server.serveStdio(System.in, System.out);
      return;
    }

    if (inputDir == null || !Files.exists(inputDir)) {
      System.err.println("Error: Input path does not exist: " + inputDir);
      System.exit(1);
    }

    if (outputDir == null) {
      outputDir = inputDir;
    }

    List<Path> runDirs = discoverRunDirectories(inputDir, owletPath);
    if (runDirs.isEmpty()) {
      System.err.println("No .wpilog or .hoot run folders found under: " + inputDir);
      System.exit(1);
    }

    System.out.println(
        "==========================================================================================");
    System.out.printf(
        Locale.US,
        " FLUX PROCESSOR v2.0  •  Robot Profile: %s  •  Found %d run(s)%n",
        profile.robotName(),
        runDirs.size());
    System.out.println(
        "==========================================================================================");

    List<RunAnalysisReport> reports = new ArrayList<>();
    for (Path runDir : runDirs) {
      Path targetRunOut =
          outputDir.equals(inputDir) ? runDir : outputDir.resolve(runDir.getFileName());
      Files.createDirectories(targetRunOut);
      HootConverter.convertDirectoryIfNeeded(runDir, targetRunOut, owletPath);
      RunAnalysisReport report =
          RunAnalyzer.analyzeRunDirectory(runDir, targetRunOut, profile, writeMerged);
      reports.add(report);

      Path htmlPath = targetRunOut.resolve("report.html");
      HtmlReportGenerator.writeRunReport(report, htmlPath);

      String fuelSimStr =
          report.fuelSim() != null
              ? String.format(
                  Locale.US,
                  "%2d/%2d (%3.0f%%)",
                  report.fuelSim().ballsScoredInHub(),
                  report.fuelSim().ballsLaunched(),
                  report.fuelSim().hubScoringAccuracyPct())
              : "N/A";
      System.out.printf(
          Locale.US,
          " [%s] %-38s | Batt: %-16s (R_int: %4.1f mΩ, Min: %5.2fV) | Peak Stator: %4.0fA | Shots: %2d (FuelSim: %s) | %s%n",
          report.overallVerdict(),
          report.runId(),
          report.battery().batteryName(),
          report.battery().estimatedRintMilliOhms(),
          report.battery().minVoltage(),
          report.battery().peakDriveStatorAmps(),
          report.shots().size(),
          fuelSimStr,
          htmlPath);
    }

    HtmlReportGenerator.writeBatchSummary(reports, outputDir);

    if (calibrateSimRequested) {
      Path targetCfgDir = calibrateSimDir != null ? calibrateSimDir : configDir;
      DigitalTwinCalibrator.SimCalibrationSummary calSummary =
          DigitalTwinCalibrator.calibrateAndWrite(
              reports, runDirs, targetCfgDir, outputDir, requestedRobotName);
      System.out.println(
          "------------------------------------------------------------------------------------------");
      System.out.printf(
          Locale.US,
          " Digital-Twin Sim Calibration (%s): wheelRadius=%.5f m, coupling=%.4f, batteries=%d, mechs=%d, ConfigSHA=%s%n",
          calSummary.targetProfile(),
          calSummary.calibratedWheelRadiusMeters(),
          calSummary.calibratedCouplingRatio(),
          calSummary.calibratedBatteries().size(),
          calSummary.calibratedMechanisms().size(),
          calSummary.updatedConfigSha256());
      System.out.println(" Sim Calibration JSON: " + calSummary.summaryJsonPath());
    }

    System.out.println(
        "------------------------------------------------------------------------------------------");
    System.out.println(" Season Index HTML : " + outputDir.resolve("runs-index.html"));
    System.out.println(" Season Index CSV  : " + outputDir.resolve("runs-index.csv"));
    System.out.println(" Battery Health CSV: " + outputDir.resolve("batteries-health.csv"));
    System.out.println(
        "==========================================================================================");
  }

  private static List<Path> discoverRunDirectories(Path root, Path owletPath)
      throws IOException, InterruptedException {
    if (!Files.isDirectory(root)) {
      return List.of();
    }
    if (hasLogFiles(root)) {
      return List.of(root);
    }
    List<Path> subDirs;
    try (Stream<Path> s = Files.list(root)) {
      subDirs =
          s.filter(Files::isDirectory)
              .sorted(Comparator.comparing(p -> p.getFileName().toString()))
              .toList();
    }
    List<Path> valid = new ArrayList<>();
    for (Path sub : subDirs) {
      if (hasLogFiles(sub)) {
        valid.add(sub);
      }
    }
    return valid;
  }

  private static boolean hasLogFiles(Path dir) throws IOException {
    try (Stream<Path> s = Files.list(dir)) {
      return s.anyMatch(
          p -> {
            String n = p.getFileName().toString().toLowerCase(Locale.ROOT);
            return (n.endsWith(".wpilog") && !n.equals("merged.wpilog")) || n.endsWith(".hoot");
          });
    }
  }

  private static void printUsage() {
    System.out.println(
        """
        FluxProcessor v2.0 — FRC Team 10413 (FLUX Robotics) Automated Log, Voice, Video, Digital-Twin & MCP Analyzer
        Usage:
          java -jar flux-processor.jar <input-dir> [options]
          java -jar flux-processor.jar --mcp [<input-dir>] [--robot AUTO|COMP_2027|FUEL|CORAL|<NAME>]

        Options:
          --out, -o <dir>              Output directory for reports and merged.wpilog (default: <input-dir>)
          --robot, -r <name>           Robot profile: AUTO (default), COMP_2027, PRACTICE_2027, FUEL, CORAL, or custom
          --config-dir <dir>           Load robot parameters & mechanisms directly from a FluxFig CSV directory
          --profile, -p <file.json>    Load custom robot CAN/topic mapping from robot-profile.json
          --calibrate-sim [<cfg-dir>]  Extract battery R_int, wheel radius, coupling & mechanism (kS,kV,kA,kG) into FluxFig CSVs
          --mcp [<input-dir>]          Start JSON-RPC 2.0 stdio Model Context Protocol (MCP) server (8 multimodal tools)
          --owlet <path>               Path to CTRE owlet CLI for .hoot -> .wpilog conversion
          --no-merge                   Skip writing merged.wpilog
        """);
  }
}
