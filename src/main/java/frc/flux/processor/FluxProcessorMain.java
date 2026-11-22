package frc.flux.processor;

import frc.flux.processor.analysis.DigitalTwinCalibrator;
import frc.flux.processor.analysis.RunAnalyzer;
import frc.flux.processor.hoot.HootConverter;
import frc.flux.processor.mcp.FluxMcpServer;
import frc.flux.processor.model.RobotProfile;
import frc.flux.processor.model.RunAnalysisReport;
import frc.flux.processor.model.RunAnalysisReport.CandidateTuningReview;
import frc.flux.processor.model.RunAnalysisReport.TuningDiffEntry;
import frc.flux.processor.report.AutoSimVideoRenderer;
import frc.flux.processor.report.HtmlReportGenerator;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.stream.Stream;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;

/**
 * CLI &amp; MCP server entry point for {@code FluxProcessor} v2027.1.0.
 *
 * <pre>
 * Usage:
 *   java -jar flux-processor.jar &lt;input-dir-or-zip&gt; [--out &lt;output-dir&gt;] [--robot AUTO|COMP_2027|PRACTICE_2027|CHASSIS_3|FUEL|CORAL|&lt;NAME&gt;]
 *        [--config-dir &lt;fluxfig-csv-dir&gt;] [--profile &lt;robot-profile.json&gt;]
 *        [--calibrate-sim [&lt;fluxfig-csv-dir&gt;]] [--owlet &lt;path&gt;] [--no-merge]
 *   java -jar flux-processor.jar --mcp [&lt;input-dir&gt;] [--robot AUTO|COMP_2027|PRACTICE_2027|CHASSIS_3|FUEL|CORAL]
 * </pre>
 */
public final class FluxProcessorMain {

  public static void main(String[] args) throws Exception {
    int code = runCli(args);
    if (code != 0) {
      System.exit(code);
    }
  }

  /**
   * Executes the CLI workflow and returns an exit code without calling {@code System.exit},
   * guaranteeing temp-dir ZIP cleanup in {@code finally} on both success and failure.
   */
  public static int runCli(String[] args) throws Exception {
    if (args.length == 0 || "--help".equals(args[0]) || "-h".equals(args[0])) {
      printUsage();
      return 0;
    }

    Path inputDir = null;
    Path outputDir = null;
    Path owletPath = null;
    Path configDir = null;
    Path calibrateSimDir = null;
    Path renderAutoJson = null;
    boolean calibrateSimRequested = false;
    boolean mcpMode = false;
    String requestedRobotName = "AUTO";
    RobotProfile profile = RobotProfile.AUTO;
    boolean writeMerged = true;

    for (int i = 0; i < args.length; i++) {
      String arg = args[i];
      switch (arg) {
        case "--render-auto" -> {
          if (i + 1 < args.length) {
            renderAutoJson = Paths.get(args[++i]);
          }
        }
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
            Path candidate = Paths.get(args[i + 1]);
            boolean hasAnotherPositional = hasRemainingPositionalArg(args, i + 2);
            if (isFluxFigConfigDir(candidate) || inputDir != null || hasAnotherPositional) {
              calibrateSimDir = candidate;
              i++;
            }
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
            profile = RobotProfile.fromPresetName(raw);
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

    if (inputDir == null && calibrateSimDir != null && !isFluxFigConfigDir(calibrateSimDir)) {
      inputDir = calibrateSimDir;
      calibrateSimDir = null;
    }

    if (configDir != null && Files.isDirectory(configDir)) {
      profile = RobotProfile.fromFluxFigDirectory(configDir, requestedRobotName);
    }

    if (mcpMode) {
      Path mcpRoot = inputDir != null ? inputDir : Paths.get(".");
      FluxMcpServer server = new FluxMcpServer(mcpRoot, profile);
      server.serveStdio(System.in, System.out);
      return 0;
    }

    if (renderAutoJson == null
        && inputDir != null
        && Files.isRegularFile(inputDir)
        && inputDir.getFileName().toString().toLowerCase(Locale.ROOT).endsWith(".json")) {
      renderAutoJson = inputDir;
    }

    if (renderAutoJson != null) {
      if (Files.isDirectory(renderAutoJson)) {
        renderAutoJson = renderAutoJson.resolve("student-auto-frames.json");
      }
      if (!Files.exists(renderAutoJson)) {
        System.err.println("Error: Autonomous frames JSON does not exist: " + renderAutoJson);
        return 1;
      }
      Path autoOutDir =
          outputDir != null
              ? outputDir
              : (renderAutoJson.getParent() != null ? renderAutoJson.getParent() : Paths.get("."));
      AutoSimVideoRenderer.AutoSimPayload payload =
          AutoSimVideoRenderer.renderFromJson(renderAutoJson, autoOutDir);
      System.out.printf(
          Locale.US,
          " [AUTO-SIM] Rendered %s (%s) • %d frames, %d shots -> %s%n",
          payload.routineName(),
          payload.robotName(),
          payload.frames().size(),
          payload.shots().size(),
          autoOutDir.resolve("student-auto.mp4"));
      return 0;
    }

    if (inputDir == null || !Files.exists(inputDir)) {
      System.err.println("Error: Input path does not exist: " + inputDir);
      return 1;
    }

    Path tempExtractDir = null;
    boolean inputWasZip = false;
    String zipBaseName = null;
    try {
      if (Files.isRegularFile(inputDir)
          && inputDir.getFileName().toString().toLowerCase(Locale.ROOT).endsWith(".zip")) {
        inputWasZip = true;
        String zipFileName = inputDir.getFileName().toString();
        zipBaseName = zipFileName.substring(0, zipFileName.length() - 4);
        tempExtractDir = Files.createTempDirectory("flux-processor-zip-");
        Path extractRoot = tempExtractDir.resolve(zipBaseName);
        extractZipArchive(inputDir, extractRoot);
        inputDir = extractRoot;
        if (outputDir == null) {
          outputDir = Files.createTempDirectory("flux-processor-out-");
        }
      }

      if (outputDir == null) {
        outputDir = inputDir;
      }

      List<Path> runDirs = discoverRunDirectories(inputDir, owletPath);
      if (runDirs.isEmpty()) {
        System.err.println("No .wpilog or .hoot run folders found under: " + inputDir);
        return 1;
      }

      System.out.println(
          "==========================================================================================");
      System.out.printf(
          Locale.US,
          " FLUX PROCESSOR v2027.1.0  •  Robot Profile: %s  •  Found %d run(s)%n",
          profile.robotName(),
          runDirs.size());
      if (inputWasZip) {
        System.out.println(" Output Directory  : " + outputDir.toAbsolutePath());
      }
      System.out.println(
          "==========================================================================================");

      List<RunAnalysisReport> reports = new ArrayList<>();
      for (Path runDir : runDirs) {
        Path targetRunOut;
        if (outputDir.equals(inputDir)) {
          targetRunOut = runDir;
        } else if (inputWasZip && runDir.equals(tempExtractDir) && zipBaseName != null) {
          targetRunOut = outputDir.resolve(zipBaseName);
        } else {
          targetRunOut = outputDir.resolve(runDir.getFileName());
        }
        Files.createDirectories(targetRunOut);
        HootConverter.convertDirectoryIfNeeded(runDir, targetRunOut, owletPath);
        RunAnalysisReport report =
            RunAnalyzer.analyzeRunDirectory(
                runDir, targetRunOut, inputDir, configDir, profile, writeMerged);
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
        String rintStr =
            Double.isFinite(report.battery().estimatedRintMilliOhms())
                ? String.format(Locale.US, "%4.1f mΩ", report.battery().estimatedRintMilliOhms())
                : "NO DATA";
        String minVoltStr =
            Double.isFinite(report.battery().minVoltage())
                ? String.format(Locale.US, "%5.2fV", report.battery().minVoltage())
                : "  N/A";
        System.out.printf(
            Locale.US,
            " [%s] %-38s | Batt: %-16s (R_int: %s, Min: %s) | Peak Stator: %4.0fA | Shots: %2d (FuelSim: %s) | %s%n",
            report.overallVerdict(),
            report.runId(),
            report.battery().batteryName(),
            rintStr,
            minVoltStr,
            report.battery().peakDriveStatorAmps(),
            report.shots().size(),
            fuelSimStr,
            htmlPath);

        printCandidateTuningSummary(report.candidateTuning());
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
      return 0;
    } finally {
      deleteDirectoryRecursively(tempExtractDir);
    }
  }

  private static void printCandidateTuningSummary(CandidateTuningReview review) {
    if (review == null || !review.present()) {
      return;
    }
    System.out.printf(
        Locale.US,
        "   [CANDIDATE TUNING - UNMERGED] Verdict: %s | Profile: %s | Snapshot: %s | Base SHA: %s | Overrides: %d%n",
        review.verdict(),
        review.activeProfile(),
        review.sourceFilePath(),
        review.baseConfigSha(),
        review.entries().size());
    for (TuningDiffEntry entry : review.entries()) {
      System.out.printf(
          Locale.US,
          "     - [%s] %s: %s -> %s%s%n",
          entry.scope(),
          entry.key(),
          entry.baseValue(),
          entry.candidateValue(),
          entry.structural() ? " [STRUCTURAL - BLOCKED]" : "");
    }
    for (String reason : review.verdictReasons()) {
      System.out.println("     * " + reason);
    }
    System.out.println("     Promote CLI: " + review.promoteCommand());
  }

  private static boolean isFluxFigConfigDir(Path dir) {
    return dir != null
        && Files.isDirectory(dir)
        && (Files.isRegularFile(dir.resolve("parameters.csv"))
            || Files.isRegularFile(dir.resolve("mechanisms.csv"))
            || Files.isRegularFile(dir.resolve("swerve_modules.csv")));
  }

  private static boolean hasRemainingPositionalArg(String[] args, int startIdx) {
    for (int j = startIdx; j < args.length; j++) {
      String a = args[j];
      if ("--out".equals(a)
          || "-o".equals(a)
          || "--owlet".equals(a)
          || "--config-dir".equals(a)
          || "--fluxfig".equals(a)
          || "--profile".equals(a)
          || "-p".equals(a)
          || "--robot".equals(a)
          || "-r".equals(a)
          || "--render-auto".equals(a)) {
        j++;
        continue;
      }
      if (!a.startsWith("-")) {
        return true;
      }
    }
    return false;
  }

  private static void deleteDirectoryRecursively(Path dir) {
    if (dir == null || !Files.exists(dir)) {
      return;
    }
    try (Stream<Path> walk = Files.walk(dir)) {
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

  private static List<Path> discoverRunDirectories(Path root, Path owletPath)
      throws IOException, InterruptedException {
    if (!Files.isDirectory(root)) {
      return List.of();
    }
    if (hasDirectLogFiles(root)
        || (hasRunMarkerFile(root) && hasRecursiveLogFiles(root))) {
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
      if (hasRecursiveLogFiles(sub)) {
        valid.add(sub);
      }
    }
    if (!valid.isEmpty()) {
      return valid;
    }
    return hasRecursiveLogFiles(root) ? List.of(root) : List.of();
  }

  private static boolean hasRunMarkerFile(Path dir) {
    return Files.isRegularFile(dir.resolve("run-meta.json"))
        || Files.isRegularFile(dir.resolve("system-snapshot.txt"))
        || Files.isRegularFile(dir.resolve("audio-sync.json"));
  }

  private static boolean hasDirectLogFiles(Path dir) throws IOException {
    try (Stream<Path> s = Files.list(dir)) {
      return s.filter(Files::isRegularFile).anyMatch(FluxProcessorMain::isRawLogFile);
    }
  }

  private static boolean hasRecursiveLogFiles(Path dir) throws IOException {
    try (Stream<Path> s = Files.walk(dir, 4)) {
      return s.filter(Files::isRegularFile).anyMatch(FluxProcessorMain::isRawLogFile);
    }
  }

  private static boolean isRawLogFile(Path p) {
    String n = p.getFileName().toString().toLowerCase(Locale.ROOT);
    return (n.endsWith(".wpilog") && !n.equals("merged.wpilog")) || n.endsWith(".hoot");
  }

  private static void extractZipArchive(Path zipFile, Path targetDir) throws IOException {
    Files.createDirectories(targetDir);
    Path normalizedRoot = targetDir.toAbsolutePath().normalize();
    try (InputStream fis = Files.newInputStream(zipFile);
        ZipInputStream zis = new ZipInputStream(fis, StandardCharsets.UTF_8)) {
      ZipEntry entry;
      while ((entry = zis.getNextEntry()) != null) {
        Path outPath = normalizedRoot.resolve(entry.getName()).normalize();
        if (!outPath.startsWith(normalizedRoot)) {
          throw new IOException(
              "Blocked zip-slip entry outside target directory: " + entry.getName());
        }
        if (entry.isDirectory()) {
          Files.createDirectories(outPath);
        } else {
          if (outPath.getParent() != null) {
            Files.createDirectories(outPath.getParent());
          }
          Files.copy(zis, outPath, StandardCopyOption.REPLACE_EXISTING);
        }
        zis.closeEntry();
      }
    }
  }

  private static void printUsage() {
    System.out.println(
        """
        FluxProcessor v2027.1.0 — FRC Team 10413 (FLUX Robotics) Automated Log, Voice, Video, Digital-Twin & MCP Analyzer
        Usage:
          java -jar dist/flux-processor.jar <input-dir-or-zip> [options]
          java -jar dist/flux-processor.jar --render-auto <student-auto-frames.json> [--out <output-dir>]
          java -jar dist/flux-processor.jar --mcp [<input-dir>] [--robot AUTO|COMP_2027|PRACTICE_2027|CHASSIS_3|FUEL|CORAL|<NAME>]

        Options:
          --out, -o <dir>              Output directory for reports and merged.wpilog (default: <input-dir>, or temp dir for .zip)
          --render-auto <frames.json>  Render 3D student-auto.mp4 and student-auto.html from autonomous simulation JSON
          --robot, -r <name>           Robot profile: AUTO (default), COMP_2027, PRACTICE_2027, CHASSIS_3, FUEL, CORAL, or custom
          --config-dir <dir>           Load robot parameters & mechanisms directly from a FluxFig CSV directory
          --profile, -p <file.json>    Load custom robot CAN/topic mapping from robot-profile.json
          --calibrate-sim [<cfg-dir>]  Extract gated battery R_int, wheel radius, coupling & mechanism plant constants into sim-calibration.json
          --mcp [<input-dir>]          Start JSON-RPC 2.0 stdio Model Context Protocol (MCP) server (8 multimodal tools)
          --owlet <path>               Path to CTRE owlet CLI for .hoot -> .wpilog conversion
          --no-merge                   Skip writing merged.wpilog
        """);
  }
}
