package frc.flux.processor;

import frc.flux.processor.analysis.RunAnalyzer;
import frc.flux.processor.hoot.HootConverter;
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
import java.util.stream.Stream;

/**
 * CLI entry point for {@code FluxProcessor}.
 *
 * <pre>
 * Usage:
 *   java -jar flux-processor.jar &lt;input-dir&gt; [--out &lt;output-dir&gt;] [--robot FUEL|CORAL] [--no-merge]
 * </pre>
 */
public final class FluxProcessorMain {

  public static void main(String[] args) throws Exception {
    if (args.length == 0 || "--help".equals(args[0]) || "-h".equals(args[0])) {
      printUsage();
      return;
    }

    Path inputDir = Paths.get(args[0]);
    Path outputDir = null;
    Path owletPath = null;
    RobotProfile profile = RobotProfile.FUEL;
    boolean writeMerged = true;

    for (int i = 1; i < args.length; i++) {
      switch (args[i]) {
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
        case "--robot", "-r" -> {
          if (i + 1 < args.length) {
            String r = args[++i].toUpperCase(Locale.ROOT);
            profile = "CORAL".equals(r) ? RobotProfile.CORAL : RobotProfile.FUEL;
          }
        }
        case "--no-merge" -> writeMerged = false;
        default -> {}
      }
    }

    if (!Files.exists(inputDir)) {
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
        " FLUX PROCESSOR v0.1  •  Robot Profile: %s  •  Found %d run(s)%n",
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

      System.out.printf(
          Locale.US,
          " [%s] %-38s | Batt: %-16s (R_int: %4.1f mΩ, Min: %5.2fV) | Peak Stator: %4.0fA | Shots: %3d | %s%n",
          report.overallVerdict(),
          report.runId(),
          report.battery().batteryName(),
          report.battery().estimatedRintMilliOhms(),
          report.battery().minVoltage(),
          report.battery().peakDriveStatorAmps(),
          report.shots().size(),
          htmlPath);
    }

    HtmlReportGenerator.writeBatchSummary(reports, outputDir);
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
        FluxProcessor — FRC Team 10413 (FLUX Robotics) Automated Log, Voice & Video Analyzer
        Usage:
          java -jar flux-processor.jar <input-dir> [--out <output-dir>] [--robot FUEL|CORAL] [--owlet <path>] [--no-merge]
        """);
  }
}
