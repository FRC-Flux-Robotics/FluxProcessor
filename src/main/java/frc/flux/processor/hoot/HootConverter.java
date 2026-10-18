package frc.flux.processor.hoot;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Stream;

/**
 * Automatically converts CTRE Phoenix 6 {@code .hoot} files into WPILib {@code .wpilog} files using
 * the {@code owlet} CLI when present.
 *
 * <p>Uses {@code owlet -S -F} (full scan) to extract the essential telemetry signal IDs across the
 * entire {@code .hoot} file and converts without {@code -e} so multi-enable matches or late-match
 * enables after long queue times are never truncated. Also handles {@code owlet} exit code {@code
 * 1} at 100% completion when the final frame of a {@code .hoot} file was cut off at robot power
 * down.
 */
public final class HootConverter {

  private static final List<String> ESSENTIAL_SIGNAL_KEYWORDS =
      List.of(
          "RobotEnable",
          "SupplyVoltage",
          "MotorVoltage",
          "StatorCurrent",
          "SupplyCurrent",
          "Velocity",
          "Position",
          "Yaw",
          "Pitch",
          "Roll",
          "AngularVelocityZWorld",
          "DeviceTemp",
          "Temperature",
          "FaultField",
          "StickyFaultField",
          "ResetCount",
          "ClosedLoopReference",
          "ClosedLoopError");

  private static final Pattern SCAN_ID_PATTERN = Pattern.compile(":\\s*([0-9a-fA-F]+)\\s*$");

  private HootConverter() {}

  /** Locates the {@code owlet} executable on disk or in {@code PATH}. */
  public static Path findOwlet(Path explicitPath) {
    if (explicitPath != null && Files.isExecutable(explicitPath)) {
      return explicitPath;
    }
    List<Path> candidates =
        List.of(
            Paths.get("/tmp/owlet"),
            Paths.get(System.getProperty("user.home"), ".flux", "owlet"),
            Paths.get(System.getProperty("user.home"), "bin", "owlet"),
            Paths.get("/usr/local/bin/owlet"));
    for (Path c : candidates) {
      if (Files.isExecutable(c)) {
        return c;
      }
    }
    String pathEnv = System.getenv("PATH");
    if (pathEnv != null) {
      for (String dir : pathEnv.split(java.io.File.pathSeparator)) {
        Path bin = Paths.get(dir, "owlet");
        if (Files.isExecutable(bin)) {
          return bin;
        }
        Path binExe = Paths.get(dir, "owlet.exe");
        if (Files.isExecutable(binExe)) {
          return binExe;
        }
      }
    }
    return null;
  }

  /**
   * Converts any {@code .hoot} files in {@code runDir} into {@code targetOutDir} (leaving {@code
   * runDir} untouched when {@code targetOutDir} differs).
   */
  public static List<Path> convertDirectoryIfNeeded(
      Path runDir, Path targetOutDir, Path owletBinary) throws IOException, InterruptedException {
    List<Path> produced = new ArrayList<>();
    if (!Files.isDirectory(runDir)) {
      return produced;
    }
    List<Path> hootFiles;
    try (Stream<Path> s = Files.list(runDir)) {
      hootFiles =
          s.filter(
                  p -> p.getFileName().toString().toLowerCase(Locale.ROOT).endsWith(".hoot"))
              .sorted()
              .toList();
    }
    if (hootFiles.isEmpty()) {
      return produced;
    }
    Files.createDirectories(targetOutDir);
    Path owlet = findOwlet(owletBinary);

    for (Path hoot : hootFiles) {
      String fileName = hoot.getFileName().toString();
      String baseName = fileName.substring(0, fileName.length() - 5);
      Path existingInRunDir = runDir.resolve(baseName + ".wpilog");
      Path outWpilog = targetOutDir.resolve(baseName + ".wpilog");
      Path tmpCached = Paths.get("/tmp", baseName + ".wpilog");

      if (Files.exists(outWpilog) && Files.size(outWpilog) > 1000) {
        produced.add(outWpilog);
        continue;
      }
      if (Files.exists(existingInRunDir) && Files.size(existingInRunDir) > 1000) {
        produced.add(existingInRunDir);
        continue;
      }
      if (Files.exists(tmpCached) && Files.size(tmpCached) > 1000) {
        Files.copy(tmpCached, outWpilog, StandardCopyOption.REPLACE_EXISTING);
        produced.add(outWpilog);
        continue;
      }
      if (owlet == null) {
        continue;
      }

      List<String> signalIds = scanEssentialSignalIds(owlet, hoot);
      List<String> cmd = new ArrayList<>();
      cmd.add(owlet.toAbsolutePath().toString());
      cmd.add(hoot.toAbsolutePath().toString());
      cmd.add(outWpilog.toAbsolutePath().toString());
      cmd.add("-f");
      cmd.add("wpilog");
      cmd.add("--unlicensed");
      if (!signalIds.isEmpty()) {
        cmd.add("-s");
        cmd.add(String.join(",", signalIds));
      }

      ProcessBuilder pb = new ProcessBuilder(cmd);
      pb.redirectErrorStream(true);
      Process p = pb.start();
      p.getInputStream().readAllBytes();
      p.waitFor();

      // owlet may return exit code 1 at 100% if the final frame of the .hoot file was truncated at
      // robot power-off; verify that the output .wpilog file was written and non-empty.
      if (Files.exists(outWpilog) && Files.size(outWpilog) > 1000) {
        produced.add(outWpilog);
      }
    }
    return produced;
  }

  /** Backward-compatible overload writing into {@code runDir}. */
  public static List<Path> convertDirectoryIfNeeded(Path runDir, Path owletBinary)
      throws IOException, InterruptedException {
    return convertDirectoryIfNeeded(runDir, runDir, owletBinary);
  }

  private static List<String> scanEssentialSignalIds(Path owlet, Path hoot)
      throws IOException, InterruptedException {
    ProcessBuilder pb =
        new ProcessBuilder(
            owlet.toAbsolutePath().toString(), hoot.toAbsolutePath().toString(), "-S", "-F");
    pb.redirectErrorStream(true);
    Process p = pb.start();
    String out = new String(p.getInputStream().readAllBytes(), StandardCharsets.UTF_8);
    p.waitFor();

    List<String> ids = new ArrayList<>();
    for (String line : out.split("\\R")) {
      boolean match = false;
      for (String kw : ESSENTIAL_SIGNAL_KEYWORDS) {
        if (line.contains(kw)) {
          match = true;
          break;
        }
      }
      if (match) {
        Matcher m = SCAN_ID_PATTERN.matcher(line);
        if (m.find()) {
          ids.add(m.group(1));
        }
      }
    }
    return ids;
  }
}
