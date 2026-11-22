package frc.flux.processor.analysis;

import frc.flux.processor.model.RobotProfile;
import frc.flux.processor.model.RunAnalysisReport.BatteryMetrics;
import frc.flux.processor.model.RunAnalysisReport.CandidateTuningReview;
import frc.flux.processor.model.RunAnalysisReport.CornerMetrics;
import frc.flux.processor.model.RunAnalysisReport.MechanismMetrics;
import frc.flux.processor.model.RunAnalysisReport.ShopStressGateMetrics;
import frc.flux.processor.model.RunAnalysisReport.SystemHealthMetrics;
import frc.flux.processor.model.RunAnalysisReport.SystemWatchDecision;
import frc.flux.processor.model.RunAnalysisReport.TuningDiffEntry;
import frc.lib.fluxfig.FluxFig;
import frc.lib.fluxfig.FluxWpiLogCodec.ParsedLog;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Stream;

/**
 * Detects and audits unmerged practice live-tuning snapshots ({@code tuned-snapshot.txt}) against
 * the {@code STABLE} baseline and run telemetry, producing a promotion safety verdict and an
 * explicit {@code fluxfig.jar promote} CLI command for human sign-off.
 */
public final class CandidateTuningAnalyzer {

  public static final String VERDICT_READY = "READY_TO_PROMOTE";
  public static final String VERDICT_REVIEW = "REVIEW_WARNINGS";
  public static final String VERDICT_BLOCK = "DO_NOT_PROMOTE";

  private static final Pattern TABLE_OVERRIDE_PATTERN =
      Pattern.compile("^#?\\s*([^\\[\\s#]+)\\[([^\\]]+)\\]\\.([^:\\s]+)\\s*:\\s*(.*?)\\s*->\\s*(.+)$");

  private static final Pattern PARAM_OVERRIDE_PATTERN =
      Pattern.compile("^#?\\s*([A-Za-z0-9_:@-]+)\\.([A-Za-z0-9_]+)\\s*:\\s*(.*?)\\s*->\\s*(.+)$");

  private static final Pattern FALLBACK_KV_PATTERN =
      Pattern.compile("^([A-Za-z0-9_:@.-]+)\\s*=\\s*(.+)$");

  private static final Set<String> STRUCTURAL_MECHANISM_COLUMNS =
      Set.of(
          "robot",
          "mechanism",
          "motorid",
          "canbus",
          "statorlimitamps",
          "supplylimitamps",
          "slipcurrentamps",
          "gravitytype",
          "followermotorid",
          "invertfollower",
          "motorinverted",
          "gearratio");

  private static final Set<String> STRUCTURAL_HARDWARE_TABLES =
      Set.of("swerve_modules", "cameras");

  private CandidateTuningAnalyzer() {}

  /**
   * Locates {@code tuned-snapshot.txt} inside {@code runDir} or {@code inputRoot}, diffs its
   * overrides against the {@code STABLE} baseline, evaluates telemetry/structural safety gates, and
   * returns {@code null} when no candidate tuning snapshot is present.
   */
  public static CandidateTuningReview analyze(
      Path runDir,
      Path outRunDir,
      Path inputRoot,
      Path configDir,
      String activeRobotName,
      List<ParsedLog> allLogs,
      BatteryMetrics battery,
      List<CornerMetrics> swerveCorners,
      List<MechanismMetrics> mechanisms,
      ShopStressGateMetrics shopStressGate,
      SystemHealthMetrics systemHealth)
      throws IOException {
    Path foundSnapshot = findTunedSnapshot(runDir, inputRoot);
    if (foundSnapshot == null) {
      return null;
    }

    String rawText = Files.readString(foundSnapshot, StandardCharsets.UTF_8);
    if (rawText.isBlank()) {
      return null;
    }

    Path effectiveSnapshotPath = foundSnapshot;
    if (outRunDir != null && !outRunDir.equals(foundSnapshot.getParent())) {
      Files.createDirectories(outRunDir);
      Path copied = outRunDir.resolve("tuned-snapshot.txt");
      Files.copy(foundSnapshot, copied, StandardCopyOption.REPLACE_EXISTING);
      effectiveSnapshotPath = copied;
    }

    FluxFig stableBaseFig = loadStableBaseFig(configDir, allLogs, activeRobotName);
    String baseSha =
        stableBaseFig != null ? stableBaseFig.getConfigSha256() : extractHeaderField(rawText, "# ConfigSHA256:");
    if (baseSha == null || baseSha.isBlank()) {
      baseSha = "unknown";
    }

    String snapshotProfile = extractHeaderField(rawText, "# ActiveProfile:");
    String activeProfile =
        (snapshotProfile != null && !snapshotProfile.isBlank())
            ? snapshotProfile
            : (activeRobotName != null && !activeRobotName.isBlank() ? activeRobotName : "DEFAULT");

    List<TuningDiffEntry> entries = parseAndDiffEntries(rawText, activeProfile, stableBaseFig);

    List<String> reasons = new ArrayList<>();
    String verdict =
        evaluateSafetyVerdict(
            entries, battery, swerveCorners, mechanisms, shopStressGate, systemHealth, reasons);

    String configDirArg =
        configDir != null ? quoteCliArg(configDir.toString()) : "<config-dir>";
    String snapArg = quoteCliArg(effectiveSnapshotPath.toString());
    String promoteCommand =
        "java -jar FluxFig/dist/fluxfig.jar promote " + snapArg + " " + configDirArg;

    return new CandidateTuningReview(
        true,
        effectiveSnapshotPath.toString(),
        activeProfile,
        baseSha,
        verdict,
        List.copyOf(reasons),
        List.copyOf(entries),
        promoteCommand);
  }

  private static Path findTunedSnapshot(Path runDir, Path inputRoot) {
    List<String> names = List.of("tuned-snapshot.txt", "fluxfig-tuned-snapshot.txt");
    if (runDir != null && Files.isDirectory(runDir)) {
      for (String name : names) {
        Path direct = runDir.resolve(name);
        if (Files.isRegularFile(direct)) {
          return direct;
        }
      }
      try (Stream<Path> walk = Files.walk(runDir, 3)) {
        Path nested =
            walk.filter(Files::isRegularFile)
                .filter(
                    p -> {
                      String fn = p.getFileName().toString().toLowerCase(Locale.ROOT);
                      return fn.equals("tuned-snapshot.txt")
                          || fn.equals("fluxfig-tuned-snapshot.txt");
                    })
                .sorted(Comparator.comparing(Path::toString))
                .findFirst()
                .orElse(null);
        if (nested != null) {
          return nested;
        }
      } catch (IOException ignored) {
      }
    }
    if (inputRoot != null && Files.isDirectory(inputRoot) && !inputRoot.equals(runDir)) {
      for (String name : names) {
        Path direct = inputRoot.resolve(name);
        if (Files.isRegularFile(direct)) {
          return direct;
        }
      }
    }
    return null;
  }

  private static FluxFig loadStableBaseFig(
      Path configDir, List<ParsedLog> allLogs, String activeRobotName) {
    if (configDir != null
        && Files.isDirectory(configDir)
        && Files.isRegularFile(configDir.resolve("parameters.csv"))) {
      try {
        return (activeRobotName != null
                && !activeRobotName.isBlank()
                && !"AUTO".equalsIgnoreCase(activeRobotName))
            ? FluxFig.load(configDir, activeRobotName)
            : FluxFig.load(configDir);
      } catch (RuntimeException ignored) {
      }
    }
    if (allLogs != null && !allLogs.isEmpty()) {
      String snapshot = RobotProfile.extractFluxFigSnapshotFromLogs(allLogs);
      if (snapshot != null && !snapshot.isBlank()) {
        try {
          FluxFig fromLog = FluxFig.fromEffectiveSnapshot(snapshot);
          fromLog.clearOverrides();
          return fromLog;
        } catch (RuntimeException ignored) {
        }
      }
    }
    return null;
  }

  private static String extractHeaderField(String rawText, String prefix) {
    for (String line : rawText.split("\\R")) {
      String trimmed = line.trim();
      if (trimmed.startsWith("=== ")) {
        break;
      }
      if (trimmed.startsWith(prefix)) {
        return trimmed.substring(prefix.length()).trim();
      }
    }
    return "";
  }

  static List<TuningDiffEntry> parseAndDiffEntries(
      String rawText, String defaultProfile, FluxFig stableBaseFig) {
    Map<String, TuningDiffEntry> byKey = new LinkedHashMap<>();
    boolean inCsvSection = false;

    for (String rawLine : rawText.split("\\R")) {
      String trimmed = rawLine.trim();
      if (trimmed.startsWith("=== ") && trimmed.endsWith(" ===")) {
        inCsvSection = true;
        continue;
      }
      if (inCsvSection || trimmed.isEmpty()) {
        continue;
      }
      if (trimmed.equals("# FluxFig Effective Snapshot")
          || trimmed.startsWith("# Mode:")
          || trimmed.startsWith("# EffectiveVersion:")
          || trimmed.startsWith("# ConfigSHA256:")
          || trimmed.startsWith("# ActiveProfile:")
          || trimmed.equals("# RuntimeOverrides:")
          || trimmed.equals("# TableCellOverrides:")) {
        continue;
      }

      Matcher tableMatcher = TABLE_OVERRIDE_PATTERN.matcher(trimmed);
      if (tableMatcher.matches()) {
        String table = stripCsvExt(tableMatcher.group(1).trim()).toLowerCase(Locale.ROOT);
        String bracketContent = tableMatcher.group(2).trim();
        int lastColon = bracketContent.lastIndexOf(':');
        String prof =
            lastColon > 0 ? bracketContent.substring(0, lastColon).trim() : defaultProfile;
        String rowKey =
            lastColon > 0 ? bracketContent.substring(lastColon + 1).trim() : bracketContent;
        String col = tableMatcher.group(3).trim();
        String snapOld = tableMatcher.group(4).trim();
        String newVal = tableMatcher.group(5).trim();
        String resolvedBase = resolveBaseTableCell(stableBaseFig, table, prof, rowKey, col, snapOld);
        String fullKey = table + "[" + prof + ":" + rowKey + "]." + col;
        boolean structural = isStructuralTableCell(table, col);
        if (!resolvedBase.equals(newVal)) {
          byKey.put(
              fullKey,
              new TuningDiffEntry("table:" + table, fullKey, resolvedBase, newVal, structural));
        }
        continue;
      }

      Matcher paramMatcher = PARAM_OVERRIDE_PATTERN.matcher(trimmed);
      if (paramMatcher.matches()) {
        String prof = paramMatcher.group(1).trim();
        String key = paramMatcher.group(2).trim();
        String snapOld = paramMatcher.group(3).trim();
        String newVal = paramMatcher.group(4).trim();
        String resolvedBase = resolveBaseParameter(stableBaseFig, prof, key, snapOld);
        String fullKey = prof + "." + key;
        boolean structural = FluxFig.isStructuralParameterKey(key);
        if (!resolvedBase.equals(newVal)) {
          byKey.put(
              fullKey,
              new TuningDiffEntry("param:" + prof, fullKey, resolvedBase, newVal, structural));
        }
        continue;
      }

      if (!trimmed.startsWith("#")) {
        Matcher kvMatcher = FALLBACK_KV_PATTERN.matcher(trimmed);
        if (kvMatcher.matches()) {
          String rawKey = kvMatcher.group(1).trim();
          String newVal = kvMatcher.group(2).trim();
          String prof = defaultProfile;
          String paramKey = rawKey;
          int dotIdx = rawKey.indexOf('.');
          if (dotIdx > 0 && dotIdx < rawKey.length() - 1) {
            prof = rawKey.substring(0, dotIdx).trim();
            paramKey = rawKey.substring(dotIdx + 1).trim();
          }
          String resolvedBase = resolveBaseParameter(stableBaseFig, prof, paramKey, "<unset>");
          String fullKey = prof + "." + paramKey;
          boolean structural = FluxFig.isStructuralParameterKey(paramKey);
          if (!resolvedBase.equals(newVal)) {
            byKey.put(
                fullKey,
                new TuningDiffEntry("param:" + prof, fullKey, resolvedBase, newVal, structural));
          }
        }
      }
    }

    return new ArrayList<>(byKey.values());
  }

  private static String resolveBaseParameter(
      FluxFig baseFig, String profile, String key, String fallback) {
    if (baseFig != null) {
      try {
        if (baseFig.hasParameter(profile, key)) {
          String prevProfile = baseFig.getActiveProfile();
          baseFig.setActiveProfile(profile);
          try {
            return baseFig.getString(key);
          } finally {
            baseFig.setActiveProfile(prevProfile);
          }
        }
      } catch (RuntimeException ignored) {
      }
    }
    return (fallback != null && !fallback.isBlank()) ? fallback : "<unset>";
  }

  private static String resolveBaseTableCell(
      FluxFig baseFig,
      String table,
      String profile,
      String rowKey,
      String col,
      String fallback) {
    if (baseFig != null && baseFig.getTableNames().contains(table)) {
      List<Map<String, String>> rows = baseFig.getTableForProfile(table, profile);
      String normRowKey = normalizeId(rowKey);
      String normCol = normalizeId(col);
      for (Map<String, String> row : rows) {
        boolean rowMatch = false;
        for (Map.Entry<String, String> cell : row.entrySet()) {
          String kNorm = normalizeId(cell.getKey());
          if ((kNorm.equals("mechanism")
                  || kNorm.equals("preset")
                  || kNorm.equals("presetname")
                  || kNorm.equals("corner")
                  || kNorm.equals("cameraname")
                  || kNorm.equals("batteryname"))
              && normalizeId(cell.getValue()).equals(normRowKey)) {
            rowMatch = true;
            break;
          }
        }
        if (rowMatch) {
          for (Map.Entry<String, String> cell : row.entrySet()) {
            if (normalizeId(cell.getKey()).equals(normCol)) {
              return cell.getValue();
            }
          }
        }
      }
    }
    return (fallback != null && !fallback.isBlank()) ? fallback : "<unset>";
  }

  private static boolean isStructuralTableCell(String table, String column) {
    String normTable = stripCsvExt(table).toLowerCase(Locale.ROOT);
    String normCol = normalizeId(column);
    if (STRUCTURAL_HARDWARE_TABLES.contains(normTable)) {
      return true;
    }
    if ("mechanisms".equals(normTable) && STRUCTURAL_MECHANISM_COLUMNS.contains(normCol)) {
      return true;
    }
    return FluxFig.isStructuralParameterKey(column);
  }

  private static String evaluateSafetyVerdict(
      List<TuningDiffEntry> entries,
      BatteryMetrics battery,
      List<CornerMetrics> swerveCorners,
      List<MechanismMetrics> mechanisms,
      ShopStressGateMetrics shopStressGate,
      SystemHealthMetrics systemHealth,
      List<String> reasons) {
    boolean blocked = false;
    boolean warn = false;

    List<String> structuralKeys = new ArrayList<>();
    for (TuningDiffEntry e : entries) {
      if (e.structural()) {
        structuralKeys.add(e.key());
      }
    }
    if (!structuralKeys.isEmpty()) {
      blocked = true;
      reasons.add(
          "Blocked structural/hardware-limit key(s) modified in live tuning: "
              + String.join(", ", structuralKeys)
              + ".");
    }

    if (battery != null) {
      if (Double.isFinite(battery.minVoltage())
          && battery.minVoltage() < BatteryAnalyzer.BROWNOUT_CRIT_VOLTS) {
        blocked = true;
        reasons.add(
            String.format(
                Locale.US,
                "Critical voltage sag to %.2f V (< %.1f V brownout limit).",
                battery.minVoltage(),
                BatteryAnalyzer.BROWNOUT_CRIT_VOLTS));
      } else if (Double.isFinite(battery.minVoltage()) && battery.minVoltage() < 7.5) {
        warn = true;
        reasons.add(
            String.format(
                Locale.US,
                "Battery voltage dipped to %.2f V (< 7.5 V warning threshold).",
                battery.minVoltage()));
      }
      if (battery.enableDropouts() > 0) {
        blocked = true;
        reasons.add(
            String.format(
                Locale.US,
                "%d unexpected enable dropout(s) observed during candidate run.",
                battery.enableDropouts()));
      }
    }

    double maxCornerStator = 0.0;
    String maxCornerName = "";
    if (swerveCorners != null && !swerveCorners.isEmpty()) {
      for (CornerMetrics c : swerveCorners) {
        if (c.peakDriveStatorAmps() > maxCornerStator) {
          maxCornerStator = c.peakDriveStatorAmps();
          maxCornerName = c.corner();
        }
        if ("FAIL".equals(c.status()) || "WARN".equals(c.status())) {
          warn = true;
          reasons.add(
              String.format(
                  Locale.US,
                  "Swerve corner %s flagged %s (asymmetry %+.1f%%, peak %.1f A).",
                  c.corner(),
                  c.status(),
                  c.driveAsymmetryPct(),
                  c.peakDriveStatorAmps()));
        }
      }
    } else if (battery != null && battery.peakDriveStatorAmps() > 0.0) {
      maxCornerStator = battery.peakDriveStatorAmps() / 4.0;
      maxCornerName = "4-corner avg";
    }
    if (maxCornerStator > SwerveSymmetryAnalyzer.DRIVE_STATOR_LIMIT_AMPS) {
      blocked = true;
      reasons.add(
          String.format(
              Locale.US,
              "Peak drive stator current %.1f A (%s) exceeded %.0f A safety limit.",
              maxCornerStator,
              maxCornerName,
              SwerveSymmetryAnalyzer.DRIVE_STATOR_LIMIT_AMPS));
    }

    if (mechanisms != null) {
      for (MechanismMetrics m : mechanisms) {
        if ("WARN".equals(m.status())) {
          warn = true;
          reasons.add(
              String.format(
                  Locale.US,
                  "Mechanism %s flagged WARN (peak %.1f A, %d resets).",
                  m.name(),
                  m.peakStatorAmps(),
                  m.resetCount()));
        }
      }
    }

    if (shopStressGate != null && shopStressGate.present()) {
      if ("FAIL".equals(shopStressGate.overallGateStatus())) {
        blocked = true;
        reasons.add("ShopStress 5m x 3m gate FAILED.");
      } else if ("WARN".equals(shopStressGate.overallGateStatus())) {
        warn = true;
        reasons.add("ShopStress 5m x 3m gate reported WARN.");
      }
    }

    if (systemHealth != null) {
      boolean hasLoopDecision = false;
      for (SystemWatchDecision wd : systemHealth.watchDecisions()) {
        if (wd.metricName().toLowerCase(Locale.ROOT).contains("loop")) {
          hasLoopDecision = true;
        }
        if ("CRITICAL".equals(wd.severity())) {
          blocked = true;
          reasons.add("CRITICAL system watch alert: " + wd.metricName() + " (" + wd.measuredValue() + ").");
        } else {
          warn = true;
          reasons.add("WARN system watch alert: " + wd.metricName() + " (" + wd.measuredValue() + ").");
        }
      }
      if (!hasLoopDecision
          && Double.isFinite(systemHealth.loopP99Ms())
          && Double.isFinite(systemHealth.loopPeriodMs())
          && systemHealth.loopP99Ms() > systemHealth.loopPeriodMs()) {
        warn = true;
        reasons.add(
            String.format(
                Locale.US,
                "Loop p99 latency %.2f ms exceeded %.1f ms period.",
                systemHealth.loopP99Ms(),
                systemHealth.loopPeriodMs()));
      }
    }

    if (entries.isEmpty()) {
      warn = true;
      reasons.add("No net parameter or table-cell diffs against STABLE baseline were found.");
    }

    if (blocked) {
      return VERDICT_BLOCK;
    }
    if (warn) {
      return VERDICT_REVIEW;
    }
    reasons.add(
        "All telemetry and structural safety gates passed (no brownouts, per-module drive stator <= 60 A, 0 critical watch alerts).");
    return VERDICT_READY;
  }

  private static String stripCsvExt(String name) {
    return name.toLowerCase(Locale.ROOT).endsWith(".csv")
        ? name.substring(0, name.length() - 4)
        : name;
  }

  private static String normalizeId(String raw) {
    if (raw == null) {
      return "";
    }
    return raw.replace("_", "").replace("-", "").replace(" ", "").trim().toLowerCase(Locale.ROOT);
  }

  private static String quoteCliArg(String value) {
    if (value == null || value.isEmpty()) {
      return "\"\"";
    }
    if (value.contains(" ") || value.contains("\t") || value.contains("\"")) {
      return "\"" + value.replace("\"", "\\\"") + "\"";
    }
    return value;
  }
}
