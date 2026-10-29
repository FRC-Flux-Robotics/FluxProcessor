package frc.flux.processor.analysis;

import frc.flux.processor.model.RunAnalysisReport.SystemHealthMetrics;
import frc.flux.processor.model.RunAnalysisReport.SystemWatchDecision;
import frc.flux.processor.wpilog.WpiLogReader.DoubleSample;
import frc.flux.processor.wpilog.WpiLogReader.ParsedLog;
import frc.lib.fluxfig.FluxSystemSnapshot;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.stream.Stream;

/**
 * Offline evaluator for SystemCore / roboRIO watch metrics combining {@code system-snapshot.txt}
 * (when pulled over SSH by {@code FluxTray.jar} or {@code pull-logs.bat}) with {@code .wpilog}
 * timing and {@code SystemStats} topics via the shared {@link FluxSystemSnapshot} engine in {@code
 * FluxFig}.
 */
public final class SystemHealthWatchAnalyzer {

  private SystemHealthWatchAnalyzer() {}

  public static SystemHealthMetrics analyze(
      Path runDir, List<ParsedLog> allLogs, long t0Us, long windowEndUs, long firstEnableUs) {
    Path snapFile = findSnapshotFile(runDir);
    String rawText = null;
    if (snapFile != null) {
      try {
        rawText = Files.readString(snapFile, StandardCharsets.UTF_8);
      } catch (IOException ignored) {
      }
    }

    double wpiDriveMaxMs = maxFromTopicSuffix(allLogs, "Performance/DriveMs", t0Us, windowEndUs);
    double wpiVisionMaxMs = maxFromTopicSuffix(allLogs, "Performance/VisionMs", t0Us, windowEndUs);
    double wpiSuperMaxMs =
        maxFromTopicSuffix(allLogs, "Performance/SuperstructureMs", t0Us, windowEndUs);
    double wpiCpuTempC = maxFromTopicSuffix(allLogs, "CPUTemp", t0Us, windowEndUs);

    boolean hasSnapshot = rawText != null && !rawText.isBlank();
    if (!hasSnapshot && wpiDriveMaxMs <= 0.0 && wpiVisionMaxMs <= 0.0 && wpiCpuTempC <= 0.0) {
      return null;
    }

    FluxSystemSnapshot parsed = FluxSystemSnapshot.parse(hasSnapshot ? rawText : "");
    boolean hasFreeStorageKey = hasSnapshot && rawText.contains("freeStorageMb=");

    String defaultController = inferControllerFromLogs(allLogs);
    String controllerType =
        hasSnapshot
            ? parsed.controllerType()
            : defaultController + " (WPILOG-ONLY)";
    boolean isRoboRio = controllerType.toUpperCase(Locale.ROOT).contains("ROBORIO");
    double defaultPeriod = isRoboRio ? 20.0 : 5.0;
    double loopPeriodMs =
        hasSnapshot && rawText.contains("loopPeriodMs=") ? parsed.loopPeriodMs() : defaultPeriod;
    double driveTraceMaxMs = Math.max(wpiDriveMaxMs, parsed.driveTraceMaxMs());
    double visionTraceMaxMs = Math.max(wpiVisionMaxMs, parsed.visionTraceMaxMs());
    double superTraceMaxMs = Math.max(wpiSuperMaxMs, parsed.superstructureTraceMaxMs());
    double loopMaxMs =
        hasSnapshot && rawText.contains("loopMaxMs=")
            ? parsed.loopMaxMs()
            : Math.max(wpiDriveMaxMs + wpiVisionMaxMs + wpiSuperMaxMs, 0.0);
    double cpuTempC = Math.max(wpiCpuTempC, parsed.cpuTempC());
    double cpuFreqMHz =
        hasSnapshot
            ? (rawText.contains("cpuFreqMHz=") ? parsed.cpuFreqMHz() : (isRoboRio ? 667.0 : 2400.0))
            : 0.0;

    FluxSystemSnapshot merged =
        hasSnapshot
            ? new FluxSystemSnapshot(
                controllerType,
                parsed.hostname(),
                parsed.kernelVersion(),
                parsed.uptimeSec(),
                loopPeriodMs,
                parsed.loopMedianMs(),
                parsed.loopP99Ms(),
                loopMaxMs,
                parsed.firstEnablePeakMs(),
                parsed.odometryJitterMaxMs(),
                driveTraceMaxMs,
                visionTraceMaxMs,
                superTraceMaxMs,
                parsed.totalCpuPct(),
                parsed.javaProcessCpuPct(),
                parsed.visionDaemonCpuPct(),
                parsed.softIrqPct(),
                parsed.hardIrqPct(),
                parsed.ioWaitPct(),
                parsed.spiCanIrqCount(),
                parsed.usbXhciIrqCount(),
                cpuTempC,
                cpuFreqMHz,
                parsed.throttledFlags(),
                parsed.heapAllocRateMbPerSec(),
                parsed.gcCount(),
                parsed.gcTotalPauseMs(),
                parsed.gcCorrelatedLoopSpikes(),
                parsed.freeStorageMb(),
                parsed.logWriteLatencyMaxMs(),
                parsed.ipc(),
                parsed.l1dCacheMissPct(),
                parsed.llcMissPct(),
                parsed.involuntaryContextSwitches(),
                parsed.canInterfaces())
            : new FluxSystemSnapshot(
                controllerType,
                "unknown",
                "unknown",
                0.0,
                loopPeriodMs,
                0.0,
                0.0,
                loopMaxMs,
                0.0,
                0.0,
                driveTraceMaxMs,
                visionTraceMaxMs,
                superTraceMaxMs,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0L,
                0L,
                cpuTempC,
                0.0,
                0,
                0.0,
                0,
                0.0,
                0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0L,
                Collections.emptyList());

    List<FluxSystemSnapshot.WatchDecision> rawDecisions =
        FluxSystemSnapshot.evaluate(merged, hasSnapshot, hasFreeStorageKey);
    List<SystemWatchDecision> decisions = new ArrayList<>();
    for (FluxSystemSnapshot.WatchDecision d : rawDecisions) {
      if (!"OK".equals(d.severity())) {
        decisions.add(
            new SystemWatchDecision(
                d.category(),
                d.metricName(),
                d.measuredValue(),
                d.threshold(),
                d.severity(),
                d.anomalySummary(),
                d.actionableDecision()));
      }
    }

    long totalCanDrops = 0L;
    int totalCanBusErrors = 0;
    for (FluxSystemSnapshot.CanInterfaceStats c : merged.canInterfaces()) {
      totalCanDrops += c.rxDropped() + c.txDropped() + c.rxOverErrors();
      totalCanBusErrors += c.busErrorCount() + c.busRestarts();
    }

    return new SystemHealthMetrics(
        merged.controllerType(),
        merged.loopPeriodMs(),
        merged.loopMedianMs(),
        merged.loopP99Ms(),
        merged.loopMaxMs(),
        merged.firstEnablePeakMs(),
        merged.totalCpuPct(),
        merged.javaProcessCpuPct(),
        merged.visionDaemonCpuPct(),
        merged.softIrqPct(),
        merged.cpuTempC(),
        merged.cpuFreqMHz(),
        merged.throttledFlags(),
        totalCanDrops,
        totalCanBusErrors,
        merged.heapAllocRateMbPerSec(),
        merged.gcCorrelatedLoopSpikes(),
        merged.freeStorageMb(),
        merged.ipc(),
        merged.llcMissPct(),
        Collections.unmodifiableList(decisions));
  }

  private static String inferControllerFromLogs(List<ParsedLog> allLogs) {
    for (ParsedLog log : allLogs) {
      for (String t : log.entriesByName().keySet()) {
        String lower = t.toLowerCase(Locale.ROOT);
        if (lower.contains("systemcore") || lower.contains("limelightsc")) {
          return "SYSTEMCORE";
        }
      }
    }
    return "ROBORIO";
  }

  private static Path findSnapshotFile(Path runDir) {
    if (runDir == null || !Files.isDirectory(runDir)) {
      return null;
    }
    Path exact = runDir.resolve("system-snapshot.txt");
    if (Files.isRegularFile(exact)) {
      return exact;
    }
    try (Stream<Path> s = Files.list(runDir)) {
      return s.filter(
              p -> {
                String n = p.getFileName().toString().toLowerCase(Locale.ROOT);
                return n.startsWith("system-snapshot") && n.endsWith(".txt");
              })
          .findFirst()
          .orElse(null);
    } catch (IOException e) {
      return null;
    }
  }

  private static double maxFromTopicSuffix(
      List<ParsedLog> allLogs, String suffix, long t0Us, long windowEndUs) {
    double max = 0.0;
    for (ParsedLog log : allLogs) {
      for (String topic : log.entriesByName().keySet()) {
        if (topic.endsWith(suffix)) {
          for (DoubleSample s : log.getDoubleSeries(topic)) {
            if (s.timestampUs() >= t0Us && s.timestampUs() <= windowEndUs && s.value() > max) {
              max = s.value();
            }
          }
        }
      }
    }
    return max;
  }
}
