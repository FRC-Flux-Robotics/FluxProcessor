package frc.flux.processor.analysis;

import frc.flux.processor.model.RunAnalysisReport.SystemHealthMetrics;
import frc.flux.processor.model.RunAnalysisReport.SystemWatchDecision;
import frc.lib.fluxfig.FluxSystemSnapshot;
import frc.lib.fluxfig.FluxWpiLogCodec.DoubleSample;
import frc.lib.fluxfig.FluxWpiLogCodec.ParsedLog;
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
 * timing ({@code Performance/TotalLoopMs}, {@code Performance/PhoenixSignalsMs}, {@code
 * Performance/CommandSchedulerMs}, {@code Performance/RobotContainerMs}, plus SystemCore {@code
 * Performance/{Drive,Vision,Superstructure}Ms}) and {@code SystemStats} topics via the shared
 * {@link FluxSystemSnapshot} engine in {@code FluxFig}.
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

    List<Double> totalLoopSamples =
        samplesFromTopicSuffix(allLogs, "Performance/TotalLoopMs", t0Us, windowEndUs);
    double wpiLoopMedianMs = percentile(totalLoopSamples, 0.50);
    double wpiLoopP99Ms = percentile(totalLoopSamples, 0.99);
    double wpiTotalLoopMaxMs = maxOf(totalLoopSamples);
    double wpiFirstEnablePeakMs =
        firstEnableUs > 0L
            ? maxFromTopicSuffix(
                allLogs, "Performance/TotalLoopMs", firstEnableUs, firstEnableUs + 500_000L)
            : 0.0;

    double wpiPhoenixSignalsMaxMs =
        maxFromTopicSuffix(allLogs, "Performance/PhoenixSignalsMs", t0Us, windowEndUs);
    double wpiCommandSchedulerMaxMs =
        maxFromTopicSuffix(allLogs, "Performance/CommandSchedulerMs", t0Us, windowEndUs);
    double wpiRobotContainerMaxMs =
        maxFromTopicSuffix(allLogs, "Performance/RobotContainerMs", t0Us, windowEndUs);

    double wpiDriveMaxMs = maxFromTopicSuffix(allLogs, "Performance/DriveMs", t0Us, windowEndUs);
    double wpiVisionMaxMs = maxFromTopicSuffix(allLogs, "Performance/VisionMs", t0Us, windowEndUs);
    double wpiSuperMaxMs =
        maxFromTopicSuffix(allLogs, "Performance/SuperstructureMs", t0Us, windowEndUs);
    double wpiCpuTempC = maxFromTopicSuffix(allLogs, "CPUTemp", t0Us, windowEndUs);

    boolean hasSnapshot = rawText != null && !rawText.isBlank();
    boolean hasWpiTiming =
        Double.isFinite(wpiTotalLoopMaxMs)
            || wpiPhoenixSignalsMaxMs > 0.0
            || wpiCommandSchedulerMaxMs > 0.0
            || wpiRobotContainerMaxMs > 0.0
            || wpiDriveMaxMs > 0.0
            || wpiVisionMaxMs > 0.0
            || wpiSuperMaxMs > 0.0
            || wpiCpuTempC > 0.0;
    if (!hasSnapshot && !hasWpiTiming) {
      return null;
    }

    FluxSystemSnapshot parsed = FluxSystemSnapshot.parse(hasSnapshot ? rawText : "");
    boolean hasFreeStorageKey =
        hasSnapshot
            && (Double.isFinite(parsed.freeStorageMb())
                || Double.isFinite(parsed.internalFreeStorageMb())
                || Double.isFinite(parsed.usbFreeStorageMb()));

    String defaultController = inferControllerFromLogs(allLogs);
    String controllerType =
        hasSnapshot && !"UNKNOWN".equalsIgnoreCase(parsed.controllerType())
            ? parsed.controllerType()
            : defaultController + (hasSnapshot ? "" : " (WPILOG-ONLY)");
    boolean isRoboRio = controllerType.toUpperCase(Locale.ROOT).contains("ROBORIO");
    double defaultPeriod = isRoboRio ? 20.0 : 5.0;
    double loopPeriodMs =
        hasSnapshot && Double.isFinite(parsed.loopPeriodMs())
            ? parsed.loopPeriodMs()
            : defaultPeriod;
    double driveTraceMaxMs = finitePositiveMax(wpiDriveMaxMs, parsed.driveTraceMaxMs());
    double visionTraceMaxMs = finitePositiveMax(wpiVisionMaxMs, parsed.visionTraceMaxMs());
    double superTraceMaxMs = finitePositiveMax(wpiSuperMaxMs, parsed.superstructureTraceMaxMs());
    double phoenixSignalsTraceMaxMs =
        finitePositiveMax(wpiPhoenixSignalsMaxMs, parsed.phoenixSignalsTraceMaxMs());
    double commandSchedulerTraceMaxMs =
        finitePositiveMax(wpiCommandSchedulerMaxMs, parsed.commandSchedulerTraceMaxMs());
    double robotContainerTraceMaxMs =
        finitePositiveMax(wpiRobotContainerMaxMs, parsed.robotContainerTraceMaxMs());
    double wpiLoopSum =
        (wpiDriveMaxMs > 0.0 ? wpiDriveMaxMs : 0.0)
            + (wpiVisionMaxMs > 0.0 ? wpiVisionMaxMs : 0.0)
            + (wpiSuperMaxMs > 0.0 ? wpiSuperMaxMs : 0.0);
    double wpiRioTraceSum =
        (wpiPhoenixSignalsMaxMs > 0.0 ? wpiPhoenixSignalsMaxMs : 0.0)
            + (wpiCommandSchedulerMaxMs > 0.0 ? wpiCommandSchedulerMaxMs : 0.0)
            + (wpiRobotContainerMaxMs > 0.0 ? wpiRobotContainerMaxMs : 0.0);
    double fallbackWpiLoopMax =
        Double.isFinite(wpiTotalLoopMaxMs)
            ? wpiTotalLoopMaxMs
            : (wpiRioTraceSum > 0.0
                ? wpiRioTraceSum
                : (wpiLoopSum > 0.0 ? wpiLoopSum : Double.NaN));
    double loopMedianMs = finitePositiveMax(wpiLoopMedianMs, parsed.loopMedianMs());
    double loopP99Ms = finitePositiveMax(wpiLoopP99Ms, parsed.loopP99Ms());
    double loopMaxMs = finitePositiveMax(fallbackWpiLoopMax, parsed.loopMaxMs());
    double firstEnablePeakMs = finitePositiveMax(wpiFirstEnablePeakMs, parsed.firstEnablePeakMs());
    double cpuTempC = finitePositiveMax(wpiCpuTempC, parsed.cpuTempC());
    double cpuFreqMHz = hasSnapshot ? parsed.cpuFreqMHz() : Double.NaN;

    FluxSystemSnapshot merged =
        hasSnapshot
            ? new FluxSystemSnapshot(
                controllerType,
                parsed.hostname(),
                parsed.kernelVersion(),
                parsed.uptimeSec(),
                loopPeriodMs,
                loopMedianMs,
                loopP99Ms,
                loopMaxMs,
                firstEnablePeakMs,
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
                parsed.canInterfaces(),
                phoenixSignalsTraceMaxMs,
                commandSchedulerTraceMaxMs,
                robotContainerTraceMaxMs,
                parsed.internalFreeStorageMb(),
                parsed.usbFreeStorageMb())
            : new FluxSystemSnapshot(
                controllerType,
                "NOT MEASURED",
                "NOT MEASURED",
                Double.NaN,
                loopPeriodMs,
                loopMedianMs,
                loopP99Ms,
                loopMaxMs,
                firstEnablePeakMs,
                Double.NaN,
                driveTraceMaxMs,
                visionTraceMaxMs,
                superTraceMaxMs,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                -1L,
                -1L,
                cpuTempC,
                Double.NaN,
                -1,
                Double.NaN,
                -1,
                Double.NaN,
                -1,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                -1L,
                Collections.emptyList(),
                phoenixSignalsTraceMaxMs,
                commandSchedulerTraceMaxMs,
                robotContainerTraceMaxMs,
                Double.NaN,
                Double.NaN);

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

  private static double finitePositiveMax(double wpiVal, double snapVal) {
    boolean hasWpi = Double.isFinite(wpiVal) && wpiVal > 0.0;
    boolean hasSnap = Double.isFinite(snapVal);
    if (hasWpi && hasSnap) {
      return Math.max(wpiVal, snapVal);
    }
    if (hasWpi) {
      return wpiVal;
    }
    if (hasSnap) {
      return snapVal;
    }
    return Double.NaN;
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

  private static List<Double> samplesFromTopicSuffix(
      List<ParsedLog> allLogs, String suffix, long t0Us, long windowEndUs) {
    List<Double> values = new ArrayList<>();
    for (ParsedLog log : allLogs) {
      for (String topic : log.entriesByName().keySet()) {
        if (topic.endsWith(suffix)) {
          for (DoubleSample s : log.getDoubleSeries(topic)) {
            if (s.timestampUs() >= t0Us
                && s.timestampUs() <= windowEndUs
                && Double.isFinite(s.value())
                && s.value() >= 0.0) {
              values.add(s.value());
            }
          }
        }
      }
    }
    Collections.sort(values);
    return values;
  }

  private static double percentile(List<Double> sortedValues, double p) {
    if (sortedValues.isEmpty()) {
      return Double.NaN;
    }
    int idx = (int) Math.ceil(p * sortedValues.size()) - 1;
    idx = Math.max(0, Math.min(sortedValues.size() - 1, idx));
    return sortedValues.get(idx);
  }

  private static double maxOf(List<Double> sortedValues) {
    return sortedValues.isEmpty() ? Double.NaN : sortedValues.get(sortedValues.size() - 1);
  }

  private static double maxFromTopicSuffix(
      List<ParsedLog> allLogs, String suffix, long t0Us, long windowEndUs) {
    double max = 0.0;
    for (ParsedLog log : allLogs) {
      for (String topic : log.entriesByName().keySet()) {
        if (topic.endsWith(suffix)) {
          for (DoubleSample s : log.getDoubleSeries(topic)) {
            if (s.timestampUs() >= t0Us
                && s.timestampUs() <= windowEndUs
                && Double.isFinite(s.value())
                && s.value() > max) {
              max = s.value();
            }
          }
        }
      }
    }
    return max;
  }
}
