package frc.flux.processor.analysis;

import frc.flux.processor.model.RunAnalysisReport.SystemHealthMetrics;
import frc.flux.processor.model.RunAnalysisReport.SystemWatchDecision;
import frc.flux.processor.wpilog.WpiLogReader.DoubleSample;
import frc.flux.processor.wpilog.WpiLogReader.ParsedLog;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.stream.Stream;

/**
 * Offline evaluator for SystemCore / roboRIO watch metrics combining {@code system-snapshot.txt}
 * (when pulled over SSH by {@code FluxTray.jar} or {@code pull-logs.bat}) with {@code .wpilog}
 * timing and {@code SystemStats} topics.
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

    // Also inspect .wpilog for Performance/*Ms and SystemStats/* if present
    double wpiDriveMaxMs = maxFromTopicSuffix(allLogs, "Performance/DriveMs", t0Us, windowEndUs);
    double wpiVisionMaxMs = maxFromTopicSuffix(allLogs, "Performance/VisionMs", t0Us, windowEndUs);
    double wpiSuperMaxMs =
        maxFromTopicSuffix(allLogs, "Performance/SuperstructureMs", t0Us, windowEndUs);
    double wpiCpuTempC = maxFromTopicSuffix(allLogs, "CPUTemp", t0Us, windowEndUs);

    boolean hasSnapshot = rawText != null && !rawText.isBlank();
    if (!hasSnapshot && wpiDriveMaxMs <= 0.0 && wpiVisionMaxMs <= 0.0 && wpiCpuTempC <= 0.0) {
      return null;
    }

    Map<String, String> kv = new LinkedHashMap<>();
    long totalCanDrops = 0L;
    int totalCanBusErrors = 0;
    boolean oddCanPortUsed = false;
    String oddCanPortName = "";
    boolean canBusOffOrPassive = false;
    String faultedCanBus = "";

    if (hasSnapshot) {
      for (String rawLine : rawText.split("\\R")) {
        String line = rawLine.trim();
        if (line.isEmpty() || line.startsWith("#") || line.startsWith("[")) {
          continue;
        }
        int eq = line.indexOf('=');
        if (eq <= 0) {
          continue;
        }
        String k = line.substring(0, eq).trim();
        String v = line.substring(eq + 1).trim();
        if (k.startsWith("can.")) {
          String iface = k.substring(4);
          String[] p = v.split(",", -1);
          if (p.length >= 8) {
            long rxPkts = parseLong(p[0], 0L);
            long txPkts = parseLong(p[1], 0L);
            long rxDrop = parseLong(p[2], 0L);
            long txDrop = parseLong(p[3], 0L);
            long rxOver = parseLong(p[4], 0L);
            int berr = (int) parseLong(p[5], 0L);
            int restarts = (int) parseLong(p[6], 0L);
            String state = p[7].trim();
            if (("can_s1".equalsIgnoreCase(iface) || "can_s3".equalsIgnoreCase(iface))
                && (rxPkts > 0 || txPkts > 0)) {
              oddCanPortUsed = true;
              oddCanPortName = iface;
            }
            totalCanDrops += rxDrop + txDrop + rxOver;
            totalCanBusErrors += berr + restarts;
            if (!"ERROR-ACTIVE".equalsIgnoreCase(state) && !"UNKNOWN".equalsIgnoreCase(state)) {
              canBusOffOrPassive = true;
              faultedCanBus = iface + " (" + state + ")";
            } else if ((berr > 0 || restarts > 0) && faultedCanBus.isEmpty()) {
              faultedCanBus = iface;
            }
          }
        } else {
          kv.put(k, v);
        }
      }
    }

    // Never invent synthetic SYSTEMCORE metrics when system-snapshot.txt is absent
    String defaultController = inferControllerFromLogs(allLogs);
    String controllerType =
        kv.getOrDefault(
            "controllerType", hasSnapshot ? "SYSTEMCORE" : defaultController + " (WPILOG-ONLY)");
    boolean isRoboRio = controllerType.toUpperCase(Locale.ROOT).contains("ROBORIO");
    double defaultPeriod = isRoboRio ? 20.0 : 5.0;
    double loopPeriodMs = parseDouble(kv.get("loopPeriodMs"), defaultPeriod);
    double loopMedianMs = parseDouble(kv.get("loopMedianMs"), hasSnapshot ? 1.5 : 0.0);
    double loopP99Ms = parseDouble(kv.get("loopP99Ms"), hasSnapshot ? 2.8 : 0.0);
    double loopMaxMs =
        parseDouble(
            kv.get("loopMaxMs"),
            hasSnapshot ? 4.1 : Math.max(wpiDriveMaxMs + wpiVisionMaxMs + wpiSuperMaxMs, 0.0));
    double firstEnablePeakMs = parseDouble(kv.get("firstEnablePeakMs"), hasSnapshot ? 3.9 : 0.0);
    double odometryJitterMaxMs =
        parseDouble(kv.get("odometryJitterMaxMs"), hasSnapshot ? 0.4 : 0.0);
    double driveTraceMaxMs =
        Math.max(wpiDriveMaxMs, parseDouble(kv.get("driveTraceMaxMs"), hasSnapshot ? 0.5 : 0.0));
    double visionTraceMaxMs =
        Math.max(wpiVisionMaxMs, parseDouble(kv.get("visionTraceMaxMs"), hasSnapshot ? 0.6 : 0.0));

    double totalCpuPct = parseDouble(kv.get("totalCpuPct"), 0.0);
    double javaCpuPct = parseDouble(kv.get("javaProcessCpuPct"), 0.0);
    double visionCpuPct = parseDouble(kv.get("visionDaemonCpuPct"), 0.0);
    double softIrqPct = parseDouble(kv.get("softIrqPct"), 0.0);
    double ioWaitPct = parseDouble(kv.get("ioWaitPct"), 0.0);
    double cpuTempC = Math.max(wpiCpuTempC, parseDouble(kv.get("cpuTempC"), 0.0));
    double cpuFreqMHz =
        parseDouble(kv.get("cpuFreqMHz"), hasSnapshot ? (isRoboRio ? 667.0 : 2400.0) : 0.0);
    int throttledFlags = parseHexOrInt(kv.get("throttledHex"), 0);
    double heapAllocMbPerSec = parseDouble(kv.get("heapAllocRateMbPerSec"), 0.0);
    int gcCorrelatedSpikes = (int) parseLong(kv.get("gcCorrelatedLoopSpikes"), 0L);
    double freeStorageMb = parseDouble(kv.get("freeStorageMb"), hasSnapshot ? 1024.0 : 0.0);
    double logWriteMaxMs = parseDouble(kv.get("logWriteLatencyMaxMs"), 0.0);
    double ipc = parseDouble(kv.get("ipc"), 0.0);
    double llcMissPct = parseDouble(kv.get("llcMissPct"), 0.0);
    long involCtxSw = parseLong(kv.get("involuntaryContextSwitches"), 0L);

    List<SystemWatchDecision> decisions = new ArrayList<>();

    // 1. Loop Period Jitter
    double p99Limit = loopPeriodMs * 1.30;
    if (loopP99Ms > p99Limit) {
      decisions.add(
          new SystemWatchDecision(
              "Loop & Timing",
              "Loop Period Jitter (p99 Δt)",
              String.format(Locale.US, "p99=%.2f ms (max=%.2f ms)", loopP99Ms, loopMaxMs),
              String.format(Locale.US, "<= %.2f ms", p99Limit),
              loopP99Ms > loopPeriodMs * 1.75 ? "CRITICAL" : "WARN",
              "Control loop p99 exceeded target period budget.",
              "Decision: Lower loopPeriodMs in parameters.csv (FluxFig) from 5 ms (200 Hz) to 10 ms (100 Hz) so PID discretize() dt stays deterministic."));
    }

    // 2. Subsystem Breakdown
    if (visionTraceMaxMs > 2.5 || driveTraceMaxMs > 2.0) {
      String culprit = visionTraceMaxMs >= driveTraceMaxMs ? "VisionMs" : "DriveMs";
      double peak = Math.max(visionTraceMaxMs, driveTraceMaxMs);
      decisions.add(
          new SystemWatchDecision(
              "Loop & Timing",
              "Subsystem Execution Spike (" + culprit + ")",
              String.format(
                  Locale.US, "Drive=%.2f ms, Vision=%.2f ms", driveTraceMaxMs, visionTraceMaxMs),
              "Drive <= 2.0 ms, Vision <= 2.5 ms",
              peak > 4.0 ? "CRITICAL" : "WARN",
              culprit + " spiked in LoggedTracer.",
              "VisionMs".equals(culprit)
                  ? "Decision: Cap per-loop camera queue drain or tighten AprilTag distance/ambiguity filtering in Vision.periodic()."
                  : "Decision: Move synchronous CTRE configurator .apply() calls out of Drive.periodic()."));
    }

    // 3. First-Enable Cold JIT Spike
    if (firstEnablePeakMs > Math.max(12.0, loopPeriodMs * 2.4)
        && loopMedianMs > 0.0
        && loopMedianMs < loopPeriodMs * 0.7) {
      decisions.add(
          new SystemWatchDecision(
              "Loop & Timing",
              "First-Enable Cold JIT Spike",
              String.format(
                  Locale.US, "1stEnable=%.2f ms vs p50=%.2f ms", firstEnablePeakMs, loopMedianMs),
              "<= 12.0 ms",
              "WARN",
              "Initial Autonomous loop spiked due to cold JVM C2 JIT compilation.",
              "Decision: Add dummy warmup iterations for PathPlanner, ShotCalculator, and ZeroPriorBootstrapSolver inside robotInit()."));
    }

    // 4. 250 Hz Odometry Jitter
    if (odometryJitterMaxMs > 1.5) {
      decisions.add(
          new SystemWatchDecision(
              "Loop & Timing",
              "250 Hz Odometry Thread Jitter",
              String.format(Locale.US, "±%.2f ms", odometryJitterMaxMs),
              "<= ±1.50 ms",
              odometryJitterMaxMs > 2.5 ? "CRITICAL" : "WARN",
              "250 Hz PhoenixOdometryThread experienced scheduling jitter.",
              "Decision: Pin PhoenixOdometryThread to Core 3 (taskset -c 3) away from USB/Network IRQs on Core 0."));
    }

    // 5. SocketCAN Odd Port / Drops / Bus Errors / softirq
    if (oddCanPortUsed) {
      decisions.add(
          new SystemWatchDecision(
              "SocketCAN & SPI",
              "Forbidden Odd CAN FD Port (" + oddCanPortName + ")",
              "Traffic detected on " + oddCanPortName,
              "0 packets on can_s1 / can_s3",
              "CRITICAL",
              oddCanPortName
                  + " shares an internal 12,500 FPS SPI bus with an even port (Issue #342).",
              "Decision: Move CAN harness to even port can_s0, can_s2, or can_s4 and update parameters.csv."));
    }
    if (totalCanDrops > 0) {
      decisions.add(
          new SystemWatchDecision(
              "SocketCAN & SPI",
              "SocketCAN RX/TX Drops & Overruns",
              totalCanDrops + " dropped/overrun frames",
              "0 drops",
              totalCanDrops > 10 ? "CRITICAL" : "WARN",
              "MCP2518FD SPI buffer overflowed.",
              "Decision: Reduce non-essential TalonFX status signal frequencies (motor temp 50 Hz -> 4 Hz) in PhoenixSignals."));
    }
    if (canBusOffOrPassive || totalCanBusErrors > 0) {
      decisions.add(
          new SystemWatchDecision(
              "SocketCAN & SPI",
              "CAN Error State / Bus-Off",
              String.format(
                  Locale.US, "%s (errors/restarts=%d)", faultedCanBus, totalCanBusErrors),
              "ERROR-ACTIVE, 0 errors",
              canBusOffOrPassive ? "CRITICAL" : "WARN",
              "Physical CAN FD frame errors or bus-off transition detected.",
              "Decision: Pit Electrical Work Order — inspect 120-Ohm termination resistors and CAN twisted-pair splices before next match (Issue #163)."));
    }
    if (softIrqPct > 25.0) {
      decisions.add(
          new SystemWatchDecision(
              "SocketCAN & SPI",
              "Kernel softirq Load",
              String.format(Locale.US, "%.1f%%", softIrqPct),
              "<= 25.0%",
              softIrqPct > 35.0 ? "CRITICAL" : "WARN",
              "SPI SocketCAN + USB UVC kernel interrupt handlers are saturating Core 0.",
              "Decision: Balance CAN status signal rates or lower USB camera capture FPS from 90 -> 60."));
    }

    // 6. CPU, Vision Daemon, Thermal & 5V Rail
    if (totalCpuPct > 80.0 && javaCpuPct < 30.0) {
      decisions.add(
          new SystemWatchDecision(
              "CPU, Vision & Thermal",
              "Java vs. Vision Daemon CPU Split",
              String.format(
                  Locale.US,
                  "Total=%.1f%% (Java=%.1f%%, Vision C++=%.1f%%)",
                  totalCpuPct,
                  javaCpuPct,
                  visionCpuPct),
              "Total CPU <= 80.0%",
              "WARN",
              "Native C++ limelightsc0..3 AprilTag daemons are saturating the Cortex-A76 cores.",
              "Decision: Enable 2x detector decimation during Teleop or reduce camera FPS in cameras.csv."));
    }
    if (cpuTempC >= 80.0 || (throttledFlags & 0x40004) != 0) {
      decisions.add(
          new SystemWatchDecision(
              "CPU, Vision & Thermal",
              "CPU Die Temp & CM5 Thermal Throttling",
              String.format(
                  Locale.US, "%.1f °C @ %.0f MHz (0x%x)", cpuTempC, cpuFreqMHz, throttledFlags),
              "< 80.0 °C, 0x0",
              cpuTempC >= 85.0 ? "CRITICAL" : "WARN",
              "CM5 SoC reached thermal throttle threshold inside electronics bay.",
              "Decision: Add a 5V pit cooling fan over SystemCore heatsink or throttle idle camera processing while Disabled."));
    }
    if ((throttledFlags & 0x10001) != 0) {
      decisions.add(
          new SystemWatchDecision(
              "CPU, Vision & Thermal",
              "CM5 5V Rail Undervoltage Bit",
              String.format(Locale.US, "throttled=0x%x", throttledFlags),
              "0x0",
              (throttledFlags & 0x1) != 0 ? "CRITICAL" : "WARN",
              "SystemCore internal 5V rail drooped (distinct from 12V battery sag).",
              "Decision: Inspect USB 3.0 camera 5V current draw and 3.3V/5V SmartIO sensors for shorts."));
    }

    // 7. JVM Heap & GC
    if (heapAllocMbPerSec > 2.0) {
      decisions.add(
          new SystemWatchDecision(
              "JVM Memory & GC",
              "Heap Sawtooth Allocation Rate",
              String.format(Locale.US, "%.2f MB/s", heapAllocMbPerSec),
              "<= 2.00 MB/s",
              heapAllocMbPerSec > 5.0 ? "CRITICAL" : "WARN",
              "High per-loop object allocation rate while Enabled.",
              "Decision: Audit student commands for per-loop Stream, Optional, String concatenation, or array allocations."));
    }
    if (gcCorrelatedSpikes > 0) {
      decisions.add(
          new SystemWatchDecision(
              "JVM Memory & GC",
              "GC-Correlated Loop Overruns",
              gcCorrelatedSpikes + " overruns",
              "0 GC overruns",
              gcCorrelatedSpikes >= 3 ? "CRITICAL" : "WARN",
              "Garbage Collection pauses directly caused control loop overruns.",
              "Decision: Update JVM launch flags in build.gradle to -XX:+UseZGC -XX:+AlwaysPreTouch -Xms512m -Xmx512m."));
    }

    // 8. Storage & Hardware PMU (only evaluate when snapshot is present)
    if (hasSnapshot && (freeStorageMb < 400.0 || ioWaitPct > 5.0 || logWriteMaxMs > 25.0)) {
      decisions.add(
          new SystemWatchDecision(
              "Storage & eMMC I/O",
              "eMMC Free Space & Log I/O Wait",
              String.format(
                  Locale.US, "Free=%.0f MB, iowait=%.1f%%", freeStorageMb, ioWaitPct),
              "Free >= 400 MB, iowait <= 5.0%",
              freeStorageMb < 150.0 ? "CRITICAL" : "WARN",
              "eMMC storage pressure or disk write latency spikes detected.",
              "Decision: Switch camera recording to the Hybrid Event-Driven preset (~4.5 MB/match) and clean old logs."));
    }
    if (hasSnapshot && ((ipc > 0.0 && ipc < 0.80) || llcMissPct > 15.0 || involCtxSw > 500L)) {
      decisions.add(
          new SystemWatchDecision(
              "ARMv8 Hardware PMU",
              "Cortex-A76 IPC & L3 Cache Misses",
              String.format(
                  Locale.US, "IPC=%.2f, LLC Miss=%.1f%%, CtxSw=%d", ipc, llcMissPct, involCtxSw),
              "IPC >= 0.80, LLC Miss <= 15.0%",
              "WARN",
              "4x OV9281 camera pipelines are thrashing the shared 2 MB L3 cache.",
              "Decision: Isolate Core 3 (isolcpus=3 / taskset) for Java + 250 Hz odometry and restrict limelightsc0..3 daemons to Cores 0-2."));
    }

    return new SystemHealthMetrics(
        controllerType,
        loopPeriodMs,
        loopMedianMs,
        loopP99Ms,
        loopMaxMs,
        firstEnablePeakMs,
        totalCpuPct,
        javaCpuPct,
        visionCpuPct,
        softIrqPct,
        cpuTempC,
        cpuFreqMHz,
        throttledFlags,
        totalCanDrops,
        totalCanBusErrors,
        heapAllocMbPerSec,
        gcCorrelatedSpikes,
        freeStorageMb,
        ipc,
        llcMissPct,
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

  private static double parseDouble(String s, double def) {
    if (s == null || s.isEmpty()) {
      return def;
    }
    try {
      return Double.parseDouble(s.trim());
    } catch (NumberFormatException e) {
      return def;
    }
  }

  private static long parseLong(String s, long def) {
    if (s == null || s.isEmpty()) {
      return def;
    }
    try {
      return Long.parseLong(s.trim());
    } catch (NumberFormatException e) {
      return def;
    }
  }

  private static int parseHexOrInt(String s, int def) {
    if (s == null || s.isEmpty()) {
      return def;
    }
    String clean = s.trim().toLowerCase(Locale.ROOT);
    try {
      if (clean.startsWith("0x")) {
        return Integer.parseUnsignedInt(clean.substring(2), 16);
      }
      return Integer.parseInt(clean);
    } catch (NumberFormatException e) {
      return def;
    }
  }
}
