package frc.flux.processor.mcp;

import frc.flux.processor.analysis.DigitalTwinCalibrator;
import frc.flux.processor.analysis.RunAnalyzer;
import frc.flux.processor.hoot.HootConverter;
import frc.flux.processor.model.RobotProfile;
import frc.flux.processor.model.RunAnalysisReport;
import frc.flux.processor.model.RunAnalysisReport.CameraSnapshotRef;
import frc.flux.processor.model.RunAnalysisReport.TimelinePoint;
import frc.flux.processor.model.RunAnalysisReport.VoiceCorrelation;
import frc.flux.processor.wpilog.WpiLogReader;
import frc.flux.processor.wpilog.WpiLogReader.DoubleSample;
import frc.flux.processor.wpilog.WpiLogReader.EntryDef;
import frc.flux.processor.wpilog.WpiLogReader.ParsedLog;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Stream;

/**
 * Zero-dependency JSON-RPC 2.0 Model Context Protocol (MCP) server ({@code --mcp [log-dir]},
 * PLAN-2027 Section 6.B Part 2).
 *
 * <p>Exposes 8 multimodal telemetry, voice, and camera/video tools over stdio so coding agents
 * (Claude Code, Gemini, Cursor) can inspect {@code .wpilog} files, correlate voice notes, and audit
 * robot runs without writing custom Python scripts.
 */
public final class FluxMcpServer {

  private final Path rootDir;
  private final RobotProfile defaultProfile;
  private final Map<String, Path> runDirsById = new LinkedHashMap<>();
  private final Map<String, RunAnalysisReport> reportCache = new LinkedHashMap<>();
  private final Map<String, List<ParsedLog>> logCache = new LinkedHashMap<>();

  public FluxMcpServer(Path rootDir, RobotProfile defaultProfile) throws IOException {
    this.rootDir = rootDir != null ? rootDir : Path.of(".");
    this.defaultProfile = defaultProfile != null ? defaultProfile : RobotProfile.AUTO;
    refreshRuns();
  }

  public void refreshRuns() throws IOException {
    runDirsById.clear();
    if (!Files.isDirectory(rootDir)) {
      return;
    }
    if (hasWpiLogs(rootDir)) {
      String id =
          rootDir.getFileName() != null ? rootDir.getFileName().toString() : "current-run";
      runDirsById.put(id, rootDir);
      return;
    }
    try (Stream<Path> s = Files.list(rootDir)) {
      for (Path sub :
          s.filter(Files::isDirectory)
              .sorted(Comparator.comparing(p -> p.getFileName().toString()))
              .toList()) {
        if (hasWpiLogs(sub)) {
          runDirsById.put(sub.getFileName().toString(), sub);
        }
      }
    }
  }

  private static boolean hasWpiLogs(Path dir) throws IOException {
    try (Stream<Path> s = Files.list(dir)) {
      return s.anyMatch(
          p -> {
            String n = p.getFileName().toString().toLowerCase(Locale.ROOT);
            return (n.endsWith(".wpilog") && !n.equals("merged.wpilog")) || n.endsWith(".hoot");
          });
    }
  }

  /** Starts the blocking stdio JSON-RPC 2.0 loop. */
  public void serveStdio(InputStream in, OutputStream out) throws IOException {
    BufferedReader reader = new BufferedReader(new InputStreamReader(in, StandardCharsets.UTF_8));
    PrintWriter writer = new PrintWriter(out, true, StandardCharsets.UTF_8);
    String line;
    while ((line = reader.readLine()) != null) {
      String trimmed = line.trim();
      if (trimmed.isEmpty() || trimmed.startsWith("Content-Length:")) {
        continue;
      }
      String response = handleRequest(trimmed);
      if (response != null && !response.isBlank()) {
        writer.println(response);
        writer.flush();
      }
    }
  }

  /** Processes a single JSON-RPC 2.0 request line and returns the JSON-RPC 2.0 response string. */
  public String handleRequest(String requestJson) {
    String idRaw = extractJsonId(requestJson);
    String method = RunAnalyzer.extractJsonString(requestJson, "method", "");

    if (method.isEmpty()) {
      return buildError(idRaw, -32600, "Invalid JSON-RPC request: missing method");
    }

    // Notifications have no id and require no response
    if (method.startsWith("notifications/")) {
      return "";
    }

    try {
      return switch (method) {
        case "initialize" ->
            buildResult(
                idRaw,
                """
                {"protocolVersion":"2024-11-05","serverInfo":{"name":"flux-processor-mcp","version":"2.0.0"},"capabilities":{"tools":{}}}""");
        case "ping" -> buildResult(idRaw, "{}");
        case "tools/list" -> buildResult(idRaw, buildToolsListJson());
        case "tools/call" -> handleToolCall(idRaw, requestJson);
        default -> buildError(idRaw, -32601, "Method not found: " + method);
      };
    } catch (Exception e) {
      return buildError(idRaw, -32000, "Server error: " + e.getMessage());
    }
  }

  private String handleToolCall(String idRaw, String requestJson) throws Exception {
    String paramsBlock = extractParamsBlock(requestJson);
    String toolName = RunAnalyzer.extractJsonString(paramsBlock, "name", "");
    String argsBlock = extractArgumentsBlock(paramsBlock);

    String textResult =
        switch (toolName) {
          case "list_entries" -> toolListEntries(argsBlock);
          case "find_condition" -> toolFindCondition(argsBlock);
          case "time_correlate" -> toolTimeCorrelate(argsBlock);
          case "compare_runs" -> toolCompareRuns(argsBlock);
          case "moi_regression" -> toolMoiRegression(argsBlock);
          case "search_voice_notes" -> toolSearchVoiceNotes(argsBlock);
          case "get_camera_snapshots" -> toolGetCameraSnapshots(argsBlock);
          case "get_video_frames" -> toolGetVideoFrames(argsBlock);
          default -> throw new IllegalArgumentException("Unknown MCP tool: " + toolName);
        };

    String resultObj =
        "{\"content\":[{\"type\":\"text\",\"text\":\""
            + escJson(textResult)
            + "\"}],\"isError\":false}";
    return buildResult(idRaw, resultObj);
  }

  private String toolListEntries(String argsJson) throws Exception {
    String runId = resolveRunId(RunAnalyzer.extractJsonString(argsJson, "runId", ""));
    List<ParsedLog> logs = getLogsForRun(runId);
    StringBuilder sb = new StringBuilder();
    sb.append("{\"runId\":\"").append(escJson(runId)).append("\",\"entries\":[");
    boolean first = true;
    for (ParsedLog log : logs) {
      String file = log.sourcePath().getFileName().toString();
      for (EntryDef e : log.entriesByName().values()) {
        int samples =
            log.getDoubleSeries(e.name()).size()
                + log.getBooleanSeries(e.name()).size()
                + log.getStringSeries(e.name()).size();
        if (!first) {
          sb.append(",");
        }
        first = false;
        sb.append(
            String.format(
                Locale.US,
                "{\"file\":\"%s\",\"name\":\"%s\",\"type\":\"%s\",\"samples\":%d}",
                escJson(file),
                escJson(e.name()),
                escJson(e.type()),
                samples));
      }
    }
    sb.append("]}");
    return sb.toString();
  }

  private String toolFindCondition(String argsJson) throws Exception {
    String runId = resolveRunId(RunAnalyzer.extractJsonString(argsJson, "runId", ""));
    String topic = RunAnalyzer.extractJsonString(argsJson, "topic", "/SystemStats/BatteryVoltage");
    String op = RunAnalyzer.extractJsonString(argsJson, "operator", "<");
    double threshold =
        parseDouble(RunAnalyzer.extractJsonString(argsJson, "threshold", "7.0"), 7.0);

    List<ParsedLog> logs = getLogsForRun(runId);
    List<DoubleSample> series = RunAnalyzer.findDoubleSeriesByTopic(logs, null, topic);
    if (series.isEmpty()) {
      // Fallback to timeline batteryVoltage if topic is BatteryVoltage
      RunAnalysisReport rep = getReportForRun(runId);
      int hits = 0;
      double firstSec = -1.0;
      double worstVal = Double.NaN;
      for (TimelinePoint pt : rep.timeline()) {
        double val =
            topic.toLowerCase(Locale.ROOT).contains("stator")
                ? pt.totalDriveStatorAmps()
                : pt.batteryVoltage();
        if (matchesOp(val, op, threshold)) {
          hits++;
          if (firstSec < 0) {
            firstSec = pt.timeSec();
          }
          worstVal = Double.isNaN(worstVal) ? val : (op.startsWith("<") ? Math.min(worstVal, val) : Math.max(worstVal, val));
        }
      }
      return String.format(
          Locale.US,
          "{\"runId\":\"%s\",\"topic\":\"%s\",\"operator\":\"%s\",\"threshold\":%.3f,\"matchingSamples\":%d,\"firstMatchSec\":%.2f,\"extremeValue\":%.3f}",
          escJson(runId),
          escJson(topic),
          escJson(op),
          threshold,
          hits,
          firstSec,
          Double.isNaN(worstVal) ? 0.0 : worstVal);
    }

    long t0Us = logs.get(0).minTimestampUs();
    int hits = 0;
    double firstSec = -1.0;
    double extreme = Double.NaN;
    for (DoubleSample s : series) {
      if (matchesOp(s.value(), op, threshold)) {
        hits++;
        double tSec = Math.max(0.0, (s.timestampUs() - t0Us) / 1_000_000.0);
        if (firstSec < 0) {
          firstSec = tSec;
        }
        extreme =
            Double.isNaN(extreme)
                ? s.value()
                : (op.startsWith("<") ? Math.min(extreme, s.value()) : Math.max(extreme, s.value()));
      }
    }
    return String.format(
        Locale.US,
        "{\"runId\":\"%s\",\"topic\":\"%s\",\"operator\":\"%s\",\"threshold\":%.3f,\"matchingSamples\":%d,\"firstMatchSec\":%.2f,\"extremeValue\":%.3f}",
        escJson(runId),
        escJson(topic),
        escJson(op),
        threshold,
        hits,
        firstSec,
        Double.isNaN(extreme) ? 0.0 : extreme);
  }

  private String toolTimeCorrelate(String argsJson) throws Exception {
    String runId = resolveRunId(RunAnalyzer.extractJsonString(argsJson, "runId", ""));
    double centerSec =
        parseDouble(RunAnalyzer.extractJsonString(argsJson, "timestampSec", "5.0"), 5.0);
    double windowSec =
        parseDouble(RunAnalyzer.extractJsonString(argsJson, "windowSec", "3.0"), 3.0);

    RunAnalysisReport rep = getReportForRun(runId);
    double tMin = Math.max(0.0, centerSec - windowSec);
    double tMax = centerSec + windowSec;

    double minV = 14.0;
    double peakDriveStator = 0.0;
    double peakSpeed = 0.0;
    for (TimelinePoint pt : rep.timeline()) {
      if (pt.timeSec() >= tMin && pt.timeSec() <= tMax) {
        minV = Math.min(minV, pt.batteryVoltage());
        peakDriveStator = Math.max(peakDriveStator, pt.totalDriveStatorAmps());
        peakSpeed = Math.max(peakSpeed, pt.chassisSpeedMps());
      }
    }
    if (minV == 14.0) {
      minV = rep.battery().minVoltage();
    }
    return String.format(
        Locale.US,
        "{\"runId\":\"%s\",\"windowStartSec\":%.2f,\"windowEndSec\":%.2f,\"minBatteryVoltage\":%.2f,\"peakDriveStatorAmps\":%.1f,\"peakChassisSpeedMps\":%.2f,\"verdict\":\"%s\"}",
        escJson(runId),
        tMin,
        tMax,
        minV,
        peakDriveStator,
        peakSpeed,
        escJson(rep.overallVerdict()));
  }

  private String toolCompareRuns(String argsJson) throws Exception {
    List<String> ids = new ArrayList<>(runDirsById.keySet());
    String runA =
        RunAnalyzer.extractJsonString(argsJson, "runA", !ids.isEmpty() ? ids.get(0) : "");
    String runB =
        RunAnalyzer.extractJsonString(
            argsJson, "runB", ids.size() > 1 ? ids.get(1) : runA);
    RunAnalysisReport repA = getReportForRun(resolveRunId(runA));
    RunAnalysisReport repB = getReportForRun(resolveRunId(runB));

    return String.format(
        Locale.US,
        "{\"runA\":{\"id\":\"%s\",\"battery\":\"%s\",\"rIntMilliOhms\":%.1f,\"minVoltage\":%.2f,\"peakStatorAmps\":%.0f,\"shots\":%d,\"elevatorCycles\":%d,\"verdict\":\"%s\"},"
            + "\"runB\":{\"id\":\"%s\",\"battery\":\"%s\",\"rIntMilliOhms\":%.1f,\"minVoltage\":%.2f,\"peakStatorAmps\":%.0f,\"shots\":%d,\"elevatorCycles\":%d,\"verdict\":\"%s\"},"
            + "\"deltaMinVoltage\":%.2f,\"deltaRintMilliOhms\":%.1f}",
        escJson(repA.runId()),
        escJson(repA.battery().batteryName()),
        repA.battery().estimatedRintMilliOhms(),
        repA.battery().minVoltage(),
        repA.battery().peakDriveStatorAmps(),
        repA.shots().size(),
        repA.elevatorCycles().size(),
        escJson(repA.overallVerdict()),
        escJson(repB.runId()),
        escJson(repB.battery().batteryName()),
        repB.battery().estimatedRintMilliOhms(),
        repB.battery().minVoltage(),
        repB.battery().peakDriveStatorAmps(),
        repB.shots().size(),
        repB.elevatorCycles().size(),
        escJson(repB.overallVerdict()),
        repB.battery().minVoltage() - repA.battery().minVoltage(),
        repB.battery().estimatedRintMilliOhms() - repA.battery().estimatedRintMilliOhms());
  }

  private String toolMoiRegression(String argsJson) throws Exception {
    String runId = resolveRunId(RunAnalyzer.extractJsonString(argsJson, "runId", ""));
    RunAnalysisReport rep = getReportForRun(runId);
    Path runDir = runDirsById.getOrDefault(runId, rootDir);
    DigitalTwinCalibrator.SimCalibrationSummary summary =
        DigitalTwinCalibrator.calibrateAndWrite(
            List.of(rep), List.of(runDir), null, runDir, rep.robotName());
    return String.format(
        Locale.US,
        "{\"runId\":\"%s\",\"profile\":\"%s\",\"batteryRintMilliOhms\":%.1f,\"batteryRSquared\":%.3f,\"wheelRadiusMeters\":%.5f,\"swerveCouplingRatio\":%.4f,\"calibratedMechanisms\":%d}",
        escJson(runId),
        escJson(summary.targetProfile()),
        rep.battery().estimatedRintMilliOhms(),
        rep.battery().rSquared(),
        summary.calibratedWheelRadiusMeters(),
        summary.calibratedCouplingRatio(),
        summary.calibratedMechanisms().size());
  }

  private String toolSearchVoiceNotes(String argsJson) throws Exception {
    String query =
        RunAnalyzer.extractJsonString(argsJson, "query", "").toLowerCase(Locale.ROOT);
    String roleFilter =
        RunAnalyzer.extractJsonString(argsJson, "role", "").toLowerCase(Locale.ROOT);

    StringBuilder sb = new StringBuilder();
    sb.append("{\"matches\":[");
    boolean first = true;
    for (String runId : runDirsById.keySet()) {
      RunAnalysisReport rep = getReportForRun(runId);
      for (VoiceCorrelation vc : rep.voiceCorrelations()) {
        boolean matchQuery =
            query.isEmpty()
                || vc.transcript().toLowerCase(Locale.ROOT).contains(query)
                || vc.sensorDiagnosis().toLowerCase(Locale.ROOT).contains(query)
                || vc.correlatedEventSummary().toLowerCase(Locale.ROOT).contains(query);
        boolean matchRole =
            roleFilter.isEmpty()
                || vc.speakerRole().toLowerCase(Locale.ROOT).contains(roleFilter);
        if (matchQuery && matchRole) {
          if (!first) {
            sb.append(",");
          }
          first = false;
          sb.append(
              String.format(
                  Locale.US,
                  "{\"runId\":\"%s\",\"role\":\"%s\",\"spokenTimeSec\":%.1f,\"transcript\":\"%s\",\"event\":\"%s\",\"diagnosis\":\"%s\",\"videoLink\":\"%s\"}",
                  escJson(runId),
                  escJson(vc.speakerRole()),
                  vc.spokenTimeSec(),
                  escJson(vc.transcript()),
                  escJson(vc.correlatedEventSummary()),
                  escJson(vc.sensorDiagnosis()),
                  escJson(vc.videoLink())));
        }
      }
    }
    sb.append("]}");
    return sb.toString();
  }

  private String toolGetCameraSnapshots(String argsJson) throws Exception {
    String runId = resolveRunId(RunAnalyzer.extractJsonString(argsJson, "runId", ""));
    String reasonFilter =
        RunAnalyzer.extractJsonString(argsJson, "triggerReason", "").toUpperCase(Locale.ROOT);
    RunAnalysisReport rep = getReportForRun(runId);

    StringBuilder sb = new StringBuilder();
    sb.append("{\"runId\":\"").append(escJson(runId)).append("\",\"snapshots\":[");
    boolean first = true;
    for (CameraSnapshotRef snap : rep.cameraSnapshots()) {
      if (!reasonFilter.isEmpty() && !snap.triggerReason().contains(reasonFilter)) {
        continue;
      }
      if (!first) {
        sb.append(",");
      }
      first = false;
      sb.append(
          String.format(
              Locale.US,
              "{\"fileName\":\"%s\",\"camera\":\"%s\",\"timestampSec\":%.2f,\"triggerReason\":\"%s\",\"relativePath\":\"%s\"}",
              escJson(snap.fileName()),
              escJson(snap.cameraName()),
              snap.timestampSec(),
              escJson(snap.triggerReason()),
              escJson(snap.relativePath())));
    }
    sb.append("]}");
    return sb.toString();
  }

  private String toolGetVideoFrames(String argsJson) throws Exception {
    String runId = resolveRunId(RunAnalyzer.extractJsonString(argsJson, "runId", ""));
    double eventSec =
        parseDouble(RunAnalyzer.extractJsonString(argsJson, "timestampSec", "10.0"), 10.0);
    RunAnalysisReport rep = getReportForRun(runId);
    double clipStartSec = Math.max(0.0, rep.videoOffsetSeconds() + eventSec - 2.0);
    double clipEndSec = Math.max(clipStartSec + 1.0, rep.videoOffsetSeconds() + eventSec + 1.0);
    String jumpLink =
        RunAnalyzer.buildVideoJumpLink(rep.videoUrl(), rep.videoOffsetSeconds(), eventSec);

    return String.format(
        Locale.US,
        "{\"runId\":\"%s\",\"eventTimestampSec\":%.2f,\"videoUrl\":\"%s\",\"clipWindowStartSec\":%.2f,\"clipWindowEndSec\":%.2f,\"jumpLink\":\"%s\"}",
        escJson(runId),
        eventSec,
        escJson(rep.videoUrl()),
        clipStartSec,
        clipEndSec,
        escJson(jumpLink));
  }

  private String resolveRunId(String requested) {
    if (requested != null && !requested.isBlank() && runDirsById.containsKey(requested)) {
      return requested;
    }
    if (!runDirsById.isEmpty()) {
      return runDirsById.keySet().iterator().next();
    }
    return requested != null && !requested.isBlank() ? requested : "default";
  }

  private RunAnalysisReport getReportForRun(String runId) throws Exception {
    if (reportCache.containsKey(runId)) {
      return reportCache.get(runId);
    }
    Path dir = runDirsById.getOrDefault(runId, rootDir);
    HootConverter.convertDirectoryIfNeeded(dir, dir, null);
    RunAnalysisReport rep = RunAnalyzer.analyzeRunDirectory(dir, dir, defaultProfile, false);
    reportCache.put(runId, rep);
    return rep;
  }

  private List<ParsedLog> getLogsForRun(String runId) throws Exception {
    if (logCache.containsKey(runId)) {
      return logCache.get(runId);
    }
    Path dir = runDirsById.getOrDefault(runId, rootDir);
    HootConverter.convertDirectoryIfNeeded(dir, dir, null);
    List<ParsedLog> logs = new ArrayList<>();
    try (Stream<Path> s = Files.list(dir)) {
      for (Path p : s.sorted().toList()) {
        String n = p.getFileName().toString().toLowerCase(Locale.ROOT);
        if (n.endsWith(".wpilog") && !n.equals("merged.wpilog")) {
          logs.add(WpiLogReader.read(p));
        }
      }
    }
    logCache.put(runId, logs);
    return logs;
  }

  private static boolean matchesOp(double val, String op, double threshold) {
    return switch (op) {
      case "<" -> val < threshold;
      case "<=" -> val <= threshold;
      case ">" -> val > threshold;
      case ">=" -> val >= threshold;
      case "==" -> Math.abs(val - threshold) < 1e-6;
      default -> val < threshold;
    };
  }

  private static String buildToolsListJson() {
    return """
        {"tools":[
          {"name":"list_entries","description":"Lists all .wpilog telemetry entries, types, and sample counts in a run."},
          {"name":"find_condition","description":"Finds all timestamps where a telemetry topic crosses a threshold (<, <=, >, >=, ==)."},
          {"name":"time_correlate","description":"Slices battery voltage, drive stator current, and chassis speed around a timestamp."},
          {"name":"compare_runs","description":"Compares two runs (runA vs runB) across battery R_int, min voltage, stator current, and cycles."},
          {"name":"moi_regression","description":"Runs Digital-Twin OLS regression for battery R_int, wheel radius, and mechanism (kS, kV, kA, kG)."},
          {"name":"search_voice_notes","description":"Searches Driver/Operator/Coach voice notes and 6-second sensor lookback diagnoses."},
          {"name":"get_camera_snapshots","description":"Lists event-triggered robot camera snapshots (cam-snapshots/*.jpg) for a run."},
          {"name":"get_video_frames","description":"Computes synchronized match video jump links and [t-2s .. t+1s] frame windows."}
        ]}"""
        .replace("\n", "")
        .replace("  ", "");
  }

  private static String extractJsonId(String json) {
    Pattern p = Pattern.compile("\"id\"\\s*:\\s*(\"[^\"]*\"|-?[0-9]+|null)");
    Matcher m = p.matcher(json);
    return m.find() ? m.group(1) : "1";
  }

  private static String extractParamsBlock(String json) {
    int idx = json.indexOf("\"params\"");
    return idx >= 0 ? json.substring(idx) : json;
  }

  private static String extractArgumentsBlock(String paramsJson) {
    int idx = paramsJson.indexOf("\"arguments\"");
    return idx >= 0 ? paramsJson.substring(idx) : paramsJson;
  }

  private static double parseDouble(String s, double fallback) {
    try {
      return Double.parseDouble(s.trim());
    } catch (NumberFormatException e) {
      return fallback;
    }
  }

  private static String buildResult(String idRaw, String resultJson) {
    return "{\"jsonrpc\":\"2.0\",\"id\":" + idRaw + ",\"result\":" + resultJson + "}";
  }

  private static String buildError(String idRaw, int code, String message) {
    return "{\"jsonrpc\":\"2.0\",\"id\":"
        + idRaw
        + ",\"error\":{\"code\":"
        + code
        + ",\"message\":\""
        + escJson(message)
        + "\"}}";
  }

  private static String escJson(String s) {
    if (s == null) {
      return "";
    }
    return s.replace("\\", "\\\\")
        .replace("\"", "\\\"")
        .replace("\n", "\\n")
        .replace("\r", "");
  }
}
