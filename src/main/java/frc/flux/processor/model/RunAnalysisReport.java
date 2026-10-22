package frc.flux.processor.model;

import java.util.List;

/** Complete structured analysis output for a single match or practice run. */
public record RunAnalysisReport(
    String runId,
    String robotName,
    String matchType,
    String allianceStation,
    boolean isFmsMatch,
    String namedBattery,
    String gitSha,
    String configSha,
    String videoUrl,
    double videoOffsetSeconds,
    double totalDurationSec,
    double enabledDurationSec,
    double totalDistanceMeters,
    double peakSpeedMps,
    String overallVerdict,
    List<String> keyAlerts,
    BatteryMetrics battery,
    List<CornerMetrics> swerveCorners,
    List<MechanismMetrics> mechanisms,
    List<ShotEvent> shots,
    List<VoiceCorrelation> voiceCorrelations,
    List<TimelinePoint> timeline,
    SystemHealthMetrics systemHealth) {

  public RunAnalysisReport(
      String runId,
      String robotName,
      String matchType,
      String allianceStation,
      boolean isFmsMatch,
      String namedBattery,
      String gitSha,
      String configSha,
      String videoUrl,
      double videoOffsetSeconds,
      double totalDurationSec,
      double enabledDurationSec,
      double totalDistanceMeters,
      double peakSpeedMps,
      String overallVerdict,
      List<String> keyAlerts,
      BatteryMetrics battery,
      List<CornerMetrics> swerveCorners,
      List<MechanismMetrics> mechanisms,
      List<ShotEvent> shots,
      List<VoiceCorrelation> voiceCorrelations,
      List<TimelinePoint> timeline) {
    this(
        runId,
        robotName,
        matchType,
        allianceStation,
        isFmsMatch,
        namedBattery,
        gitSha,
        configSha,
        videoUrl,
        videoOffsetSeconds,
        totalDurationSec,
        enabledDurationSec,
        totalDistanceMeters,
        peakSpeedMps,
        overallVerdict,
        keyAlerts,
        battery,
        swerveCorners,
        mechanisms,
        shots,
        voiceCorrelations,
        timeline,
        null);
  }

  public record BatteryMetrics(
      String batteryName,
      double restingVoltage,
      double minVoltage,
      double minVoltageTimeSec,
      double estimatedRintMilliOhms,
      double rSquared,
      double peakSupplyCurrentAmps,
      double peakDriveStatorAmps,
      double brownoutWarningSeconds,
      double brownoutCriticalSeconds,
      int enableDropouts,
      String healthGrade) {}

  public record CornerMetrics(
      String corner,
      int driveId,
      int steerId,
      double meanDriveStatorAmps,
      double peakDriveStatorAmps,
      double meanDriveSupplyAmps,
      double peakDriveVelocityRps,
      double meanSteerStatorAmps,
      double peakSteerStatorAmps,
      double driveAsymmetryPct,
      String status) {}

  public record MechanismMetrics(
      String name,
      int motorId,
      String role,
      double meanStatorAmps,
      double peakStatorAmps,
      double peakSupplyAmps,
      double peakVelocityRps,
      double minPositionRot,
      double maxPositionRot,
      int resetCount,
      String status) {}

  public record ShotEvent(
      int shotNumber,
      double timestampSec,
      double durationSec,
      double shooterLeftRps,
      double shooterRightRps,
      double shooterAsymmetryRps,
      double hoodPositionRot,
      double feederPeakStatorAmps,
      double minBatteryVoltageDuringShot,
      String videoLink,
      String status) {}

  public record VoiceCorrelation(
      String speakerRole,
      double spokenTimeSec,
      String transcript,
      double lookbackStartSec,
      String correlatedEventSummary,
      String sensorDiagnosis,
      String videoLink,
      String severity) {}

  public record TimelinePoint(
      double timeSec,
      boolean enabled,
      String mode,
      double batteryVoltage,
      double totalDriveStatorAmps,
      double totalSupplyAmps,
      double chassisSpeedMps,
      double shooterLeftRps,
      double feederStatorAmps) {}

  /** Offline SystemCore / roboRIO watch metrics and triggered engineering/pit decisions. */
  public record SystemHealthMetrics(
      String controllerType,
      double loopPeriodMs,
      double loopMedianMs,
      double loopP99Ms,
      double loopMaxMs,
      double firstEnablePeakMs,
      double totalCpuPct,
      double javaProcessCpuPct,
      double visionDaemonCpuPct,
      double softIrqPct,
      double cpuTempC,
      double cpuFreqMHz,
      int throttledFlags,
      long socketCanDropsAndOverruns,
      int socketCanBusErrors,
      double heapAllocRateMbPerSec,
      int gcCorrelatedLoopSpikes,
      double freeStorageMb,
      double ipc,
      double llcMissPct,
      List<SystemWatchDecision> watchDecisions) {}

  public record SystemWatchDecision(
      String category,
      String metricName,
      String measuredValue,
      String threshold,
      String severity,
      String anomalySummary,
      String actionableDecision) {}
}
