package frc.flux.processor.analysis;

import frc.flux.processor.model.RobotProfile;
import frc.flux.processor.model.RunAnalysisReport.CornerMetrics;
import frc.flux.processor.wpilog.WpiLogReader.DoubleSample;
import frc.flux.processor.wpilog.WpiLogReader.ParsedLog;
import java.util.ArrayList;
import java.util.List;

/** 4-Corner Swerve drive and steer current symmetry analyzer. */
public final class SwerveSymmetryAnalyzer {

  private SwerveSymmetryAnalyzer() {}

  public static List<CornerMetrics> analyze(
      RobotProfile profile,
      List<ParsedLog> allLogs,
      ParsedLog driveLog,
      long minTsUs,
      long maxTsUs) {
    List<CornerMetrics> rawCorners = new ArrayList<>();
    String[] order = {"FL", "FR", "BL", "BR"};

    for (String key : order) {
      RobotProfile.CornerConfig cfg = profile.corners().get(key);
      if (cfg == null) {
        continue;
      }
      List<DoubleSample> dStator =
          RunAnalyzer.findDoubleSeriesByTopic(allLogs, driveLog, cfg.driveStatorTopic());
      List<DoubleSample> dSupply =
          RunAnalyzer.findDoubleSeriesByTopic(allLogs, driveLog, cfg.driveSupplyTopic());
      List<DoubleSample> dVel =
          RunAnalyzer.findDoubleSeriesByTopic(allLogs, driveLog, cfg.driveVelocityTopic());
      List<DoubleSample> sStator =
          RunAnalyzer.findDoubleSeriesByTopic(allLogs, driveLog, cfg.steerStatorTopic());

      double meanDStator = RunAnalyzer.meanAbsActive(dStator, 2.0, minTsUs, maxTsUs);
      double peakDStator = RunAnalyzer.maxAbs(dStator, minTsUs, maxTsUs);
      double meanDSupply = RunAnalyzer.meanAbsActive(dSupply, 1.0, minTsUs, maxTsUs);
      double peakDVel = RunAnalyzer.maxAbs(dVel, minTsUs, maxTsUs);
      double meanSStator = RunAnalyzer.meanAbsActive(sStator, 1.0, minTsUs, maxTsUs);
      double peakSStator = RunAnalyzer.maxAbs(sStator, minTsUs, maxTsUs);

      rawCorners.add(
          new CornerMetrics(
              key,
              cfg.driveMotorId(),
              cfg.steerMotorId(),
              meanDStator,
              peakDStator,
              meanDSupply,
              peakDVel,
              meanSStator,
              peakSStator,
              0.0,
              "PASS"));
    }

    // Compute median of meanDriveStatorAmps across the 4 corners
    List<Double> means =
        rawCorners.stream().map(CornerMetrics::meanDriveStatorAmps).sorted().toList();
    double medianMean =
        means.size() == 4
            ? 0.5 * (means.get(1) + means.get(2))
            : (means.isEmpty() ? 1.0 : means.get(0));
    if (medianMean < 1.0) {
      medianMean = 1.0;
    }

    List<CornerMetrics> result = new ArrayList<>(rawCorners.size());
    for (CornerMetrics c : rawCorners) {
      double asymPct = ((c.meanDriveStatorAmps() - medianMean) / medianMean) * 100.0;
      String status = "PASS";
      if (Math.abs(asymPct) > 35.0) {
        status = "FAIL";
      } else if (Math.abs(asymPct) > 22.0 || c.peakDriveStatorAmps() > 140.0) {
        status = "WARN";
      }
      result.add(
          new CornerMetrics(
              c.corner(),
              c.driveId(),
              c.steerId(),
              c.meanDriveStatorAmps(),
              c.peakDriveStatorAmps(),
              c.meanDriveSupplyAmps(),
              c.peakDriveVelocityRps(),
              c.meanSteerStatorAmps(),
              c.peakSteerStatorAmps(),
              asymPct,
              status));
    }
    return result;
  }
}
