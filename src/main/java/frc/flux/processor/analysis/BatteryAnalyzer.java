package frc.flux.processor.analysis;

import frc.flux.processor.model.RunAnalysisReport.BatteryMetrics;
import frc.flux.processor.model.RunAnalysisReport.TimelinePoint;
import java.util.List;

/**
 * Battery internal resistance ({@code R_int = dV / dI}) linear regression, voltage sag, and
 * brownout dropout analyzer.
 */
public final class BatteryAnalyzer {

  public static final double BROWNOUT_WARN_VOLTS = 7.0;
  public static final double BROWNOUT_CRIT_VOLTS = 6.8;

  private BatteryAnalyzer() {}

  public static BatteryMetrics analyze(
      String batteryName, List<TimelinePoint> timeline, int enableDropouts) {
    double restingV = 12.6;
    for (TimelinePoint pt : timeline) {
      if (!pt.enabled() && pt.totalSupplyAmps() < 5.0 && pt.batteryVoltage() > 11.0) {
        restingV = pt.batteryVoltage();
        break;
      }
    }

    double minV = 14.0;
    double minVTime = 0.0;
    double peakSupply = 0.0;
    double peakDriveStator = 0.0;
    double warnSec = 0.0;
    double critSec = 0.0;

    // Linear regression: V_drop = (restingV - V) vs I_effective
    // Use effective battery current (max of measured supply current and 0.38 * total stator current)
    double sumX = 0.0;
    double sumY = 0.0;
    double sumXX = 0.0;
    double sumXY = 0.0;
    double sumYY = 0.0;
    int n = 0;

    for (int i = 0; i < timeline.size(); i++) {
      TimelinePoint pt = timeline.get(i);
      double dt = (i > 0) ? (pt.timeSec() - timeline.get(i - 1).timeSec()) : 0.2;
      double v = pt.batteryVoltage();
      if (v > 3.0 && v < minV) {
        minV = v;
        minVTime = pt.timeSec();
      }
      double effSupply = Math.max(pt.totalSupplyAmps(), pt.totalDriveStatorAmps() * 0.38);
      if (effSupply > peakSupply) {
        peakSupply = effSupply;
      }
      if (pt.totalDriveStatorAmps() > peakDriveStator) {
        peakDriveStator = pt.totalDriveStatorAmps();
      }
      if (v < BROWNOUT_WARN_VOLTS) {
        warnSec += dt;
      }
      if (v < BROWNOUT_CRIT_VOLTS) {
        critSec += dt;
      }

      if (pt.enabled() && effSupply > 15.0 && v > 5.0 && v <= restingV) {
        double x = effSupply;
        double y = restingV - v;
        sumX += x;
        sumY += y;
        sumXX += x * x;
        sumXY += x * y;
        sumYY += y * y;
        n++;
      }
    }

    if (minV == 14.0) {
      minV = restingV;
    }

    double rintOhms = 0.016;
    double rSquared = 0.0;
    if (n >= 5 && sumXX > 1e-6) {
      rintOhms = sumXY / sumXX;
      double denom = (n * sumXX - sumX * sumX) * (n * sumYY - sumY * sumY);
      if (denom > 1e-9) {
        double num = (n * sumXY - sumX * sumY);
        rSquared = Math.max(0.0, Math.min(1.0, (num * num) / denom));
      }
    }
    double rintMilliOhms = Math.max(8.0, Math.min(60.0, rintOhms * 1000.0));

    String grade;
    if (minV < BROWNOUT_CRIT_VOLTS || enableDropouts > 0) {
      grade = "BROWNOUT RISK";
    } else if (rintMilliOhms <= 16.5) {
      grade = "EXCELLENT (Comp Tier)";
    } else if (rintMilliOhms <= 20.5) {
      grade = "GOOD (Practice/Comp)";
    } else if (rintMilliOhms <= 25.0) {
      grade = "FAIR (Practice Only)";
    } else {
      grade = "WEAK (Retire / Recell)";
    }

    return new BatteryMetrics(
        batteryName,
        restingV,
        minV,
        minVTime,
        rintMilliOhms,
        rSquared,
        peakSupply,
        peakDriveStator,
        warnSec,
        critSec,
        enableDropouts,
        grade);
  }
}
