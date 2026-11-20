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
    if (timeline == null || timeline.isEmpty()) {
      return new BatteryMetrics(
          batteryName,
          Double.NaN,
          Double.NaN,
          0.0,
          Double.NaN,
          Double.NaN,
          0.0,
          0.0,
          0.0,
          0.0,
          enableDropouts,
          enableDropouts > 0 ? "BROWNOUT RISK" : "NO DATA");
    }

    double restingV = Double.NaN;
    for (TimelinePoint pt : timeline) {
      if (!pt.enabled() && pt.totalSupplyAmps() < 5.0 && pt.batteryVoltage() > 11.0) {
        restingV = pt.batteryVoltage();
        break;
      }
    }

    double minV = Double.POSITIVE_INFINITY;
    double maxV = Double.NEGATIVE_INFINITY;
    double minVTime = 0.0;
    double peakSupply = 0.0;
    double peakDriveStator = 0.0;
    double warnSec = 0.0;
    double critSec = 0.0;

    for (int i = 0; i < timeline.size(); i++) {
      TimelinePoint pt = timeline.get(i);
      double v = pt.batteryVoltage();
      if (Double.isFinite(v) && v > 3.0) {
        if (v < minV) {
          minV = v;
          minVTime = pt.timeSec();
        }
        if (v > maxV) {
          maxV = v;
        }
      }
    }

    boolean hasVoltageData = Double.isFinite(minV);
    if (!hasVoltageData) {
      return new BatteryMetrics(
          batteryName,
          Double.NaN,
          Double.NaN,
          0.0,
          Double.NaN,
          Double.NaN,
          0.0,
          0.0,
          0.0,
          0.0,
          enableDropouts,
          enableDropouts > 0 ? "BROWNOUT RISK" : "NO DATA");
    }

    boolean hasRestingV = Double.isFinite(restingV);

    // Linear regression: V_drop = (restingV - V) vs I_effective, or 2-parameter OLS (V = Voc - R*I)
    // when the log window contains no disabled resting sample.
    double sumX = 0.0;
    double sumY = 0.0;
    double sumXX = 0.0;
    double sumXY = 0.0;
    double sumYY = 0.0;
    double sumV = 0.0;
    double sumXV = 0.0;
    double sumVV = 0.0;
    int n = 0;

    for (int i = 0; i < timeline.size(); i++) {
      TimelinePoint pt = timeline.get(i);
      double dt = (i > 0) ? (pt.timeSec() - timeline.get(i - 1).timeSec()) : 0.2;
      double v = pt.batteryVoltage();
      double effSupply = Math.max(pt.totalSupplyAmps(), pt.totalDriveStatorAmps() * 0.38);
      if (effSupply > peakSupply) {
        peakSupply = effSupply;
      }
      if (pt.totalDriveStatorAmps() > peakDriveStator) {
        peakDriveStator = pt.totalDriveStatorAmps();
      }
      if (Double.isFinite(v) && v > 3.0 && v < BROWNOUT_WARN_VOLTS) {
        warnSec += dt;
      }
      if (Double.isFinite(v) && v > 3.0 && v < BROWNOUT_CRIT_VOLTS) {
        critSec += dt;
      }

      if (pt.enabled() && effSupply > 15.0 && Double.isFinite(v) && v > 5.0) {
        if (!hasRestingV || v <= restingV) {
          double x = effSupply;
          sumX += x;
          sumXX += x * x;
          sumV += v;
          sumXV += x * v;
          sumVV += v * v;
          if (hasRestingV) {
            double y = restingV - v;
            sumY += y;
            sumXY += x * y;
            sumYY += y * y;
          }
          n++;
        }
      }
    }

    double rintMilliOhms = Double.NaN;
    double rSquared = Double.NaN;
    if (n >= 5 && sumXX > 1e-6) {
      if (hasRestingV) {
        double rintOhms = sumXY / sumXX;
        rintMilliOhms = Math.max(8.0, Math.min(60.0, rintOhms * 1000.0));
        double denom = (n * sumXX - sumX * sumX) * (n * sumYY - sumY * sumY);
        if (denom > 1e-9) {
          double num = (n * sumXY - sumX * sumY);
          rSquared = Math.max(0.0, Math.min(1.0, (num * num) / denom));
        } else {
          rSquared = 0.0;
        }
      } else {
        double varX = n * sumXX - sumX * sumX;
        double varV = n * sumVV - sumV * sumV;
        if (varX > 1.0) {
          double slope = (n * sumXV - sumX * sumV) / varX;
          double rintOhms = Math.max(0.0, -slope);
          restingV = (sumV + rintOhms * sumX) / n;
          rintMilliOhms = Math.max(8.0, Math.min(60.0, rintOhms * 1000.0));
          double denom = varX * varV;
          if (denom > 1e-9) {
            double num = n * sumXV - sumX * sumV;
            rSquared = Math.max(0.0, Math.min(1.0, (num * num) / denom));
          } else {
            rSquared = 0.0;
          }
        } else {
          restingV = Math.max(12.6, maxV);
          double estSumXY = restingV * sumX - sumXV;
          double rintOhms = estSumXY / sumXX;
          rintMilliOhms = Math.max(8.0, Math.min(60.0, rintOhms * 1000.0));
          rSquared = 0.0;
        }
      }
    } else if (!hasRestingV) {
      restingV = maxV;
    }

    String grade;
    if (minV < BROWNOUT_CRIT_VOLTS || enableDropouts > 0) {
      grade = "BROWNOUT RISK";
    } else if (!Double.isFinite(rintMilliOhms)) {
      grade = "NO DATA";
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
