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
  public static final int MIN_REGRESSION_SAMPLES = 10;
  public static final double MIN_CURRENT_SPREAD_AMPS = 15.0;
  public static final double MIN_R_SQUARED = 0.35;

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
      double supply = Double.isFinite(pt.totalSupplyAmps()) ? pt.totalSupplyAmps() : 0.0;
      if (!pt.enabled() && supply < 5.0 && Double.isFinite(pt.batteryVoltage()) && pt.batteryVoltage() > 11.0) {
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
      double dt = (i > 0) ? (pt.timeSec() - timeline.get(i - 1).timeSec()) : 0.2;
      double v = pt.batteryVoltage();
      if (Double.isFinite(pt.totalSupplyAmps()) && pt.totalSupplyAmps() > peakSupply) {
        peakSupply = pt.totalSupplyAmps();
      }
      if (Double.isFinite(pt.totalDriveStatorAmps()) && pt.totalDriveStatorAmps() > peakDriveStator) {
        peakDriveStator = pt.totalDriveStatorAmps();
      }
      if (Double.isFinite(v) && v > 3.0) {
        if (v < minV) {
          minV = v;
          minVTime = pt.timeSec();
        }
        if (v > maxV) {
          maxV = v;
        }
        if (v < BROWNOUT_WARN_VOLTS) {
          warnSec += dt;
        }
        if (v < BROWNOUT_CRIT_VOLTS) {
          critSec += dt;
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
          peakSupply,
          peakDriveStator,
          0.0,
          0.0,
          enableDropouts,
          enableDropouts > 0 ? "BROWNOUT RISK" : "NO DATA");
    }

    boolean hasRestingV = Double.isFinite(restingV);

    // 2-parameter OLS regression V = Voc + slope * I (requiring slope < 0, R^2 >= 0.35,
    // n >= 10, and current spread >= 15 A before clamping R_int to [8.0, 60.0] mOhm).
    double sumX = 0.0;
    double sumXX = 0.0;
    double sumV = 0.0;
    double sumXV = 0.0;
    double sumVV = 0.0;
    double minI = Double.POSITIVE_INFINITY;
    double maxI = Double.NEGATIVE_INFINITY;
    int n = 0;

    for (TimelinePoint pt : timeline) {
      double v = pt.batteryVoltage();
      double supply = pt.totalSupplyAmps();
      if (pt.enabled()
          && Double.isFinite(supply)
          && supply >= 5.0
          && Double.isFinite(v)
          && v > 5.0) {
        sumX += supply;
        sumXX += supply * supply;
        sumV += v;
        sumXV += supply * v;
        sumVV += v * v;
        if (supply < minI) {
          minI = supply;
        }
        if (supply > maxI) {
          maxI = supply;
        }
        n++;
      }
    }

    double rintMilliOhms = Double.NaN;
    double rSquared = Double.NaN;
    double currentSpread = (n > 0) ? (maxI - minI) : 0.0;
    if (n >= MIN_REGRESSION_SAMPLES && currentSpread >= MIN_CURRENT_SPREAD_AMPS) {
      double varX = n * sumXX - sumX * sumX;
      double varV = n * sumVV - sumV * sumV;
      if (varX > 1e-6 && varV > 1e-9) {
        double covXV = n * sumXV - sumX * sumV;
        double slope = covXV / varX;
        double fitR2 = (covXV * covXV) / (varX * varV);
        if (Double.isFinite(fitR2)) {
          rSquared = Math.max(0.0, Math.min(1.0, fitR2));
        }
        if (slope < 0.0 && Double.isFinite(rSquared) && rSquared >= MIN_R_SQUARED) {
          double rintOhms = -slope;
          if (!hasRestingV) {
            restingV = (sumV + rintOhms * sumX) / n;
            hasRestingV = Double.isFinite(restingV);
          }
          rintMilliOhms = Math.max(8.0, Math.min(60.0, rintOhms * 1000.0));
        }
      }
    }
    if (!hasRestingV) {
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
