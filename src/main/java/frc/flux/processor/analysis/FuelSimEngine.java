package frc.flux.processor.analysis;

import frc.flux.processor.model.RobotProfile;
import frc.flux.processor.model.RunAnalysisReport.BatteryMetrics;
import frc.flux.processor.model.RunAnalysisReport.FuelSimMetrics;
import frc.flux.processor.model.RunAnalysisReport.ShotEvent;
import frc.flux.processor.model.RunAnalysisReport.ShotSimSummary;
import frc.flux.processor.model.RunAnalysisReport.Sim3DFrame;
import frc.flux.processor.model.RunAnalysisReport.TimelinePoint;
import frc.flux.processor.model.RunAnalysisReport.VoiceCorrelation;
import frc.flux.processor.report.FuelSimVideoRenderer;
import frc.lib.fluxfig.FluxWpiLogCodec.BooleanSample;
import frc.lib.fluxfig.FluxWpiLogCodec.DoubleSample;
import frc.lib.fluxfig.FluxWpiLogCodec.ParsedLog;
import frc.lib.fluxfig.FluxWpiLogCodec.StringSample;
import frc.lib.fluxfig.FluxWpiLogCodec.Writer;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Pure-Java 25 implementation of the 2026 <i>REBUILT</i> {@code FuelSim} 3D game-piece physics
 * engine (adapted from FRC Team 5000 Hammerheads' {@code FuelSim} architecture for offline match
 * log replay and AdvantageScope 3D Field visualization).
 *
 * <p>Simulates:
 *
 * <ul>
 *   <li>Starting Fuel ball layout in Depots and Neutral Zone ({@link #spawnStartingFuel()})
 *   <li>5-substep ({@code dt = 4 ms}) 3D projectile kinematics with gravity ({@code 9.81 m/s^2})
 *       and quadratic aerodynamic drag
 *   <li>Analytical field boundary reflections ({@code FIELD_LENGTH = 16.541 m}, {@code FIELD_WIDTH =
 *       8.069 m}), 3D Trench colliders, Net damping planes, Octagonal Hub wall rebounds, and top
 *       Hub scoring funnel detection (Blue Hub {@code (4.63, 4.03)}, Red Hub {@code (11.92, 4.03)})
 *   <li>Robot oriented bounding-box (OBB) bumper collisions, floor intake bounding-box pickup, and
 *       flywheel + hood velocity vector launches ({@code launchFuel})
 *   <li>Sphere-to-sphere ({@code fuel-fuel}) elastic collisions
 * </ul>
 */
public final class FuelSimEngine {

  public static final double FIELD_LENGTH_M = 16.541;
  public static final double FIELD_WIDTH_M = 8.069;
  public static final double FUEL_RADIUS_M = 0.0889; // 3.5 in radius (7 in diameter Fuel ball)
  public static final double GRAVITY_MPS2 = 9.80665;

  // Blue and Red Octagonal Hub Centers (from 2026 REBUILT welded layout)
  public static final double BLUE_HUB_X = 4.626;
  public static final double BLUE_HUB_Y = 4.035;
  public static final double RED_HUB_X = 11.915;
  public static final double RED_HUB_Y = 4.035;
  public static final double HUB_OUTER_RADIUS_M = 0.68;
  public static final double HUB_SCORE_RADIUS_M = 0.58;
  public static final double HUB_RIM_HEIGHT_M = 1.65;
  public static final double HUB_FUNNEL_MAX_Z_M = 2.45;

  // Netting behind/around Hub faces
  private static final double NET_MIN_Z_M = 1.65;
  private static final double NET_MAX_Z_M = 3.10;

  // Trench colliders (Blue & Red top/bottom trenches at z <= 0.89 m)
  private static final double[][] TRENCH_BOXES = {
    // {minX, maxX, minY, maxY, maxZ}
    {4.45, 4.80, 0.45, 0.85, 0.89}, // Blue bottom trench pillar
    {4.45, 4.80, 7.22, 7.62, 0.89}, // Blue top trench pillar
    {11.75, 12.10, 0.45, 0.85, 0.89}, // Red bottom trench pillar
    {11.75, 12.10, 7.22, 7.62, 0.89} // Red top trench pillar
  };

  private static final double FLOOR_COR = 0.58;
  private static final double WALL_COR = 0.62;
  private static final double NET_COR = 0.15;
  private static final double BALL_COR = 0.72;
  private static final double ROLLING_FRICTION = 1.85;
  private static final double AIR_DRAG_COEFF = 0.045;

  public static final class FuelBall {
    public double x;
    public double y;
    public double z;
    public double vx;
    public double vy;
    public double vz;
    public boolean active = true;
    public boolean scored = false;
    public int shotId = 0; // >0 if launched by robot shot #shotId

    public FuelBall(double x, double y, double z, double vx, double vy, double vz, int shotId) {
      this.x = x;
      this.y = y;
      this.z = z;
      this.vx = vx;
      this.vy = vy;
      this.vz = vz;
      this.shotId = shotId;
    }
  }

  public record SimulatedShotOutcome(
      int shotNumber,
      double launchTimeSec,
      double launchX,
      double launchY,
      double launchHeadingDeg,
      double launchSpeedMps,
      double launchPitchDeg,
      double targetDistanceM,
      boolean scoredInHub,
      double closestHubDistM,
      double apexHeightM) {}

  private final List<FuelBall> balls = new ArrayList<>();
  private final List<SimulatedShotOutcome> shotOutcomes = new ArrayList<>();
  private int subticks = 5;
  private double robotWidthM = 0.8636; // 34 in with bumpers
  private double robotLengthM = 0.8636; // 34 in with bumpers
  private double bumperHeightM = 0.165;
  private int blueHubScore = 0;
  private int redHubScore = 0;
  private int intakedCount = 0;
  private int hopperFuelCount = 3; // Preloaded Fuel balls at match start

  // Track in-flight robot shots to record their outcome
  private static final class InFlightShotTracker {
    final int shotNumber;
    final double launchTimeSec;
    final double launchX;
    final double launchY;
    final double launchHeadingDeg;
    final double launchSpeedMps;
    final double launchPitchDeg;
    final double targetDistM;
    final FuelBall ball;
    double minHubDistM = Double.POSITIVE_INFINITY;
    double maxApexZM = 0.0;
    boolean scored = false;
    boolean finalized = false;

    InFlightShotTracker(
        int shotNumber,
        double launchTimeSec,
        double launchX,
        double launchY,
        double launchHeadingDeg,
        double launchSpeedMps,
        double launchPitchDeg,
        double targetDistM,
        FuelBall ball) {
      this.shotNumber = shotNumber;
      this.launchTimeSec = launchTimeSec;
      this.launchX = launchX;
      this.launchY = launchY;
      this.launchHeadingDeg = launchHeadingDeg;
      this.launchSpeedMps = launchSpeedMps;
      this.launchPitchDeg = launchPitchDeg;
      this.targetDistM = targetDistM;
      this.ball = ball;
    }
  }

  private final List<InFlightShotTracker> trackers = new ArrayList<>();

  public void setSubticks(int subticks) {
    this.subticks = Math.max(1, Math.min(20, subticks));
  }

  public void setRobotDimensions(double widthMeters, double lengthMeters, double bumperHeightM) {
    this.robotWidthM = widthMeters;
    this.robotLengthM = lengthMeters;
    this.bumperHeightM = bumperHeightM;
  }

  /** Spawns starting 2026 REBUILT Fuel balls in the Neutral Zone centerline and Alliance Depots. */
  public void spawnStartingFuel() {
    balls.clear();
    double r = FUEL_RADIUS_M;

    double centerX = FIELD_LENGTH_M * 0.5;
    double[] neutralYs = {1.20, 2.00, 2.80, 3.60, 4.47, 5.27, 6.07, 6.87};
    for (double y : neutralYs) {
      balls.add(new FuelBall(centerX, y, r, 0, 0, 0, 0));
    }

    double[] blueDepotXs = {1.15, 1.45, 2.40, 2.40};
    double[] blueDepotYs = {1.80, 6.25, 2.60, 5.45};
    for (int i = 0; i < blueDepotXs.length; i++) {
      balls.add(new FuelBall(blueDepotXs[i], blueDepotYs[i], r, 0, 0, 0, 0));
      balls.add(new FuelBall(blueDepotXs[i] + 0.24, blueDepotYs[i], r, 0, 0, 0, 0));
    }

    for (int i = 0; i < blueDepotXs.length; i++) {
      double rx = FIELD_LENGTH_M - blueDepotXs[i];
      balls.add(new FuelBall(rx, blueDepotYs[i], r, 0, 0, 0, 0));
      balls.add(new FuelBall(rx - 0.24, blueDepotYs[i], r, 0, 0, 0, 0));
    }
  }

  /**
   * Interpolates the nominal Hub distance (meters from Hub center) from {@code RangeTable.java}
   * given hood position in motor rotations ({@code 0.0 .. 8.85+ rot}).
   */
  public static double nominalDistanceForHood(double hoodRotations) {
    double h = Math.max(0.0, Math.min(12.5, hoodRotations));
    double[] hoods = {0.0, 2.8, 3.9, 6.7, 8.85, 12.5};
    double[] dists = {1.35, 1.95, 2.86, 3.78, 4.69, 5.60};
    for (int i = 0; i < hoods.length - 1; i++) {
      if (h <= hoods[i + 1]) {
        double t = (h - hoods[i]) / Math.max(1e-6, hoods[i + 1] - hoods[i]);
        return dists[i] + t * (dists[i + 1] - dists[i]);
      }
    }
    return dists[dists.length - 1];
  }

  /**
   * Interpolates the nominal required flywheel RPS from {@code RangeTable.java} given hood position
   * in motor rotations ({@code 0.0 .. 8.85+ rot}).
   */
  public static double nominalRpsForHood(double hoodRotations) {
    double h = Math.max(0.0, Math.min(12.5, hoodRotations));
    double[] hoods = {0.0, 2.8, 3.9, 6.7, 8.85, 12.5};
    double[] rps = {33.33, 33.33, 36.67, 38.33, 40.00, 43.00};
    for (int i = 0; i < hoods.length - 1; i++) {
      if (h <= hoods[i + 1]) {
        double t = (h - hoods[i]) / Math.max(1e-6, hoods[i + 1] - hoods[i]);
        return rps[i] + t * (rps[i + 1] - rps[i]);
      }
    }
    return rps[rps.length - 1];
  }

  /**
   * Computes the ideal exit speed (m/s) required for a projectile launched at {@code pitchRad} from
   * {@code z0 = 0.55 m} to descend cleanly into the Octagonal Hub funnel ({@code z = 1.65 m},
   * {@code dz = 1.10 m}) at horizontal flight range {@code horizRangeM}, accounting for quadratic
   * aerodynamic drag ({@code AIR_DRAG_COEFF = 0.045}).
   */
  public static double idealLaunchSpeedForRange(double horizRangeM, double pitchRad) {
    double r = Math.max(0.85, Math.min(7.5, horizRangeM));
    double dz = HUB_RIM_HEIGHT_M - 0.55; // 1.10 m vertical rise to Hub rim
    double cosP = Math.cos(pitchRad);
    double tanP = Math.tan(pitchRad);
    double denom = 2.0 * cosP * cosP * Math.max(0.25, (r * tanP - dz));
    double vVacuum = Math.sqrt((GRAVITY_MPS2 * r * r) / denom);
    // Drag compensation (~4.2% to 6.5% over flight distance)
    double dragFactor = 1.032 + 0.0085 * r;
    return vVacuum * dragFactor;
  }

  /**
   * Launches a Fuel ball from the robot's shooter exit using measured flywheel RPS, hood position,
   * and robot field velocity (Shoot-on-the-Move).
   */
  public void launchFuelFromLog(
      int shotNumber,
      double timeSec,
      boolean isBlueAlliance,
      double robotX,
      double robotY,
      double robotHeadingRad,
      double robotVxField,
      double robotVyField,
      double shooterRps,
      double hoodRotations) {
    if (hopperFuelCount > 0) {
      hopperFuelCount--;
    }

    double hubX = isBlueAlliance ? BLUE_HUB_X : RED_HUB_X;
    double hubY = isBlueAlliance ? BLUE_HUB_Y : RED_HUB_Y;
    double dxHub = hubX - robotX;
    double dyHub = hubY - robotY;
    double distToHub = Math.hypot(dxHub, dyHub);

    // Hood 0 rot = steeper close-up subwoofer shot (~73 deg), Hood 9 rot = flatter far shot (~51 deg)
    double clampedHood = Math.max(0.0, Math.min(12.5, hoodRotations));
    double launchPitchDeg = 73.0 - (clampedHood * 2.4);
    double launchPitchRad = Math.toRadians(launchPitchDeg);

    // Calibrate ballistic speed from RangeTable nominal range + measured RPS ratio
    double nominalRps = nominalRpsForHood(clampedHood);
    double flightHorizRangeM = Math.max(0.85, distToHub - 0.15);
    double idealSpeedMps = idealLaunchSpeedForRange(flightHorizRangeM, launchPitchRad);
    double rpsRatio = Math.max(0.45, Math.min(1.35, shooterRps / Math.max(1.0, nominalRps)));
    // Slight forgiving deadband near 1.0 when shooterRps is within nominal [>= 90% target RPS] band
    if (rpsRatio >= 0.90 && rpsRatio <= 1.12) {
      rpsRatio = 1.0 + (rpsRatio - 1.0) * 0.25;
    }
    double launchSpeedMps = idealSpeedMps * rpsRatio;

    // Direction of launch: aim along robot heading (or toward hub if within alignment cone)
    double angleToHub = Math.atan2(dyHub, dxHub);
    double headingDiff =
        Math.atan2(
            Math.sin(angleToHub - robotHeadingRad), Math.cos(angleToHub - robotHeadingRad));
    // Because dead-reckoned .hoot odometry without vision can drift in heading over a 2.5-min match,
    // blend robot heading toward the Hub when aligned within 45 deg
    double effectiveYawRad =
        Math.abs(headingDiff) <= Math.toRadians(45.0)
            ? angleToHub + headingDiff * 0.12
            : angleToHub + Math.copySign(Math.toRadians(4.0), headingDiff);

    double horizSpeed = launchSpeedMps * Math.cos(launchPitchRad);
    double vz = launchSpeedMps * Math.sin(launchPitchRad);
    // Include shoot-on-the-move chassis velocity component (damped if stationary/near-stationary)
    double sotmScale = Math.hypot(robotVxField, robotVyField) > 1.2 ? 0.35 : 0.10;
    double vx = robotVxField * sotmScale + horizSpeed * Math.cos(effectiveYawRad);
    double vy = robotVyField * sotmScale + horizSpeed * Math.sin(effectiveYawRad);

    double launchHeightM = 0.55; // Shooter exit height on FUEL
    FuelBall launched =
        new FuelBall(
            robotX + 0.15 * Math.cos(effectiveYawRad),
            robotY + 0.15 * Math.sin(effectiveYawRad),
            launchHeightM,
            vx,
            vy,
            vz,
            shotNumber);
    balls.add(launched);

    trackers.add(
        new InFlightShotTracker(
            shotNumber,
            timeSec,
            robotX,
            robotY,
            Math.toDegrees(effectiveYawRad),
            launchSpeedMps,
            launchPitchDeg,
            distToHub,
            launched));
  }

  /** Steps the 3D Fuel physics simulation across {@code subticks} for one {@code dtSec} step. */
  public void stepSimulation(
      double dtSec,
      boolean isBlueAlliance,
      double robotX,
      double robotY,
      double robotHeadingRad,
      double robotVxField,
      double robotVyField,
      boolean intakeActive) {
    double subDt = dtSec / subticks;
    double cosH = Math.cos(robotHeadingRad);
    double sinH = Math.sin(robotHeadingRad);
    double halfL = robotLengthM * 0.5;
    double halfW = robotWidthM * 0.5;

    for (int s = 0; s < subticks; s++) {
      for (FuelBall b : balls) {
        if (!b.active) {
          continue;
        }

        // Aerodynamic drag + gravity
        double speed = Math.sqrt(b.vx * b.vx + b.vy * b.vy + b.vz * b.vz);
        if (speed > 0.01) {
          double dragAccel = AIR_DRAG_COEFF * speed;
          b.vx -= (b.vx / speed) * dragAccel * subDt;
          b.vy -= (b.vy / speed) * dragAccel * subDt;
          b.vz -= (b.vz / speed) * dragAccel * subDt;
        }
        b.vz -= GRAVITY_MPS2 * subDt;

        b.x += b.vx * subDt;
        b.y += b.vy * subDt;
        b.z += b.vz * subDt;

        // Floor bounce & rolling friction
        if (b.z <= FUEL_RADIUS_M) {
          b.z = FUEL_RADIUS_M;
          if (b.vz < -0.25) {
            b.vz = -b.vz * FLOOR_COR;
          } else {
            b.vz = 0.0;
          }
          double groundSpeed = Math.hypot(b.vx, b.vy);
          if (groundSpeed > 0.01) {
            double decel = Math.min(groundSpeed, ROLLING_FRICTION * subDt);
            b.vx -= (b.vx / groundSpeed) * decel;
            b.vy -= (b.vy / groundSpeed) * decel;
          } else {
            b.vx = 0.0;
            b.vy = 0.0;
          }
        }

        // Field perimeter walls (FIELD_LINES)
        if (b.x < FUEL_RADIUS_M) {
          b.x = FUEL_RADIUS_M;
          b.vx = Math.abs(b.vx) * WALL_COR;
        } else if (b.x > FIELD_LENGTH_M - FUEL_RADIUS_M) {
          b.x = FIELD_LENGTH_M - FUEL_RADIUS_M;
          b.vx = -Math.abs(b.vx) * WALL_COR;
        }
        if (b.y < FUEL_RADIUS_M) {
          b.y = FUEL_RADIUS_M;
          b.vy = Math.abs(b.vy) * WALL_COR;
        } else if (b.y > FIELD_WIDTH_M - FUEL_RADIUS_M) {
          b.y = FIELD_WIDTH_M - FUEL_RADIUS_M;
          b.vy = -Math.abs(b.vy) * WALL_COR;
        }

        // Trench collisions (TRENCH_COLLIDERS)
        for (double[] tr : TRENCH_BOXES) {
          if (b.z <= tr[4]
              && b.x + FUEL_RADIUS_M > tr[0]
              && b.x - FUEL_RADIUS_M < tr[1]
              && b.y + FUEL_RADIUS_M > tr[2]
              && b.y - FUEL_RADIUS_M < tr[3]) {
            double cx = 0.5 * (tr[0] + tr[1]);
            double cy = 0.5 * (tr[2] + tr[3]);
            if (Math.abs(b.x - cx) > Math.abs(b.y - cy)) {
              b.vx = -b.vx * WALL_COR;
              b.x = b.x < cx ? tr[0] - FUEL_RADIUS_M : tr[1] + FUEL_RADIUS_M;
            } else {
              b.vy = -b.vy * WALL_COR;
              b.y = b.y < cy ? tr[2] - FUEL_RADIUS_M : tr[3] + FUEL_RADIUS_M;
            }
          }
        }

        // Hub scoring & Hub/Net collisions (Blue Hub & Red Hub)
        checkHubAndNetInteraction(b, BLUE_HUB_X, BLUE_HUB_Y, true);
        checkHubAndNetInteraction(b, RED_HUB_X, RED_HUB_Y, false);

        // Robot intake & bumper collision (when ball is near floor)
        if (b.active && b.z <= bumperHeightM + FUEL_RADIUS_M) {
          double relX = b.x - robotX;
          double relY = b.y - robotY;
          // Transform into robot local frame (+X forward, +Y left)
          double localX = relX * cosH + relY * sinH;
          double localY = -relX * sinH + relY * cosH;

          // Intake bounding box at front of robot: X in [halfL - 0.15 .. halfL + 0.32], |Y| <= 0.36
          if (intakeActive
              && b.shotId == 0
              && localX >= halfL - 0.15
              && localX <= halfL + 0.32
              && Math.abs(localY) <= 0.36) {
            b.active = false;
            intakedCount++;
            hopperFuelCount = Math.min(5, hopperFuelCount + 1);
            continue;
          }

          // Oriented bounding box bumper push
          if (Math.abs(localX) < halfL + FUEL_RADIUS_M
              && Math.abs(localY) < halfW + FUEL_RADIUS_M) {
            double penX = (halfL + FUEL_RADIUS_M) - Math.abs(localX);
            double penY = (halfW + FUEL_RADIUS_M) - Math.abs(localY);
            if (penX < penY) {
              double signX = localX >= 0 ? 1.0 : -1.0;
              localX = signX * (halfL + FUEL_RADIUS_M);
              double nx = signX * cosH;
              double ny = signX * sinH;
              b.vx = robotVxField + nx * 1.6;
              b.vy = robotVyField + ny * 1.6;
            } else {
              double signY = localY >= 0 ? 1.0 : -1.0;
              localY = signY * (halfW + FUEL_RADIUS_M);
              double nx = -signY * sinH;
              double ny = signY * cosH;
              b.vx = robotVxField + nx * 1.6;
              b.vy = robotVyField + ny * 1.6;
            }
            b.x = robotX + (localX * cosH - localY * sinH);
            b.y = robotY + (localX * sinH + localY * cosH);
          }
        }
      }

      int n = balls.size();
      double minDist = 2.0 * FUEL_RADIUS_M;
      for (int i = 0; i < n; i++) {
        FuelBall a = balls.get(i);
        if (!a.active) {
          continue;
        }
        for (int j = i + 1; j < n; j++) {
          FuelBall b = balls.get(j);
          if (!b.active) {
            continue;
          }
          double dx = b.x - a.x;
          double dy = b.y - a.y;
          double dz = b.z - a.z;
          double distSq = dx * dx + dy * dy + dz * dz;
          if (distSq > 1e-6 && distSq < minDist * minDist) {
            double dist = Math.sqrt(distSq);
            double nx = dx / dist;
            double ny = dy / dist;
            double nz = dz / dist;
            double overlap = 0.5 * (minDist - dist);
            a.x -= nx * overlap;
            a.y -= ny * overlap;
            a.z = Math.max(FUEL_RADIUS_M, a.z - nz * overlap);
            b.x += nx * overlap;
            b.y += ny * overlap;
            b.z = Math.max(FUEL_RADIUS_M, b.z + nz * overlap);

            double rvx = b.vx - a.vx;
            double rvy = b.vy - a.vy;
            double rvz = b.vz - a.vz;
            double velAlongNormal = rvx * nx + rvy * ny + rvz * nz;
            if (velAlongNormal < 0) {
              double impulse = -(1.0 + BALL_COR) * velAlongNormal * 0.5;
              a.vx -= impulse * nx;
              a.vy -= impulse * ny;
              a.vz -= impulse * nz;
              b.vx += impulse * nx;
              b.vy += impulse * ny;
              b.vz += impulse * nz;
            }
          }
        }
      }

      double targetHubX = isBlueAlliance ? BLUE_HUB_X : RED_HUB_X;
      double targetHubY = isBlueAlliance ? BLUE_HUB_Y : RED_HUB_Y;
      for (InFlightShotTracker tr : trackers) {
        if (tr.finalized) {
          continue;
        }
        if (tr.ball.z > tr.maxApexZM) {
          tr.maxApexZM = tr.ball.z;
        }
        double dHub = Math.hypot(tr.ball.x - targetHubX, tr.ball.y - targetHubY);
        if (dHub < tr.minHubDistM) {
          tr.minHubDistM = dHub;
        }
        if (tr.ball.scored) {
          tr.scored = true;
          tr.finalized = true;
          recordTrackerOutcome(tr);
        } else if (!tr.ball.active || (tr.ball.z <= FUEL_RADIUS_M + 0.02 && tr.ball.vz <= 0.0)) {
          tr.finalized = true;
          recordTrackerOutcome(tr);
        }
      }
    }
  }

  private void checkHubAndNetInteraction(
      FuelBall b, double hubX, double hubY, boolean isBlueHub) {
    double dx = b.x - hubX;
    double dy = b.y - hubY;
    double radialDist = Math.hypot(dx, dy);

    if (b.vz < 0
        && b.z >= HUB_RIM_HEIGHT_M - 0.15
        && b.z <= HUB_FUNNEL_MAX_Z_M
        && radialDist <= HUB_SCORE_RADIUS_M) {
      b.scored = true;
      if (isBlueHub) {
        blueHubScore++;
      } else {
        redHubScore++;
      }
      // Route scored ball out the bottom return chute onto the alliance floor with gentle roll
      double ejectAngle = isBlueHub ? Math.PI : 0.0;
      b.x = hubX + Math.cos(ejectAngle) * (HUB_OUTER_RADIUS_M + 0.18);
      b.y = hubY + ( (blueHubScore + redHubScore) % 2 == 0 ? 0.25 : -0.25);
      b.z = FUEL_RADIUS_M;
      b.vx = Math.cos(ejectAngle) * 1.1;
      b.vy = 0.0;
      b.vz = 0.0;
      b.shotId = 0;
      return;
    }

    if (b.z > NET_MIN_Z_M && b.z < NET_MAX_Z_M && radialDist <= 0.25 && Math.abs(dx) < 0.18) {
      b.vx *= NET_COR;
      b.vy *= NET_COR;
    }

    double outerWallRadius = b.z < 1.35 ? 0.45 : 0.58;
    if (b.z < HUB_RIM_HEIGHT_M - 0.08 && radialDist < outerWallRadius + FUEL_RADIUS_M) {
      if (radialDist > 1e-4) {
        double nx = dx / radialDist;
        double ny = dy / radialDist;
        b.x = hubX + nx * (outerWallRadius + FUEL_RADIUS_M);
        b.y = hubY + ny * (outerWallRadius + FUEL_RADIUS_M);
        double vn = b.vx * nx + b.vy * ny;
        if (vn < 0) {
          b.vx -= (1.0 + WALL_COR) * vn * nx;
          b.vy -= (1.0 + WALL_COR) * vn * ny;
        }
      }
    }
  }

  private void recordTrackerOutcome(InFlightShotTracker tr) {
    shotOutcomes.add(
        new SimulatedShotOutcome(
            tr.shotNumber,
            tr.launchTimeSec,
            tr.launchX,
            tr.launchY,
            tr.launchHeadingDeg,
            tr.launchSpeedMps,
            tr.launchPitchDeg,
            tr.targetDistM,
            tr.scored,
            tr.minHubDistM,
            tr.maxApexZM));
  }

  /** Finalizes any still-airborne shots at the end of the log. */
  public void finishSimulation() {
    for (InFlightShotTracker tr : trackers) {
      if (!tr.finalized) {
        tr.finalized = true;
        tr.scored = tr.minHubDistM <= HUB_SCORE_RADIUS_M * 1.15;
        recordTrackerOutcome(tr);
      }
    }
  }

  /**
   * Returns packed {@code [x0, y0, z0, x1, y1, z1, ...]} coordinates for all active Fuel balls so
   * they can be written directly to a {@code struct:Translation3d[]} entry in {@code .wpilog}.
   */
  public double[] getActiveFuelTranslations3d() {
    int activeCount = 0;
    for (FuelBall b : balls) {
      if (b.active) {
        activeCount++;
      }
    }
    double[] out = new double[activeCount * 3];
    int idx = 0;
    for (FuelBall b : balls) {
      if (b.active) {
        out[idx++] = b.x;
        out[idx++] = b.y;
        out[idx++] = b.z;
      }
    }
    return out;
  }

  public int getBlueHubScore() {
    return blueHubScore;
  }

  public int getRedHubScore() {
    return redHubScore;
  }

  public int getIntakedCount() {
    return intakedCount;
  }

  public int getHopperFuelCount() {
    return hopperFuelCount;
  }

  public List<SimulatedShotOutcome> getShotOutcomes() {
    return shotOutcomes;
  }

  static FuelSimMetrics writeMergedLog(
      Path outPath,
      String runId,
      long t0Us,
      long windowEndUs,
      String alliance,
      RobotProfile profile,
      List<ParsedLog> allLogs,
      ParsedLog rioLog,
      ParsedLog driveLog,
      ParsedLog mechLog,
      List<ShotEvent> shots,
      List<VoiceCorrelation> voiceCorrelations,
      BatteryMetrics battery,
      List<TimelinePoint> timeline)
      throws IOException {
    FuelSimMetrics fuelSimMetrics;
    try (Writer writer = new Writer(outPath, "FluxProcessor Merged Log + FuelSim 3D")) {
      writer.registerStandardGeometrySchemas(t0Us);

      int eVolt = writer.startEntry("/FluxAudit/BatteryVoltage", "double", "", t0Us);
      int eDriveStator = writer.startEntry("/FluxAudit/TotalDriveStatorAmps", "double", "", t0Us);
      int eSupply = writer.startEntry("/FluxAudit/TotalSupplyAmps", "double", "", t0Us);
      int eSpeed = writer.startEntry("/FluxAudit/ChassisSpeedMps", "double", "", t0Us);
      int eRint = writer.startEntry("/FluxAudit/BatteryEstimatedRintMilliOhms", "double", "", t0Us);
      int eBattName = writer.startEntry("/FluxAudit/NamedBattery", "string", "", t0Us);

      int eRobotPose = writer.startEntry("/Odometry/Robot", "struct:Pose2d", "", t0Us);
      int eAkitPose =
          writer.startEntry("/AdvantageKit/RealOutputs/Odometry/Robot", "struct:Pose2d", "", t0Us);
      int eFuels = writer.startEntry("/FuelSim/Fuels", "struct:Translation3d[]", "", t0Us);
      int eBlueScore = writer.startEntry("/FuelSim/BlueHubScore", "int64", "", t0Us);
      int eRedScore = writer.startEntry("/FuelSim/RedHubScore", "int64", "", t0Us);
      int eIntaked = writer.startEntry("/FuelSim/IntakedCount", "int64", "", t0Us);
      int eHopper = writer.startEntry("/FuelSim/HopperCount", "int64", "", t0Us);
      int eShotStatus = writer.startEntry("/FuelSim/LastShotOutcome", "string", "", t0Us);

      writer.appendDouble(eRint, t0Us, battery.estimatedRintMilliOhms());
      writer.appendString(eBattName, t0Us, battery.batteryName());

      for (TimelinePoint pt : timeline) {
        long ts = t0Us + Math.round(pt.timeSec() * 1_000_000.0);
        writer.appendDouble(eVolt, ts, pt.batteryVoltage());
        writer.appendDouble(eDriveStator, ts, pt.totalDriveStatorAmps());
        writer.appendDouble(eSupply, ts, pt.totalSupplyAmps());
        writer.appendDouble(eSpeed, ts, pt.chassisSpeedMps());
      }

      fuelSimMetrics =
          simulateAndWriteFuelSimTimeline(
              writer,
              outPath.getParent(),
              runId,
              t0Us,
              windowEndUs,
              alliance,
              battery.batteryName(),
              profile,
              allLogs,
              driveLog,
              mechLog,
              shots,
              timeline,
              eRobotPose,
              eAkitPose,
              eFuels,
              eBlueScore,
              eRedScore,
              eIntaked,
              eHopper,
              eShotStatus);

      if (!voiceCorrelations.isEmpty()) {
        int eVoice = writer.startEntry("/VoiceNotes/CorrelatedDiagnosis", "string", "", t0Us);
        for (VoiceCorrelation vc : voiceCorrelations) {
          long ts = t0Us + Math.round(vc.spokenTimeSec() * 1_000_000.0);
          writer.appendString(
              eVoice,
              ts,
              "["
                  + vc.speakerRole()
                  + "] \""
                  + vc.transcript()
                  + "\" -> "
                  + vc.correlatedEventSummary()
                  + ": "
                  + vc.sensorDiagnosis());
        }
      }

      copyLogSeries(writer, "/Rio/", rioLog);
      if (driveLog != rioLog) {
        copyLogSeries(writer, "/DriveBus/", driveLog);
      }
      if (mechLog != rioLog && mechLog != driveLog) {
        copyLogSeries(writer, "/MechBus/", mechLog);
      }
    }
    return fuelSimMetrics;
  }

  private static FuelSimMetrics simulateAndWriteFuelSimTimeline(
      Writer writer,
      Path targetRunDir,
      String runId,
      long t0Us,
      long windowEndUs,
      String alliance,
      String batteryName,
      RobotProfile profile,
      List<ParsedLog> allLogs,
      ParsedLog driveLog,
      ParsedLog mechLog,
      List<ShotEvent> shots,
      List<TimelinePoint> timeline,
      int eRobotPose,
      int eAkitPose,
      int eFuels,
      int eBlueScore,
      int eRedScore,
      int eIntaked,
      int eHopper,
      int eShotStatus)
      throws IOException {
    boolean isBlue = alliance == null || !alliance.toLowerCase(Locale.ROOT).startsWith("red");
    double hubX = isBlue ? BLUE_HUB_X : RED_HUB_X;
    double hubY = isBlue ? BLUE_HUB_Y : RED_HUB_Y;
    double startX = isBlue ? 3.55 : 12.99;
    double startY =
        alliance != null && alliance.contains("1")
            ? 4.65
            : (alliance != null && alliance.contains("3") ? 3.45 : 4.21);
    double startHeadingDeg = isBlue ? 0.0 : 180.0;

    List<DoubleSample> akitX =
        RunAnalyzer.findDoubleSeries(
            allLogs,
            driveLog,
            List.of(
                "/RealOutputs/Odometry/Robot.x",
                "RealOutputs/Odometry/Robot.x",
                "/AdvantageKit/RealOutputs/Odometry/Robot.x",
                "/Odometry/Robot.x"));
    List<DoubleSample> akitY =
        RunAnalyzer.findDoubleSeries(
            allLogs,
            driveLog,
            List.of(
                "/RealOutputs/Odometry/Robot.y",
                "RealOutputs/Odometry/Robot.y",
                "/AdvantageKit/RealOutputs/Odometry/Robot.y",
                "/Odometry/Robot.y"));
    List<DoubleSample> akitRot =
        RunAnalyzer.findDoubleSeries(
            allLogs,
            driveLog,
            List.of(
                "/RealOutputs/Odometry/Robot.rotRad",
                "RealOutputs/Odometry/Robot.rotRad",
                "/AdvantageKit/RealOutputs/Odometry/Robot.rotRad",
                "/Odometry/Robot.rotRad"));

    int pigeonId = profile.pigeonId() > 0 ? profile.pigeonId() : 20;
    List<DoubleSample> yawSeries =
        RunAnalyzer.findDoubleSeries(
            allLogs, driveLog, List.of("Phoenix6/Pigeon2-" + pigeonId + "/Yaw", "/Drive/GyroYawDeg"));

    List<List<DoubleSample>> drivePosSeries = new ArrayList<>();
    List<List<DoubleSample>> steerPosSeries = new ArrayList<>();
    for (RobotProfile.CornerConfig c : profile.corners().values()) {
      drivePosSeries.add(
          RunAnalyzer.findDoubleSeries(
              allLogs,
              driveLog,
              List.of(
                  "Phoenix6/TalonFX-" + c.driveMotorId() + "/Position",
                  c.driveVelocityTopic().replace("Velocity", "Position"))));
      steerPosSeries.add(
          RunAnalyzer.findDoubleSeries(
              allLogs,
              driveLog,
              List.of(
                  "Phoenix6/TalonFX-" + c.steerMotorId() + "/Position",
                  "Phoenix6/CANcoder-" + c.cancoderId() + "/AbsolutePosition")));
    }

    List<DoubleSample> intakeVelSeries = Collections.emptyList();
    List<DoubleSample> shooterVelSeries = Collections.emptyList();
    List<DoubleSample> hoodPosSeries = Collections.emptyList();
    List<DoubleSample> elevPosSeries =
        RunAnalyzer.findDoubleSeries(
            allLogs,
            mechLog,
            List.of(
                "/Elevator/PositionRot",
                "Elevator/PositionRot",
                "/Superstructure/ElevatorHeightMeters"));
    List<DoubleSample> armPosSeries =
        RunAnalyzer.findDoubleSeries(
            allLogs,
            mechLog,
            List.of("/ArmPivot/PositionRot", "ArmPivot/PositionRot", "/Superstructure/ArmAngleRad"));
    for (RobotProfile.MechMotorSpec m : profile.mechanisms()) {
      String lower = m.name().toLowerCase(Locale.ROOT);
      if (lower.contains("intake") && intakeVelSeries.isEmpty()) {
        intakeVelSeries = RunAnalyzer.findDoubleSeriesByTopic(allLogs, mechLog, m.velocityTopic());
      } else if (m.isShooter() && shooterVelSeries.isEmpty()) {
        shooterVelSeries = RunAnalyzer.findDoubleSeriesByTopic(allLogs, mechLog, m.velocityTopic());
      } else if (lower.contains("hood") && hoodPosSeries.isEmpty()) {
        hoodPosSeries = RunAnalyzer.findDoubleSeriesByTopic(allLogs, mechLog, m.positionTopic());
      } else if (lower.contains("elevator") && elevPosSeries.isEmpty()) {
        elevPosSeries = RunAnalyzer.findDoubleSeriesByTopic(allLogs, mechLog, m.positionTopic());
      } else if (lower.contains("arm") && armPosSeries.isEmpty()) {
        armPosSeries = RunAnalyzer.findDoubleSeriesByTopic(allLogs, mechLog, m.positionTopic());
      }
    }

    FuelSimEngine fuelSim = new FuelSimEngine();
    fuelSim.spawnStartingFuel();

    double x = startX;
    double y = startY;
    double yaw0 = RunAnalyzer.sampleDoubleAt(yawSeries, t0Us, 0.0);
    double couplingRatio = 4.5;
    double metersPerRotorRot =
        (2.0 * Math.PI * profile.wheelRadiusMeters()) / Math.max(1.0, profile.driveGearRatio());

    double[] prevDrivePos = new double[drivePosSeries.size()];
    double[] prevSteerPos = new double[steerPosSeries.size()];
    for (int m = 0; m < drivePosSeries.size(); m++) {
      prevDrivePos[m] = RunAnalyzer.sampleDoubleAt(drivePosSeries.get(m), t0Us, 0.0);
      prevSteerPos[m] = RunAnalyzer.sampleDoubleAt(steerPosSeries.get(m), t0Us, 0.0);
    }

    long dtUs = 20_000L;
    double dtSec = 0.020;
    long maxSpanUs = Math.min(Math.max(t0Us + dtUs, windowEndUs), t0Us + 200_000_000L);
    int nextShotIdx = 0;
    int prevOutcomesCount = 0;
    int stepCounter = 0;
    int tlIdx = 0;
    String lastShotBanner = "";
    List<Sim3DFrame> simFrames = new ArrayList<>();

    for (long tsUs = t0Us; tsUs <= maxSpanUs; tsUs += dtUs) {
      stepCounter++;
      double relTimeSec = (tsUs - t0Us) / 1_000_000.0;
      double prevX = x;
      double prevY = y;
      double headingRad;

      if (!akitX.isEmpty() && !akitY.isEmpty() && !akitRot.isEmpty()) {
        x = RunAnalyzer.sampleDoubleAt(akitX, tsUs, x);
        y = RunAnalyzer.sampleDoubleAt(akitY, tsUs, y);
        headingRad = RunAnalyzer.sampleDoubleAt(akitRot, tsUs, Math.toRadians(startHeadingDeg));
      } else {
        double rawYawDeg = RunAnalyzer.sampleDoubleAt(yawSeries, tsUs, yaw0);
        double headingDeg = (rawYawDeg - yaw0) + startHeadingDeg;
        headingRad = Math.toRadians(headingDeg);

        if (!drivePosSeries.isEmpty()) {
          double dxRob = 0.0;
          double dyRob = 0.0;
          int nMod = drivePosSeries.size();
          for (int m = 0; m < nMod; m++) {
            double curDrive = RunAnalyzer.sampleDoubleAt(drivePosSeries.get(m), tsUs, prevDrivePos[m]);
            double curSteer = RunAnalyzer.sampleDoubleAt(steerPosSeries.get(m), tsUs, prevSteerPos[m]);
            double dSteer = curSteer - prevSteerPos[m];
            double dRotor = (curDrive - prevDrivePos[m]) - (dSteer * couplingRatio);
            double dMeters = dRotor * metersPerRotorRot;
            double modAngleRad = 2.0 * Math.PI * curSteer;
            dxRob += dMeters * Math.cos(modAngleRad) / nMod;
            dyRob += dMeters * Math.sin(modAngleRad) / nMod;
            prevDrivePos[m] = curDrive;
            prevSteerPos[m] = curSteer;
          }
          double dxField = dxRob * Math.cos(headingRad) - dyRob * Math.sin(headingRad);
          double dyField = dxRob * Math.sin(headingRad) + dyRob * Math.cos(headingRad);
          if (Math.hypot(dxField, dyField) < 0.35) {
            x = Math.max(0.45, Math.min(FIELD_LENGTH_M - 0.45, x + dxField));
            y = Math.max(0.45, Math.min(FIELD_WIDTH_M - 0.45, y + dyField));
          }
        }

        if (nextShotIdx < shots.size()) {
          ShotEvent upcoming = shots.get(nextShotIdx);
          double timeToShot = upcoming.timestampSec() - relTimeSec;
          if (timeToShot >= 0.0 && timeToShot <= 2.5) {
            double expectedDist = nominalDistanceForHood(upcoming.hoodPositionRot());
            double dxFromHub = x - hubX;
            double dyFromHub = y - hubY;
            if (isBlue && dxFromHub > -0.45) {
              dxFromHub = -Math.max(0.85, Math.abs(dxFromHub));
            } else if (!isBlue && dxFromHub < 0.45) {
              dxFromHub = Math.max(0.85, Math.abs(dxFromHub));
            }
            double approachAngle = Math.atan2(dyFromHub, dxFromHub);
            double targetX = hubX + expectedDist * Math.cos(approachAngle);
            double targetY =
                Math.max(
                    0.75,
                    Math.min(
                        FIELD_WIDTH_M - 0.75,
                        hubY + expectedDist * Math.sin(approachAngle)));
            double alpha = timeToShot <= 0.15 ? 0.38 : 0.08;
            x = x + alpha * (targetX - x);
            y = y + alpha * (targetY - y);
          }
        }
      }

      double vxField = (x - prevX) / dtSec;
      double vyField = (y - prevY) / dtSec;
      boolean intakeActive = Math.abs(RunAnalyzer.sampleDoubleAt(intakeVelSeries, tsUs, 0.0)) > 15.0;

      while (nextShotIdx < shots.size()
          && relTimeSec >= shots.get(nextShotIdx).timestampSec()) {
        ShotEvent sh = shots.get(nextShotIdx);
        int ballsInBurst = Math.max(1, Math.min(4, (int) Math.round(sh.durationSec() / 0.40)));
        for (int bIdx = 0; bIdx < ballsInBurst; bIdx++) {
          fuelSim.launchFuelFromLog(
              sh.shotNumber() * 10 + (bIdx + 1),
              relTimeSec,
              isBlue,
              x,
              y,
              headingRad,
              vxField + (bIdx * 0.06),
              vyField,
              sh.shooterLeftRps(),
              sh.hoodPositionRot());
        }
        nextShotIdx++;
      }

      fuelSim.stepSimulation(dtSec, isBlue, x, y, headingRad, vxField, vyField, intakeActive);

      writer.appendPose2d(eRobotPose, tsUs, x, y, headingRad);
      writer.appendPose2d(eAkitPose, tsUs, x, y, headingRad);
      writer.appendTranslation3dArray(eFuels, tsUs, fuelSim.getActiveFuelTranslations3d());
      writer.appendInt64(eBlueScore, tsUs, fuelSim.getBlueHubScore());
      writer.appendInt64(eRedScore, tsUs, fuelSim.getRedHubScore());
      writer.appendInt64(eIntaked, tsUs, fuelSim.getIntakedCount());
      writer.appendInt64(eHopper, tsUs, fuelSim.getHopperFuelCount());

      List<SimulatedShotOutcome> outcomes = fuelSim.getShotOutcomes();
      while (prevOutcomesCount < outcomes.size()) {
        SimulatedShotOutcome o = outcomes.get(prevOutcomesCount++);
        lastShotBanner =
            String.format(
                Locale.US,
                "Ball #%d @ %.1fs (dist=%.2fm, v=%.1fm/s, pitch=%.1f°): %s (minHubDist=%.2fm, apex=%.2fm)",
                o.shotNumber(),
                o.launchTimeSec(),
                o.targetDistanceM(),
                o.launchSpeedMps(),
                o.launchPitchDeg(),
                o.scoredInHub() ? "SCORED IN HUB" : "MISSED / REBOUND",
                o.closestHubDistM(),
                o.apexHeightM());
        writer.appendString(eShotStatus, tsUs, lastShotBanner);
      }

      double[] activeBalls = fuelSim.getActiveFuelTranslations3d();
      boolean anyAirborne = false;
      for (int bi = 2; bi < activeBalls.length; bi += 3) {
        if (activeBalls[bi] > 0.14) {
          anyAirborne = true;
          break;
        }
      }
      if ((stepCounter % 4 == 1) || (anyAirborne && stepCounter % 2 == 1)) {
        while (tlIdx + 1 < timeline.size() && timeline.get(tlIdx + 1).timeSec() <= relTimeSec) {
          tlIdx++;
        }
        TimelinePoint tp = !timeline.isEmpty() ? timeline.get(tlIdx) : null;
        double vBatt = tp != null ? tp.batteryVoltage() : 12.4;
        double dStator = tp != null ? tp.totalDriveStatorAmps() : 0.0;
        double sRps =
            Math.abs(
                RunAnalyzer.sampleDoubleAt(
                    shooterVelSeries, tsUs, tp != null ? tp.shooterLeftRps() : 0.0));
        double hRot = Math.abs(RunAnalyzer.sampleDoubleAt(hoodPosSeries, tsUs, 0.0));
        double elevRot = Math.max(0.0, RunAnalyzer.sampleDoubleAt(elevPosSeries, tsUs, 0.0));
        double elevM = elevRot > 2.0 ? Math.min(1.45, elevRot * 0.022) : Math.min(1.45, elevRot);
        double armRot = RunAnalyzer.sampleDoubleAt(armPosSeries, tsUs, 0.0);
        double armRad = Math.abs(armRot) <= 1.5 ? armRot * 2.0 * Math.PI : Math.toRadians(armRot);
        simFrames.add(
            new Sim3DFrame(
                relTimeSec,
                x,
                y,
                headingRad,
                vBatt,
                dStator,
                sRps,
                hRot,
                elevM,
                armRad,
                fuelSim.getBlueHubScore(),
                fuelSim.getRedHubScore(),
                fuelSim.getIntakedCount(),
                lastShotBanner,
                activeBalls));
      }
    }
    fuelSim.finishSimulation();

    String videoFileName =
        targetRunDir != null
            ? FuelSimVideoRenderer.renderMp4IfAvailable(
                targetRunDir, runId, profile.robotName(), alliance, batteryName, simFrames)
            : null;

    List<SimulatedShotOutcome> allOutcomes = fuelSim.getShotOutcomes();
    int totalLaunched = allOutcomes.size();
    int totalScored = 0;
    for (SimulatedShotOutcome o : allOutcomes) {
      if (o.scoredInHub()) {
        totalScored++;
      }
    }
    List<ShotSimSummary> shotSummaries = new ArrayList<>();
    for (ShotEvent sh : shots) {
      int burstCount = 0;
      int burstScored = 0;
      double distM = nominalDistanceForHood(sh.hoodPositionRot());
      double speedMps = 0.0;
      double pitchDeg = 65.0 - sh.hoodPositionRot() * 2.1;
      double apexM = 0.0;
      for (SimulatedShotOutcome o : allOutcomes) {
        if (o.shotNumber() / 10 == sh.shotNumber()) {
          burstCount++;
          if (o.scoredInHub()) {
            burstScored++;
          }
          distM = o.targetDistanceM();
          speedMps = o.launchSpeedMps();
          pitchDeg = o.launchPitchDeg();
          apexM = Math.max(apexM, o.apexHeightM());
        }
      }
      String outcomeLabel =
          burstCount == 0
              ? "N/A"
              : (burstScored == burstCount
                  ? "SCORED (" + burstScored + "/" + burstCount + ")"
                  : (burstScored > 0
                      ? "PARTIAL (" + burstScored + "/" + burstCount + ")"
                      : "MISSED / REBOUND (0/" + burstCount + ")"));
      shotSummaries.add(
          new ShotSimSummary(
              sh.shotNumber(),
              sh.timestampSec(),
              burstCount,
              burstScored,
              distM,
              speedMps,
              pitchDeg,
              apexM,
              outcomeLabel));
    }
    double accuracyPct = totalLaunched > 0 ? (100.0 * totalScored) / totalLaunched : 0.0;
    return new FuelSimMetrics(
        totalLaunched,
        totalScored,
        totalLaunched - totalScored,
        accuracyPct,
        fuelSim.getIntakedCount(),
        fuelSim.getBlueHubScore(),
        fuelSim.getRedHubScore(),
        videoFileName,
        shotSummaries,
        simFrames);
  }

  private static void copyLogSeries(Writer writer, String prefix, ParsedLog log)
      throws IOException {
    if (log == null) {
      return;
    }
    for (Map.Entry<String, List<BooleanSample>> e : log.allBooleanSeries().entrySet()) {
      List<BooleanSample> samples = e.getValue();
      if (!samples.isEmpty()) {
        int id = writer.startEntry(prefix + e.getKey(), "boolean", "", samples.get(0).timestampUs());
        for (BooleanSample s : samples) {
          writer.appendBoolean(id, s.timestampUs(), s.value());
        }
      }
    }
    for (Map.Entry<String, List<StringSample>> e : log.allStringSeries().entrySet()) {
      List<StringSample> samples = e.getValue();
      if (!samples.isEmpty()) {
        int id = writer.startEntry(prefix + e.getKey(), "string", "", samples.get(0).timestampUs());
        for (StringSample s : samples) {
          writer.appendString(id, s.timestampUs(), s.value());
        }
      }
    }
    for (Map.Entry<String, List<DoubleSample>> e : log.allDoubleSeries().entrySet()) {
      List<DoubleSample> samples = e.getValue();
      if (!samples.isEmpty()) {
        int id = writer.startEntry(prefix + e.getKey(), "double", "", samples.get(0).timestampUs());
        int step = samples.size() > 25000 ? 5 : 1;
        for (int i = 0; i < samples.size(); i += step) {
          DoubleSample s = samples.get(i);
          writer.appendDouble(id, s.timestampUs(), s.value());
        }
      }
    }
  }
}
