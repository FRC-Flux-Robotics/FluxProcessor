package frc.flux.processor.analysis;

import java.util.ArrayList;
import java.util.List;

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

    // 1. Neutral Zone centerline staged Fuel (8 balls along X = 8.27m)
    double centerX = FIELD_LENGTH_M * 0.5;
    double[] neutralYs = {1.20, 2.00, 2.80, 3.60, 4.47, 5.27, 6.07, 6.87};
    for (double y : neutralYs) {
      balls.add(new FuelBall(centerX, y, r, 0, 0, 0, 0));
    }

    // 2. Blue Alliance Depot & Staging (2x4 grid near Blue wall / staging zone)
    double[] blueDepotXs = {1.15, 1.45, 2.40, 2.40};
    double[] blueDepotYs = {1.80, 6.25, 2.60, 5.45};
    for (int i = 0; i < blueDepotXs.length; i++) {
      balls.add(new FuelBall(blueDepotXs[i], blueDepotYs[i], r, 0, 0, 0, 0));
      balls.add(new FuelBall(blueDepotXs[i] + 0.24, blueDepotYs[i], r, 0, 0, 0, 0));
    }

    // 3. Red Alliance Depot & Staging (mirrored)
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
      // 1. Update each active Fuel ball
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

      // 2. Fuel-to-Fuel sphere collisions
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

      // 3. Update shot trackers
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

    // 1. Check top funnel scoring into the Octagonal Hub
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

    // 2. Check Netting above the Hub backstop (absorbs momentum so ball drops into/near hub)
    if (b.z > NET_MIN_Z_M && b.z < NET_MAX_Z_M && radialDist <= 0.25 && Math.abs(dx) < 0.18) {
      b.vx *= NET_COR;
      b.vy *= NET_COR;
    }

    // 3. Check Octagonal Hub outer wall rebound (narrower base below 1.35m, flared funnel above 1.35m)
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
}
