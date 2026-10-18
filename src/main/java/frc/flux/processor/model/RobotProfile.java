package frc.flux.processor.model;

import java.util.List;
import java.util.Map;

/**
 * Hardware CAN ID mapping for FRC Team 10413 (FLUX Robotics) robots ({@code FUEL} and {@code
 * CORAL}).
 */
public record RobotProfile(
    String robotName,
    int pigeonId,
    double wheelRadiusMeters,
    double driveGearRatio,
    Map<String, CornerConfig> corners,
    List<MechMotorSpec> mechanisms) {

  public record CornerConfig(String corner, int driveMotorId, int steerMotorId, int cancoderId) {}

  public record MechMotorSpec(
      String name, int motorId, String role, double currentLimitAmps, boolean isShooter) {}

  /** Official 2026 {@code FUEL} robot CAN configuration (matches {@code Robots.FUEL}). */
  public static final RobotProfile FUEL =
      new RobotProfile(
          "FUEL",
          20,
          0.0508, // 2.0 in wheel radius
          6.394736842105262,
          Map.of(
              "FL", new CornerConfig("FL", 2, 1, 21),
              "FR", new CornerConfig("FR", 4, 3, 22),
              "BL", new CornerConfig("BL", 6, 5, 23),
              "BR", new CornerConfig("BR", 8, 7, 24)),
          List.of(
              new MechMotorSpec("Intake", 1, "Velocity Roller", 40.0, false),
              new MechMotorSpec("Feeder", 3, "Velocity Mecanum", 40.0, false),
              new MechMotorSpec("Indexer", 4, "Velocity Roller", 40.0, false),
              new MechMotorSpec("Shooter Right", 10, "Flywheel Follower", 40.0, true),
              new MechMotorSpec("Shooter Left", 11, "Flywheel Leader", 40.0, true),
              new MechMotorSpec("Hood", 12, "Position Tilt", 40.0, false),
              new MechMotorSpec("Tilter", 20, "Intake Deploy", 40.0, false)));

  /** Official 2026 {@code CORAL} robot CAN configuration (matches {@code Robots.CORAL}). */
  public static final RobotProfile CORAL =
      new RobotProfile(
          "CORAL",
          24,
          0.0508,
          6.394736842105262,
          Map.of(
              "FL", new CornerConfig("FL", 7, 8, 23),
              "FR", new CornerConfig("FR", 1, 2, 20),
              "BL", new CornerConfig("BL", 5, 6, 22),
              "BR", new CornerConfig("BR", 3, 4, 21)),
          List.of());
}
