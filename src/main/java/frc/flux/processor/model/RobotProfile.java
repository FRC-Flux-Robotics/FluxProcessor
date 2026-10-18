package frc.flux.processor.model;

import frc.flux.processor.wpilog.WpiLogReader.ParsedLog;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Hardware CAN ID and telemetry topic mapping for FRC robots.
 *
 * <p>Supports built-in Team 10413 presets ({@link #FUEL}, {@link #CORAL}), external JSON profiles
 * ({@code --profile robot-profile.json} or {@code robot-profile.json} in the log folder), and
 * zero-config {@link #AUTO} discovery across CTRE Phoenix 6 ({@code .hoot}/{@code .wpilog}),
 * AdvantageKit ({@code /Drive/Module0..3/*}), WPILib {@code DataLogManager}/DogLog/Epilogue, and
 * REV URCL logs.
 */
public record RobotProfile(
    String robotName,
    int pigeonId,
    double wheelRadiusMeters,
    double driveGearRatio,
    Map<String, CornerConfig> corners,
    List<MechMotorSpec> mechanisms) {

  public record CornerConfig(
      String corner,
      int driveMotorId,
      int steerMotorId,
      int cancoderId,
      String driveStatorTopic,
      String driveSupplyTopic,
      String driveVelocityTopic,
      String steerStatorTopic) {

    public CornerConfig(String corner, int driveMotorId, int steerMotorId, int cancoderId) {
      this(
          corner,
          driveMotorId,
          steerMotorId,
          cancoderId,
          "Phoenix6/TalonFX-" + driveMotorId + "/StatorCurrent",
          "Phoenix6/TalonFX-" + driveMotorId + "/SupplyCurrent",
          "Phoenix6/TalonFX-" + driveMotorId + "/Velocity",
          steerMotorId > 0 ? "Phoenix6/TalonFX-" + steerMotorId + "/StatorCurrent" : "");
    }
  }

  public record MechMotorSpec(
      String name,
      int motorId,
      String role,
      double currentLimitAmps,
      boolean isShooter,
      String statorTopic,
      String supplyTopic,
      String velocityTopic,
      String positionTopic,
      String resetTopic) {

    public MechMotorSpec(
        String name, int motorId, String role, double currentLimitAmps, boolean isShooter) {
      this(
          name,
          motorId,
          role,
          currentLimitAmps,
          isShooter,
          "Phoenix6/TalonFX-" + motorId + "/StatorCurrent",
          "Phoenix6/TalonFX-" + motorId + "/SupplyCurrent",
          "Phoenix6/TalonFX-" + motorId + "/Velocity",
          "Phoenix6/TalonFX-" + motorId + "/Position",
          "Phoenix6/TalonFX-" + motorId + "/ResetCount");
    }
  }

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

  /**
   * Sentinel profile that inspects each run's {@code .wpilog} entry table (or {@code
   * robot-profile.json}) to automatically detect {@code FUEL}, {@code CORAL}, or any other FRC
   * team's robot topology.
   */
  public static final RobotProfile AUTO =
      new RobotProfile("AUTO", -1, 0.0508, 6.394736842105262, Map.of(), List.of());

  private static final Pattern TALON_PATTERN =
      Pattern.compile("(?:^|/)Phoenix6/TalonFX-(\\d+)/([A-Za-z0-9_]+)$");
  private static final Pattern PIGEON_PATTERN =
      Pattern.compile("(?:^|/)Phoenix6/Pigeon2-(\\d+)/SupplyVoltage$");
  private static final Pattern AKIT_MOD_PATTERN =
      Pattern.compile(
          "(?i)^/?(?:AdvantageKit/RealOutputs/|RealOutputs/)?(?:Drive|Swerve)/Module\\s*([0-3]|FL|FR|BL|BR)/([A-Za-z0-9_]+)$");
  private static final Pattern URCL_MOTOR_PATTERN =
      Pattern.compile(
          "(?i)^/?(?:URCL|SparkMax|SparkFlex|REV)/([A-Za-z0-9_\\-]+?)(?:[-_/](\\d+))?/([A-Za-z0-9_]+)$");

  /**
   * Loads a custom {@link RobotProfile} from a JSON file (`--profile <path>` or
   * `robot-profile.json`).
   *
   * <p>Supported JSON structure:
   *
   * <pre>
   * {
   *   "robotName": "KRAKEN_BOT",
   *   "pigeonId": 1,
   *   "wheelRadiusMeters": 0.0508,
   *   "driveGearRatio": 6.12,
   *   "corners": {
   *     "FL": { "driveId": 10, "steerId": 11, "cancoderId": 12 },
   *     "FR": { "driveId": 20, "steerId": 21, "cancoderId": 22 },
   *     "BL": { "driveId": 30, "steerId": 31, "cancoderId": 32 },
   *     "BR": { "driveId": 40, "steerId": 41, "cancoderId": 42 }
   *   },
   *   "mechanisms": [
   *     { "name": "Elevator", "motorId": 50, "role": "Position Lift", "currentLimitAmps": 60.0, "isShooter": false },
   *     { "name": "Flywheel", "motorId": 51, "role": "Flywheel Leader", "currentLimitAmps": 80.0, "isShooter": true }
   *   ]
   * }
   * </pre>
   */
  public static RobotProfile fromJson(Path jsonPath) throws IOException {
    String json = Files.readString(jsonPath, StandardCharsets.UTF_8);
    return fromJsonString(json);
  }

  public static RobotProfile fromJsonString(String json) {
    String robotName = extractString(json, "robotName", "CUSTOM_ROBOT");
    int pigeonId = (int) Math.round(extractDouble(json, "pigeonId", 0));
    double wheelRadius = extractDouble(json, "wheelRadiusMeters", 0.0508);
    double gearRatio = extractDouble(json, "driveGearRatio", 6.12);

    Map<String, CornerConfig> corners = new LinkedHashMap<>();
    String[] keys = {"FL", "FR", "BL", "BR"};
    for (String k : keys) {
      Pattern cp =
          Pattern.compile("\"" + k + "\"\\s*:\\s*\\{([^}]*)\\}", Pattern.CASE_INSENSITIVE);
      Matcher cm = cp.matcher(json);
      if (cm.find()) {
        String body = cm.group(1);
        int dId =
            (int)
                Math.round(
                    extractDouble(
                        body, "driveId", extractDouble(body, "driveMotorId", 0)));
        int sId =
            (int)
                Math.round(
                    extractDouble(
                        body, "steerId", extractDouble(body, "steerMotorId", 0)));
        int cId = (int) Math.round(extractDouble(body, "cancoderId", 0));
        String dStator =
            extractString(
                body,
                "driveStatorTopic",
                "Phoenix6/TalonFX-" + dId + "/StatorCurrent");
        String dSupply =
            extractString(
                body,
                "driveSupplyTopic",
                "Phoenix6/TalonFX-" + dId + "/SupplyCurrent");
        String dVel =
            extractString(
                body, "driveVelocityTopic", "Phoenix6/TalonFX-" + dId + "/Velocity");
        String sStator =
            extractString(
                body,
                "steerStatorTopic",
                sId > 0 ? "Phoenix6/TalonFX-" + sId + "/StatorCurrent" : "");
        corners.put(
            k, new CornerConfig(k, dId, sId, cId, dStator, dSupply, dVel, sStator));
      }
    }

    List<MechMotorSpec> mechanisms = new ArrayList<>();
    Pattern mechArrPattern =
        Pattern.compile("\"mechanisms\"\\s*:\\s*\\[(.*?)\\]", Pattern.DOTALL);
    Matcher arrMatcher = mechArrPattern.matcher(json);
    if (arrMatcher.find()) {
      String arrBody = arrMatcher.group(1);
      Pattern objPattern = Pattern.compile("\\{([^}]*)\\}");
      Matcher objMatcher = objPattern.matcher(arrBody);
      int idx = 1;
      while (objMatcher.find()) {
        String body = objMatcher.group(1);
        String name = extractString(body, "name", "Mechanism-" + idx);
        int mId =
            (int)
                Math.round(
                    extractDouble(body, "motorId", extractDouble(body, "id", idx)));
        String role = extractString(body, "role", "Mechanism");
        double limit = extractDouble(body, "currentLimitAmps", 40.0);
        boolean isShooter =
            extractBoolean(
                body,
                "isShooter",
                role.toLowerCase(Locale.ROOT).contains("shooter")
                    || role.toLowerCase(Locale.ROOT).contains("flywheel")
                    || name.toLowerCase(Locale.ROOT).contains("shooter"));
        String statorT =
            extractString(
                body, "statorTopic", "Phoenix6/TalonFX-" + mId + "/StatorCurrent");
        String supplyT =
            extractString(
                body, "supplyTopic", "Phoenix6/TalonFX-" + mId + "/SupplyCurrent");
        String velT =
            extractString(body, "velocityTopic", "Phoenix6/TalonFX-" + mId + "/Velocity");
        String posT =
            extractString(body, "positionTopic", "Phoenix6/TalonFX-" + mId + "/Position");
        String resetT =
            extractString(body, "resetTopic", "Phoenix6/TalonFX-" + mId + "/ResetCount");
        mechanisms.add(
            new MechMotorSpec(
                name, mId, role, limit, isShooter, statorT, supplyT, velT, posT, resetT));
        idx++;
      }
    }

    return new RobotProfile(robotName, pigeonId, wheelRadius, gearRatio, corners, mechanisms);
  }

  /**
   * Resolves the effective {@link RobotProfile} for a given run directory and its parsed logs.
   *
   * <ul>
   *   <li>If {@code runDir/robot-profile.json} exists (and {@code requested} is {@link #AUTO}),
   *       loads the JSON profile.
   *   <li>If {@code requested} has non-empty corners (e.g., explicit {@code FUEL}, {@code CORAL},
   *       or {@code --profile}), checks whether those topics actually exist in {@code allLogs}; if
   *       none match (e.g., another team ran {@code FluxProcessor} without flags on their own
   *       AdvantageKit/REV/CTRE logs), falls back to auto-discovery so metrics are never blank.
   * </ul>
   */
  public static RobotProfile resolveForLogs(
      RobotProfile requested, List<ParsedLog> allLogs, Path runDir) {
    if (runDir != null) {
      Path localProfile = runDir.resolve("robot-profile.json");
      Path parentProfile =
          runDir.getParent() != null ? runDir.getParent().resolve("robot-profile.json") : null;
      try {
        if (Files.exists(localProfile)) {
          return fromJson(localProfile);
        }
        if ((requested == null || "AUTO".equals(requested.robotName()))
            && parentProfile != null
            && Files.exists(parentProfile)) {
          return fromJson(parentProfile);
        }
      } catch (IOException ignored) {
      }
    }

    Set<String> allTopics = new LinkedHashSet<>();
    for (ParsedLog log : allLogs) {
      allTopics.addAll(log.entriesByName().keySet());
      allTopics.addAll(log.allDoubleSeries().keySet());
    }

    // If an explicit profile was given and its drive topics exist in the logs, use it as-is
    if (requested != null
        && !"AUTO".equals(requested.robotName())
        && !requested.corners().isEmpty()
        && matchesAnyCornerTopic(requested, allTopics)) {
      return requested;
    }

    // Check if logs match Team 10413 FUEL or CORAL presets
    if ( allTopics.contains("Phoenix6/Pigeon2-20/Yaw")
        || allTopics.contains("Phoenix6/Pigeon2-20/SupplyVoltage")
        || (allTopics.contains("Phoenix6/TalonFX-2/StatorCurrent")
            && allTopics.contains("Phoenix6/TalonFX-11/Velocity"))) {
      String customName =
          (requested != null && !"AUTO".equals(requested.robotName()))
              ? requested.robotName()
              : FUEL.robotName();
      return new RobotProfile(
          customName,
          FUEL.pigeonId(),
          FUEL.wheelRadiusMeters(),
          FUEL.driveGearRatio(),
          FUEL.corners(),
          FUEL.mechanisms());
    }
    if (allTopics.contains("Phoenix6/Pigeon2-24/Yaw")
        || allTopics.contains("Phoenix6/Pigeon2-24/SupplyVoltage")) {
      String customName =
          (requested != null && !"AUTO".equals(requested.robotName()))
              ? requested.robotName()
              : CORAL.robotName();
      return new RobotProfile(
          customName,
          CORAL.pigeonId(),
          CORAL.wheelRadiusMeters(),
          CORAL.driveGearRatio(),
          CORAL.corners(),
          CORAL.mechanisms());
    }

    // Auto-discover from arbitrary FRC team logs (AdvantageKit, Phoenix 6, REV URCL, or generic)
    String inferredName =
        (requested != null
                && !"AUTO".equals(requested.robotName())
                && !"FUEL".equals(requested.robotName()))
            ? requested.robotName()
            : inferRobotNameFromLogs(allLogs, runDir);
    return autoDiscoverProfile(inferredName, allLogs, allTopics);
  }

  private static boolean matchesAnyCornerTopic(RobotProfile profile, Set<String> allTopics) {
    for (CornerConfig c : profile.corners().values()) {
      if (allTopics.contains(c.driveStatorTopic()) || allTopics.contains(c.driveVelocityTopic())) {
        return true;
      }
    }
    return false;
  }

  private static String inferRobotNameFromLogs(List<ParsedLog> allLogs, Path runDir) {
    String[] metaKeys = {
      "/AdvantageKit/Robot",
      "/RealOutputs/Robot",
      "Robot",
      "/Metadata/Robot",
      "/Metadata/ProjectName",
      "ProjectName"
    };
    for (ParsedLog log : allLogs) {
      for (String k : metaKeys) {
        var s = log.getStringSeries(k);
        if (!s.isEmpty() && !s.get(0).value().isBlank()) {
          return s.get(0).value().trim();
        }
      }
    }
    if (runDir != null && runDir.getFileName() != null) {
      String folder = runDir.getFileName().toString();
      if (folder.toUpperCase(Locale.ROOT).contains("CORAL")) {
        return "CORAL";
      }
      if (folder.toUpperCase(Locale.ROOT).contains("FUEL")) {
        return "FUEL";
      }
    }
    return "FRC-ROBOT";
  }

  private static RobotProfile autoDiscoverProfile(
      String robotName, List<ParsedLog> allLogs, Set<String> allTopics) {
    int pigeonId = 0;
    for (String t : allTopics) {
      Matcher pm = PIGEON_PATTERN.matcher(t);
      if (pm.find()) {
        pigeonId = Integer.parseInt(pm.group(1));
        break;
      }
    }

    // 1. Check AdvantageKit / Swerve Module0..3 or ModuleFL..BR topics first
    Map<String, CornerConfig> akitCorners = discoverAdvantageKitCorners(allTopics);
    if (akitCorners.size() == 4) {
      List<MechMotorSpec> mechs = discoverGenericMechanisms(allTopics, Set.of());
      return new RobotProfile(robotName, pigeonId, 0.0508, 6.12, akitCorners, mechs);
    }

    // 2. Check CTRE Phoenix 6 TalonFX-<ID> topics across logs
    Set<Integer> driveBusTalons = new LinkedHashSet<>();
    Set<Integer> allTalons = new LinkedHashSet<>();
    for (ParsedLog log : allLogs) {
      String fName = log.sourcePath().getFileName().toString().toLowerCase(Locale.ROOT);
      boolean isDriveFile = fName.contains("drive") || fName.contains("swerve");
      for (String t : log. allDoubleSeries().keySet()) {
        Matcher tm = TALON_PATTERN.matcher(t);
        if (tm.find()) {
          int id = Integer.parseInt(tm.group(1));
          allTalons.add(id);
          if (isDriveFile) {
            driveBusTalons.add(id);
          }
        }
      }
    }

    Set<Integer> candidateDriveBus = driveBusTalons.size() >= 4 ? driveBusTalons : allTalons;
    Map<String, CornerConfig> corners = new LinkedHashMap<>();
    Set<Integer> usedCornerIds = new LinkedHashSet<>();

    if (candidateDriveBus.size() >= 4) {
      // Rank TalonFX IDs by mean active velocity to separate the 4 drive motors from steer/mech motors
      List<Integer> sortedByVel = new ArrayList<>(candidateDriveBus);
      sortedByVel.sort(
          Comparator.comparingDouble(
                  (Integer id) -> -maxSeriesValue(allLogs, "Phoenix6/TalonFX-" + id + "/Velocity"))
              .thenComparingInt(Integer::intValue));

      List<Integer> driveIds = new ArrayList<>(sortedByVel.subList(0, 4));
      Collections.sort(driveIds);
      List<Integer> remainingOnDriveBus = new ArrayList<>(candidateDriveBus);
      remainingOnDriveBus.removeAll(driveIds);
      Collections.sort(remainingOnDriveBus);

      String[] cornerNames = {"FL", "FR", "BL", "BR"};
      for (int i = 0; i < 4; i++) {
        int dId = driveIds.get(i);
        int sId = i < remainingOnDriveBus.size() ? remainingOnDriveBus.get(i) : 0;
        usedCornerIds.add(dId);
        if (sId > 0) {
          usedCornerIds.add(sId);
        }
        corners.put(cornerNames[i], new CornerConfig(cornerNames[i], dId, sId, 0));
      }
    }

    List<MechMotorSpec> mechanisms = new ArrayList<>();
    List<Integer> mechIds = new ArrayList<>(allTalons);
    mechIds.removeAll(usedCornerIds);
    Collections.sort(mechIds);
    for (int mId : mechIds) {
      double peakVel = maxSeriesValue(allLogs, "Phoenix6/TalonFX-" + mId + "/Velocity");
      boolean isFastFlywheel = peakVel > 35.0;
      String role = isFastFlywheel ? "High-Speed Roller / Shooter" : "Mechanism Motor";
      mechanisms.add(new MechMotorSpec("Motor " + mId, mId, role, 40.0, isFastFlywheel));
    }

    if (mechanisms.isEmpty()) {
      mechanisms.addAll(discoverGenericMechanisms(allTopics, usedCornerIds));
    }

    return new RobotProfile(robotName, pigeonId, 0.0508, 6.12, corners, mechanisms);
  }

  private static Map<String, CornerConfig> discoverAdvantageKitCorners(Set<String> allTopics) {
    Map<String, CornerConfig> corners = new LinkedHashMap<>();
    String[][] mappings = {
      {"FL", "0", "FL", "FrontLeft"},
      {"FR", "1", "FR", "FrontRight"},
      {"BL", "2", "BL", "BackLeft"},
      {"BR", "3", "BR", "BackRight"}
    };
    for (int idx = 0; idx < mappings.length; idx++) {
      String corner = mappings[idx][0];
      String dStator =
          findFirstMatchingTopic(
              allTopics,
              List.of(
                  "Drive/Module" + idx + "/DriveCurrentAmps",
                  "Drive/Module" + corner + "/DriveCurrentAmps",
                  "Drive/Module" + mappings[idx][3] + "/DriveCurrentAmps",
                  "Swerve/Module" + idx + "/DriveCurrentAmps",
                  "Drive/DriveCurrentAmps[" + idx + "]"));
      String dSupply =
          findFirstMatchingTopic(
              allTopics,
              List.of(
                  "Drive/Module" + idx + "/DriveSupplyCurrentAmps",
                  "Drive/Module" + corner + "/DriveSupplyCurrentAmps",
                  "Drive/DriveSupplyCurrentAmps[" + idx + "]"));
      String dVel =
          findFirstMatchingTopic(
              allTopics,
              List.of(
                  "Drive/Module" + idx + "/DriveVelocityRadPerSec",
                  "Drive/Module" + idx + "/DriveVelocityMetersPerSec",
                  "Drive/Module" + idx + "/DriveVelocityRps",
                  "Drive/Module" + corner + "/DriveVelocityRadPerSec",
                  "Drive/Module" + corner + "/DriveVelocityMetersPerSec"));
      String sStator =
          findFirstMatchingTopic(
              allTopics,
              List.of(
                  "Drive/Module" + idx + "/TurnCurrentAmps",
                  "Drive/Module" + idx + "/SteerCurrentAmps",
                  "Drive/Module" + corner + "/TurnCurrentAmps",
                  "Drive/TurnCurrentAmps[" + idx + "]"));
      if (dStator != null || dVel != null) {
        corners.put(
            corner,
            new CornerConfig(
                corner,
                idx + 1,
                idx + 5,
                idx + 9,
                dStator != null ? dStator : "",
                dSupply != null ? dSupply : (dStator != null ? dStator : ""),
                dVel != null ? dVel : "",
                sStator != null ? sStator : ""));
      }
    }
    return corners;
  }

  private static List<MechMotorSpec> discoverGenericMechanisms(
      Set<String> allTopics, Set<Integer> excludeIds) {
    List<MechMotorSpec> mechs = new ArrayList<>();
    Map<String, String> statorBySubsys = new LinkedHashMap<>();
    Map<String, String> velBySubsys = new LinkedHashMap<>();
    Map<String, String> posBySubsys = new LinkedHashMap<>();

    for (String t : allTopics) {
      String lower = t.toLowerCase(Locale.ROOT);
      if (lower.contains("drive") || lower.contains("swerve") || lower.contains("module") || lower.contains("systemstats")) {
        continue;
      }
      int lastSlash = t.lastIndexOf('/');
      if (lastSlash <= 0) {
        continue;
      }
      String prefix = t.substring(0, lastSlash);
      String leaf = t.substring(lastSlash + 1).toLowerCase(Locale.ROOT);
      if (leaf.contains("current") || leaf.contains("stator") || leaf.contains("amps")) {
        statorBySubsys.putIfAbsent(prefix, t);
      } else if (leaf.contains("velocity") || leaf.contains("rpm") || leaf.contains("rps")) {
        velBySubsys.putIfAbsent(prefix, t);
      } else if (leaf.contains("position") || leaf.contains("angle") || leaf.contains("height")) {
        posBySubsys.putIfAbsent(prefix, t);
      }
    }

    int syntheticId = 1;
    for (Map.Entry<String, String> e : statorBySubsys.entrySet()) {
      String prefix = e.getKey();
      String cleanName = prefix.startsWith("/") ? prefix.substring(1) : prefix;
      cleanName = cleanName.replace("AdvantageKit/RealOutputs/", "").replace("RealOutputs/", "");
      String statorT = e.getValue();
      String velT = velBySubsys.getOrDefault(prefix, "");
      String posT = posBySubsys.getOrDefault(prefix, "");
      boolean isShooter =
          cleanName.toLowerCase(Locale.ROOT).contains("shoot")
              || cleanName.toLowerCase(Locale.ROOT).contains("flywheel")
              || cleanName.toLowerCase(Locale.ROOT).contains("launcher");
      mechs.add(
          new MechMotorSpec(
              cleanName,
              syntheticId++,
              isShooter ? "Shooter / Launcher" : "Subsystem",
              40.0,
              isShooter,
              statorT,
              statorT,
              velT,
              posT,
              ""));
    }
    return mechs;
  }

  private static String findFirstMatchingTopic(Set<String> allTopics, List<String> suffixes) {
    for (String suffix : suffixes) {
      if (allTopics.contains(suffix)) {
        return suffix;
      }
      if (allTopics.contains("/" + suffix)) {
        return "/" + suffix;
      }
      if (allTopics.contains("/AdvantageKit/" + suffix)) {
        return "/AdvantageKit/" + suffix;
      }
      if (allTopics.contains("/AdvantageKit/RealOutputs/" + suffix)) {
        return "/AdvantageKit/RealOutputs/" + suffix;
      }
      if (allTopics.contains("/RealOutputs/" + suffix)) {
        return "/RealOutputs/" + suffix;
      }
    }
    return null;
  }

  private static double maxSeriesValue(List<ParsedLog> logs, String topic) {
    double max = 0.0;
    for (ParsedLog log : logs) {
      for (var s : log.getDoubleSeries(topic)) {
        double v = Math.abs(s.value());
        if (v > max) {
          max = v;
        }
      }
    }
    return max;
  }

  private static String extractString(String json, String key, String fallback) {
    Pattern p = Pattern.compile("\"" + Pattern.quote(key) + "\"\\s*:\\s*\"([^\"]*)\"");
    Matcher m = p.matcher(json);
    return m.find() ? m.group(1).trim() : fallback;
  }

  private static double extractDouble(String json, String key, double fallback) {
    Pattern p = Pattern.compile("\"" + Pattern.quote(key) + "\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?)");
    Matcher m = p.matcher(json);
    if (m.find()) {
      try {
        return Double.parseDouble(m.group(1));
      } catch (NumberFormatException ignored) {
      }
    }
    return fallback;
  }

  private static boolean extractBoolean(String json, String key, boolean fallback) {
    Pattern p = Pattern.compile("\"" + Pattern.quote(key) + "\"\\s*:\\s*(true|false)", Pattern.CASE_INSENSITIVE);
    Matcher m = p.matcher(json);
    return m.find() ? Boolean.parseBoolean(m.group(1)) : fallback;
  }
}
