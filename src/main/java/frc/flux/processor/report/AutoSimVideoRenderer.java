package frc.flux.processor.report;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.geom.Path2D;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Standalone Headless 3D Autonomous Simulation Video ({@code .mp4}) and Interactive WebGL/Canvas
 * ({@code .html}) Renderer inside {@code FluxProcessor}.
 *
 * <p>Consumes {@code student-auto-frames.json} exported by JUnit simulation tests in {@code
 * Flux-SystemCore-2027} or {@code Flux-RoboRIO-2027} via:
 *
 * <pre>
 *   java -jar dist/flux-processor.jar --render-auto &lt;student-auto-frames.json&gt; [--out &lt;output-dir&gt;]
 * </pre>
 *
 * <p>Supports two field &amp; robot rendering modes:
 *
 * <ol>
 *   <li><b>2027 Empty Field ({@code EMPTY_FIELD_2027})</b>: SystemCore robot ({@code COMP_2027})
 *       starting at {@code (0, 0)} with 3D rising elevator, pivoting arm, and roller on an empty
 *       1 m grid.
 *   <li><b>2026 REBUILT Field with Shooting ({@code REBUILT_2026_SHOOTING})</b>: 2026 {@code FUEL}
 *       robot on the {@code 16.54m x 8.21m} 2026 REBUILT field with the 3D Hexagonal Blue Hub at
 *       {@code (4.63m, 4.21m, Z=1.83m)}, floor {@code FUEL} game-piece pickups, deploying intake
 *       tilter, adjustable shooter hood, and 3D ballistic {@code FUEL} shot trajectories.
 * </ol>
 */
public final class AutoSimVideoRenderer {
  public static final int WIDTH = 960;
  public static final int HEIGHT = 540;
  public static final int FPS = 25;

  /** 2026 Blue Alliance Hexagonal Hub center in Wall/Blue coordinates. */
  public static final double HUB_BLUE_X = 4.63;

  public static final double HUB_BLUE_Y = 4.21;
  public static final double HUB_TOP_Z = 1.83;
  public static final double HUB_RADIUS_M = 0.62;

  private AutoSimVideoRenderer() {}

  /** Single 25 Hz (40 ms) snapshot of the simulated robot during an autonomous run. */
  public record AutoFrame(
      double timeSec,
      double xMeters,
      double yMeters,
      double headingRad,
      double vxMps,
      double vyMps,
      double omegaRadPerSec,
      double elevatorHeightM,
      double armAngleRad,
      double intakeRollerRps,
      double batteryVoltage,
      double driveStatorAmps,
      double accelScale,
      String stateLabel) {
    /** 2026 FUEL shooter velocity in RPS (stored in the shared telemetry slot). */
    public double shooterRps() {
      return accelScale;
    }

    /** 2026 FUEL hood angle in motor rotations (stored in the shared telemetry slot). */
    public double hoodRotations() {
      return elevatorHeightM;
    }
  }

  /** 3D parabolic ballistic trajectory of a fired 2026 {@code FUEL} game piece. */
  public record ShotArc(
      int shotId,
      double launchTimeSec,
      double flightTimeSec,
      double startX,
      double startY,
      double startZ,
      double targetX,
      double targetY,
      double targetZ,
      double apexZ,
      boolean scored) {

    /** Evaluates the 3D ball position {@code {x, y, z}} at normalized flight fraction {@code u}. */
    public double[] positionAt(double u) {
      double clamped = Math.max(0.0, Math.min(1.0, u));
      double x = startX + clamped * (targetX - startX);
      double y = startY + clamped * (targetY - startY);
      double midLinearZ = 0.5 * (startZ + targetZ);
      double z =
          (1.0 - clamped) * startZ
              + clamped * targetZ
              + 4.0 * clamped * (1.0 - clamped) * (apexZ - midLinearZ);
      return new double[] {x, y, z};
    }
  }

  /** Floor {@code FUEL} game piece that disappears once picked up at {@code pickedUpTimeSec}. */
  public record FloorFuelBall(double xMeters, double yMeters, double pickedUpTimeSec) {}

  /** Parsed payload from {@code student-auto-frames.json}. */
  public record AutoSimPayload(
      String routineName,
      String robotName,
      boolean field2026Shooting,
      List<AutoFrame> frames,
      List<ShotArc> shots,
      List<FloorFuelBall> floorBalls,
      String autoWarning) {
    public AutoSimPayload(
        String routineName,
        String robotName,
        boolean field2026Shooting,
        List<AutoFrame> frames,
        List<ShotArc> shots,
        List<FloorFuelBall> floorBalls) {
      this(routineName, robotName, field2026Shooting, frames, shots, floorBalls, "");
    }
  }

  /**
   * Reads {@code student-auto-frames.json} from {@code jsonFile} and renders {@code
   * student-auto.html} and {@code student-auto.mp4} into {@code outputDir}.
   */
  public static AutoSimPayload renderFromJson(Path jsonFile, Path outputDir) throws IOException {
    String raw = Files.readString(jsonFile, StandardCharsets.UTF_8);
    AutoSimPayload payload = parseFramesJson(raw);
    if (payload.autoWarning() != null && !payload.autoWarning().isBlank()) {
      System.out.println("[renderAuto] WARNING: " + payload.autoWarning());
    }
    Files.createDirectories(outputDir);
    Path targetJson = outputDir.resolve("student-auto-frames.json");
    if (!jsonFile.toAbsolutePath().normalize().equals(targetJson.toAbsolutePath().normalize())) {
      writeFramesJson(
          targetJson,
          payload.routineName(),
          payload.robotName(),
          payload.frames(),
          payload.shots(),
          payload.floorBalls(),
          payload.field2026Shooting(),
          payload.autoWarning());
    }
    writeInteractiveHtml(
        outputDir.resolve("student-auto.html"),
        payload.routineName(),
        payload.robotName(),
        payload.frames(),
        payload.shots(),
        payload.floorBalls(),
        payload.field2026Shooting(),
        payload.autoWarning());
    renderMp4(
        outputDir.resolve("student-auto.mp4"),
        payload.routineName(),
        payload.robotName(),
        payload.frames(),
        payload.shots(),
        payload.floorBalls(),
        payload.field2026Shooting(),
        payload.autoWarning());
    return payload;
  }

  /**
   * Renders either the 2027 empty-field simulation or the 2026 REBUILT field shooting simulation
   * (when {@code field2026Shooting == true}) into {@code outputDir}.
   */
  public static void renderAll(
      Path outputDir,
      String routineName,
      String robotName,
      List<AutoFrame> frames,
      List<ShotArc> shots,
      List<FloorFuelBall> floorBalls,
      boolean field2026Shooting)
      throws IOException {
    Files.createDirectories(outputDir);
    Path jsonFile = outputDir.resolve("student-auto-frames.json");
    Path htmlFile = outputDir.resolve("student-auto.html");
    Path mp4File = outputDir.resolve("student-auto.mp4");

    writeFramesJson(
        jsonFile, routineName, robotName, frames, shots, floorBalls, field2026Shooting, "");
    writeInteractiveHtml(
        htmlFile, routineName, robotName, frames, shots, floorBalls, field2026Shooting, "");
    renderMp4(mp4File, routineName, robotName, frames, shots, floorBalls, field2026Shooting, "");
  }

  /** Parses {@code student-auto-frames.json} into {@link AutoSimPayload}. */
  public static AutoSimPayload parseFramesJson(String json) {
    String routineName = extractStringField(json, "routineName", "STUDENT: Default Autonomous");
    String robotName = extractStringField(json, "robotName", "COMP_2027");
    String fieldMode = extractStringField(json, "fieldMode", "EMPTY_FIELD_2027");
    String autoWarning = extractStringField(json, "autoWarning", "");
    boolean field2026 = "REBUILT_2026_SHOOTING".equalsIgnoreCase(fieldMode);

    List<FloorFuelBall> floorBalls = new ArrayList<>();
    String floorBlock = extractArrayBlock(json, "floorBalls");
    if (floorBlock != null) {
      Matcher m = Pattern.compile("\\{([^}]+)\\}").matcher(floorBlock);
      while (m.find()) {
        String obj = m.group(1);
        floorBalls.add(
            new FloorFuelBall(
                extractDouble(obj, "x", 0.0),
                extractDouble(obj, "y", 0.0),
                extractDouble(obj, "tPick", 99.0)));
      }
    }

    List<ShotArc> shots = new ArrayList<>();
    String shotsBlock = extractArrayBlock(json, "shots");
    if (shotsBlock != null) {
      Matcher m = Pattern.compile("\\{([^}]+)\\}").matcher(shotsBlock);
      while (m.find()) {
        String obj = m.group(1);
        shots.add(
            new ShotArc(
                (int) Math.round(extractDouble(obj, "id", shots.size() + 1)),
                extractDouble(obj, "t0", 0.0),
                extractDouble(obj, "tof", 0.65),
                extractDouble(obj, "x0", 0.0),
                extractDouble(obj, "y0", 0.0),
                extractDouble(obj, "z0", 0.52),
                extractDouble(obj, "x1", HUB_BLUE_X),
                extractDouble(obj, "y1", HUB_BLUE_Y),
                extractDouble(obj, "z1", HUB_TOP_Z),
                extractDouble(obj, "apexZ", 2.45),
                extractBoolean(obj, "scored", true)));
      }
    }

    List<AutoFrame> frames = new ArrayList<>();
    String framesBlock = extractArrayBlock(json, "frames");
    if (framesBlock != null) {
      Matcher m = Pattern.compile("\\{([^}]+)\\}").matcher(framesBlock);
      while (m.find()) {
        String obj = m.group(1);
        frames.add(
            new AutoFrame(
                extractDouble(obj, "t", 0.0),
                extractDouble(obj, "x", 0.0),
                extractDouble(obj, "y", 0.0),
                extractDouble(obj, "h", 0.0),
                extractDouble(obj, "vx", 0.0),
                extractDouble(obj, "vy", 0.0),
                extractDouble(obj, "om", 0.0),
                extractDouble(obj, "elev", 0.0),
                extractDouble(obj, "arm", 0.0),
                extractDouble(obj, "rol", 0.0),
                extractDouble(obj, "vBat", 12.5),
                extractDouble(obj, "iDrv", 0.0),
                extractDouble(obj, "scale", 1.0),
                extractStringField(obj, "st", "IDLE")));
      }
    }

    return new AutoSimPayload(
        routineName, robotName, field2026, frames, shots, floorBalls, autoWarning);
  }

  private static String extractArrayBlock(String json, String arrayKey) {
    int keyIdx = json.indexOf("\"" + arrayKey + "\"");
    if (keyIdx < 0) {
      return null;
    }
    int openBracket = json.indexOf('[', keyIdx);
    if (openBracket < 0) {
      return null;
    }
    int depth = 0;
    for (int i = openBracket; i < json.length(); i++) {
      char c = json.charAt(i);
      if (c == '[') {
        depth++;
      } else if (c == ']') {
        depth--;
        if (depth == 0) {
          return json.substring(openBracket + 1, i);
        }
      }
    }
    return null;
  }

  private static String extractStringField(String text, String key, String defaultVal) {
    Matcher m = Pattern.compile("\"" + Pattern.quote(key) + "\"\\s*:\\s*\"([^\"]*)\"").matcher(text);
    return m.find() ? m.group(1) : defaultVal;
  }

  private static double extractDouble(String text, String key, double defaultVal) {
    Matcher m =
        Pattern.compile("\"" + Pattern.quote(key) + "\"\\s*:\\s*([-+]?[0-9]*\\.?[0-9]+(?:[eE][-+]?[0-9]+)?)")
            .matcher(text);
    if (m.find()) {
      try {
        return Double.parseDouble(m.group(1));
      } catch (NumberFormatException ignored) {
      }
    }
    return defaultVal;
  }

  private static boolean extractBoolean(String text, String key, boolean defaultVal) {
    Matcher m = Pattern.compile("\"" + Pattern.quote(key) + "\"\\s*:\\s*(true|false)").matcher(text);
    return m.find() ? Boolean.parseBoolean(m.group(1)) : defaultVal;
  }

  private static void renderMp4(
      Path mp4File,
      String routineName,
      String robotName,
      List<AutoFrame> frames,
      List<ShotArc> shots,
      List<FloorFuelBall> floorBalls,
      boolean field2026Shooting,
      String autoWarning)
      throws IOException {
    if (frames.isEmpty()) {
      return;
    }
    if (!FuelSimVideoRenderer.isFfmpegAvailable()) {
      System.out.println(
          "[renderAuto] Info: ffmpeg not found on PATH — skipped student-auto.mp4 (open student-auto.html in your browser for the full 3D viewer).");
      return;
    }

    ProcessBuilder pb =
        new ProcessBuilder(
            "ffmpeg",
            "-y",
            "-loglevel",
            "error",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "bgr24",
            "-s",
            WIDTH + "x" + HEIGHT,
            "-r",
            String.valueOf(FPS),
            "-i",
            "-",
            "-c:v",
            "libx264",
            "-preset",
            "fast",
            "-crf",
            "22",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            mp4File.toAbsolutePath().toString());
    pb.redirectOutput(ProcessBuilder.Redirect.DISCARD);
    pb.redirectError(ProcessBuilder.Redirect.DISCARD);

    try {
      Process proc = pb.start();
      BufferedImage img = new BufferedImage(WIDTH, HEIGHT, BufferedImage.TYPE_3BYTE_BGR);
      byte[] bgrBytes = ((DataBufferByte) img.getRaster().getDataBuffer()).getData();
      Graphics2D g = img.createGraphics();
      g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
      g.setRenderingHint(
          RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);

      double camX = 0.4;
      double camY = 0.2;
      double totalDuration = frames.get(frames.size() - 1).timeSec();

      try (OutputStream stdin = proc.getOutputStream()) {
        for (int idx = 0; idx < frames.size(); idx++) {
          AutoFrame f = frames.get(idx);
          if (field2026Shooting) {
            camX = 3.15;
            camY = 4.10;
          } else if (idx == 0) {
            camX = f.xMeters() * 0.35;
            camY = f.yMeters() * 0.35;
          } else {
            camX = camX * 0.90 + (f.xMeters() * 0.35) * 0.10;
            camY = camY * 0.90 + (f.yMeters() * 0.35) * 0.10;
          }

          renderFrame(
              g,
              routineName,
              robotName,
              frames,
              shots,
              floorBalls,
              field2026Shooting,
              idx,
              camX,
              camY,
              totalDuration,
              autoWarning);
          stdin.write(bgrBytes);
        }
        stdin.flush();
      } finally {
        g.dispose();
      }

      if (!proc.waitFor(60, TimeUnit.SECONDS)) {
        proc.destroyForcibly();
      }
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }
  }

  private static double[] project(
      double wx, double wy, double wz, double camX, double camY, boolean field2026) {
    double dx = wx - camX;
    double dy = wy - camY;
    double az = Math.toRadians(-38.0);
    double el = Math.toRadians(34.0);
    double rx = dx * Math.cos(az) - dy * Math.sin(az);
    double ry = dx * Math.sin(az) + dy * Math.cos(az);
    double scale = field2026 ? 96.0 : 128.0;
    double sx = (WIDTH * 0.45) + rx * scale;
    double sy = (HEIGHT * 0.56) - (ry * Math.sin(el) + wz * Math.cos(el)) * scale;
    return new double[] {sx, sy};
  }

  private static void drawLine3D(
      Graphics2D g,
      double x0,
      double y0,
      double z0,
      double x1,
      double y1,
      double z1,
      double camX,
      double camY,
      boolean field2026) {
    double[] p0 = project(x0, y0, z0, camX, camY, field2026);
    double[] p1 = project(x1, y1, z1, camX, camY, field2026);
    g.drawLine(
        (int) Math.round(p0[0]),
        (int) Math.round(p0[1]),
        (int) Math.round(p1[0]),
        (int) Math.round(p1[1]));
  }

  private static void fillQuad3D(
      Graphics2D g,
      double x0,
      double y0,
      double z0,
      double x1,
      double y1,
      double z1,
      double x2,
      double y2,
      double z2,
      double x3,
      double y3,
      double z3,
      double camX,
      double camY,
      boolean field2026,
      Color fill,
      Color stroke) {
    double[] p0 = project(x0, y0, z0, camX, camY, field2026);
    double[] p1 = project(x1, y1, z1, camX, camY, field2026);
    double[] p2 = project(x2, y2, z2, camX, camY, field2026);
    double[] p3 = project(x3, y3, z3, camX, camY, field2026);
    Path2D path = new Path2D.Double();
    path.moveTo(p0[0], p0[1]);
    path.lineTo(p1[0], p1[1]);
    path.lineTo(p2[0], p2[1]);
    path.lineTo(p3[0], p3[1]);
    path.closePath();
    if (fill != null) {
      g.setColor(fill);
      g.fill(path);
    }
    if (stroke != null) {
      g.setColor(stroke);
      g.draw(path);
    }
  }

  private static void renderFrame(
      Graphics2D g,
      String routineName,
      String robotName,
      List<AutoFrame> frames,
      List<ShotArc> shots,
      List<FloorFuelBall> floorBalls,
      boolean field2026,
      int idx,
      double camX,
      double camY,
      double totalDuration,
      String autoWarning) {
    AutoFrame f = frames.get(idx);

    g.setColor(new Color(10, 14, 23));
    g.fillRect(0, 0, WIDTH, HEIGHT);

    if (field2026) {
      draw2026RebuiltField3D(g, f, frames, shots, floorBalls, idx, camX, camY);
    } else {
      draw2027EmptyField3D(g, f, frames, idx, camX, camY);
    }

    drawTopDownMiniMap(g, frames, shots, floorBalls, field2026, idx);

    g.setColor(new Color(15, 23, 42, 235));
    g.fillRect(0, 0, WIDTH, 46);
    g.setColor(new Color(56, 189, 248));
    g.fillRect(0, 44, WIDTH, 2);

    g.setFont(new Font("SansSerif", Font.BOLD, 13));
    g.setColor(Color.WHITE);
    String headerTitle = "FLUX 10413 • " + routineName + " (" + robotName + ")";
    while (headerTitle.length() > 24 && g.getFontMetrics().stringWidth(headerTitle) > 528) {
      headerTitle = headerTitle.substring(0, headerTitle.length() - 4) + "...";
    }
    g.drawString(headerTitle, 16, 28);

    if (autoWarning != null && !autoWarning.isBlank()) {
      g.setColor(new Color(185, 28, 28, 235));
      g.fillRoundRect(16, 54, 700, 26, 8, 8);
      g.setFont(new Font("SansSerif", Font.BOLD, 11));
      g.setColor(Color.WHITE);
      String clipped =
          autoWarning.length() > 96 ? autoWarning.substring(0, 93) + "..." : autoWarning;
      g.drawString("SafeAuto Warning: " + clipped, 26, 71);
    } else if (isEmptyStudentStarter(routineName, frames, shots)) {
      g.setColor(new Color(29, 78, 216, 232));
      g.fillRoundRect(16, 56, 706, 26, 8, 8);
      g.setFont(new Font("SansSerif", Font.BOLD, 11));
      g.setColor(Color.WHITE);
      g.drawString(
          "STUDENT NOTE: Empty starter — edit createStudentDefaultAuto() & run ./gradlew renderAuto",
          26,
          73);
    }

    // Match Clock
    g.setFont(new Font("Monospaced", Font.BOLD, 15));
    g.setColor(new Color(56, 189, 248));
    g.drawString(
        String.format(Locale.US, "T = %5.2fs / %4.1fs", f.timeSec(), totalDuration), 560, 29);

    // State Badge Pill
    Color badgeCol = stateColor(f.stateLabel(), 230);
    g.setColor(badgeCol);
    g.fillRoundRect(745, 10, 195, 26, 8, 8);
    g.setFont(new Font("SansSerif", Font.BOLD, 12));
    g.setColor(new Color(15, 23, 42));
    g.drawString("STATE: " + f.stateLabel(), 757, 27);

    g.setColor(new Color(15, 23, 42, 238));
    g.fillRect(0, HEIGHT - 56, WIDTH, 56);
    g.setColor(new Color(51, 65, 85));
    g.fillRect(0, HEIGHT - 56, WIDTH, 1);

    double speedMps = Math.hypot(f.vxMps(), f.vyMps());
    g.setFont(new Font("Monospaced", Font.BOLD, 13));
    g.setColor(new Color(226, 232, 240));
    g.drawString(
        String.format(
            Locale.US,
            "POSE: X=%+5.2fm  Y=%+5.2fm  Yaw=%+6.1f°   |   SPEED: %4.2f m/s (ω=%+4.2f rad/s)",
            f.xMeters(),
            f.yMeters(),
            Math.toDegrees(f.headingRad()),
            speedMps,
            f.omegaRadPerSec()),
        16,
        HEIGHT - 32);

    g.setColor(new Color(148, 163, 184));
    if (field2026) {
      long scoredSoFar =
          shots.stream()
              .filter(s -> s.scored() && (s.launchTimeSec() + s.flightTimeSec()) <= f.timeSec())
              .count();
      long launchedSoFar =
          shots.stream().filter(s -> s.launchTimeSec() <= f.timeSec()).count();
      g.drawString(
          String.format(
              Locale.US,
              "SHOOTER: %5.1f RPS  Hood=%4.2f rot  Intake=%+5.1f RPS  |  BATT: %5.2fV (%3.0fA)  HUB SCORES: %d / %d",
              f.shooterRps(),
              f.hoodRotations(),
              f.intakeRollerRps(),
              f.batteryVoltage(),
              f.driveStatorAmps(),
              scoredSoFar,
              launchedSoFar),
          16,
          HEIGHT - 12);
    } else {
      g.drawString(
          String.format(
              Locale.US,
              "SUPERSTRUCTURE: Elev=%4.2fm  Arm=%+5.1f°  Roller=%+5.1f RPS   |   BATT: %5.2fV (%3.0fA)  CG Scale: %3.2fx",
              f.elevatorHeightM(),
              Math.toDegrees(f.armAngleRad()),
              f.intakeRollerRps(),
              f.batteryVoltage(),
              f.driveStatorAmps(),
              f.accelScale()),
          16,
          HEIGHT - 12);
    }
  }

  private static void draw2027EmptyField3D(
      Graphics2D g, AutoFrame f, List<AutoFrame> frames, int idx, double camX, double camY) {
    fillQuad3D(
        g,
        -6.0,
        -4.0,
        0.0,
        6.0,
        -4.0,
        0.0,
        6.0,
        4.0,
        0.0,
        -6.0,
        4.0,
        0.0,
        camX,
        camY,
        false,
        new Color(18, 25, 38),
        new Color(45, 58, 82));

    g.setStroke(new BasicStroke(1.0f));
    for (int gx = -6; gx <= 6; gx++) {
      g.setColor(gx == 0 ? new Color(56, 189, 248, 120) : new Color(51, 65, 85, 110));
      drawLine3D(g, gx, -4.0, 0.0, gx, 4.0, 0.0, camX, camY, false);
    }
    for (int gy = -4; gy <= 4; gy++) {
      g.setColor(gy == 0 ? new Color(52, 211, 153, 120) : new Color(51, 65, 85, 110));
      drawLine3D(g, -6.0, gy, 0.0, 6.0, gy, 0.0, camX, camY, false);
    }

    // Dashed 5m x 3m Shop Micro-Arena reference boundary
    g.setColor(new Color(56, 189, 248, 95));
    g.setStroke(
        new BasicStroke(
            1.5f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER, 10.0f, new float[] {6.0f, 6.0f}, 0.0f));
    drawLine3D(g, -2.5, -1.5, 0.01, 2.5, -1.5, 0.01, camX, camY, false);
    drawLine3D(g, 2.5, -1.5, 0.01, 2.5, 1.5, 0.01, camX, camY, false);
    drawLine3D(g, 2.5, 1.5, 0.01, -2.5, 1.5, 0.01, camX, camY, false);
    drawLine3D(g, -2.5, 1.5, 0.01, -2.5, -1.5, 0.01, camX, camY, false);

    // Coordinate Origin Triad at (0, 0, 0)
    g.setStroke(new BasicStroke(2.5f));
    g.setColor(new Color(56, 189, 248));
    drawLine3D(g, 0.0, 0.0, 0.01, 1.0, 0.0, 0.01, camX, camY, false);
    g.setColor(new Color(52, 211, 153));
    drawLine3D(g, 0.0, 0.0, 0.01, 0.0, 1.0, 0.01, camX, camY, false);
    double[] originScreen = project(0.0, 0.0, 0.0, camX, camY, false);
    g.setColor(new Color(250, 204, 21));
    g.fillOval((int) originScreen[0] - 5, (int) originScreen[1] - 5, 10, 10);
    g.setFont(new Font("Monospaced", Font.BOLD, 12));
    g.drawString("(0,0)", (int) originScreen[0] - 18, (int) originScreen[1] + 18);

    // Full Planned Ghost Path
    g.setColor(new Color(148, 163, 184, 70));
    g.setStroke(
        new BasicStroke(
            1.4f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND, 10.0f, new float[] {4.0f, 5.0f}, 0.0f));
    for (int i = 1; i < frames.size(); i++) {
      AutoFrame a = frames.get(i - 1);
      AutoFrame b = frames.get(i);
      drawLine3D(
          g, a.xMeters(), a.yMeters(), 0.01, b.xMeters(), b.yMeters(), 0.01, camX, camY, false);
    }

    // Driven Odometry Ribbon Trail (0..idx)
    g.setStroke(new BasicStroke(3.2f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
    for (int i = 1; i <= idx; i++) {
      AutoFrame a = frames.get(i - 1);
      AutoFrame b = frames.get(i);
      g.setColor(stateColor(b.stateLabel(), 215));
      drawLine3D(
          g, a.xMeters(), a.yMeters(), 0.02, b.xMeters(), b.yMeters(), 0.02, camX, camY, false);
    }

    drawSystemCoreRobot3D(g, f, camX, camY);
  }

  private static void draw2026RebuiltField3D(
      Graphics2D g,
      AutoFrame f,
      List<AutoFrame> frames,
      List<ShotArc> shots,
      List<FloorFuelBall> floorBalls,
      int idx,
      double camX,
      double camY) {
    fillQuad3D(
        g,
        0.0,
        0.5,
        0.0,
        8.3,
        0.5,
        0.0,
        8.3,
        7.7,
        0.0,
        0.0,
        7.7,
        0.0,
        camX,
        camY,
        true,
        new Color(18, 25, 38),
        new Color(59, 130, 246, 150));

    g.setStroke(new BasicStroke(1.0f));
    for (int gx = 0; gx <= 8; gx++) {
      g.setColor(gx == 5 ? new Color(59, 130, 246, 130) : new Color(51, 65, 85, 100));
      drawLine3D(g, gx, 0.5, 0.0, gx, 7.7, 0.0, camX, camY, true);
    }
    for (int gy = 1; gy <= 7; gy++) {
      g.setColor(new Color(51, 65, 85, 100));
      drawLine3D(g, 0.0, gy, 0.0, 8.3, gy, 0.0, camX, camY, true);
    }

    // Blue Alliance Wall at X = 0.0
    fillQuad3D(
        g,
        0.0,
        0.5,
        0.0,
        0.0,
        7.7,
        0.0,
        0.0,
        7.7,
        0.65,
        0.0,
        0.5,
        0.65,
        camX,
        camY,
        true,
        new Color(30, 64, 175, 110),
        new Color(96, 165, 250, 180));

    // Blue Depot floor strip (X = 0.55..1.18m, Y = 5.32..6.52m)
    fillQuad3D(
        g,
        0.55,
        5.32,
        0.005,
        1.18,
        5.32,
        0.005,
        1.18,
        6.52,
        0.005,
        0.55,
        6.52,
        0.005,
        camX,
        camY,
        true,
        new Color(16, 185, 129, 55),
        new Color(52, 211, 153, 185));
    double[] depotLbl = project(0.86, 6.62, 0.14, camX, camY, true);
    g.setFont(new Font("SansSerif", Font.BOLD, 10));
    g.setColor(new Color(110, 231, 183));
    g.drawString("BLUE DEPOT", (int) depotLbl[0] - 32, (int) depotLbl[1]);

    for (FloorFuelBall ball : floorBalls) {
      if (f.timeSec() <= ball.pickedUpTimeSec()) {
        double[] bp = project(ball.xMeters(), ball.yMeters(), 0.08, camX, camY, true);
        g.setColor(new Color(0, 0, 0, 120));
        g.fillOval((int) bp[0] - 6, (int) bp[1] - 3, 12, 7);
        g.setColor(new Color(250, 204, 21));
        g.fillOval((int) bp[0] - 6, (int) bp[1] - 6, 12, 12);
        g.setColor(new Color(202, 138, 4));
        g.setStroke(new BasicStroke(1.2f));
        g.drawOval((int) bp[0] - 6, (int) bp[1] - 6, 12, 12);
      }
    }

    g.setColor(new Color(148, 163, 184, 65));
    g.setStroke(
        new BasicStroke(
            1.3f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND, 10.0f, new float[] {4.0f, 5.0f}, 0.0f));
    for (int i = 1; i < frames.size(); i++) {
      AutoFrame a = frames.get(i - 1);
      AutoFrame b = frames.get(i);
      drawLine3D(
          g, a.xMeters(), a.yMeters(), 0.01, b.xMeters(), b.yMeters(), 0.01, camX, camY, true);
    }

    g.setStroke(new BasicStroke(3.2f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
    for (int i = 1; i <= idx; i++) {
      AutoFrame a = frames.get(i - 1);
      AutoFrame b = frames.get(i);
      g.setColor(stateColor(b.stateLabel(), 215));
      drawLine3D(
          g, a.xMeters(), a.yMeters(), 0.02, b.xMeters(), b.yMeters(), 0.02, camX, camY, true);
    }

    if ("PREPPING_SHOT".equals(f.stateLabel()) || "SHOOTING".equals(f.stateLabel())) {
      g.setColor(new Color(56, 189, 248, 130));
      g.setStroke(
          new BasicStroke(
              1.6f,
              BasicStroke.CAP_ROUND,
              BasicStroke.JOIN_ROUND,
              10.0f,
              new float[] {5.0f, 5.0f},
              0.0f));
      drawLine3D(
          g, f.xMeters(), f.yMeters(), 0.45, HUB_BLUE_X, HUB_BLUE_Y, HUB_TOP_Z, camX, camY, true);
    }

    draw2026FuelRobot3D(g, f, camX, camY);

    draw2026HexHub3D(g, camX, camY);

    for (ShotArc shot : shots) {
      if (f.timeSec() < shot.launchTimeSec()) {
        continue;
      }
      double elapsed = f.timeSec() - shot.launchTimeSec();
      double progress = Math.min(1.0, elapsed / Math.max(0.05, shot.flightTimeSec()));

      int segments = 24;
      int maxSeg = Math.max(1, (int) Math.round(progress * segments));
      g.setStroke(new BasicStroke(progress < 1.0 ? 2.8f : 1.8f));
      g.setColor(
          progress < 1.0
              ? new Color(250, 204, 21, 235)
              : (shot.scored() ? new Color(52, 211, 153, 150) : new Color(248, 113, 113, 150)));
      double[] prev = shot.positionAt(0.0);
      for (int s = 1; s <= maxSeg; s++) {
        double u = Math.min(progress, (double) s / segments);
        double[] cur = shot.positionAt(u);
        drawLine3D(g, prev[0], prev[1], prev[2], cur[0], cur[1], cur[2], camX, camY, true);
        prev = cur;
      }

      if (progress < 1.0) {
        double[] ballPos = shot.positionAt(progress);
        double[] shadowPt = project(ballPos[0], ballPos[1], 0.01, camX, camY, true);
        g.setColor(new Color(0, 0, 0, 110));
        g.fillOval((int) shadowPt[0] - 5, (int) shadowPt[1] - 3, 10, 6);

        double[] ballPt = project(ballPos[0], ballPos[1], ballPos[2], camX, camY, true);
        g.setColor(new Color(250, 204, 21));
        g.fillOval((int) ballPt[0] - 7, (int) ballPt[1] - 7, 14, 14);
        g.setColor(new Color(254, 240, 138));
        g.fillOval((int) ballPt[0] - 4, (int) ballPt[1] - 5, 5, 5);
      } else if (elapsed < shot.flightTimeSec() + 0.65 && shot.scored()) {
        double[] hubTop = project(HUB_BLUE_X, HUB_BLUE_Y, HUB_TOP_Z + 0.08, camX, camY, true);
        g.setColor(new Color(52, 211, 153, 210));
        g.setStroke(new BasicStroke(2.5f));
        g.drawOval((int) hubTop[0] - 18, (int) hubTop[1] - 10, 36, 20);
      }
    }
  }

  private static void draw2026HexHub3D(Graphics2D g, double camX, double camY) {
    double rBase = 0.78;
    double rTop = HUB_RADIUS_M;
    double zMid = 1.15;
    double zTop = HUB_TOP_Z;

    double[][] baseHex = new double[6][2];
    double[][] midHex = new double[6][2];
    double[][] topHex = new double[6][2];
    for (int i = 0; i < 6; i++) {
      double a = Math.toRadians(60.0 * i + 30.0);
      baseHex[i][0] = HUB_BLUE_X + rBase * Math.cos(a);
      baseHex[i][1] = HUB_BLUE_Y + rBase * Math.sin(a);
      midHex[i][0] = HUB_BLUE_X + (rBase * 0.72) * Math.cos(a);
      midHex[i][1] = HUB_BLUE_Y + (rBase * 0.72) * Math.sin(a);
      topHex[i][0] = HUB_BLUE_X + rTop * Math.cos(a);
      topHex[i][1] = HUB_BLUE_Y + rTop * Math.sin(a);
    }

    g.setStroke(new BasicStroke(1.3f));
    for (int i = 0; i < 6; i++) {
      int n = (i + 1) % 6;
      fillQuad3D(
          g,
          baseHex[i][0],
          baseHex[i][1],
          0.0,
          baseHex[n][0],
          baseHex[n][1],
          0.0,
          midHex[n][0],
          midHex[n][1],
          zMid,
          midHex[i][0],
          midHex[i][1],
          zMid,
          camX,
          camY,
          true,
          new Color(30, 58, 138, 125),
          new Color(59, 130, 246, 170));
      fillQuad3D(
          g,
          midHex[i][0],
          midHex[i][1],
          zMid,
          midHex[n][0],
          midHex[n][1],
          zMid,
          topHex[n][0],
          topHex[n][1],
          zTop,
          topHex[i][0],
          topHex[i][1],
          zTop,
          camX,
          camY,
          true,
          new Color(37, 99, 235, 115),
          new Color(96, 165, 250, 210));
    }

    g.setColor(new Color(250, 204, 21, 230));
    g.setStroke(new BasicStroke(2.6f));
    for (int i = 0; i < 6; i++) {
      int n = (i + 1) % 6;
      drawLine3D(
          g, topHex[i][0], topHex[i][1], zTop, topHex[n][0], topHex[n][1], zTop, camX, camY, true);
    }
    double[] lbl = project(HUB_BLUE_X, HUB_BLUE_Y, zTop + 0.28, camX, camY, true);
    g.setFont(new Font("SansSerif", Font.BOLD, 11));
    g.setColor(new Color(147, 197, 253));
    g.drawString("2026 BLUE HUB (Z=1.83m)", (int) lbl[0] - 64, (int) lbl[1]);
  }

  private static void draw2026FuelRobot3D(Graphics2D g, AutoFrame f, double camX, double camY) {
    double rx = f.xMeters();
    double ry = f.yMeters();
    double h = f.headingRad();
    double cos = Math.cos(h);
    double sin = Math.sin(h);
    double half = 0.42;
    double zBottom = 0.05;
    double zTop = 0.20;

    double[][] localCorners = {{-half, -half}, {half, -half}, {half, half}, {-half, half}};
    double[][] worldCorners = new double[4][2];
    for (int i = 0; i < 4; i++) {
      worldCorners[i][0] = rx + localCorners[i][0] * cos - localCorners[i][1] * sin;
      worldCorners[i][1] = ry + localCorners[i][0] * sin + localCorners[i][1] * cos;
    }

    Color bumperFill = new Color(37, 99, 235, 225);
    Color bumperStroke = new Color(147, 197, 253);
    for (int i = 0; i < 4; i++) {
      int next = (i + 1) % 4;
      fillQuad3D(
          g,
          worldCorners[i][0],
          worldCorners[i][1],
          zBottom,
          worldCorners[next][0],
          worldCorners[next][1],
          zBottom,
          worldCorners[next][0],
          worldCorners[next][1],
          zTop,
          worldCorners[i][0],
          worldCorners[i][1],
          zTop,
          camX,
          camY,
          true,
          bumperFill,
          bumperStroke);
    }
    fillQuad3D(
        g,
        worldCorners[0][0],
        worldCorners[0][1],
        zTop,
        worldCorners[1][0],
        worldCorners[1][1],
        zTop,
        worldCorners[2][0],
        worldCorners[2][1],
        zTop,
        worldCorners[3][0],
        worldCorners[3][1],
        zTop,
        camX,
        camY,
        true,
        new Color(30, 41, 59, 235),
        bumperStroke);

    boolean intaking = Math.abs(f.intakeRollerRps()) > 5.0 || "INTAKING".equals(f.stateLabel());
    double intakeExt = intaking ? 0.62 : 0.44;
    double ixL = rx + intakeExt * cos - (-0.28) * sin;
    double iyL = ry + intakeExt * sin + (-0.28) * cos;
    double ixR = rx + intakeExt * cos - (0.28) * sin;
    double iyR = ry + intakeExt * sin + (0.28) * cos;
    g.setColor(intaking ? new Color(52, 211, 153) : new Color(148, 163, 184));
    g.setStroke(new BasicStroke(3.2f));
    drawLine3D(g, ixL, iyL, 0.12, ixR, iyR, 0.12, camX, camY, true);

    double fx0 = rx + 0.08 * cos;
    double fy0 = ry + 0.08 * sin;
    double fx1 = rx + 0.48 * cos;
    double fy1 = ry + 0.48 * sin;
    g.setColor(new Color(56, 189, 248));
    g.setStroke(new BasicStroke(2.6f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
    drawLine3D(g, fx0, fy0, zTop + 0.02, fx1, fy1, zTop + 0.02, camX, camY, true);

    double sx0 = rx - 0.10 * cos;
    double sy0 = ry - 0.10 * sin;
    double hoodPitchRad = Math.toRadians(52.0) - (f.elevatorHeightM() / 10.0) * Math.toRadians(24.0);
    double barrelLen = 0.36;
    double sx1 = sx0 + barrelLen * Math.cos(hoodPitchRad) * cos;
    double sy1 = sy0 + barrelLen * Math.cos(hoodPitchRad) * sin;
    double sz1 = 0.25 + barrelLen * Math.sin(hoodPitchRad);

    g.setColor(new Color(250, 204, 21));
    g.setStroke(new BasicStroke(4.0f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
    drawLine3D(g, sx0, sy0, 0.22, sx1, sy1, sz1, camX, camY, true);

    double[] muzzlePt = project(sx1, sy1, sz1, camX, camY, true);
    g.setColor(f.accelScale() > 25.0 ? new Color(250, 204, 21) : new Color(148, 163, 184));
    g.fillOval((int) muzzlePt[0] - 5, (int) muzzlePt[1] - 5, 10, 10);
  }

  private static void drawSystemCoreRobot3D(
      Graphics2D g, AutoFrame f, double camX, double camY) {
    double rx = f.xMeters();
    double ry = f.yMeters();
    double h = f.headingRad();
    double cos = Math.cos(h);
    double sin = Math.sin(h);
    double half = 0.42;
    double zBottom = 0.05;
    double zTop = 0.20;

    double[][] localCorners = {{-half, -half}, {half, -half}, {half, half}, {-half, half}};
    double[][] worldCorners = new double[4][2];
    for (int i = 0; i < 4; i++) {
      worldCorners[i][0] = rx + localCorners[i][0] * cos - localCorners[i][1] * sin;
      worldCorners[i][1] = ry + localCorners[i][0] * sin + localCorners[i][1] * cos;
    }

    fillQuad3D(
        g,
        worldCorners[0][0],
        worldCorners[0][1],
        0.005,
        worldCorners[1][0],
        worldCorners[1][1],
        0.005,
        worldCorners[2][0],
        worldCorners[2][1],
        0.005,
        worldCorners[3][0],
        worldCorners[3][1],
        0.005,
        camX,
        camY,
        false,
        new Color(0, 0, 0, 110),
        null);

    Color bumperFill = new Color(37, 99, 235, 225);
    Color bumperStroke = new Color(147, 197, 253);
    g.setStroke(new BasicStroke(1.6f));
    for (int i = 0; i < 4; i++) {
      int next = (i + 1) % 4;
      fillQuad3D(
          g,
          worldCorners[i][0],
          worldCorners[i][1],
          zBottom,
          worldCorners[next][0],
          worldCorners[next][1],
          zBottom,
          worldCorners[next][0],
          worldCorners[next][1],
          zTop,
          worldCorners[i][0],
          worldCorners[i][1],
          zTop,
          camX,
          camY,
          false,
          bumperFill,
          bumperStroke);
    }

    fillQuad3D(
        g,
        worldCorners[0][0],
        worldCorners[0][1],
        zTop,
        worldCorners[1][0],
        worldCorners[1][1],
        zTop,
        worldCorners[2][0],
        worldCorners[2][1],
        zTop,
        worldCorners[3][0],
        worldCorners[3][1],
        zTop,
        camX,
        camY,
        false,
        new Color(30, 41, 59, 235),
        bumperStroke);

    double modOff = 0.29;
    double[][] modLocals = {
      {modOff, modOff}, {modOff, -modOff}, {-modOff, modOff}, {-modOff, -modOff}
    };
    g.setStroke(new BasicStroke(2.0f));
    for (double[] ml : modLocals) {
      double mx = rx + ml[0] * cos - ml[1] * sin;
      double my = ry + ml[0] * sin + ml[1] * cos;
      double[] mp = project(mx, my, zTop + 0.01, camX, camY, false);
      g.setColor(new Color(148, 163, 184));
      g.fillOval((int) mp[0] - 4, (int) mp[1] - 4, 8, 8);

      double wvx = f.vxMps() - f.omegaRadPerSec() * (ml[0] * sin + ml[1] * cos);
      double wvy = f.vyMps() + f.omegaRadPerSec() * (ml[0] * cos - ml[1] * sin);
      g.setColor(new Color(56, 189, 248));
      drawLine3D(
          g,
          mx,
          my,
          zTop + 0.02,
          mx + wvx * 0.12,
          my + wvy * 0.12,
          zTop + 0.02,
          camX,
          camY,
          false);
    }

    double fx0 = rx + 0.10 * cos;
    double fy0 = ry + 0.10 * sin;
    double fx1 = rx + 0.52 * cos;
    double fy1 = ry + 0.52 * sin;
    g.setColor(new Color(250, 204, 21));
    g.setStroke(new BasicStroke(3.0f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
    drawLine3D(g, fx0, fy0, zTop + 0.02, fx1, fy1, zTop + 0.02, camX, camY, false);

    double mastLeftX = rx - 0.12 * sin;
    double mastLeftY = ry + 0.12 * cos;
    double mastRightX = rx + 0.12 * sin;
    double mastRightY = ry - 0.12 * cos;
    double fixedMastTopZ = zTop + 0.65;
    g.setColor(new Color(148, 163, 184));
    g.setStroke(new BasicStroke(2.6f));
    drawLine3D(
        g, mastLeftX, mastLeftY, zTop, mastLeftX, mastLeftY, fixedMastTopZ, camX, camY, false);
    drawLine3D(
        g,
        mastRightX,
        mastRightY,
        zTop,
        mastRightX,
        mastRightY,
        fixedMastTopZ,
        camX,
        camY,
        false);
    drawLine3D(
        g,
        mastLeftX,
        mastLeftY,
        fixedMastTopZ,
        mastRightX,
        mastRightY,
        fixedMastTopZ,
        camX,
        camY,
        false);

    double carriageZ = zTop + 0.18 + Math.max(0.0, f.elevatorHeightM());
    g.setColor(new Color(56, 189, 248));
    g.setStroke(new BasicStroke(3.4f));
    drawLine3D(g, rx, ry, zTop, rx, ry, carriageZ, camX, camY, false);
    drawLine3D(
        g, mastLeftX, mastLeftY, carriageZ, mastRightX, mastRightY, carriageZ, camX, camY, false);

    double armLen = 0.42;
    double armPitch = f.armAngleRad();
    double armEndX = rx + (armLen * Math.cos(armPitch)) * cos;
    double armEndY = ry + (armLen * Math.cos(armPitch)) * sin;
    double armEndZ = carriageZ + armLen * Math.sin(armPitch);
    g.setColor(new Color(250, 204, 21));
    g.setStroke(new BasicStroke(3.8f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
    drawLine3D(g, rx, ry, carriageZ, armEndX, armEndY, armEndZ, camX, camY, false);

    double[] rollerPt = project(armEndX, armEndY, armEndZ, camX, camY, false);
    Color rollerCol =
        Math.abs(f.intakeRollerRps()) > 1.0
            ? (f.intakeRollerRps() > 0 ? new Color(52, 211, 153) : new Color(248, 113, 113))
            : new Color(226, 232, 240);
    g.setColor(rollerCol);
    g.fillOval((int) rollerPt[0] - 7, (int) rollerPt[1] - 7, 14, 14);
    g.setColor(new Color(15, 23, 42));
    g.setStroke(new BasicStroke(1.5f));
    g.drawOval((int) rollerPt[0] - 7, (int) rollerPt[1] - 7, 14, 14);
  }

  private static void drawTopDownMiniMap(
      Graphics2D g,
      List<AutoFrame> frames,
      List<ShotArc> shots,
      List<FloorFuelBall> floorBalls,
      boolean field2026,
      int idx) {
    int mapW = 210;
    int mapH = 140;
    int mapX = WIDTH - mapW - 14;
    int mapY = 58;

    g.setColor(new Color(15, 23, 42, 225));
    g.fillRoundRect(mapX, mapY, mapW, mapH, 10, 10);
    g.setColor(new Color(71, 85, 105));
    g.setStroke(new BasicStroke(1.4f));
    g.drawRoundRect(mapX, mapY, mapW, mapH, 10, 10);

    g.setFont(new Font("SansSerif", Font.BOLD, 10));
    g.setColor(new Color(148, 163, 184));
    g.drawString(
        field2026 ? "2D TOP-DOWN (2026 Blue Wing)" : "2D TOP-DOWN MAP (5m × 3m)",
        mapX + 12,
        mapY + 15);

    AutoFrame cur = frames.get(idx);
    if (field2026) {
      double scale = 19.0;
      double ox = mapX + 18.0;
      double oy = mapY + mapH - 12.0;

      g.setColor(new Color(59, 130, 246, 90));
      g.drawRect((int) ox, (int) (oy - 6.0 * scale), (int) (5.5 * scale), (int) (5.5 * scale));

      int hx = (int) Math.round(ox + HUB_BLUE_X * scale);
      int hy = (int) Math.round(oy - (HUB_BLUE_Y - 1.2) * scale);
      g.setColor(new Color(250, 204, 21));
      g.fillOval(hx - 5, hy - 5, 10, 10);

      for (FloorFuelBall b : floorBalls) {
        if (cur.timeSec() <= b.pickedUpTimeSec()) {
          int bx = (int) Math.round(ox + b.xMeters() * scale);
          int by = (int) Math.round(oy - (b.yMeters() - 1.2) * scale);
          g.setColor(new Color(250, 204, 21, 180));
          g.fillOval(bx - 2, by - 2, 5, 5);
        }
      }

      g.setStroke(new BasicStroke(1.8f));
      for (int i = 1; i <= idx; i++) {
        AutoFrame a = frames.get(i - 1);
        AutoFrame b = frames.get(i);
        g.setColor(stateColor(b.stateLabel(), 220));
        int x0 = (int) Math.round(ox + a.xMeters() * scale);
        int y0 = (int) Math.round(oy - (a.yMeters() - 1.2) * scale);
        int x1 = (int) Math.round(ox + b.xMeters() * scale);
        int y1 = (int) Math.round(oy - (b.yMeters() - 1.2) * scale);
        g.drawLine(x0, y0, x1, y1);
      }

      g.setColor(new Color(250, 204, 21, 130));
      g.setStroke(new BasicStroke(1.1f));
      for (ShotArc s : shots) {
        if (cur.timeSec() >= s.launchTimeSec()) {
          int sx = (int) Math.round(ox + s.startX() * scale);
          int sy = (int) Math.round(oy - (s.startY() - 1.2) * scale);
          g.drawLine(sx, sy, hx, hy);
        }
      }

      int rx = (int) Math.round(ox + cur.xMeters() * scale);
      int ry = (int) Math.round(oy - (cur.yMeters() - 1.2) * scale);
      g.setColor(new Color(56, 189, 248));
      g.fillOval(rx - 4, ry - 4, 8, 8);
      return;
    }

    double cx = mapX + mapW * 0.46;
    double cy = mapY + mapH * 0.56;
    double scale = 28.0;

    int boxW = (int) Math.round(5.0 * scale);
    int boxH = (int) Math.round(3.0 * scale);
    g.setColor(new Color(56, 189, 248, 70));
    g.drawRect((int) Math.round(cx - boxW / 2.0), (int) Math.round(cy - boxH / 2.0), boxW, boxH);

    g.setColor(new Color(148, 163, 184, 90));
    g.drawLine((int) cx - 60, (int) cy, (int) cx + 80, (int) cy);
    g.drawLine((int) cx, (int) cy - 45, (int) cx, (int) cy + 45);

    g.setStroke(new BasicStroke(2.0f));
    for (int i = 1; i <= idx; i++) {
      AutoFrame a = frames.get(i - 1);
      AutoFrame b = frames.get(i);
      g.setColor(stateColor(b.stateLabel(), 220));
      int x0 = (int) Math.round(cx + a.xMeters() * scale);
      int y0 = (int) Math.round(cy - a.yMeters() * scale);
      int x1 = (int) Math.round(cx + b.xMeters() * scale);
      int y1 = (int) Math.round(cy - b.yMeters() * scale);
      g.drawLine(x0, y0, x1, y1);
    }

    int rx = (int) Math.round(cx + cur.xMeters() * scale);
    int ry = (int) Math.round(cy - cur.yMeters() * scale);
    g.setColor(new Color(250, 204, 21));
    g.fillOval(rx - 4, ry - 4, 8, 8);
    int hx = (int) Math.round(rx + Math.cos(cur.headingRad()) * 10.0);
    int hy = (int) Math.round(ry - Math.sin(cur.headingRad()) * 10.0);
    g.drawLine(rx, ry, hx, hy);
  }

  private static Color stateColor(String stateLabel, int alpha) {
    if (stateLabel == null) {
      return new Color(56, 189, 248, alpha);
    }
    return switch (stateLabel) {
      case "INTAKING" -> new Color(52, 211, 153, alpha);
      case "SCORING_LOW", "PREPPING_SHOT" -> new Color(56, 189, 248, alpha);
      case "SCORING_HIGH", "SHOOTING" -> new Color(250, 204, 21, alpha);
      case "EJECTING", "OUTTAKING" -> new Color(248, 113, 113, alpha);
      default -> new Color(129, 140, 248, alpha);
    };
  }

  public static void writeFramesJson(
      Path jsonFile,
      String routineName,
      String robotName,
      List<AutoFrame> frames,
      List<ShotArc> shots,
      List<FloorFuelBall> floorBalls,
      boolean field2026)
      throws IOException {
    writeFramesJson(
        jsonFile, routineName, robotName, frames, shots, floorBalls, field2026, "");
  }

  public static void writeFramesJson(
      Path jsonFile,
      String routineName,
      String robotName,
      List<AutoFrame> frames,
      List<ShotArc> shots,
      List<FloorFuelBall> floorBalls,
      boolean field2026,
      String autoWarning)
      throws IOException {
    StringBuilder sb = new StringBuilder(32768);
    sb.append("{\n");
    sb.append("  \"routineName\": \"").append(escapeJson(routineName)).append("\",\n");
    sb.append("  \"robotName\": \"").append(escapeJson(robotName)).append("\",\n");
    sb.append("  \"fieldMode\": \"")
        .append(field2026 ? "REBUILT_2026_SHOOTING" : "EMPTY_FIELD_2027")
        .append("\",\n");
    if (autoWarning != null && !autoWarning.isBlank()) {
      sb.append("  \"autoWarning\": \"").append(escapeJson(autoWarning)).append("\",\n");
    }
    sb.append("  \"fps\": ").append(FPS).append(",\n");
    sb.append("  \"floorBalls\": [\n");
    for (int i = 0; i < floorBalls.size(); i++) {
      FloorFuelBall b = floorBalls.get(i);
      sb.append(
          String.format(
              Locale.US,
              "    {\"x\":%.3f,\"y\":%.3f,\"tPick\":%.3f}",
              b.xMeters(),
              b.yMeters(),
              b.pickedUpTimeSec()));
      if (i + 1 < floorBalls.size()) {
        sb.append(",");
      }
      sb.append("\n");
    }
    sb.append("  ],\n");
    sb.append("  \"shots\": [\n");
    for (int i = 0; i < shots.size(); i++) {
      ShotArc s = shots.get(i);
      sb.append(
          String.format(
              Locale.US,
              "    {\"id\":%d,\"t0\":%.3f,\"tof\":%.3f,\"x0\":%.3f,\"y0\":%.3f,\"z0\":%.3f,\"x1\":%.3f,\"y1\":%.3f,\"z1\":%.3f,\"apexZ\":%.3f,\"scored\":%s}",
              s.shotId(),
              s.launchTimeSec(),
              s.flightTimeSec(),
              s.startX(),
              s.startY(),
              s.startZ(),
              s.targetX(),
              s.targetY(),
              s.targetZ(),
              s.apexZ(),
              s.scored() ? "true" : "false"));
      if (i + 1 < shots.size()) {
        sb.append(",");
      }
      sb.append("\n");
    }
    sb.append("  ],\n");
    sb.append("  \"frames\": [\n");
    for (int i = 0; i < frames.size(); i++) {
      AutoFrame f = frames.get(i);
      sb.append(
          String.format(
              Locale.US,
              "    {\"t\":%.3f,\"x\":%.4f,\"y\":%.4f,\"h\":%.4f,\"vx\":%.3f,\"vy\":%.3f,\"om\":%.3f,\"elev\":%.3f,\"arm\":%.3f,\"rol\":%.2f,\"vBat\":%.2f,\"iDrv\":%.1f,\"scale\":%.2f,\"st\":\"%s\"}",
              f.timeSec(),
              f.xMeters(),
              f.yMeters(),
              f.headingRad(),
              f.vxMps(),
              f.vyMps(),
              f.omegaRadPerSec(),
              f.elevatorHeightM(),
              f.armAngleRad(),
              f.intakeRollerRps(),
              f.batteryVoltage(),
              f.driveStatorAmps(),
              f.accelScale(),
              escapeJson(f.stateLabel())));
      if (i + 1 < frames.size()) {
        sb.append(",");
      }
      sb.append("\n");
    }
    sb.append("  ]\n}\n");
    Files.writeString(jsonFile, sb.toString(), StandardCharsets.UTF_8);
  }

  private static void writeInteractiveHtml(
      Path htmlFile,
      String routineName,
      String robotName,
      List<AutoFrame> frames,
      List<ShotArc> shots,
      List<FloorFuelBall> floorBalls,
      boolean field2026,
      String autoWarning)
      throws IOException {
    StringBuilder framesJs = new StringBuilder(24576);
    framesJs.append("[");
    for (int i = 0; i < frames.size(); i++) {
      AutoFrame f = frames.get(i);
      framesJs.append(
          String.format(
              Locale.US,
              "{t:%.3f,x:%.4f,y:%.4f,h:%.4f,vx:%.3f,vy:%.3f,om:%.3f,elev:%.3f,arm:%.3f,rol:%.2f,vBat:%.2f,iDrv:%.1f,scale:%.2f,st:\"%s\"}",
              f.timeSec(),
              f.xMeters(),
              f.yMeters(),
              f.headingRad(),
              f.vxMps(),
              f.vyMps(),
              f.omegaRadPerSec(),
              f.elevatorHeightM(),
              f.armAngleRad(),
              f.intakeRollerRps(),
              f.batteryVoltage(),
              f.driveStatorAmps(),
              f.accelScale(),
              escapeJson(f.stateLabel())));
      if (i + 1 < frames.size()) {
        framesJs.append(",");
      }
    }
    framesJs.append("]");

    StringBuilder shotsJs = new StringBuilder(4096);
    shotsJs.append("[");
    for (int i = 0; i < shots.size(); i++) {
      ShotArc s = shots.get(i);
      shotsJs.append(
          String.format(
              Locale.US,
              "{id:%d,t0:%.3f,tof:%.3f,x0:%.3f,y0:%.3f,z0:%.3f,x1:%.3f,y1:%.3f,z1:%.3f,apex:%.3f,scored:%s}",
              s.shotId(),
              s.launchTimeSec(),
              s.flightTimeSec(),
              s.startX(),
              s.startY(),
              s.startZ(),
              s.targetX(),
              s.targetY(),
              s.targetZ(),
              s.apexZ(),
              s.scored() ? "true" : "false"));
      if (i + 1 < shots.size()) {
        shotsJs.append(",");
      }
    }
    shotsJs.append("]");

    StringBuilder ballsJs = new StringBuilder(2048);
    ballsJs.append("[");
    for (int i = 0; i < floorBalls.size(); i++) {
      FloorFuelBall b = floorBalls.get(i);
      ballsJs.append(
          String.format(
              Locale.US, "{x:%.3f,y:%.3f,tPick:%.3f}", b.xMeters(), b.yMeters(), b.pickedUpTimeSec()));
      if (i + 1 < floorBalls.size()) {
        ballsJs.append(",");
      }
    }
    ballsJs.append("]");

    String faviconDataUri =
        "data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 64 64'%3E"
            + "%3Crect width='64' height='64' rx='14' fill='%230f172a' stroke='%2338bdf8' stroke-width='4'/%3E"
            + "%3Cpolygon points='36,8 16,35 30,35 26,56 48,28 34,28' fill='%23facc15'/%3E"
            + "%3C/svg%3E";

    boolean emptyStudentStarter = isEmptyStudentStarter(routineName, frames, shots);
    String warningBannerHtml =
        (autoWarning != null && !autoWarning.isBlank())
            ? "<div class=\"warn-banner\">SafeAuto Warning: " + escapeHtml(autoWarning) + "</div>\n"
            : (emptyStudentStarter
                ? "<div class=\"student-banner\">STUDENT NOTE: Empty starter auto — open <code>src/main/java/frc/robot/autos/StudentAutoRoutines.java</code> (<code>createStudentDefaultAuto</code>), add your commands, and re-run <code>./gradlew renderAuto</code>!</div>\n"
                : "");

    String html =
        "<!DOCTYPE html>\n"
            + "<html lang=\"en\">\n"
            + "<head>\n"
            + "<meta charset=\"UTF-8\">\n"
            + "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\n"
            + "<link rel=\"icon\" type=\"image/svg+xml\" href=\""
            + faviconDataUri
            + "\">\n"
            + "<title>FLUX 10413 — "
            + escapeHtml(routineName)
            + "</title>\n"
            + "<style>\n"
            + "  body{margin:0;background:#0a0e17;color:#e2e8f0;font-family:system-ui,-apple-system,sans-serif;display:flex;flex-direction:column;height:100vh;overflow:hidden;}\n"
            + "  header{padding:10px 18px;background:#0f172a;border-bottom:2px solid #38bdf8;display:flex;justify-content:space-between;align-items:center;gap:12px;}\n"
            + "  .warn-banner{background:#991b1b;color:#fff;padding:7px 18px;font-size:12.5px;font-weight:700;border-bottom:1px solid #f87171;}\n"
            + "  .student-banner{background:#1e3a8a;color:#e0f2fe;padding:7px 18px;font-size:12.5px;font-weight:700;border-bottom:1px solid #38bdf8;}\n"
            + "  .student-banner code{background:rgba(15,23,42,0.55);padding:1px 5px;border-radius:4px;color:#facc15;}\n"
            + "  .badge{padding:4px 12px;border-radius:6px;background:#38bdf8;color:#0f172a;font-weight:800;font-size:12.5px;letter-spacing:0.02em;}\n"
            + "  #viewport{flex:1;position:relative;display:flex;justify-content:center;align-items:center;min-height:0;}\n"
            + "  canvas{width:100%;height:100%;display:block;cursor:grab;}\n"
            + "  canvas:active{cursor:grabbing;}\n"
            + "  .hint{position:absolute;left:16px;top:12px;background:rgba(15,23,42,0.82);border:1px solid #334155;border-radius:7px;padding:5px 10px;font-size:11.5px;color:#94a3b8;pointer-events:none;}\n"
            + "  footer{padding:9px 18px 11px 18px;background:#0f172a;border-top:1px solid #334155;display:flex;flex-direction:column;gap:7px;font-family:ui-monospace,SFMono-Regular,monospace;font-size:12.5px;}\n"
            + "  .controls-row{display:flex;gap:10px;align-items:center;}\n"
            + "  .telem-row{display:flex;justify-content:space-between;flex-wrap:wrap;gap:10px;color:#cbd5e1;}\n"
            + "  button{background:#2563eb;color:#fff;border:1px solid #3b82f6;padding:5px 12px;border-radius:6px;font-weight:700;font-size:12px;cursor:pointer;}\n"
            + "  button.secondary{background:#1e293b;border-color:#475569;color:#e2e8f0;}\n"
            + "  button:hover{filter:brightness(1.1);}\n"
            + "  input[type=range]{flex:1;accent-color:#38bdf8;cursor:pointer;}\n"
            + "</style>\n"
            + "</head>\n"
            + "<body>\n"
            + "<header>\n"
            + "  <div><strong>FLUX 10413 • "
            + escapeHtml(routineName)
            + "</strong> <span style=\"color:#94a3b8;margin-left:8px;\">("
            + escapeHtml(robotName)
            + ")</span></div>\n"
            + "  <div id=\"stateBadge\" class=\"badge\">STATE: IDLE</div>\n"
            + "</header>\n"
            + warningBannerHtml
            + "<div id=\"viewport\">\n"
            + "  <canvas id=\"simCanvas\"></canvas>\n"
            + "  <div class=\"hint\">Drag to orbit 3D • Scroll to zoom • Space: Play/Pause • ←/→: Step frame</div>\n"
            + "</div>\n"
            + "<footer>\n"
            + "  <div class=\"controls-row\">\n"
            + "    <button id=\"playBtn\">Pause</button>\n"
            + "    <button id=\"speedBtn\" class=\"secondary\">1.0x</button>\n"
            + "    <button id=\"resetCamBtn\" class=\"secondary\">Reset View</button>\n"
            + "    <span id=\"timeLbl\" style=\"color:#38bdf8;font-weight:700;min-width:128px;\">T = 0.00s / 15.00s</span>\n"
            + "    <input id=\"scrubber\" type=\"range\" min=\"0\" max=\""
            + Math.max(0, frames.size() - 1)
            + "\" value=\"0\">\n"
            + "  </div>\n"
            + "  <div class=\"telem-row\">\n"
            + "    <span id=\"poseLbl\">POSE: X=+0.00m Y=+0.00m Yaw=+0.0°</span>\n"
            + "    <span id=\"mechLbl\" style=\"color:#94a3b8;\">HUB SCORES: 0 / 0</span>\n"
            + "  </div>\n"
            + "</footer>\n"
            + "<script>\n"
            + "const FRAMES = "
            + framesJs
            + ";\n"
            + "const SHOTS = "
            + shotsJs
            + ";\n"
            + "const FLOOR_BALLS = "
            + ballsJs
            + ";\n"
            + "const FIELD_2026 = "
            + (field2026 ? "true" : "false")
            + ";\n"
            + "const HUB_X = 4.63, HUB_Y = 4.21, HUB_Z = 1.83, HUB_R = 0.62;\n"
            + "const canvas = document.getElementById('simCanvas');\n"
            + "const ctx = canvas.getContext('2d');\n"
            + "const SPEEDS = [1.0, 0.5, 2.0];\n"
            + "let idx = 0, playing = true, speedIdx = 0, tickAcc = 0;\n"
            + "let az = -0.663, el = 0.593, zoom = FIELD_2026 ? 98 : 130, panX = 0, panY = 0;\n"
            + "let dragging = false, lx = 0, ly = 0;\n"
            + "canvas.addEventListener('mousedown', e => { dragging = true; lx = e.clientX; ly = e.clientY; });\n"
            + "window.addEventListener('mouseup', () => dragging = false);\n"
            + "window.addEventListener('mousemove', e => {\n"
            + "  if(!dragging) return;\n"
            + "  az += (e.clientX - lx) * 0.007;\n"
            + "  el = Math.max(0.18, Math.min(1.42, el + (e.clientY - ly) * 0.007));\n"
            + "  lx = e.clientX; ly = e.clientY; draw();\n"
            + "});\n"
            + "canvas.addEventListener('wheel', e => { e.preventDefault(); zoom = Math.max(48, Math.min(280, zoom - e.deltaY * 0.11)); draw(); }, {passive:false});\n"
            + "const scrubber = document.getElementById('scrubber');\n"
            + "const playBtn = document.getElementById('playBtn');\n"
            + "const speedBtn = document.getElementById('speedBtn');\n"
            + "const resetCamBtn = document.getElementById('resetCamBtn');\n"
            + "playBtn.onclick = () => { playing = !playing; playBtn.textContent = playing ? 'Pause' : 'Play'; };\n"
            + "speedBtn.onclick = () => { speedIdx = (speedIdx + 1) % SPEEDS.length; speedBtn.textContent = SPEEDS[speedIdx].toFixed(1) + 'x'; };\n"
            + "resetCamBtn.onclick = () => { az = -0.663; el = 0.593; zoom = FIELD_2026 ? 98 : 130; panX = 0; panY = 0; draw(); };\n"
            + "scrubber.oninput = e => { playing = false; playBtn.textContent = 'Play'; idx = parseInt(e.target.value, 10); draw(); };\n"
            + "window.addEventListener('keydown', e => {\n"
            + "  if(e.code === 'Space') { e.preventDefault(); playBtn.click(); }\n"
            + "  else if(e.code === 'ArrowRight' && FRAMES.length) { playing = false; playBtn.textContent = 'Play'; idx = (idx + 1) % FRAMES.length; draw(); }\n"
            + "  else if(e.code === 'ArrowLeft' && FRAMES.length) { playing = false; playBtn.textContent = 'Play'; idx = (idx - 1 + FRAMES.length) % FRAMES.length; draw(); }\n"
            + "});\n"
            + "function stateColor(st, alpha) {\n"
            + "  const a = alpha === undefined ? 1.0 : alpha;\n"
            + "  if(st === 'INTAKING') return `rgba(52,211,153,${a})`;\n"
            + "  if(st === 'SCORING_LOW' || st === 'PREPPING_SHOT') return `rgba(56,189,248,${a})`;\n"
            + "  if(st === 'SCORING_HIGH' || st === 'SHOOTING') return `rgba(250,204,21,${a})`;\n"
            + "  if(st === 'EJECTING' || st === 'OUTTAKING') return `rgba(248,113,113,${a})`;\n"
            + "  return `rgba(129,140,248,${a})`;\n"
            + "}\n"
            + "function proj(wx, wy, wz, cx, cy) {\n"
            + "  const dx = wx - cx, dy = wy - cy;\n"
            + "  const rx = dx * Math.cos(az) - dy * Math.sin(az);\n"
            + "  const ry = dx * Math.sin(az) + dy * Math.cos(az);\n"
            + "  return [canvas.clientWidth * 0.44 + panX + rx * zoom, canvas.clientHeight * 0.56 + panY - (ry * Math.sin(el) + wz * Math.cos(el)) * zoom];\n"
            + "}\n"
            + "function line3D(x0,y0,z0,x1,y1,z1,cx,cy) {\n"
            + "  const p0 = proj(x0,y0,z0,cx,cy), p1 = proj(x1,y1,z1,cx,cy);\n"
            + "  ctx.beginPath(); ctx.moveTo(p0[0],p0[1]); ctx.lineTo(p1[0],p1[1]); ctx.stroke();\n"
            + "}\n"
            + "function quad3D(pts, cx, cy, fill, stroke, lw) {\n"
            + "  const pr = pts.map(p => proj(p[0],p[1],p[2],cx,cy));\n"
            + "  ctx.beginPath(); ctx.moveTo(pr[0][0],pr[0][1]);\n"
            + "  for(let i=1;i<pr.length;i++) ctx.lineTo(pr[i][0],pr[i][1]);\n"
            + "  ctx.closePath();\n"
            + "  if(fill) { ctx.fillStyle = fill; ctx.fill(); }\n"
            + "  if(stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = lw || 1.3; ctx.stroke(); }\n"
            + "}\n"
            + "function draw2026Field(f, cx, cy) {\n"
            + "  quad3D([[0,0.5,0],[8.3,0.5,0],[8.3,7.7,0],[0,7.7,0]], cx, cy, '#121926', 'rgba(59,130,246,0.6)', 1.5);\n"
            + "  for(let gx=0; gx<=8; gx++) {\n"
            + "    ctx.strokeStyle = gx === 5 ? 'rgba(59,130,246,0.55)' : 'rgba(51,65,85,0.45)'; ctx.lineWidth = 1;\n"
            + "    line3D(gx,0.5,0, gx,7.7,0, cx,cy);\n"
            + "  }\n"
            + "  for(let gy=1; gy<=7; gy++) {\n"
            + "    ctx.strokeStyle = 'rgba(51,65,85,0.45)'; ctx.lineWidth = 1;\n"
            + "    line3D(0,gy,0, 8.3,gy,0, cx,cy);\n"
            + "  }\n"
            + "  quad3D([[0,0.5,0],[0,7.7,0],[0,7.7,0.65],[0,0.5,0.65]], cx, cy, 'rgba(30,64,175,0.42)', 'rgba(96,165,250,0.75)', 1.5);\n"
            + "  quad3D([[0.55,5.32,0.005],[1.18,5.32,0.005],[1.18,6.52,0.005],[0.55,6.52,0.005]], cx, cy, 'rgba(16,185,129,0.22)', 'rgba(52,211,153,0.78)', 1.6);\n"
            + "  const dLbl = proj(0.86, 6.62, 0.14, cx, cy);\n"
            + "  ctx.fillStyle = '#6ee7b7'; ctx.font = 'bold 11px system-ui,sans-serif'; ctx.fillText('BLUE DEPOT', dLbl[0]-34, dLbl[1]);\n"
            + "  for(const b of FLOOR_BALLS) {\n"
            + "    if(f.t <= b.tPick) {\n"
            + "      const bp = proj(b.x, b.y, 0.08, cx, cy);\n"
            + "      ctx.fillStyle = 'rgba(0,0,0,0.45)'; ctx.beginPath(); ctx.ellipse(bp[0], bp[1]+2, 6, 3.5, 0, 0, Math.PI*2); ctx.fill();\n"
            + "      ctx.fillStyle = '#facc15'; ctx.beginPath(); ctx.arc(bp[0], bp[1]-1, 6, 0, Math.PI*2); ctx.fill();\n"
            + "      ctx.strokeStyle = '#ca8a04'; ctx.lineWidth = 1.2; ctx.stroke();\n"
            + "    }\n"
            + "  }\n"
            + "  ctx.setLineDash([4,5]); ctx.strokeStyle = 'rgba(148,163,184,0.3)'; ctx.lineWidth = 1.3;\n"
            + "  ctx.beginPath();\n"
            + "  for(let i=0;i<FRAMES.length;i++) { const p = proj(FRAMES[i].x, FRAMES[i].y, 0.01, cx, cy); if(i===0) ctx.moveTo(p[0],p[1]); else ctx.lineTo(p[0],p[1]); }\n"
            + "  ctx.stroke(); ctx.setLineDash([]);\n"
            + "  ctx.lineWidth = 3.2; ctx.lineCap = 'round';\n"
            + "  for(let i=1;i<=idx;i++) {\n"
            + "    ctx.strokeStyle = stateColor(FRAMES[i].st, 0.88);\n"
            + "    line3D(FRAMES[i-1].x, FRAMES[i-1].y, 0.02, FRAMES[i].x, FRAMES[i].y, 0.02, cx, cy);\n"
            + "  }\n"
            + "  if(f.st === 'PREPPING_SHOT' || f.st === 'SHOOTING') {\n"
            + "    ctx.setLineDash([5,5]); ctx.strokeStyle = 'rgba(56,189,248,0.6)'; ctx.lineWidth = 1.7;\n"
            + "    line3D(f.x, f.y, 0.45, HUB_X, HUB_Y, HUB_Z, cx, cy);\n"
            + "    ctx.setLineDash([]);\n"
            + "  }\n"
            + "  draw2026Robot(f, cx, cy);\n"
            + "  draw2026HexHub(cx, cy);\n"
            + "  for(const s of SHOTS) {\n"
            + "    if(f.t < s.t0) continue;\n"
            + "    const elapsed = f.t - s.t0;\n"
            + "    const prog = Math.min(1.0, elapsed / Math.max(0.05, s.tof));\n"
            + "    ctx.strokeStyle = prog < 1.0 ? 'rgba(250,204,21,0.92)' : (s.scored ? 'rgba(52,211,153,0.6)' : 'rgba(248,113,113,0.6)');\n"
            + "    ctx.lineWidth = prog < 1.0 ? 2.8 : 1.8; ctx.beginPath();\n"
            + "    const maxSeg = Math.max(1, Math.round(prog * 24));\n"
            + "    for(let k=0; k<=maxSeg; k++) {\n"
            + "      const u = Math.min(prog, k / 24.0);\n"
            + "      const bx = s.x0 + u*(s.x1-s.x0), by = s.y0 + u*(s.y1-s.y0);\n"
            + "      const bz = (1-u)*s.z0 + u*s.z1 + 4*u*(1-u)*(s.apex - 0.5*(s.z0+s.z1));\n"
            + "      const sp = proj(bx, by, bz, cx, cy);\n"
            + "      if(k===0) ctx.moveTo(sp[0],sp[1]); else ctx.lineTo(sp[0],sp[1]);\n"
            + "    }\n"
            + "    ctx.stroke();\n"
            + "    if(prog < 1.0) {\n"
            + "      const bx = s.x0 + prog*(s.x1-s.x0), by = s.y0 + prog*(s.y1-s.y0);\n"
            + "      const bz = (1-prog)*s.z0 + prog*s.z1 + 4*prog*(1-prog)*(s.apex - 0.5*(s.z0+s.z1));\n"
            + "      const sh = proj(bx, by, 0.01, cx, cy), bp = proj(bx, by, bz, cx, cy);\n"
            + "      ctx.fillStyle = 'rgba(0,0,0,0.45)'; ctx.beginPath(); ctx.ellipse(sh[0], sh[1], 5, 3, 0, 0, Math.PI*2); ctx.fill();\n"
            + "      ctx.fillStyle = '#facc15'; ctx.beginPath(); ctx.arc(bp[0], bp[1], 7, 0, Math.PI*2); ctx.fill();\n"
            + "      ctx.fillStyle = '#fef08a'; ctx.beginPath(); ctx.arc(bp[0]-2, bp[1]-2, 2.5, 0, Math.PI*2); ctx.fill();\n"
            + "    } else if(elapsed < s.tof + 0.65 && s.scored) {\n"
            + "      const ht = proj(HUB_X, HUB_Y, HUB_Z + 0.08, cx, cy);\n"
            + "      ctx.strokeStyle = 'rgba(52,211,153,0.88)'; ctx.lineWidth = 2.6;\n"
            + "      ctx.beginPath(); ctx.ellipse(ht[0], ht[1], 19, 10, 0, 0, Math.PI*2); ctx.stroke();\n"
            + "    }\n"
            + "  }\n"
            + "}\n"
            + "function draw2026HexHub(cx, cy) {\n"
            + "  const rBase = 0.78, zMid = 1.15, baseH = [], midH = [], topH = [];\n"
            + "  for(let i=0;i<6;i++) {\n"
            + "    const a = (60*i + 30) * Math.PI / 180.0;\n"
            + "    baseH.push([HUB_X + rBase*Math.cos(a), HUB_Y + rBase*Math.sin(a)]);\n"
            + "    midH.push([HUB_X + rBase*0.72*Math.cos(a), HUB_Y + rBase*0.72*Math.sin(a)]);\n"
            + "    topH.push([HUB_X + HUB_R*Math.cos(a), HUB_Y + HUB_R*Math.sin(a)]);\n"
            + "  }\n"
            + "  for(let i=0;i<6;i++) {\n"
            + "    const n = (i+1)%6;\n"
            + "    quad3D([[baseH[i][0],baseH[i][1],0],[baseH[n][0],baseH[n][1],0],[midH[n][0],midH[n][1],zMid],[midH[i][0],midH[i][1],zMid]], cx, cy, 'rgba(30,58,138,0.48)', 'rgba(59,130,246,0.68)', 1.2);\n"
            + "    quad3D([[midH[i][0],midH[i][1],zMid],[midH[n][0],midH[n][1],zMid],[topH[n][0],topH[n][1],HUB_Z],[topH[i][0],topH[i][1],HUB_Z]], cx, cy, 'rgba(37,99,235,0.45)', 'rgba(96,165,250,0.82)', 1.3);\n"
            + "  }\n"
            + "  ctx.strokeStyle = 'rgba(250,204,21,0.92)'; ctx.lineWidth = 2.6;\n"
            + "  for(let i=0;i<6;i++) { const n=(i+1)%6; line3D(topH[i][0],topH[i][1],HUB_Z, topH[n][0],topH[n][1],HUB_Z, cx,cy); }\n"
            + "  const lbl = proj(HUB_X, HUB_Y, HUB_Z + 0.28, cx, cy);\n"
            + "  ctx.fillStyle = '#93c5fd'; ctx.font = 'bold 11.5px system-ui,sans-serif'; ctx.fillText('2026 BLUE HUB (Z=1.83m)', lbl[0]-66, lbl[1]);\n"
            + "}\n"
            + "function draw2026Robot(f, cx, cy) {\n"
            + "  const rx = f.x, ry = f.y, c = Math.cos(f.h), sn = Math.sin(f.h), half = 0.42, zb = 0.05, zt = 0.20;\n"
            + "  const lc = [[-half,-half],[half,-half],[half,half],[-half,half]];\n"
            + "  const wc = lc.map(p => [rx + p[0]*c - p[1]*sn, ry + p[0]*sn + p[1]*c]);\n"
            + "  for(let i=0;i<4;i++) {\n"
            + "    const n = (i+1)%4;\n"
            + "    quad3D([[wc[i][0],wc[i][1],zb],[wc[n][0],wc[n][1],zb],[wc[n][0],wc[n][1],zt],[wc[i][0],wc[i][1],zt]], cx, cy, 'rgba(37,99,235,0.88)', '#93c5fd', 1.4);\n"
            + "  }\n"
            + "  quad3D([[wc[0][0],wc[0][1],zt],[wc[1][0],wc[1][1],zt],[wc[2][0],wc[2][1],zt],[wc[3][0],wc[3][1],zt]], cx, cy, 'rgba(30,41,59,0.92)', '#93c5fd', 1.3);\n"
            + "  const intaking = Math.abs(f.rol) > 5.0 || f.st === 'INTAKING';\n"
            + "  const ext = intaking ? 0.62 : 0.44;\n"
            + "  const ixL = rx + ext*c - (-0.28)*sn, iyL = ry + ext*sn + (-0.28)*c;\n"
            + "  const ixR = rx + ext*c - (0.28)*sn, iyR = ry + ext*sn + (0.28)*c;\n"
            + "  ctx.strokeStyle = intaking ? '#34d399' : '#94a3b8'; ctx.lineWidth = 3.4;\n"
            + "  line3D(ixL,iyL,0.12, ixR,iyR,0.12, cx,cy);\n"
            + "  ctx.strokeStyle = '#38bdf8'; ctx.lineWidth = 2.8;\n"
            + "  line3D(rx + 0.08*c, ry + 0.08*sn, zt+0.02, rx + 0.48*c, ry + 0.48*sn, zt+0.02, cx, cy);\n"
            + "  const sx0 = rx - 0.10*c, sy0 = ry - 0.10*sn;\n"
            + "  const pitch = (52.0 - (f.elev / 10.0) * 24.0) * Math.PI / 180.0;\n"
            + "  const sx1 = sx0 + 0.36*Math.cos(pitch)*c, sy1 = sy0 + 0.36*Math.cos(pitch)*sn, sz1 = 0.25 + 0.36*Math.sin(pitch);\n"
            + "  ctx.strokeStyle = '#facc15'; ctx.lineWidth = 4.0;\n"
            + "  line3D(sx0,sy0,0.22, sx1,sy1,sz1, cx,cy);\n"
            + "  const mz = proj(sx1, sy1, sz1, cx, cy);\n"
            + "  ctx.fillStyle = f.scale > 25.0 ? '#facc15' : '#94a3b8'; ctx.beginPath(); ctx.arc(mz[0], mz[1], 5, 0, Math.PI*2); ctx.fill();\n"
            + "}\n"
            + "function draw2027EmptyField(f, cx, cy) {\n"
            + "  quad3D([[-6,-4,0],[6,-4,0],[6,4,0],[-6,4,0]], cx, cy, '#121926', '#2d3a52', 1.4);\n"
            + "  for(let gx=-6; gx<=6; gx++) { ctx.strokeStyle = gx===0 ? 'rgba(56,189,248,0.5)' : 'rgba(51,65,85,0.45)'; ctx.lineWidth = 1; line3D(gx,-4,0, gx,4,0, cx,cy); }\n"
            + "  for(let gy=-4; gy<=4; gy++) { ctx.strokeStyle = gy===0 ? 'rgba(52,211,153,0.5)' : 'rgba(51,65,85,0.45)'; ctx.lineWidth = 1; line3D(-6,gy,0, 6,gy,0, cx,cy); }\n"
            + "  ctx.setLineDash([6,6]); ctx.strokeStyle = 'rgba(56,189,248,0.45)'; ctx.lineWidth = 1.5;\n"
            + "  line3D(-2.5,-1.5,0.01, 2.5,-1.5,0.01, cx,cy); line3D(2.5,-1.5,0.01, 2.5,1.5,0.01, cx,cy);\n"
            + "  line3D(2.5,1.5,0.01, -2.5,1.5,0.01, cx,cy); line3D(-2.5,1.5,0.01, -2.5,-1.5,0.01, cx,cy);\n"
            + "  ctx.setLineDash([]);\n"
            + "  ctx.lineWidth = 3.2;\n"
            + "  for(let i=1;i<=idx;i++) { ctx.strokeStyle = stateColor(FRAMES[i].st, 0.88); line3D(FRAMES[i-1].x, FRAMES[i-1].y, 0.02, FRAMES[i].x, FRAMES[i].y, 0.02, cx, cy); }\n"
            + "  const rx = f.x, ry = f.y, c = Math.cos(f.h), sn = Math.sin(f.h), half = 0.42, zb = 0.05, zt = 0.20;\n"
            + "  const lc = [[-half,-half],[half,-half],[half,half],[-half,half]];\n"
            + "  const wc = lc.map(p => [rx + p[0]*c - p[1]*sn, ry + p[0]*sn + p[1]*c]);\n"
            + "  for(let i=0;i<4;i++) { const n=(i+1)%4; quad3D([[wc[i][0],wc[i][1],zb],[wc[n][0],wc[n][1],zb],[wc[n][0],wc[n][1],zt],[wc[i][0],wc[i][1],zt]], cx, cy, 'rgba(37,99,235,0.88)', '#93c5fd', 1.4); }\n"
            + "  quad3D([[wc[0][0],wc[0][1],zt],[wc[1][0],wc[1][1],zt],[wc[2][0],wc[2][1],zt],[wc[3][0],wc[3][1],zt]], cx, cy, 'rgba(30,41,59,0.92)', '#93c5fd', 1.3);\n"
            + "  ctx.strokeStyle = '#facc15'; ctx.lineWidth = 3.0; line3D(rx+0.10*c, ry+0.10*sn, zt+0.02, rx+0.52*c, ry+0.52*sn, zt+0.02, cx, cy);\n"
            + "  const cz = zt + 0.18 + Math.max(0.0, f.elev);\n"
            + "  ctx.strokeStyle = '#38bdf8'; ctx.lineWidth = 3.5; line3D(rx, ry, zt, rx, ry, cz, cx, cy);\n"
            + "  const ax = rx + 0.42*Math.cos(f.arm)*c, ay = ry + 0.42*Math.cos(f.arm)*sn, azPt = cz + 0.42*Math.sin(f.arm);\n"
            + "  ctx.strokeStyle = '#facc15'; ctx.lineWidth = 3.8; line3D(rx, ry, cz, ax, ay, azPt, cx, cy);\n"
            + "  const rp = proj(ax, ay, azPt, cx, cy);\n"
            + "  ctx.fillStyle = Math.abs(f.rol) > 1.0 ? (f.rol > 0 ? '#34d399' : '#f87171') : '#e2e8f0';\n"
            + "  ctx.beginPath(); ctx.arc(rp[0], rp[1], 6.5, 0, Math.PI*2); ctx.fill();\n"
            + "}\n"
            + "function drawMiniMap(f) {\n"
            + "  const cw = canvas.clientWidth, mw = 215, mh = 144, mx = cw - mw - 16, my = 14;\n"
            + "  if(cw < 560) return;\n"
            + "  ctx.fillStyle = 'rgba(15,23,42,0.9)'; ctx.strokeStyle = '#475569'; ctx.lineWidth = 1.4;\n"
            + "  ctx.beginPath(); ctx.roundRect(mx, my, mw, mh, 10); ctx.fill(); ctx.stroke();\n"
            + "  ctx.fillStyle = '#94a3b8'; ctx.font = 'bold 10.5px system-ui,sans-serif';\n"
            + "  ctx.fillText(FIELD_2026 ? '2D TOP-DOWN (2026 Blue Wing)' : '2D TOP-DOWN MAP (5m × 3m)', mx + 12, my + 16);\n"
            + "  if(FIELD_2026) {\n"
            + "    const sc = 19.0, ox = mx + 18.0, oy = my + mh - 12.0;\n"
            + "    ctx.strokeStyle = 'rgba(59,130,246,0.4)'; ctx.strokeRect(ox, oy - 6.0*sc, 5.5*sc, 5.5*sc);\n"
            + "    ctx.fillStyle = 'rgba(16,185,129,0.28)'; ctx.fillRect(ox + 0.55*sc, oy - (6.52-1.2)*sc, 0.63*sc, 1.20*sc);\n"
            + "    const hx = ox + HUB_X*sc, hy = oy - (HUB_Y - 1.2)*sc;\n"
            + "    ctx.fillStyle = '#facc15'; ctx.beginPath(); ctx.arc(hx, hy, 5, 0, Math.PI*2); ctx.fill();\n"
            + "    for(const b of FLOOR_BALLS) {\n"
            + "      if(f.t <= b.tPick) { ctx.fillStyle = 'rgba(250,204,21,0.8)'; ctx.beginPath(); ctx.arc(ox + b.x*sc, oy - (b.y-1.2)*sc, 2.6, 0, Math.PI*2); ctx.fill(); }\n"
            + "    }\n"
            + "    ctx.lineWidth = 1.8;\n"
            + "    for(let i=1;i<=idx;i++) {\n"
            + "      ctx.strokeStyle = stateColor(FRAMES[i].st, 0.9);\n"
            + "      ctx.beginPath(); ctx.moveTo(ox + FRAMES[i-1].x*sc, oy - (FRAMES[i-1].y-1.2)*sc); ctx.lineTo(ox + FRAMES[i].x*sc, oy - (FRAMES[i].y-1.2)*sc); ctx.stroke();\n"
            + "    }\n"
            + "    ctx.strokeStyle = 'rgba(250,204,21,0.5)'; ctx.lineWidth = 1.1;\n"
            + "    for(const s of SHOTS) { if(f.t >= s.t0) { ctx.beginPath(); ctx.moveTo(ox + s.x0*sc, oy - (s.y0-1.2)*sc); ctx.lineTo(hx, hy); ctx.stroke(); } }\n"
            + "    const rx = ox + f.x*sc, ry = oy - (f.y-1.2)*sc;\n"
            + "    ctx.fillStyle = '#38bdf8'; ctx.beginPath(); ctx.arc(rx, ry, 4.2, 0, Math.PI*2); ctx.fill();\n"
            + "    ctx.strokeStyle = '#facc15'; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(rx, ry); ctx.lineTo(rx + Math.cos(f.h)*9, ry - Math.sin(f.h)*9); ctx.stroke();\n"
            + "  } else {\n"
            + "    const cx = mx + mw*0.48, cy = my + mh*0.58, sc = 26.0;\n"
            + "    ctx.strokeStyle = 'rgba(56,189,248,0.35)'; ctx.strokeRect(cx - 2.5*sc, cy - 1.5*sc, 5.0*sc, 3.0*sc);\n"
            + "    ctx.lineWidth = 2.0;\n"
            + "    for(let i=1;i<=idx;i++) { ctx.strokeStyle = stateColor(FRAMES[i].st, 0.9); ctx.beginPath(); ctx.moveTo(cx + FRAMES[i-1].x*sc, cy - FRAMES[i-1].y*sc); ctx.lineTo(cx + FRAMES[i].x*sc, cy - FRAMES[i].y*sc); ctx.stroke(); }\n"
            + "    const rx = cx + f.x*sc, ry = cy - f.y*sc;\n"
            + "    ctx.fillStyle = '#facc15'; ctx.beginPath(); ctx.arc(rx, ry, 4.2, 0, Math.PI*2); ctx.fill();\n"
            + "    ctx.strokeStyle = '#facc15'; ctx.beginPath(); ctx.moveTo(rx, ry); ctx.lineTo(rx + Math.cos(f.h)*9, ry - Math.sin(f.h)*9); ctx.stroke();\n"
            + "  }\n"
            + "}\n"
            + "function draw() {\n"
            + "  const dpr = window.devicePixelRatio || 1;\n"
            + "  const cw = Math.max(1, canvas.clientWidth), ch = Math.max(1, canvas.clientHeight);\n"
            + "  canvas.width = Math.floor(cw * dpr); canvas.height = Math.floor(ch * dpr);\n"
            + "  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);\n"
            + "  ctx.fillStyle = '#0a0e17'; ctx.fillRect(0, 0, cw, ch);\n"
            + "  if (!FRAMES.length) return;\n"
            + "  const f = FRAMES[idx];\n"
            + "  const cx = FIELD_2026 ? 3.15 : f.x * 0.35, cy = FIELD_2026 ? 4.10 : f.y * 0.35;\n"
            + "  if(FIELD_2026) draw2026Field(f, cx, cy); else draw2027EmptyField(f, cx, cy);\n"
            + "  drawMiniMap(f);\n"
            + "  const spd = Math.hypot(f.vx, f.vy);\n"
            + "  document.getElementById('timeLbl').textContent = `T = ${f.t.toFixed(2)}s / ${FRAMES[FRAMES.length-1].t.toFixed(2)}s`;\n"
            + "  document.getElementById('poseLbl').textContent = `POSE: X=${(f.x>=0?'+':'')+f.x.toFixed(2)}m Y=${(f.y>=0?'+':'')+f.y.toFixed(2)}m Yaw=${(f.h*180/Math.PI).toFixed(1)}°  |  SPEED: ${spd.toFixed(2)} m/s (ω=${f.om.toFixed(2)} rad/s)`;\n"
            + "  if(FIELD_2026) {\n"
            + "    const scored = SHOTS.filter(s => s.scored && (s.t0 + s.tof) <= f.t).length;\n"
            + "    const launched = SHOTS.filter(s => s.t0 <= f.t).length;\n"
            + "    document.getElementById('mechLbl').textContent = `SHOOTER: ${f.scale.toFixed(1)} RPS  Hood=${f.elev.toFixed(2)} rot  Intake=${f.rol.toFixed(1)} RPS  |  BATT: ${f.vBat.toFixed(2)}V (${f.iDrv.toFixed(0)}A)  |  HUB SCORES: ${scored} / ${launched}`;\n"
            + "  } else {\n"
            + "    document.getElementById('mechLbl').textContent = `ELEV: ${f.elev.toFixed(2)}m  Arm=${(f.arm*180/Math.PI).toFixed(1)}°  Roller=${f.rol.toFixed(1)} RPS  |  BATT: ${f.vBat.toFixed(2)}V (${f.iDrv.toFixed(0)}A)  CG Scale=${f.scale.toFixed(2)}x`;\n"
            + "  }\n"
            + "  const badge = document.getElementById('stateBadge');\n"
            + "  badge.textContent = 'STATE: ' + f.st;\n"
            + "  badge.style.background = stateColor(f.st, 1.0);\n"
            + "  scrubber.value = idx;\n"
            + "}\n"
            + "setInterval(() => {\n"
            + "  if(playing && FRAMES.length) {\n"
            + "    tickAcc += SPEEDS[speedIdx];\n"
            + "    while(tickAcc >= 1.0) { idx = (idx + 1) % FRAMES.length; tickAcc -= 1.0; }\n"
            + "    draw();\n"
            + "  }\n"
            + "}, 40);\n"
            + "window.addEventListener('resize', draw);\n"
            + "draw();\n"
            + "</script>\n"
            + "</body>\n"
            + "</html>\n";
    Files.writeString(htmlFile, html, StandardCharsets.UTF_8);
  }

  private static boolean isEmptyStudentStarter(
      String routineName, List<AutoFrame> frames, List<ShotArc> shots) {
    if (routineName == null
        || !routineName.toUpperCase(Locale.US).startsWith("STUDENT")
        || !shots.isEmpty()) {
      return false;
    }
    double pathLengthM = 0.0;
    for (int i = 1; i < frames.size(); i++) {
      pathLengthM +=
          Math.hypot(
              frames.get(i).xMeters() - frames.get(i - 1).xMeters(),
              frames.get(i).yMeters() - frames.get(i - 1).yMeters());
    }
    return pathLengthM < 0.02;
  }

  private static String escapeJson(String s) {
    return (s == null) ? "" : s.replace("\\", "\\\\").replace("\"", "\\\"").replace("</", "<\\/");
  }

  private static String escapeHtml(String s) {
    if (s == null) {
      return "";
    }
    return s.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace("\"", "&quot;")
        .replace("'", "&#39;");
  }
}
