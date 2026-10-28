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
      String stateLabel) {}

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
      List<FloorFuelBall> floorBalls) {}

  /**
   * Reads {@code student-auto-frames.json} from {@code jsonFile} and renders {@code
   * student-auto.html} and {@code student-auto.mp4} into {@code outputDir}.
   */
  public static AutoSimPayload renderFromJson(Path jsonFile, Path outputDir) throws IOException {
    String raw = Files.readString(jsonFile, StandardCharsets.UTF_8);
    AutoSimPayload payload = parseFramesJson(raw);
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
          payload.field2026Shooting());
    }
    writeInteractiveHtml(
        outputDir.resolve("student-auto.html"),
        payload.routineName(),
        payload.robotName(),
        payload.frames(),
        payload.shots(),
        payload.floorBalls(),
        payload.field2026Shooting());
    renderMp4(
        outputDir.resolve("student-auto.mp4"),
        payload.routineName(),
        payload.robotName(),
        payload.frames(),
        payload.shots(),
        payload.floorBalls(),
        payload.field2026Shooting());
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

    writeFramesJson(jsonFile, routineName, robotName, frames, shots, floorBalls, field2026Shooting);
    writeInteractiveHtml(
        htmlFile, routineName, robotName, frames, shots, floorBalls, field2026Shooting);
    renderMp4(mp4File, routineName, robotName, frames, shots, floorBalls, field2026Shooting);
  }

  /** Parses {@code student-auto-frames.json} into {@link AutoSimPayload}. */
  public static AutoSimPayload parseFramesJson(String json) {
    String routineName = extractStringField(json, "routineName", "STUDENT: Default Autonomous");
    String robotName = extractStringField(json, "robotName", "COMP_2027");
    String fieldMode = extractStringField(json, "fieldMode", "EMPTY_FIELD_2027");
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

    return new AutoSimPayload(routineName, robotName, field2026, frames, shots, floorBalls);
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
      boolean field2026Shooting)
      throws IOException {
    if (frames.isEmpty() || !FuelSimVideoRenderer.isFfmpegAvailable()) {
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
              totalDuration);
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
      double totalDuration) {
    AutoFrame f = frames.get(idx);

    // 1. Dark Arena Backdrop
    g.setColor(new Color(10, 14, 23));
    g.fillRect(0, 0, WIDTH, HEIGHT);

    if (field2026) {
      draw2026RebuiltField3D(g, f, frames, shots, floorBalls, idx, camX, camY);
    } else {
      draw2027EmptyField3D(g, f, frames, idx, camX, camY);
    }

    // 6. Top-Right Picture-in-Picture (PiP) 2D Top-Down Blueprint Mini-Map
    drawTopDownMiniMap(g, frames, shots, floorBalls, field2026, idx);

    // 7. Top Broadcast HUD Bar
    g.setColor(new Color(15, 23, 42, 235));
    g.fillRect(0, 0, WIDTH, 46);
    g.setColor(new Color(56, 189, 248));
    g.fillRect(0, 44, WIDTH, 2);

    g.setFont(new Font("SansSerif", Font.BOLD, 13));
    g.setColor(Color.WHITE);
    g.drawString("FLUX 10413 • " + routineName + " (" + robotName + ")", 16, 28);

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

    // 8. Bottom Telemetry HUD Bar
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
              f.accelScale(),
              f.elevatorHeightM(),
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
    // 1. Empty Field Carpet & 1.0 m Grid Lines (-6 m to +6 m)
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
    // 1. 2026 Blue Half-Field Carpet (0.0m to 8.5m X, 0.5m to 7.7m Y)
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

    // 2. Floor FUEL Balls (15 cm yellow game pieces)
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

    // 3. Full Planned Ghost Path & Driven Odometry Trail
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

    // 4. Aiming Laser Line from Robot to Blue Hub when PREPPING_SHOT or SHOOTING
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

    // 5. 3D 2026 FUEL Robot
    draw2026FuelRobot3D(g, f, camX, camY);

    // 6. 3D Blue Hexagonal Hub at (4.63m, 4.21m, Z=1.83m)
    draw2026HexHub3D(g, camX, camY);

    // 7. 3D Parabolic FUEL Shot Trajectories & In-Flight FUEL Balls
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
    StringBuilder sb = new StringBuilder(32768);
    sb.append("{\n");
    sb.append("  \"routineName\": \"").append(escapeJson(routineName)).append("\",\n");
    sb.append("  \"robotName\": \"").append(escapeJson(robotName)).append("\",\n");
    sb.append("  \"fieldMode\": \"")
        .append(field2026 ? "REBUILT_2026_SHOOTING" : "EMPTY_FIELD_2027")
        .append("\",\n");
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
      boolean field2026)
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

    String html =
        "<!DOCTYPE html>\n"
            + "<html lang=\"en\">\n"
            + "<head>\n"
            + "<meta charset=\"UTF-8\">\n"
            + "<title>FLUX 10413 — "
            + escapeHtml(routineName)
            + "</title>\n"
            + "<style>\n"
            + "  body{margin:0;background:#0a0e17;color:#e2e8f0;font-family:system-ui,sans-serif;display:flex;flex-direction:column;height:100vh;overflow:hidden;}\n"
            + "  header{padding:10px 18px;background:#0f172a;border-bottom:2px solid #38bdf8;display:flex;justify-content:space-between;align-items:center;}\n"
            + "  .badge{padding:4px 12px;border-radius:6px;background:#38bdf8;color:#0f172a;font-weight:700;font-size:13px;}\n"
            + "  #viewport{flex:1;position:relative;display:flex;justify-content:center;align-items:center;}\n"
            + "  canvas{width:100%;height:100%;display:block;cursor:grab;}\n"
            + "  footer{padding:10px 18px;background:#0f172a;border-top:1px solid #334155;display:flex;gap:14px;align-items:center;font-family:monospace;font-size:13px;}\n"
            + "  button{background:#2563eb;color:#fff;border:none;padding:6px 14px;border-radius:6px;font-weight:700;cursor:pointer;}\n"
            + "  input[type=range]{flex:1;accent-color:#38bdf8;}\n"
            + "</style>\n"
            + "</head>\n"
            + "<body>\n"
            + "<header>\n"
            + "  <div><strong>FLUX 10413 • "
            + escapeHtml(routineName)
            + "</strong> <span style=\"color:#94a3b8;margin-left:10px;\">("
            + escapeHtml(robotName)
            + ")</span></div>\n"
            + "  <div id=\"stateBadge\" class=\"badge\">STATE: IDLE</div>\n"
            + "</header>\n"
            + "<div id=\"viewport\"><canvas id=\"simCanvas\"></canvas></div>\n"
            + "<footer>\n"
            + "  <button id=\"playBtn\">Pause</button>\n"
            + "  <span id=\"timeLbl\">0.00s / 15.00s</span>\n"
            + "  <input id=\"scrubber\" type=\"range\" min=\"0\" max=\""
            + Math.max(0, frames.size() - 1)
            + "\" value=\"0\">\n"
            + "  <span id=\"poseLbl\">X=+0.00m Y=+0.00m</span>\n"
            + "</footer>\n"
            + "<script>\n"
            + "const FRAMES = "
            + framesJs
            + ";\n"
            + "const SHOTS = "
            + shotsJs
            + ";\n"
            + "const FIELD_2026 = "
            + (field2026 ? "true" : "false")
            + ";\n"
            + "const canvas = document.getElementById('simCanvas');\n"
            + "const ctx = canvas.getContext('2d');\n"
            + "let idx = 0, playing = true, az = -0.66, el = 0.59, zoom = FIELD_2026 ? 95 : 130, dragging = false, lx = 0, ly = 0;\n"
            + "canvas.addEventListener('mousedown', e => { dragging = true; lx = e.clientX; ly = e.clientY; });\n"
            + "window.addEventListener('mouseup', () => dragging = false);\n"
            + "window.addEventListener('mousemove', e => { if(!dragging) return; az += (e.clientX - lx)*0.007; el = Math.max(0.2, Math.min(1.4, el + (e.clientY - ly)*0.007)); lx = e.clientX; ly = e.clientY; });\n"
            + "canvas.addEventListener('wheel', e => { zoom = Math.max(50, Math.min(260, zoom - e.deltaY * 0.1)); });\n"
            + "const scrubber = document.getElementById('scrubber');\n"
            + "const playBtn = document.getElementById('playBtn');\n"
            + "playBtn.onclick = () => { playing = !playing; playBtn.textContent = playing ? 'Pause' : 'Play'; };\n"
            + "scrubber.oninput = e => { playing = false; playBtn.textContent = 'Play'; idx = parseInt(e.target.value, 10); };\n"
            + "function proj(wx, wy, wz, cx, cy) {\n"
            + "  const dx = wx - cx, dy = wy - cy;\n"
            + "  const rx = dx * Math.cos(az) - dy * Math.sin(az);\n"
            + "  const ry = dx * Math.sin(az) + dy * Math.cos(az);\n"
            + "  return [canvas.width * 0.48 + rx * zoom, canvas.height * 0.56 - (ry * Math.sin(el) + wz * Math.cos(el)) * zoom];\n"
            + "}\n"
            + "function draw() {\n"
            + "  canvas.width = canvas.clientWidth; canvas.height = canvas.clientHeight;\n"
            + "  ctx.fillStyle = '#0a0e17'; ctx.fillRect(0, 0, canvas.width, canvas.height);\n"
            + "  if (!FRAMES.length) return;\n"
            + "  const f = FRAMES[idx];\n"
            + "  const cx = FIELD_2026 ? 3.15 : f.x * 0.35, cy = FIELD_2026 ? 4.10 : f.y * 0.35;\n"
            + "  ctx.strokeStyle = '#334155'; ctx.lineWidth = 1;\n"
            + "  const gxMin = FIELD_2026 ? 0 : -6, gxMax = FIELD_2026 ? 8 : 6;\n"
            + "  const gyMin = FIELD_2026 ? 1 : -4, gyMax = FIELD_2026 ? 7 : 4;\n"
            + "  for(let gx=gxMin; gx<=gxMax; gx++) { const a=proj(gx,gyMin,0,cx,cy), b=proj(gx,gyMax,0,cx,cy); ctx.beginPath(); ctx.moveTo(a[0],a[1]); ctx.lineTo(b[0],b[1]); ctx.stroke(); }\n"
            + "  for(let gy=gyMin; gy<=gyMax; gy++) { const a=proj(gxMin,gy,0,cx,cy), b=proj(gxMax,gy,0,cx,cy); ctx.beginPath(); ctx.moveTo(a[0],a[1]); ctx.lineTo(b[0],b[1]); ctx.stroke(); }\n"
            + "  ctx.strokeStyle = '#38bdf8'; ctx.lineWidth = 3; ctx.beginPath();\n"
            + "  for(let i=0; i<=idx; i++) { const p = proj(FRAMES[i].x, FRAMES[i].y, 0.02, cx, cy); if(i===0) ctx.moveTo(p[0],p[1]); else ctx.lineTo(p[0],p[1]); }\n"
            + "  ctx.stroke();\n"
            + "  for(const s of SHOTS) {\n"
            + "    if(f.t < s.t0) continue;\n"
            + "    const prog = Math.min(1.0, (f.t - s.t0) / Math.max(0.05, s.tof));\n"
            + "    ctx.strokeStyle = prog < 1.0 ? '#facc15' : '#34d399'; ctx.lineWidth = 2.5; ctx.beginPath();\n"
            + "    for(let k=0; k<=20; k++) {\n"
            + "      const u = Math.min(prog, k/20.0);\n"
            + "      const bx = s.x0 + u*(s.x1-s.x0), by = s.y0 + u*(s.y1-s.y0);\n"
            + "      const bz = (1-u)*s.z0 + u*s.z1 + 4*u*(1-u)*(s.apex - 0.5*(s.z0+s.z1));\n"
            + "      const sp = proj(bx, by, bz, cx, cy);\n"
            + "      if(k===0) ctx.moveTo(sp[0],sp[1]); else ctx.lineTo(sp[0],sp[1]);\n"
            + "    }\n"
            + "    ctx.stroke();\n"
            + "  }\n"
            + "  const rp = proj(f.x, f.y, 0.15, cx, cy);\n"
            + "  ctx.fillStyle = '#2563eb'; ctx.beginPath(); ctx.arc(rp[0], rp[1], 14, 0, Math.PI*2); ctx.fill();\n"
            + "  document.getElementById('timeLbl').textContent = f.t.toFixed(2) + 's / ' + FRAMES[FRAMES.length-1].t.toFixed(2) + 's';\n"
            + "  document.getElementById('poseLbl').textContent = `X=${f.x.toFixed(2)}m Y=${f.y.toFixed(2)}m Yaw=${(f.h*180/Math.PI).toFixed(1)}°`;\n"
            + "  document.getElementById('stateBadge').textContent = 'STATE: ' + f.st;\n"
            + "  scrubber.value = idx;\n"
            + "}\n"
            + "setInterval(() => { if(playing && FRAMES.length) { idx = (idx + 1) % FRAMES.length; } draw(); }, 40);\n"
            + "</script>\n"
            + "</body>\n"
            + "</html>\n";
    Files.writeString(htmlFile, html, StandardCharsets.UTF_8);
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
