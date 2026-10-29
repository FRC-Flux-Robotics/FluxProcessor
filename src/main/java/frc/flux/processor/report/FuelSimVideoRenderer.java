package frc.flux.processor.report;

import frc.flux.processor.analysis.FuelSimEngine;
import frc.flux.processor.model.RunAnalysisReport.Sim3DFrame;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.geom.Path2D;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.io.OutputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

/**
 * High-speed headless 3D isometric video renderer ({@code replay-3d.mp4}) for {@code FluxProcessor}.
 *
 * <p>Renders 3D field geometry (2026 <i>REBUILT</i> Octagonal Hubs, Trenches, Alliance Zones, and
 * 2027 Elevator/Arm superstructure), 3D robot odometry pose, airborne &amp; floor game pieces with
 * parabolic flight trails, and a live telemetry broadcast HUD into a zero-copy {@code
 * TYPE_3BYTE_BGR} buffer piped directly to {@code ffmpeg -pix_fmt bgr24}.
 */
public final class FuelSimVideoRenderer {

  private static final int WIDTH = 960;
  private static final int HEIGHT = 540;
  private static final int FPS = 25;

  private static volatile Boolean ffmpegAvailable = null;

  private FuelSimVideoRenderer() {}

  /** Checks once per JVM process whether {@code ffmpeg} is available on {@code PATH}. */
  public static boolean isFfmpegAvailable() {
    Boolean cached = ffmpegAvailable;
    if (cached != null) {
      return cached;
    }
    try {
      Process p =
          new ProcessBuilder("ffmpeg", "-version")
              .redirectOutput(ProcessBuilder.Redirect.DISCARD)
              .redirectError(ProcessBuilder.Redirect.DISCARD)
              .start();
      boolean finished = p.waitFor(3, TimeUnit.SECONDS);
      if (!finished) {
        p.destroyForcibly();
      }
      boolean ok = finished && p.exitValue() == 0;
      ffmpegAvailable = ok;
      return ok;
    } catch (Exception e) {
      ffmpegAvailable = false;
      return false;
    }
  }

  /**
   * Renders {@code replay-3d.mp4} into {@code targetRunDir} if {@code ffmpeg} is installed and
   * {@code frames} is non-empty. Prefers the hardware-accelerated MuJoCo 3.15 OpenGL/EGL 3D CAD
   * renderer ({@code scripts/render_mujoco_replay.py} + {@code assets/meshes/*.stl}) when
   * available, falling back to the built-in pure-Java renderer otherwise. Returns {@code
   * "replay-3d.mp4"} on success, or {@code null} if skipped.
   */
  public static String renderMp4IfAvailable(
      Path targetRunDir,
      String runId,
      String robotName,
      String alliance,
      String batteryName,
      List<Sim3DFrame> frames) {
    if (frames == null || frames.size() < 2 || !isFfmpegAvailable()) {
      return null;
    }
    Path outMp4 = targetRunDir.resolve("replay-3d.mp4");

    // 1. Write sim-frames.json and try hardware-accelerated MuJoCo 3.15 OpenGL/EGL + 14-mesh CAD renderer
    try {
      Path simJson = targetRunDir.resolve("sim-frames.json");
      writeSimFramesJson(simJson, runId, robotName, alliance, batteryName, frames);
      if (tryRenderMujocoMp4(simJson, outMp4)) {
        return outMp4.getFileName().toString();
      }
    } catch (Exception ignored) {
      // Fall back to pure-Java 3D renderer below
    }

    boolean isBlue = alliance == null || !alliance.toLowerCase(Locale.ROOT).startsWith("red");

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
            "ultrafast",
            "-crf",
            "23",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            outMp4.toAbsolutePath().toString());
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

      // Keep a short ring of recent airborne ball coordinates for smooth 3D shot trails
      List<double[]> airborneTrail = new ArrayList<>();

      // Adaptive step: render every frame during shot windows, and every 2nd frame during idle cruise
      // so a 150s match produces a crisp ~45-60s highlight/2x broadcast video in ~1 second
      try (OutputStream stdin = proc.getOutputStream()) {
        for (int i = 0; i < frames.size(); i++) {
          Sim3DFrame f = frames.get(i);
          boolean hasAirborne = hasAirborneBall(f.ballXyz()) || f.shooterRps() > 18.0;
          if (!hasAirborne && (i % 2 == 1) && frames.size() > 400) {
            continue;
          }
          updateAirborneTrail(airborneTrail, f.ballXyz());
          drawFrame(g, runId, robotName, alliance, isBlue, batteryName, f, airborneTrail);
          stdin.write(bgrBytes);
        }
        stdin.flush();
      } finally {
        g.dispose();
      }

      boolean done = proc.waitFor(30, TimeUnit.SECONDS);
      if (!done) {
        proc.destroyForcibly();
      }
      if (done && proc.exitValue() == 0 && Files.exists(outMp4) && Files.size(outMp4) > 1000) {
        return outMp4.getFileName().toString();
      }
    } catch (Exception ignored) {
      // Ignore video generation errors on headless systems without codec support
    }
    return null;
  }

  private static boolean hasAirborneBall(double[] xyz) {
    if (xyz == null) {
      return false;
    }
    for (int i = 2; i < xyz.length; i += 3) {
      if (xyz[i] > 0.14) {
        return true;
      }
    }
    return false;
  }

  private static void updateAirborneTrail(List<double[]> trail, double[] xyz) {
    if (xyz != null) {
      for (int i = 0; i + 2 < xyz.length; i += 3) {
        if (xyz[i + 2] > 0.14) {
          trail.add(new double[] {xyz[i], xyz[i + 1], xyz[i + 2]});
        }
      }
    }
    while (trail.size() > 45) {
      trail.remove(0);
    }
  }

  /**
   * Projects 3D field coordinates {@code (fx, fy, fz)} in meters into 2D screen coordinates {@code
   * [sx, sy]} using an elevated broadcast isometric camera looking across the 16.54m x 8.07m field.
   */
  private static double[] project(double fx, double fy, double fz) {
    double cx = fx - FuelSimEngine.FIELD_LENGTH_M * 0.5;
    double cy = fy - FuelSimEngine.FIELD_WIDTH_M * 0.5;
    // Slight yaw + 34-degree pitch broadcast view
    double yawRad = Math.toRadians(-12.0);
    double pitchRad = Math.toRadians(36.0);
    double rx = cx * Math.cos(yawRad) - cy * Math.sin(yawRad);
    double ry = cx * Math.sin(yawRad) + cy * Math.cos(yawRad);

    double scale = 47.0; // pixels per meter
    double sx = WIDTH * 0.50 + rx * scale;
    double sy = HEIGHT * 0.54 - (ry * Math.sin(pitchRad) + fz * Math.cos(pitchRad) * 1.18) * scale;
    return new double[] {sx, sy};
  }

  private static void drawFrame(
      Graphics2D g,
      String runId,
      String robotName,
      String alliance,
      boolean isBlue,
      String batteryName,
      Sim3DFrame f,
      List<double[]> airborneTrail) {
    g.setColor(new Color(17, 19, 24));
    g.fillRect(0, 0, WIDTH, HEIGHT);

    double L = FuelSimEngine.FIELD_LENGTH_M;
    double W = FuelSimEngine.FIELD_WIDTH_M;

    fillQuad3D(g, 0, 0, 0, 4.62, 0, 0, 4.62, W, 0, 0, W, 0, new Color(28, 48, 92)); // Blue Zone
    fillQuad3D(
        g, 4.62, 0, 0, 11.92, 0, 0, 11.92, W, 0, 4.62, W, 0, new Color(32, 36, 44)); // Neutral
    fillQuad3D(g, 11.92, 0, 0, L, 0, 0, L, W, 0, 11.92, W, 0, new Color(88, 30, 36)); // Red Zone

    g.setStroke(new BasicStroke(1.0f));
    g.setColor(new Color(255, 255, 255, 32));
    for (double x = 2.0; x < L; x += 2.0) {
      drawLine3D(g, x, 0, 0, x, W, 0);
    }
    for (double y = 2.0; y < W; y += 2.0) {
      drawLine3D(g, 0, y, 0, L, y, 0);
    }
    g.setStroke(new BasicStroke(2.0f));
    g.setColor(new Color(255, 255, 255, 85));
    drawLine3D(g, L * 0.5, 0, 0, L * 0.5, W, 0);

    // Field perimeter walls (z = 0..0.50m)
    g.setStroke(new BasicStroke(2.2f));
    g.setColor(new Color(59, 130, 246, 180));
    drawLine3D(g, 0, 0, 0.25, 0, W, 0.25);
    g.setColor(new Color(239, 68, 68, 180));
    drawLine3D(g, L, 0, 0.25, L, W, 0.25);
    g.setColor(new Color(148, 163, 184, 140));
    drawLine3D(g, 0, 0, 0.25, L, 0, 0.25);
    drawLine3D(g, 0, W, 0.25, L, W, 0.25);

    // Trench Pillars
    drawBox3D(g, 4.62, 0.65, 0, 0.35, 0.40, 0.89, new Color(59, 130, 246, 130));
    drawBox3D(g, 4.62, 7.42, 0, 0.35, 0.40, 0.89, new Color(59, 130, 246, 130));
    drawBox3D(g, 11.92, 0.65, 0, 0.35, 0.40, 0.89, new Color(239, 68, 68, 130));
    drawBox3D(g, 11.92, 7.42, 0, 0.35, 0.40, 0.89, new Color(239, 68, 68, 130));

    // Blue & Red Octagonal Hubs (z = 0..1.65m funnel + z = 1.65..2.8m net)
    drawHub3D(
        g,
        FuelSimEngine.BLUE_HUB_X,
        FuelSimEngine.BLUE_HUB_Y,
        new Color(37, 99, 235),
        new Color(96, 165, 250));
    drawHub3D(
        g,
        FuelSimEngine.RED_HUB_X,
        FuelSimEngine.RED_HUB_Y,
        new Color(220, 38, 38),
        new Color(248, 113, 113));

    if (airborneTrail.size() >= 2) {
      g.setStroke(new BasicStroke(2.5f));
      for (int i = 1; i < airborneTrail.size(); i++) {
        double[] a = airborneTrail.get(i - 1);
        double[] b = airborneTrail.get(i);
        if (Math.hypot(b[0] - a[0], b[1] - a[1]) < 1.2) {
          int alpha = Math.min(240, 40 + (i * 200) / airborneTrail.size());
          g.setColor(new Color(56, 189, 248, alpha));
          drawLine3D(g, a[0], a[1], a[2], b[0], b[1], b[2]);
        }
      }
    }

    drawRobot3D(g, f, isBlue);

    double[] balls = f.ballXyz();
    if (balls != null) {
      for (int i = 0; i + 2 < balls.length; i += 3) {
        double bx = balls[i];
        double by = balls[i + 1];
        double bz = balls[i + 2];
        double[] shadowPt = project(bx, by, 0.0);
        g.setColor(new Color(0, 0, 0, 110));
        g.fillOval((int) shadowPt[0] - 4, (int) shadowPt[1] - 2, 8, 5);

        double[] ballPt = project(bx, by, bz);
        int rad = bz > 0.15 ? 6 : 5;
        g.setColor(bz > 0.15 ? new Color(250, 204, 21) : new Color(234, 179, 8));
        g.fillOval((int) ballPt[0] - rad, (int) ballPt[1] - rad, rad * 2, rad * 2);
        g.setColor(new Color(254, 240, 138));
        g.fillOval((int) ballPt[0] - rad / 2, (int) ballPt[1] - rad / 2, rad, rad);
      }
    }

    g.setColor(new Color(15, 23, 42, 230));
    g.fillRect(0, 0, WIDTH, 46);
    g.setColor(isBlue ? new Color(37, 99, 235) : new Color(220, 38, 38));
    g.fillRect(0, 44, WIDTH, 2);

    g.setFont(new Font("SansSerif", Font.BOLD, 14));
    g.setColor(Color.WHITE);
    g.drawString("FLUX 10413 • " + runId + " (" + robotName + ")", 16, 28);

    g.setFont(new Font("Monospaced", Font.BOLD, 15));
    g.setColor(new Color(56, 189, 248));
    g.drawString(String.format(Locale.US, "T = %5.1fs", f.timeSec()), 420, 29);

    g.setFont(new Font("SansSerif", Font.BOLD, 13));
    g.setColor(new Color(96, 165, 250));
    g.drawString("BLUE HUB: " + f.blueScore(), 555, 28);
    g.setColor(new Color(248, 113, 113));
    g.drawString("RED HUB: " + f.redScore(), 680, 28);
    g.setColor(new Color(250, 204, 21));
    g.drawString("INTAKED: " + f.intakedCount(), 795, 28);

    g.setColor(new Color(15, 23, 42, 235));
    g.fillRect(0, HEIGHT - 54, WIDTH, 54);

    boolean brownout = f.batteryVoltage() < 6.8;
    g.setFont(new Font("SansSerif", Font.BOLD, 13));
    g.setColor(brownout ? new Color(239, 68, 68) : new Color(34, 197, 94));
    g.drawString(
        String.format(
            Locale.US,
            "BATT (%s): %5.2f V %s",
            batteryName != null ? batteryName : "Batt",
            f.batteryVoltage(),
            brownout ? "[BROWNOUT!]" : ""),
        16,
        HEIGHT - 30);

    g.setColor(new Color(148, 163, 184));
    g.drawString(
        String.format(
            Locale.US,
            "DRIVE STATOR: %4.0f A   •   SHOOTER: %4.1f RPS   •   HOOD: %4.2f rot",
            f.driveStatorAmps(),
            f.shooterRps(),
            f.hoodRot()),
        16,
        HEIGHT - 11);

    if (f.lastShotBanner() != null && !f.lastShotBanner().isEmpty()) {
      boolean scored = f.lastShotBanner().contains("SCORED");
      g.setColor(scored ? new Color(22, 101, 52, 220) : new Color(153, 27, 27, 220));
      g.fillRoundRect(465, HEIGHT - 44, WIDTH - 480, 34, 8, 8);
      g.setFont(new Font("SansSerif", Font.BOLD, 11));
      g.setColor(Color.WHITE);
      FontMetrics fm = g.getFontMetrics();
      String text = f.lastShotBanner();
      if (fm.stringWidth(text) > WIDTH - 500 && text.length() > 64) {
        text = text.substring(0, 64) + "...";
      }
      g.drawString(text, 475, HEIGHT - 23);
    }
  }

  private static void drawHub3D(
      Graphics2D g, double hx, double hy, Color baseColor, Color rimColor) {
    // Draw octagonal rim at z = 1.65m and base at z = 0m
    int sides = 8;
    double rBase = 0.45;
    double rRim = FuelSimEngine.HUB_SCORE_RADIUS_M;
    double zRim = FuelSimEngine.HUB_RIM_HEIGHT_M;

    Path2D rimPoly = new Path2D.Double();
    for (int i = 0; i <= sides; i++) {
      double ang = i * (2.0 * Math.PI / sides);
      double rx = hx + rRim * Math.cos(ang);
      double ry = hy + rRim * Math.sin(ang);
      double[] pt = project(rx, ry, zRim);
      if (i == 0) {
        rimPoly.moveTo(pt[0], pt[1]);
      } else {
        rimPoly.lineTo(pt[0], pt[1]);
      }
      if (i < sides) {
        double bx = hx + rBase * Math.cos(ang);
        double by = hy + rBase * Math.sin(ang);
        g.setColor(new Color(baseColor.getRed(), baseColor.getGreen(), baseColor.getBlue(), 140));
        g.setStroke(new BasicStroke(1.5f));
        drawLine3D(g, bx, by, 0, rx, ry, zRim);
      }
    }
    g.setColor(new Color(baseColor.getRed(), baseColor.getGreen(), baseColor.getBlue(), 95));
    g.fill(rimPoly);
    g.setColor(rimColor);
    g.setStroke(new BasicStroke(2.4f));
    g.draw(rimPoly);
  }

  private static void drawRobot3D(Graphics2D g, Sim3DFrame f, boolean isBlue) {
    double rx = f.robotX();
    double ry = f.robotY();
    double h = f.robotHeadingRad();
    double cos = Math.cos(h);
    double sin = Math.sin(h);
    double half = 0.43;
    double zTop = 0.22;

    double[][] localCorners = {{-half, -half}, {half, -half}, {half, half}, {-half, half}};
    double[][] worldCorners = new double[4][2];
    for (int i = 0; i < 4; i++) {
      worldCorners[i][0] = rx + localCorners[i][0] * cos - localCorners[i][1] * sin;
      worldCorners[i][1] = ry + localCorners[i][0] * sin + localCorners[i][1] * cos;
    }

    Color bumperFill = isBlue ? new Color(37, 99, 235, 210) : new Color(220, 38, 38, 210);
    Color topFill = new Color(51, 65, 85, 230);

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
        topFill);

    g.setColor(bumperFill);
    g.setStroke(new BasicStroke(3.0f));
    for (int i = 0; i < 4; i++) {
      int next = (i + 1) % 4;
      drawLine3D(
          g,
          worldCorners[i][0],
          worldCorners[i][1],
          0.10,
          worldCorners[next][0],
          worldCorners[next][1],
          0.10);
    }

    g.setColor(new Color(250, 204, 21));
    g.setStroke(new BasicStroke(2.5f));
    drawLine3D(g, rx, ry, zTop + 0.05, rx + 0.58 * cos, ry + 0.58 * sin, zTop + 0.05);

    // 2027 Elevator + Arm 3D visualization (when elevatorHeightM > 0.05m)
    if (f.elevatorHeightM() > 0.05) {
      double elevZ = zTop + f.elevatorHeightM();
      g.setColor(new Color(56, 189, 248));
      g.setStroke(new BasicStroke(3.5f));
      drawLine3D(g, rx, ry, zTop, rx, ry, elevZ);
      double armLen = 0.52;
      double armEndX = rx + armLen * Math.cos(f.armAngleRad()) * cos;
      double armEndY = ry + armLen * Math.cos(f.armAngleRad()) * sin;
      double armEndZ = elevZ + armLen * Math.sin(f.armAngleRad());
      g.setColor(new Color(250, 204, 21));
      drawLine3D(g, rx, ry, elevZ, armEndX, armEndY, armEndZ);
    }
  }

  private static void drawBox3D(
      Graphics2D g, double cx, double cy, double z0, double dx, double dy, double dz, Color c) {
    double x0 = cx - dx * 0.5, x1 = cx + dx * 0.5;
    double y0 = cy - dy * 0.5, y1 = cy + dy * 0.5;
    fillQuad3D(g, x0, y0, z0 + dz, x1, y0, z0 + dz, x1, y1, z0 + dz, x0, y1, z0 + dz, c);
    g.setColor(c.brighter());
    g.setStroke(new BasicStroke(1.4f));
    drawLine3D(g, x0, y0, z0, x0, y0, z0 + dz);
    drawLine3D(g, x1, y0, z0, x1, y0, z0 + dz);
    drawLine3D(g, x1, y1, z0, x1, y1, z0 + dz);
    drawLine3D(g, x0, y1, z0, x0, y1, z0 + dz);
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
      Color color) {
    double[] p0 = project(x0, y0, z0);
    double[] p1 = project(x1, y1, z1);
    double[] p2 = project(x2, y2, z2);
    double[] p3 = project(x3, y3, z3);
    Path2D path = new Path2D.Double();
    path.moveTo(p0[0], p0[1]);
    path.lineTo(p1[0], p1[1]);
    path.lineTo(p2[0], p2[1]);
    path.lineTo(p3[0], p3[1]);
    path.closePath();
    g.setColor(color);
    g.fill(path);
  }

  private static void drawLine3D(
      Graphics2D g, double x0, double y0, double z0, double x1, double y1, double z1) {
    double[] p0 = project(x0, y0, z0);
    double[] p1 = project(x1, y1, z1);
    g.drawLine(
        (int) Math.round(p0[0]),
        (int) Math.round(p0[1]),
        (int) Math.round(p1[0]),
        (int) Math.round(p1[1]));
  }

  private static void writeSimFramesJson(
      Path simJson,
      String runId,
      String robotName,
      String alliance,
      String batteryName,
      List<Sim3DFrame> frames)
      throws Exception {
    StringBuilder sb = new StringBuilder(frames.size() * 260);
    sb.append("{\n");
    sb.append("  \"runId\": \"").append(escapeJson(runId)).append("\",\n");
    sb.append("  \"robotName\": \"").append(escapeJson(robotName)).append("\",\n");
    sb.append("  \"alliance\": \"").append(escapeJson(alliance)).append("\",\n");
    sb.append("  \"batteryName\": \"").append(escapeJson(batteryName)).append("\",\n");
    sb.append("  \"frames\": [\n");
    for (int i = 0; i < frames.size(); i++) {
      Sim3DFrame f = frames.get(i);
      if (i > 0) {
        sb.append(",\n");
      }
      sb.append(
          String.format(
              Locale.US,
              "    {\"t\":%.2f,\"rx\":%.3f,\"ry\":%.3f,\"yaw\":%.3f,\"vBatt\":%.2f,"
                  + "\"dStator\":%.1f,\"shooterRps\":%.1f,\"hoodRot\":%.2f,\"elevM\":%.3f,"
                  + "\"armRad\":%.3f,\"blueScore\":%d,\"redScore\":%d,\"intaked\":%d,"
                  + "\"banner\":\"%s\",\"balls\":[",
              f.timeSec(),
              f.robotX(),
              f.robotY(),
              f.robotHeadingRad(),
              f.batteryVoltage(),
              f.driveStatorAmps(),
              f.shooterRps(),
              f.hoodRot(),
              f.elevatorHeightM(),
              f.armAngleRad(),
              f.blueScore(),
              f.redScore(),
              f.intakedCount(),
              escapeJson(f.lastShotBanner())));
      double[] b = f.ballXyz();
      if (b != null) {
        for (int k = 0; k < b.length; k++) {
          if (k > 0) {
            sb.append(',');
          }
          sb.append(String.format(Locale.US, "%.2f", b[k]));
        }
      }
      sb.append("]}");
    }
    sb.append("\n  ]\n}\n");
    Files.writeString(simJson, sb.toString());
  }

  private static boolean tryRenderMujocoMp4(Path simJson, Path outMp4) {
    try {
      Path repoRoot = resolveRepoRoot();
      Path scriptPath = repoRoot.resolve("scripts/render_mujoco_replay.py");
      Path meshDir = repoRoot.resolve("assets/meshes");
      if (!Files.exists(scriptPath) || !Files.isDirectory(meshDir)) {
        return false;
      }

      String envPython = System.getenv("FLUX_PYTHON");
      String[] pythonCandidates =
          (envPython != null && !envPython.isBlank())
              ? new String[] {envPython.trim(), "python3", "python"}
              : new String[] {"python3", "python"};
      String selectedPython = null;
      for (String candidate : pythonCandidates) {
        if (candidate.startsWith("/") && !Files.isExecutable(Path.of(candidate))) {
          continue;
        }
        try {
          Process check =
              new ProcessBuilder(candidate, "-c", "import mujoco, PIL")
                  .redirectOutput(ProcessBuilder.Redirect.DISCARD)
                  .redirectError(ProcessBuilder.Redirect.DISCARD)
                  .start();
          boolean checkDone = check.waitFor(4, TimeUnit.SECONDS);
          if (!checkDone) {
            check.destroyForcibly();
          }
          if (checkDone && check.exitValue() == 0) {
            selectedPython = candidate;
            break;
          }
        } catch (Exception ignored) {
        }
      }
      if (selectedPython == null) {
        return false;
      }

      ProcessBuilder pb =
          new ProcessBuilder(
              selectedPython,
              scriptPath.toAbsolutePath().toString(),
              simJson.toAbsolutePath().toString(),
              outMp4.toAbsolutePath().toString(),
              meshDir.toAbsolutePath().toString());
      pb.environment().put("MUJOCO_GL", "egl");
      pb.redirectOutput(ProcessBuilder.Redirect.DISCARD);
      pb.redirectError(ProcessBuilder.Redirect.DISCARD);
      Process proc = pb.start();
      boolean finished = proc.waitFor(180, TimeUnit.SECONDS);
      if (!finished) {
        proc.destroyForcibly();
      }
      return finished && proc.exitValue() == 0 && Files.exists(outMp4) && Files.size(outMp4) > 2000;
    } catch (Exception e) {
      return false;
    }
  }

  private static Path resolveRepoRoot() {
    try {
      Path jarOrClasses =
          Path.of(
                  FuelSimVideoRenderer.class
                      .getProtectionDomain()
                      .getCodeSource()
                      .getLocation()
                      .toURI())
              .toAbsolutePath();
      Path dir = Files.isDirectory(jarOrClasses) ? jarOrClasses : jarOrClasses.getParent();
      for (int depth = 0; depth < 4 && dir != null; depth++) {
        if (Files.exists(dir.resolve("scripts/render_mujoco_replay.py"))
            && Files.isDirectory(dir.resolve("assets/meshes"))) {
          return dir;
        }
        dir = dir.getParent();
      }
    } catch (Exception ignored) {
    }
    return Path.of("").toAbsolutePath();
  }

  private static String escapeJson(String s) {
    if (s == null) {
      return "";
    }
    return s.replace("\\", "\\\\").replace("\"", "\\\"").replace("\n", " ").replace("\r", "");
  }
}
