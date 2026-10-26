package frc.flux.processor.wpilog;

import java.io.BufferedOutputStream;
import java.io.Closeable;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Zero-dependency binary writer for WPILib DataLog ({@code .wpilog} v1.0) files.
 *
 * <p>Used to write unified {@code merged.wpilog} files that combine RoboRIO, Drivetrain CANivore,
 * Mechanism CANivore, voice transcript timelines ({@code /VoiceNotes/Driver}, {@code /VoiceNotes/Operator},
 * {@code /VoiceNotes/Coach}), and computed health signals for AdvantageScope.
 */
public final class WpiLogWriter implements Closeable {

  private static final byte[] MAGIC = "WPILOG".getBytes(StandardCharsets.US_ASCII);
  private static final int CONTROL_START = 0;

  private final OutputStream out;
  private final Map<String, Integer> entryIds = new LinkedHashMap<>();
  private int nextEntryId = 1;

  public WpiLogWriter(Path outputPath, String extraHeader) throws IOException {
    this.out = new BufferedOutputStream(Files.newOutputStream(outputPath), 65536);
    writeHeader(extraHeader != null ? extraHeader : "FluxProcessor v0.1");
  }

  private void writeHeader(String extraHeader) throws IOException {
    byte[] extraBytes = extraHeader.getBytes(StandardCharsets.UTF_8);
    ByteBuffer buf = ByteBuffer.allocate(12 + extraBytes.length).order(ByteOrder.LITTLE_ENDIAN);
    buf.put(MAGIC);
    buf.putShort((short) 0x0100);
    buf.putInt(extraBytes.length);
    buf.put(extraBytes);
    out.write(buf.array());
  }

  public int startEntry(String name, String type, String metadata, long timestampUs)
      throws IOException {
    Integer existing = entryIds.get(name);
    if (existing != null) {
      return existing;
    }
    int id = nextEntryId++;
    entryIds.put(name, id);

    byte[] nameBytes = name.getBytes(StandardCharsets.UTF_8);
    byte[] typeBytes = type.getBytes(StandardCharsets.UTF_8);
    byte[] metaBytes = (metadata != null ? metadata : "").getBytes(StandardCharsets.UTF_8);

    int payloadLen = 1 + 4 + 4 + nameBytes.length + 4 + typeBytes.length + 4 + metaBytes.length;
    ByteBuffer payload = ByteBuffer.allocate(payloadLen).order(ByteOrder.LITTLE_ENDIAN);
    payload.put((byte) CONTROL_START);
    payload.putInt(id);
    payload.putInt(nameBytes.length);
    payload.put(nameBytes);
    payload.putInt(typeBytes.length);
    payload.put(typeBytes);
    payload.putInt(metaBytes.length);
    payload.put(metaBytes);

    writeRecord(0, timestampUs, payload.array());
    return id;
  }

  public void appendDouble(int entryId, long timestampUs, double value) throws IOException {
    ByteBuffer buf = ByteBuffer.allocate(8).order(ByteOrder.LITTLE_ENDIAN);
    buf.putDouble(value);
    writeRecord(entryId, timestampUs, buf.array());
  }

  public void appendDoubleArray(int entryId, long timestampUs, double[] values) throws IOException {
    ByteBuffer buf = ByteBuffer.allocate(values.length * 8).order(ByteOrder.LITTLE_ENDIAN);
    for (double v : values) {
      buf.putDouble(v);
    }
    writeRecord(entryId, timestampUs, buf.array());
  }

  public void appendBoolean(int entryId, long timestampUs, boolean value) throws IOException {
    writeRecord(entryId, timestampUs, new byte[] {(byte) (value ? 1 : 0)});
  }

  public void appendString(int entryId, long timestampUs, String value) throws IOException {
    byte[] bytes = (value != null ? value : "").getBytes(StandardCharsets.UTF_8);
    writeRecord(entryId, timestampUs, bytes);
  }

  public void appendInt64(int entryId, long timestampUs, long value) throws IOException {
    ByteBuffer buf = ByteBuffer.allocate(8).order(ByteOrder.LITTLE_ENDIAN);
    buf.putLong(value);
    writeRecord(entryId, timestampUs, buf.array());
  }

  /**
   * Registers WPILib binary {@code structschema} entries for {@code Pose2d}, {@code Translation2d},
   * {@code Rotation2d}, and {@code Translation3d} so AdvantageScope 2D/3D Field tabs decode {@code
   * struct:Pose2d} and {@code struct:Translation3d[]} topics automatically.
   */
  public void registerStandardGeometrySchemas(long timestampUs) throws IOException {
    writeStructSchema("Translation2d", "double x;double y", timestampUs);
    writeStructSchema("Rotation2d", "double value", timestampUs);
    writeStructSchema(
        "Pose2d", "Translation2d translation;Rotation2d rotation", timestampUs);
    writeStructSchema("Translation3d", "double x;double y;double z", timestampUs);
  }

  private void writeStructSchema(String structName, String schemaDef, long timestampUs)
      throws IOException {
    String topic = "/.schema/struct:" + structName;
    if (entryIds.containsKey(topic)) {
      return;
    }
    int id = startEntry(topic, "structschema", "", timestampUs);
    writeRecord(id, timestampUs, schemaDef.getBytes(StandardCharsets.UTF_8));
  }

  /** Appends a WPILib {@code struct:Pose2d} (24 bytes: {@code x}, {@code y}, {@code rotRad}). */
  public void appendPose2d(
      int entryId, long timestampUs, double xMeters, double yMeters, double headingRad)
      throws IOException {
    ByteBuffer buf = ByteBuffer.allocate(24).order(ByteOrder.LITTLE_ENDIAN);
    buf.putDouble(xMeters);
    buf.putDouble(yMeters);
    buf.putDouble(headingRad);
    writeRecord(entryId, timestampUs, buf.array());
  }

  /**
   * Appends a WPILib {@code struct:Translation3d[]} array ({@code 24 * N} bytes: packed {@code x,
   * y, z} triplets in meters).
   */
  public void appendTranslation3dArray(int entryId, long timestampUs, double[] xyzTriplets)
      throws IOException {
    ByteBuffer buf = ByteBuffer.allocate(xyzTriplets.length * 8).order(ByteOrder.LITTLE_ENDIAN);
    for (double v : xyzTriplets) {
      buf.putDouble(v);
    }
    writeRecord(entryId, timestampUs, buf.array());
  }

  public void writeRecord(int entryId, long timestampUs, byte[] payload) throws IOException {
    // Fixed compact encoding: 4-byte entryId, 4-byte payloadSize, 8-byte timestampUs
    int headerByte = (3) | (3 << 2) | (7 << 4);
    ByteBuffer hdr = ByteBuffer.allocate(1 + 4 + 4 + 8).order(ByteOrder.LITTLE_ENDIAN);
    hdr.put((byte) headerByte);
    hdr.putInt(entryId);
    hdr.putInt(payload.length);
    hdr.putLong(Math.max(0L, timestampUs));
    out.write(hdr.array());
    out.write(payload);
  }

  @Override
  public void close() throws IOException {
    out.flush();
    out.close();
  }
}
