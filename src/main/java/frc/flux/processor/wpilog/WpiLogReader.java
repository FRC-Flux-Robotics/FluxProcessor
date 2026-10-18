package frc.flux.processor.wpilog;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.FileChannel;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Zero-dependency binary parser for WPILib DataLog ({@code .wpilog} v1.0) files.
 *
 * <p>Reads control records ({@code Start}, {@code Finish}, {@code SetMetadata}) and time-series
 * data records ({@code double}, {@code float}, {@code int64}, {@code boolean}, {@code string}) with
 * microsecond timestamps.
 */
public final class WpiLogReader {

  private static final byte[] MAGIC = "WPILOG".getBytes(StandardCharsets.US_ASCII);
  private static final int CONTROL_START = 0;
  private static final int CONTROL_FINISH = 1;
  private static final int CONTROL_SET_METADATA = 2;

  public record EntryDef(int id, String name, String type, String metadata) {}

  public record DoubleSample(long timestampUs, double value) {}

  public record BooleanSample(long timestampUs, boolean value) {}

  public record StringSample(long timestampUs, String value) {}

  public record RawRecord(int entryId, long timestampUs, byte[] payload) {}

  public static final class ParsedLog {
    private final Path sourcePath;
    private final String extraHeader;
    private final Map<Integer, EntryDef> entriesById;
    private final Map<String, EntryDef> entriesByName;
    private final Map<String, List<DoubleSample>> doubleSeries;
    private final Map<String, List<BooleanSample>> booleanSeries;
    private final Map<String, List<StringSample>> stringSeries;
    private final long minTimestampUs;
    private final long maxTimestampUs;

    public ParsedLog(
        Path sourcePath,
        String extraHeader,
        Map<Integer, EntryDef> entriesById,
        Map<String, EntryDef> entriesByName,
        Map<String, List<DoubleSample>> doubleSeries,
        Map<String, List<BooleanSample>> booleanSeries,
        Map<String, List<StringSample>> stringSeries,
        long minTimestampUs,
        long maxTimestampUs) {
      this.sourcePath = sourcePath;
      this.extraHeader = extraHeader;
      this.entriesById = entriesById;
      this.entriesByName = entriesByName;
      this.doubleSeries = doubleSeries;
      this.booleanSeries = booleanSeries;
      this.stringSeries = stringSeries;
      this.minTimestampUs = minTimestampUs;
      this.maxTimestampUs = maxTimestampUs;
    }

    public Path sourcePath() {
      return sourcePath;
    }

    public String extraHeader() {
      return extraHeader;
    }

    public Map<Integer, EntryDef> entriesById() {
      return entriesById;
    }

    public Map<String, EntryDef> entriesByName() {
      return entriesByName;
    }

    public List<DoubleSample> getDoubleSeries(String name) {
      return doubleSeries.getOrDefault(name, Collections.emptyList());
    }

    public List<BooleanSample> getBooleanSeries(String name) {
      return booleanSeries.getOrDefault(name, Collections.emptyList());
    }

    public List<StringSample> getStringSeries(String name) {
      return stringSeries.getOrDefault(name, Collections.emptyList());
    }

    public Map<String, List<DoubleSample>> allDoubleSeries() {
      return doubleSeries;
    }

    public Map<String, List<BooleanSample>> allBooleanSeries() {
      return booleanSeries;
    }

    public Map<String, List<StringSample>> allStringSeries() {
      return stringSeries;
    }

    public long minTimestampUs() {
      return minTimestampUs;
    }

    public long maxTimestampUs() {
      return maxTimestampUs;
    }

    public double durationSeconds() {
      if (maxTimestampUs <= minTimestampUs) {
        return 0.0;
      }
      return (maxTimestampUs - minTimestampUs) / 1_000_000.0;
    }
  }

  /** Reads and parses a {@code .wpilog} file into time-series maps. */
  public static ParsedLog read(Path path) throws IOException {
    try (FileChannel channel = FileChannel.open(path, StandardOpenOption.READ)) {
      long fileSize = channel.size();
      if (fileSize < 12) {
        throw new IOException("Invalid .wpilog file (too small): " + path);
      }
      ByteBuffer buf = channel.map(FileChannel.MapMode.READ_ONLY, 0, fileSize);
      buf.order(ByteOrder.LITTLE_ENDIAN);
      return parseBuffer(path, buf);
    }
  }

  public static ParsedLog parseBuffer(Path sourcePath, ByteBuffer buf) throws IOException {
    buf.order(ByteOrder.LITTLE_ENDIAN);
    for (int i = 0; i < MAGIC.length; i++) {
      if (buf.get() != MAGIC[i]) {
        throw new IOException("Invalid .wpilog magic header in " + sourcePath);
      }
    }
    int version = Short.toUnsignedInt(buf.getShort());
    if (version < 0x0100) {
      throw new IOException("Unsupported .wpilog version: " + Integer.toHexString(version));
    }
    int extraLen = buf.getInt();
    if (extraLen < 0 || extraLen > buf.remaining()) {
      throw new IOException("Corrupt extra header length in " + sourcePath);
    }
    byte[] extraBytes = new byte[extraLen];
    buf.get(extraBytes);
    String extraHeader = new String(extraBytes, StandardCharsets.UTF_8);

    Map<Integer, EntryDef> entriesById = new LinkedHashMap<>();
    Map<String, EntryDef> entriesByName = new LinkedHashMap<>();
    Map<String, List<DoubleSample>> doubleSeries = new LinkedHashMap<>();
    Map<String, List<BooleanSample>> booleanSeries = new LinkedHashMap<>();
    Map<String, List<StringSample>> stringSeries = new LinkedHashMap<>();

    long minTs = Long.MAX_VALUE;
    long maxTs = Long.MIN_VALUE;

    while (buf.hasRemaining()) {
      int headerByte = Byte.toUnsignedInt(buf.get());
      int idLen = (headerByte & 0x03) + 1;
      int sizeLen = ((headerByte >> 2) & 0x03) + 1;
      int tsLen = ((headerByte >> 4) & 0x07) + 1;

      if (buf.remaining() < idLen + sizeLen + tsLen) {
        break;
      }
      int entryId = (int) readUnsignedLe(buf, idLen);
      int payloadSize = (int) readUnsignedLe(buf, sizeLen);
      long timestampUs = readUnsignedLe(buf, tsLen);

      if (payloadSize < 0 || payloadSize > buf.remaining()) {
        break;
      }
      int nextPos = buf.position() + payloadSize;

      if (entryId == 0) {
        if (payloadSize >= 5) {
          int controlType = Byte.toUnsignedInt(buf.get());
          if (controlType == CONTROL_START && buf.remaining() >= 16) {
            int startedId = buf.getInt();
            String name = readLengthPrefixedString(buf, nextPos);
            String type = readLengthPrefixedString(buf, nextPos);
            String metadata = readLengthPrefixedString(buf, nextPos);
            if (name != null && type != null) {
              EntryDef def = new EntryDef(startedId, name, type, metadata != null ? metadata : "");
              entriesById.put(startedId, def);
              entriesByName.put(name, def);
            }
          } else if (controlType == CONTROL_SET_METADATA && buf.remaining() >= 8) {
            int targetId = buf.getInt();
            String metadata = readLengthPrefixedString(buf, nextPos);
            EntryDef old = entriesById.get(targetId);
            if (old != null && metadata != null) {
              EntryDef updated = new EntryDef(old.id(), old.name(), old.type(), metadata);
              entriesById.put(targetId, updated);
              entriesByName.put(old.name(), updated);
            }
          }
        }
      } else {
        // Ignore non-monotonic wall-clock epoch timestamps (> 100,000 seconds of robot uptime)
        boolean isMonotonicTs = timestampUs > 0 && timestampUs < 100_000_000_000L;
        if (isMonotonicTs) {
          if (timestampUs < minTs) {
            minTs = timestampUs;
          }
          if (timestampUs > maxTs) {
            maxTs = timestampUs;
          }
        }
        EntryDef def = entriesById.get(entryId);
        if (def != null && isMonotonicTs) {
          String type = def.type();
          if ("double".equals(type) && payloadSize == 8) {
            double v = buf.getDouble();
            doubleSeries.computeIfAbsent(def.name(), k -> new ArrayList<>()).add(new DoubleSample(timestampUs, v));
          } else if ("float".equals(type) && payloadSize == 4) {
            double v = buf.getFloat();
            doubleSeries.computeIfAbsent(def.name(), k -> new ArrayList<>()).add(new DoubleSample(timestampUs, v));
          } else if ("int64".equals(type) && payloadSize == 8) {
            double v = buf.getLong();
            doubleSeries.computeIfAbsent(def.name(), k -> new ArrayList<>()).add(new DoubleSample(timestampUs, v));
          } else if ("boolean".equals(type) && payloadSize == 1) {
            boolean v = buf.get() != 0;
            booleanSeries.computeIfAbsent(def.name(), k -> new ArrayList<>()).add(new BooleanSample(timestampUs, v));
          } else if (("string".equals(type) || "json".equals(type)) && payloadSize >= 0) {
            byte[] strBytes = new byte[payloadSize];
            buf.get(strBytes);
            String v = new String(strBytes, StandardCharsets.UTF_8);
            stringSeries.computeIfAbsent(def.name(), k -> new ArrayList<>()).add(new StringSample(timestampUs, v));
          }
        }
      }
      buf.position(nextPos);
    }

    if (minTs == Long.MAX_VALUE) {
      minTs = 0;
      maxTs = 0;
    }

    return new ParsedLog(
        sourcePath,
        extraHeader,
        entriesById,
        entriesByName,
        doubleSeries,
        booleanSeries,
        stringSeries,
        minTs,
        maxTs);
  }

  private static long readUnsignedLe(ByteBuffer buf, int numBytes) {
    long result = 0;
    for (int i = 0; i < numBytes; i++) {
      result |= ((long) Byte.toUnsignedInt(buf.get())) << (8 * i);
    }
    return result;
  }

  private static String readLengthPrefixedString(ByteBuffer buf, int limitPos) {
    if (buf.position() + 4 > limitPos) {
      return null;
    }
    int len = buf.getInt();
    if (len < 0 || buf.position() + len > limitPos) {
      return null;
    }
    byte[] bytes = new byte[len];
    buf.get(bytes);
    return new String(bytes, StandardCharsets.UTF_8);
  }
}
