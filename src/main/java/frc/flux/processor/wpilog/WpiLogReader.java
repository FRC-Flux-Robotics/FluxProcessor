package frc.flux.processor.wpilog;

import frc.lib.fluxfig.FluxWpiLogCodec;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Delegating binary parser for WPILib DataLog ({@code .wpilog} v1.0) files backed by the shared
 * {@link FluxWpiLogCodec} implementation in {@code FluxFig}.
 */
public final class WpiLogReader {

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

  /** Reads and parses a {@code .wpilog} file into time-series maps via {@link FluxWpiLogCodec}. */
  public static ParsedLog read(Path path) throws IOException {
    return fromCore(FluxWpiLogCodec.read(path));
  }

  /** Parses an in-memory {@code .wpilog} buffer into time-series maps via {@link FluxWpiLogCodec}. */
  public static ParsedLog parseBuffer(Path sourcePath, ByteBuffer buf) throws IOException {
    return fromCore(FluxWpiLogCodec.parseBuffer(sourcePath, buf));
  }

  private static ParsedLog fromCore(FluxWpiLogCodec.ParsedLog core) {
    Map<Integer, EntryDef> entriesById = new LinkedHashMap<>();
    Map<String, EntryDef> entriesByName = new LinkedHashMap<>();
    for (FluxWpiLogCodec.EntryDef e : core.entriesById().values()) {
      EntryDef mapped = new EntryDef(e.id(), e.name(), e.type(), e.metadata());
      entriesById.put(e.id(), mapped);
      entriesByName.put(e.name(), mapped);
    }

    Map<String, List<DoubleSample>> doubleSeries = new LinkedHashMap<>();
    for (Map.Entry<String, List<FluxWpiLogCodec.DoubleSample>> entry :
        core.allDoubleSeries().entrySet()) {
      List<DoubleSample> list = new ArrayList<>(entry.getValue().size());
      for (FluxWpiLogCodec.DoubleSample s : entry.getValue()) {
        list.add(new DoubleSample(s.timestampUs(), s.value()));
      }
      doubleSeries.put(entry.getKey(), list);
    }

    Map<String, List<BooleanSample>> booleanSeries = new LinkedHashMap<>();
    for (Map.Entry<String, List<FluxWpiLogCodec.BooleanSample>> entry :
        core.allBooleanSeries().entrySet()) {
      List<BooleanSample> list = new ArrayList<>(entry.getValue().size());
      for (FluxWpiLogCodec.BooleanSample s : entry.getValue()) {
        list.add(new BooleanSample(s.timestampUs(), s.value()));
      }
      booleanSeries.put(entry.getKey(), list);
    }

    Map<String, List<StringSample>> stringSeries = new LinkedHashMap<>();
    for (Map.Entry<String, List<FluxWpiLogCodec.StringSample>> entry :
        core.allStringSeries().entrySet()) {
      List<StringSample> list = new ArrayList<>(entry.getValue().size());
      for (FluxWpiLogCodec.StringSample s : entry.getValue()) {
        list.add(new StringSample(s.timestampUs(), s.value()));
      }
      stringSeries.put(entry.getKey(), list);
    }

    return new ParsedLog(
        core.sourcePath(),
        core.extraHeader(),
        entriesById,
        entriesByName,
        doubleSeries,
        booleanSeries,
        stringSeries,
        core.minTimestampUs(),
        core.maxTimestampUs());
  }
}
