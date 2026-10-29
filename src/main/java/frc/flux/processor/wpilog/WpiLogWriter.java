package frc.flux.processor.wpilog;

import frc.lib.fluxfig.FluxWpiLogCodec;
import java.io.Closeable;
import java.io.IOException;
import java.nio.file.Path;

/**
 * Delegating binary writer for WPILib DataLog ({@code .wpilog} v1.0) files backed by the shared
 * {@link FluxWpiLogCodec.Writer} implementation in {@code FluxFig}.
 */
public final class WpiLogWriter implements Closeable {

  private final FluxWpiLogCodec.Writer delegate;

  public WpiLogWriter(Path outputPath, String extraHeader) throws IOException {
    this.delegate =
        new FluxWpiLogCodec.Writer(
            outputPath, extraHeader != null ? extraHeader : "FluxProcessor v0.1");
  }

  public int startEntry(String name, String type, String metadata, long timestampUs)
      throws IOException {
    return delegate.startEntry(name, type, metadata, timestampUs);
  }

  public void appendDouble(int entryId, long timestampUs, double value) throws IOException {
    delegate.appendDouble(entryId, timestampUs, value);
  }

  public void appendDoubleArray(int entryId, long timestampUs, double[] values) throws IOException {
    delegate.appendDoubleArray(entryId, timestampUs, values);
  }

  public void appendBoolean(int entryId, long timestampUs, boolean value) throws IOException {
    delegate.appendBoolean(entryId, timestampUs, value);
  }

  public void appendString(int entryId, long timestampUs, String value) throws IOException {
    delegate.appendString(entryId, timestampUs, value);
  }

  public void appendInt64(int entryId, long timestampUs, long value) throws IOException {
    delegate.appendInt64(entryId, timestampUs, value);
  }

  public void registerStandardGeometrySchemas(long timestampUs) throws IOException {
    delegate.registerStandardGeometrySchemas(timestampUs);
  }

  public void appendPose2d(
      int entryId, long timestampUs, double xMeters, double yMeters, double headingRad)
      throws IOException {
    delegate.appendPose2d(entryId, timestampUs, xMeters, yMeters, headingRad);
  }

  public void appendTranslation3dArray(int entryId, long timestampUs, double[] xyzTriplets)
      throws IOException {
    delegate.appendTranslation3dArray(entryId, timestampUs, xyzTriplets);
  }

  public void writeRecord(int entryId, long timestampUs, byte[] payload) throws IOException {
    delegate.writeRecord(entryId, timestampUs, payload);
  }

  @Override
  public void close() throws IOException {
    delegate.close();
  }
}
