package frc4488.lib.misc;

public record Timed<T>(
    /** The seconds since robot start */
    double time, T value) {

  /** This <em>must</em> be relative to the robot start (<code>Timer.getFPGATimestamp()</code>) */
  public static <T> Timed<T> fromMillis(long millis, T value) {
    return new Timed<>(millis / 1000.0f, value);
  }
}
