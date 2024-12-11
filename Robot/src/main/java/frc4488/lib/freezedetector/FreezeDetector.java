package frc4488.lib.freezedetector;

import java.util.Optional;

public interface FreezeDetector {
  public String getName();

  public Optional<StackTraceElement[]> tryDetect();

  public default void periodic() {}
}
