package frc4488.lib.freezedetector;

import java.util.Optional;

public class PeriodicFreezeDetector implements FreezeDetector {
  private static final int MIN_MILLIS = 100;

  private final Thread thread;
  private long lastPeriodic;

  public PeriodicFreezeDetector(Thread thread) {
    this.thread = thread;
    this.lastPeriodic = -1;
  }

  @Override
  public String getName() {
    return "Periodic Freeze";
  }

  @Override
  public Optional<StackTraceElement[]> tryDetect() {
    if (lastPeriodic != -1 && lastPeriodic < System.currentTimeMillis() - MIN_MILLIS) {
      lastPeriodic = System.currentTimeMillis();
      return Optional.of(thread.getStackTrace());
    }
    return Optional.empty();
  }

  @Override
  public void periodic() {
    lastPeriodic = System.currentTimeMillis();
  }
}
