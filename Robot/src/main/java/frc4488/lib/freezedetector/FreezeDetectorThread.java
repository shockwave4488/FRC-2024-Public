package frc4488.lib.freezedetector;

import frc4488.lib.logging.LogFile;
import frc4488.lib.logging.LogLevel;
import java.util.function.Function;
import java.util.stream.Stream;

public class FreezeDetectorThread {

  public static final Function<Thread, FreezeDetector> STACK = StackFreezeDetector::new;
  public static final Function<Thread, FreezeDetector> PERIODIC = PeriodicFreezeDetector::new;

  private final LogFile log;
  private final boolean printToConsole;
  private final FreezeDetector[] detectors;
  private final Thread detectorThread;

  @SafeVarargs
  public FreezeDetectorThread(
      Thread thread,
      LogFile log,
      boolean printToConsole,
      Function<Thread, FreezeDetector>... detectors) {
    this.log = log;
    this.printToConsole = printToConsole;
    this.detectors =
        Stream.of(detectors).map(detector -> detector.apply(thread)).toArray(FreezeDetector[]::new);
    this.detectorThread = new Thread(this::runDetector);

    detectorThread.setName("Freeze Detector [" + thread.getName() + "]");
    detectorThread.setDaemon(true);
    detectorThread.start();
  }

  public void periodic() {
    for (FreezeDetector detector : detectors) {
      detector.periodic();
    }
  }

  private void runDetector() {
    while (true) {
      for (FreezeDetector detector : detectors) {
        detector.tryDetect().ifPresent(trace -> logStackTrace(detector.getName(), trace));
      }
      try {
        Thread.sleep(10);
      } catch (InterruptedException e) {
        return;
      }
    }
  }

  private void logStackTrace(String context, StackTraceElement[] trace) {
    StringBuilder builder = new StringBuilder(context);
    builder.append(" (");
    builder.append(System.currentTimeMillis());
    builder.append("):");
    for (StackTraceElement element : trace) {
      builder.append("\n  ");
      builder.append(element.getClassName());
      builder.append('#');
      builder.append(element.getMethodName());
      builder.append(" (");
      builder.append(element.getLineNumber());
      builder.append(')');
    }
    log.println(printToConsole ? LogLevel.ERROR_CONSOLE : LogLevel.WARN, builder.toString());
  }
}
