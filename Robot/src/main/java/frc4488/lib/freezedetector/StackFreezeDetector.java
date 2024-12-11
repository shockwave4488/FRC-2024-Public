package frc4488.lib.freezedetector;

import frc4488.lib.misc.Timed;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

public class StackFreezeDetector implements FreezeDetector {
  private static final int MAX_HISTORY_MILLIS = 250;
  private static final int MIN_HISTORY_MILLIS = 100;
  private static final int MIN_TRACES = 2;

  private final Thread thread;
  private final List<Timed<StackTraceElement[]>> traces;

  public StackFreezeDetector(Thread thread) {
    this.thread = thread;
    this.traces = new ArrayList<>();
  }

  @Override
  public String getName() {
    return "Stack Freeze";
  }

  @Override
  public Optional<StackTraceElement[]> tryDetect() {
    traces.add(new Timed<>(System.currentTimeMillis(), thread.getStackTrace()));
    while (!traces.isEmpty()
        && traces.get(0).time() < System.currentTimeMillis() - MAX_HISTORY_MILLIS) {
      traces.remove(0);
    }

    if (traces.size() >= MIN_TRACES
        && traces.get(0).time() < System.currentTimeMillis() - MIN_HISTORY_MILLIS) {
      StackTraceElement[] firstTrace = traces.get(0).value();
      boolean matching = true;
      for (int i = 1; i < traces.size(); i++) {
        StackTraceElement[] trace = traces.get(i).value();
        if (!Arrays.equals(firstTrace, trace)) {
          matching = false;
          break;
        }
      }
      if (matching) {
        traces.clear();
        return Optional.of(firstTrace);
      }
    }

    return Optional.empty();
  }
}
