package frc4488.lib.misc;

import java.util.ArrayList;
import java.util.List;

public class TimeUtil {

  private static final int MIN_JUMP_MILLIS =
      60 * 60 * 1000; // Makes this class unreliable for the first hour after flashing
  private static long lastTime = -1;
  private static boolean hasTimeJumped = false;
  private static final List<Runnable> callbacks = new ArrayList<>();

  public static void runOnTimeJump(Runnable callback) {
    if (hasTimeJumped) {
      callback.run();
    } else {
      callbacks.add(callback);
    }
  }

  public static void init() {
    lastTime = System.currentTimeMillis();
  }

  public static void checkTimeJump() {
    if (hasTimeJumped) {
      return;
    }
    if (lastTime == -1) {
      throw new IllegalStateException("Must be initialized!");
    }
    long newTime = System.currentTimeMillis();
    if (newTime - lastTime >= MIN_JUMP_MILLIS) {
      hasTimeJumped = true;

      RuntimeException toThrow = new RuntimeException("Error while calling time jump callbacks!");
      for (Runnable callback : callbacks) {
        try {
          callback.run();
        } catch (Exception e) {
          toThrow.addSuppressed(e);
        }
      }
      if (toThrow.getSuppressed().length > 0) {
        throw toThrow;
      }
    } else {
      lastTime = newTime;
    }
  }
}
