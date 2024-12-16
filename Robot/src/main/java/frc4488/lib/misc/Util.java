package frc4488.lib.misc;

import java.io.IOException;
import java.nio.file.FileVisitResult;
import java.nio.file.FileVisitor;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Predicate;
import java.util.function.Supplier;

public final class Util {
  private Util() {}

  public static <T> T returnAfterModifying(T obj, Consumer<? super T> modifier) {
    modifier.accept(obj);
    return obj;
  }

  public static <T> T lazyInitialize(
      Consumer<T> setVariable, Supplier<T> newValueSupplier, T curValue) {
    if (curValue == null) {
      T newValue = newValueSupplier.get();
      setVariable.accept(newValue);
      return newValue;
    }
    return curValue;
  }

  public static <T> boolean retry(Supplier<T> supplier, Predicate<T> check, int retries) {
    for (int i = 0; i < retries; i++) {
      if (check.test(supplier.get())) {
        return true;
      }
    }
    return false;
  }

  public static void deleteDir(Path dir) throws IOException {
    Files.walkFileTree(
        dir,
        new FileVisitor<>() {

          @Override
          public FileVisitResult preVisitDirectory(Path dir, BasicFileAttributes attrs)
              throws IOException {
            return FileVisitResult.CONTINUE;
          }

          @Override
          public FileVisitResult postVisitDirectory(Path dir, IOException e) throws IOException {
            if (e != null) {
              throw e;
            }
            Files.delete(dir);
            return FileVisitResult.CONTINUE;
          }

          @Override
          public FileVisitResult visitFile(Path file, BasicFileAttributes attrs)
              throws IOException {
            Files.delete(file);
            return FileVisitResult.CONTINUE;
          }

          @Override
          public FileVisitResult visitFileFailed(Path file, IOException e) throws IOException {
            throw e;
          }
        });
  }

  /**
   * Checks if a number is within the specified min and max values.
   *
   * @param number the number to check
   * @return a boolean that is true if the number is within the range, and false if it isn't.
   */
  public static boolean isInRangeWithMinMax(double number, double min, double max) {
    return (number >= min) && (number <= max);
  }

  /**
   * Checks if a number is within a range that is calculated using a base, and a threshold.
   *
   * @param number the number to check
   * @param base the number that is getting added to and subtracted from
   * @param threshold what to add to & subtract from the base to calculate the min and max
   * @return a boolean that is true if the number is within the range, and false if it isn't.
   */
  public static boolean isInRangeWithThreshold(double number, double base, double threshold) {
    return isInRangeWithMinMax(number, base - threshold, base + threshold);
  }

  public static boolean isInOneSidedRangeWithThreshold(
      double number, double base, double threshold) {
    double min = base;
    double max = base;
    if (threshold > 0) {
      max += threshold;
    } else {
      min += threshold;
    }
    return isInRangeWithMinMax(number, min, max);
  }

  /**
   * Checks if a number is within a range that is calculated using a base, and a percentage.
   *
   * @param number the number to check
   * @param base the number that is getting multiplied
   * @param percent what to multiply the base by to calculate the min and max (0.0-1.0)
   * @return a boolean that is true if the number is within the range, and false if it isn't.
   */
  public static boolean isInRangeWithPercentage(double number, double base, double percent) {
    return isInRangeWithThreshold(number, base, base * percent);
  }

  public static <T> Supplier<T> changeFirstReturn(Supplier<T> supplier, T firstValue) {
    return new Supplier<>() {
      private boolean first = true;

      @Override
      public T get() {
        if (first) {
          first = false;
          return firstValue;
        }
        return supplier.get();
      }
    };
  }

  public static BooleanSupplier changeFirstReturn(BooleanSupplier supplier, boolean firstValue) {
    Supplier<Boolean> boxed =
        changeFirstReturn(() -> supplier.getAsBoolean(), (Boolean) firstValue);
    return () -> boxed.get();
  }
}
