package frc4488.lib.dashboard;

import java.io.IOException;
import java.io.OutputStream;
import java.util.function.Consumer;

public class DashboardOutputStream extends OutputStream {

  private final Consumer<String> printer;
  private StringBuilder builder;

  public DashboardOutputStream(Consumer<String> printer) {
    this.printer = printer;
    this.builder = new StringBuilder();
  }

  @Override
  public void write(int b) throws IOException {
    if (b == '\n') {
      printer.accept(builder.toString());
      builder = new StringBuilder();
    } else {
      builder.append((char) b);
    }
  }
}
