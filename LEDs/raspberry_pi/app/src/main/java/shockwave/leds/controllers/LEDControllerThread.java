package shockwave.leds.controllers;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class LEDControllerThread extends Thread {

  @FunctionalInterface
  public interface RunnableIO {
    public void run() throws IOException;
  }

  private static final int FRAME_PERIOD_MILLIS = 16; // 62.5 fps, approximating 60 fps

  private final RunnableIO show;
  private final List<LEDController> controllers;

  public LEDControllerThread(RunnableIO show) {
    super("LED Controller");

    this.show = show;
    this.controllers = new ArrayList<>();
  }

  public void addController(LEDController controller) {
    synchronized (controllers) {
      controllers.add(controller);
    }
  }

  public void clearControllers() {
    synchronized (controllers) {
      controllers.clear();
    }
  }

  @Override
  public void run() {
    long queuedRenderTime = 0;
    long lastTime = System.currentTimeMillis();
    while (!Thread.interrupted()) {
      synchronized (controllers) {
        controllers.removeIf(
            controller -> {
              controller.render();
              return controller.isFinished();
            });
      }
      while (queuedRenderTime < FRAME_PERIOD_MILLIS) {
        long time = System.currentTimeMillis();
        queuedRenderTime += time - lastTime;
        lastTime = time;
      }
      queuedRenderTime -= FRAME_PERIOD_MILLIS;
      if (queuedRenderTime > FRAME_PERIOD_MILLIS) {
        queuedRenderTime = FRAME_PERIOD_MILLIS;
        System.out.println("Dropping frames!");
      }
      try {
        show.run();
      } catch (IOException e) {
        e.printStackTrace();
        break;
      }
    }
  }
}
