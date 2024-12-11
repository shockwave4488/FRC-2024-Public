package frc4488.lib.misc;

import java.util.concurrent.CountDownLatch;

public class ThreadNotifier {

  private volatile CountDownLatch latch;
  private volatile Thread waitingThread;

  public ThreadNotifier() {
    latch = new CountDownLatch(1);
  }

  /**
   * NOT TO BE CONFUSED WITH {@link #notify()} and {@link #notifyAll()}, which are from Object and
   * won't do what you expect
   */
  public void notifyThread() {
    latch.countDown();
  }

  /** NOT TO BE CONFUSED WITH {@link #wait()}, which is from Object and won't do what you expect */
  public void waitUntilNotified() throws InterruptedException {
    synchronized (this) {
      if (waitingThread != null) {
        throw new IllegalStateException("Only one thread can wait at a time!");
      }
      waitingThread = Thread.currentThread();
    }
    latch.await();
    latch = new CountDownLatch(1);
    waitingThread = null;
  }
}
