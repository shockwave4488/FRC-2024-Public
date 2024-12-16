package frc4488.lib.misc;

import java.io.IOException;
import java.io.OutputStream;
import java.util.concurrent.CountDownLatch;
import java.util.function.Consumer;

/**
 * Flushes to the provided {@link OutputStream} in a different thread to stop the main thread
 * freezing. <strong>Warning:</strong> While the flushing occurs in a different thread, writing to
 * this stream should still all occur on the same thread
 */
public class FastFlushOutputStream extends OutputStream {

  private final OutputStream out;
  private final Consumer<IOException> onError;
  private final byte[] buffer1;
  private final byte[] buffer2;
  private byte[] buffer;
  private int index;
  private final ThreadNotifier flushingNotifier;
  private final Thread flushingThread;
  private volatile byte[] toFlush;
  private volatile int toFlushLen;
  private volatile CountDownLatch flushingComplete;

  public FastFlushOutputStream(OutputStream out, int bufferSize, Consumer<IOException> onError) {
    this.out = out;
    this.onError = onError;
    this.buffer1 = new byte[bufferSize];
    this.buffer2 = new byte[bufferSize];
    this.buffer = buffer1;
    this.index = 0;
    this.flushingNotifier = new ThreadNotifier();
    this.flushingComplete = new CountDownLatch(0);

    this.flushingThread = new Thread(this::runFlushingThread, "FastFlush [" + out + "]");
    this.flushingThread.start();
  }

  @Override
  public void write(int b) throws IOException {
    if (index == buffer.length) {
      flush();
    }
    buffer[index++] = (byte) b;
  }

  @Override
  public void write(byte[] b, int off, int len) throws IOException {
    if (index == buffer.length) {
      flush();
    }
    if (index + len > buffer.length) {
      int intermediateLen = buffer.length - index;
      write(b, off, intermediateLen);
      write(b, off + intermediateLen, len - intermediateLen);
      return;
    }
    System.arraycopy(b, off, buffer, index, len);
    index += len;
  }

  @Override
  public void flush() {
    try {
      flushingComplete.await();
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      return;
    }

    toFlush = buffer;
    toFlushLen = index;
    buffer = (buffer == buffer1 ? buffer2 : buffer1);
    index = 0;
    flushingComplete = new CountDownLatch(1);
    flushingNotifier.notifyThread();
  }

  public boolean flushIfFree() {
    if (flushingComplete.getCount() == 0) {
      flush();
      return true;
    }
    return false;
  }

  public void flushAndWait() {
    flush();
    try {
      flushingComplete.await();
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }
  }

  private void runFlushingThread() {
    while (true) {
      try {
        flushingNotifier.waitUntilNotified();
      } catch (InterruptedException e) {
        return;
      }

      try {
        synchronized (out) {
          out.write(toFlush, 0, toFlushLen);
          out.flush();
        }
      } catch (IOException e) {
        onError.accept(e);
      }

      flushingComplete.countDown();
    }
  }
}
