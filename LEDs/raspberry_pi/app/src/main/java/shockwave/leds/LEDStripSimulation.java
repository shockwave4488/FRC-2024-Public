package shockwave.leds;

import com.google.gson.Gson;
import com.google.gson.JsonObject;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.event.ComponentAdapter;
import java.awt.event.ComponentEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.PipedInputStream;
import java.io.PipedOutputStream;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import javax.swing.JFrame;
import javax.swing.JPanel;
import shockwave.leds.canvas.LEDStrip;
import shockwave.leds.canvas.LEDStrip.LEDEffect;

public class LEDStripSimulation extends JFrame {

  public static LEDStrip showNewSimulation(String name, int width, int height) {
    try {
      PipedOutputStream output = new PipedOutputStream();
      PipedInputStream input = new PipedInputStream(output);
      new LEDStripSimulation(name, width, height, input);
      return new LEDStrip(Math.abs(width * height), LEDEffect.RGB, output);
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  private static final int PIXEL_SIZE = 10;

  private final int numLeds;
  private final InputStream input;
  private final byte[] data;
  private final Thread thread;

  public LEDStripSimulation(String name, int width, int height, InputStream input)
      throws IOException {
    this.numLeds = Math.abs(width * height);
    this.input = input;
    this.data = new byte[numLeds * 3];

    int x = 0;
    int y = 0;
    File config = new File("simulation/" + name + ".json");
    if (config.exists()) {
      JsonObject obj = new Gson().fromJson(Files.readString(config.toPath()), JsonObject.class);
      x = obj.get("x").getAsInt();
      y = obj.get("y").getAsInt();
    }

    setTitle(name);
    setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
    setResizable(false);
    setLocation(x, y);
    JPanel panel =
        new JPanel() {
          @Override
          public void paint(Graphics g) {
            for (int ledIndex = 0; ledIndex < numLeds; ledIndex++) {
              int red = data[ledIndex * 3];
              int green = data[ledIndex * 3 + 1];
              int blue = data[ledIndex * 3 + 2];
              if (red < 0) {
                red += 256;
              }
              if (green < 0) {
                green += 256;
              }
              if (blue < 0) {
                blue += 256;
              }
              g.setColor(new Color(red, green, blue));

              int x = ledIndex % Math.abs(width);
              int y = ledIndex / Math.abs(width);
              if (width < 0) {
                x = Math.abs(width) - x - 1;
              }
              if (height < 0) {
                y = Math.abs(height) - y - 1;
              }
              g.fillRect(x * PIXEL_SIZE, y * PIXEL_SIZE, PIXEL_SIZE, PIXEL_SIZE);
            }
          }
        };
    panel.setSize(Math.max(200, Math.abs(width) * PIXEL_SIZE), Math.abs(height) * PIXEL_SIZE);
    panel.setMinimumSize(panel.getSize());
    panel.setPreferredSize(panel.getSize());
    panel.setMaximumSize(panel.getSize());
    add(panel);
    pack();
    addWindowListener(
        new WindowAdapter() {
          @Override
          public void windowClosed(WindowEvent event) {
            try {
              input.close();
            } catch (IOException e) {
              e.printStackTrace();
            }
          }
        });
    addComponentListener(
        new ComponentAdapter() {
          @Override
          public void componentMoved(ComponentEvent event) {
            JsonObject obj = new JsonObject();
            obj.addProperty("x", getX());
            obj.addProperty("y", getY());
            try {
              Files.writeString(config.toPath(), new Gson().toJson(obj));
            } catch (IOException e) {
              e.printStackTrace();
            }
          }
        });
    setVisible(true);

    this.thread = new Thread(this::receiveData, name);
    this.thread.start();
  }

  // Based on arduino.ino
  private void receiveData() {
    try {
      byte[] data = new byte[numLeds * 3];
      int dataIndex = 0;
      while (true) {
        int first = input.readNBytes(1)[0];
        int second = input.readNBytes(1)[0];

        if ((first & 0b10000000) != (second & 0b10000000)) {
          first = second;
          second = input.readNBytes(1)[0];
          dataIndex = 0;
        }

        if ((first & 0b01000000) == 0) {
          dataIndex = 0;
        }

        data[dataIndex++] = (byte) (((first & 0xF) << 4) + (second & 0xF));

        if (dataIndex == data.length) {
          show(data);
          dataIndex = 0;
        }
      }
    } catch (ArrayIndexOutOfBoundsException e) {
      // Stream closed
    } catch (IOException e) {
      if (e.getMessage().equals("Pipe broken") || e.getMessage().equals("Write end dead")) {
        return;
      }
      e.printStackTrace();
      try {
        input.close();
      } catch (IOException e2) {
        e2.printStackTrace();
      }
    }
  }

  private void show(byte[] data) {
    System.arraycopy(data, 0, this.data, 0, data.length);
    repaint();
  }
}
