package shockwave.leds;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.TreeMap;

public class CopyMusicTask {

  public static void main(String[] args) throws IOException {
    File sourceFolder = new File("../../../Robot/src/main/deploy/songs");
    File targetFolder = new File("build/classes/java/main/songs");

    sourceFolder.mkdirs();
    targetFolder.mkdirs();

    for (File file : targetFolder.listFiles()) {
      if (file.isFile()) {
        file.delete();
      }
    }

    TreeMap<String, File> files = new TreeMap<>();
    for (File file : sourceFolder.listFiles()) {
      if (file.getName().endsWith(".wav")) {
        files.put(file.getName().substring(0, file.getName().length() - ".wav".length()), file);
      }
    }

    int i = 0;
    for (File file : files.values()) {
      Files.copy(file.toPath(), new File(targetFolder, "song" + i + ".wav").toPath());
      i++;
    }
  }
}
